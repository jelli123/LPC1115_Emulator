#include "gdb_stub.h"
#include "emulator.h"
#include "target_halt.h"
#include "usb_descriptors.h"

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "tusb.h"
#include "pico/time.h"

namespace gdb_stub {
namespace {

constexpr uint32_t MAX_PKT = 1024;
constexpr uint32_t LPC_RAM = 0x1000'0000u;

std::atomic<bool> g_active{false};
bool     g_wait_stop  = false;   // 'c'/'s'/^C gesendet, Stop-Antwort ausstehend
uint32_t g_seen_halts = 0;       // target_halt::halt_count() beim letzten Stop
char     g_pkt[MAX_PKT];
size_t   g_pkt_len = 0;

int cdc() { return usb_desc_cdc_gdb(); }

void cdc_write(const char* s, size_t n) {
    const int i = cdc();
    if (i < 0) return;
    const uint8_t itf = static_cast<uint8_t>(i);
    const absolute_time_t deadline = make_timeout_time_ms(500);
    while (n) {
        uint32_t w = tud_cdc_n_write(itf, s, static_cast<uint32_t>(n));
        s += w; n -= w;
        tud_cdc_n_write_flush(itf);
        if (n) { tud_task(); if (time_reached(deadline)) return; }
    }
}

void put_packet(const char* body, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum = static_cast<uint8_t>(sum + body[i]);
    char tail[4];
    std::snprintf(tail, sizeof tail, "#%02x", sum);
    cdc_write("$", 1);
    cdc_write(body, len);
    cdc_write(tail, 3);
}
void put_str(const char* s) { put_packet(s, std::strlen(s)); }

int hexv(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Liest eine Hex-Zahl ab p (bis zu einem Nicht-Hex-Zeichen); p wird verschoben.
uint32_t parse_hex(const char*& p, const char* end) {
    uint32_t v = 0;
    while (p < end && hexv(*p) >= 0) v = (v << 4) | static_cast<uint32_t>(hexv(*p++));
    return v;
}

uint32_t parse_hex_le(const char* s, size_t bytes) {
    uint32_t v = 0;
    for (size_t i = 0; i < bytes; ++i) {
        int hi = hexv(s[i*2]), lo = hexv(s[i*2+1]);
        if (hi < 0 || lo < 0) return v;
        v |= static_cast<uint32_t>((hi << 4) | lo) << (i * 8);
    }
    return v;
}

void emit_hex_le(char* out, uint32_t v, size_t bytes) {
    static const char* hex = "0123456789abcdef";
    for (size_t i = 0; i < bytes; ++i) {
        uint8_t b = static_cast<uint8_t>(v >> (i * 8));
        out[i*2] = hex[b >> 4]; out[i*2+1] = hex[b & 0xF];
    }
}

// --- LPC <-> Host-Adressen fuer Registerwerte -------------------------------
uint32_t to_lpc(uint32_t v) {
    const uint32_t img = emulator::load_base(), ram = emulator::guest_ram_base();
    if (v >= img && v < img + emulator::LPC_LOAD_MAX_SIZE) return v - img;
    if (v >= ram && v <= ram + emulator::LPC_GUEST_RAM_SIZE) return LPC_RAM + (v - ram);
    return v;
}
uint32_t to_host(uint32_t v) {
    if (v < emulator::LPC_LOAD_MAX_SIZE) return emulator::load_base() + v;
    if (v >= LPC_RAM && v <= LPC_RAM + emulator::LPC_GUEST_RAM_SIZE)
        return emulator::guest_ram_base() + (v - LPC_RAM);
    return v;
}

// GDB-Registernummern (ARM): 0..15 = r0..r15, 25 = xPSR (bzw. 16 in 'g').
bool reg_read(unsigned idx, uint32_t& v) {
    if (idx == 25) idx = 16;
    if (!target_halt::read_register(idx, v)) { v = 0; return false; }
    if (idx <= 15) v = to_lpc(v);
    return true;
}
bool reg_write(unsigned idx, uint32_t v) {
    if (idx == 25) idx = 16;
    if (idx == 13 || idx == 14 || idx == 15) v = to_host(v);
    return target_halt::write_register(idx, v);
}

// Haelt den Gast an (fuer '?' beim Verbinden). Wartet kurz auf den Halt.
void halt_sync() {
    if (target_halt::is_halted() || emulator::state() != emulator::State::Running) return;
    target_halt::request_halt();
    const absolute_time_t deadline = make_timeout_time_ms(300);
    while (!target_halt::is_halted() && !time_reached(deadline)) tud_task();
    g_seen_halts = target_halt::halt_count();
}

void handle_packet(const char* p, size_t n) {
    const char* end = p + n;
    char rsp[520];
    if (n == 0) { put_str(""); return; }

    switch (p[0]) {
    case '?':
        halt_sync();
        g_wait_stop = false;
        put_str("S05");
        return;

    case 'g': {
        char* w = rsp;
        for (unsigned i = 0; i < 17; ++i) {
            uint32_t v; reg_read(i == 16 ? 25u : i, v);
            emit_hex_le(w, v, 4); w += 8;
        }
        put_packet(rsp, static_cast<size_t>(w - rsp));
        return;
    }
    case 'G': {
        for (unsigned i = 0; i < 17 && 1 + (i + 1) * 8 <= n; ++i)
            reg_write(i == 16 ? 25u : i, parse_hex_le(p + 1 + i * 8, 4));
        put_str(target_halt::is_halted() ? "OK" : "E01");
        return;
    }
    case 'p': {
        const char* q = p + 1;
        unsigned idx = parse_hex(q, end);
        uint32_t v;
        if (idx > 25 || (idx > 15 && idx != 25)) { put_str("xxxxxxxx"); return; }
        reg_read(idx, v);
        char b[8]; emit_hex_le(b, v, 4); put_packet(b, 8);
        return;
    }
    case 'P': {
        const char* q = p + 1;
        unsigned idx = parse_hex(q, end);
        if (q < end && *q == '=' && end - q >= 9 && reg_write(idx, parse_hex_le(q + 1, 4))) put_str("OK");
        else put_str("E01");
        return;
    }
    case 'm': {
        const char* q = p + 1;
        uint32_t addr = parse_hex(q, end);
        uint32_t len = (q < end && *q == ',') ? (++q, parse_hex(q, end)) : 0;
        if (len > sizeof rsp / 2) len = sizeof rsp / 2;
        uint8_t buf[sizeof rsp / 2];
        if (!target_halt::read_memory(addr, buf, len)) { put_str("E01"); return; }
        for (uint32_t k = 0; k < len; ++k) emit_hex_le(rsp + k * 2, buf[k], 1);
        put_packet(rsp, len * 2);
        return;
    }
    case 'M': {
        const char* q = p + 1;
        uint32_t addr = parse_hex(q, end);
        uint32_t len = (q < end && *q == ',') ? (++q, parse_hex(q, end)) : 0;
        if (q >= end || *q != ':' || static_cast<uint32_t>(end - q - 1) < len * 2 || len > 256) {
            put_str("E01"); return;
        }
        ++q;
        uint8_t buf[256];
        for (uint32_t k = 0; k < len; ++k) buf[k] = static_cast<uint8_t>(parse_hex_le(q + k * 2, 1));
        put_str(target_halt::write_memory(addr, buf, len) ? "OK" : "E01");
        return;
    }
    case 'c':
    case 's': {
        if (n > 1) {                         // c<addr>/s<addr>: PC setzen
            const char* q = p + 1;
            reg_write(15, parse_hex(q, end));
        }
        g_seen_halts = target_halt::halt_count();
        g_wait_stop = true;
        if (target_halt::is_halted()) {
            if (p[0] == 's') target_halt::request_step();
            else             target_halt::request_resume();
        }
        return;                              // Antwort beim naechsten Halt
    }
    case 'D':
        g_wait_stop = false;
        if (target_halt::is_halted()) target_halt::request_resume();
        put_str("OK");
        return;
    case 'k':
        g_wait_stop = false;
        if (target_halt::is_halted()) target_halt::request_resume();
        return;
    case 'H':
    case 'T':
        put_str("OK");
        return;
    case 'Z':
    case 'z': {
        if (n < 4 || p[1] != '0' || p[2] != ',') { put_str(""); return; }   // nur SW-Breakpoints
        const char* q = p + 3;
        uint32_t addr = parse_hex(q, end);
        bool ok = (p[0] == 'Z') ? target_halt::set_breakpoint(addr)
                                : target_halt::clear_breakpoint(addr);
        put_str(ok || p[0] == 'z' ? "OK" : "E01");
        return;
    }
    case 'q':
        if (n >= 10 && !std::memcmp(p, "qSupported", 10)) {
            std::snprintf(rsp, sizeof rsp, "PacketSize=%lx", static_cast<unsigned long>(MAX_PKT));
            put_str(rsp);
        } else if (n >= 9 && !std::memcmp(p, "qAttached", 9)) put_str("1");
        else if (n == 2 && !std::memcmp(p, "qC", 2))            put_str("QC1");
        else if (n >= 12 && !std::memcmp(p, "qfThreadInfo", 12)) put_str("m1");
        else if (n >= 12 && !std::memcmp(p, "qsThreadInfo", 12)) put_str("l");
        else put_str("");
        return;
    default:
        put_str("");                         // unbekannt (auch vMustReplyEmpty, vCont?)
        return;
    }
}

void rx_byte(char c) {
    static enum { Idle, InPkt, CkHi, CkLo } st = Idle;
    static uint8_t calc = 0, want = 0;
    switch (st) {
    case Idle:
        if (c == '$') { g_pkt_len = 0; calc = 0; st = InPkt; }
        else if (c == 0x03) {                // ^C: asynchron anhalten
            g_seen_halts = target_halt::halt_count();
            g_wait_stop = true;
            target_halt::request_halt();
        }
        break;
    case InPkt:
        if (c == '#') st = CkHi;
        else {
            if (g_pkt_len < MAX_PKT - 1) g_pkt[g_pkt_len++] = c;
            calc = static_cast<uint8_t>(calc + c);
        }
        break;
    case CkHi: want = static_cast<uint8_t>(hexv(c) << 4); st = CkLo; break;
    case CkLo: {
        want = static_cast<uint8_t>(want | hexv(c));
        const char ack = (want == calc) ? '+' : '-';
        cdc_write(&ack, 1);
        st = Idle;
        if (ack == '+') handle_packet(g_pkt, g_pkt_len);
        break;
    }
    }
}

} // namespace

void init() {}

void poll() {
    const int i = cdc();
    if (i < 0) return;
    const uint8_t itf = static_cast<uint8_t>(i);
    if (!g_active.load()) {
        // Inaktiv: Eingang verwerfen, sonst arbeitet ein spaeteres 'gdb on'
        // veraltete Pakete einer frueheren Sitzung ab (verschobene Antworten).
        char junk[64];
        while (tud_cdc_n_available(itf)) tud_cdc_n_read(itf, junk, sizeof junk);
        return;
    }
    // Stop-Antwort nachreichen, sobald der Gast (erneut) angehalten hat.
    if (g_wait_stop && target_halt::is_halted() &&
        target_halt::halt_count() != g_seen_halts) {
        g_wait_stop = false;
        g_seen_halts = target_halt::halt_count();
        put_str("S05");
    }
    char buf[64];
    while (tud_cdc_n_available(itf)) {
        uint32_t r = tud_cdc_n_read(itf, buf, sizeof buf);
        for (uint32_t k = 0; k < r; ++k) rx_byte(buf[k]);
    }
}

void start() {
    int itf = cdc();
    g_active.store(true);
    g_wait_stop = false;
    if (itf >= 0) std::printf("[GDB] aktiviert (CDC #%d)\n", itf);
    else          std::printf("[GDB] aktiviert, aber GDB-CDC ist per Config deaktiviert\n");
}
void stop()  {
    g_active.store(false);
    if (target_halt::is_halted()) target_halt::request_resume();
    std::printf("[GDB] deaktiviert\n");
}
bool active() { return g_active.load(); }
uint16_t port_index() { int i = cdc(); return i >= 0 ? static_cast<uint16_t>(i) : 0xFFFFu; }

} // namespace gdb_stub
