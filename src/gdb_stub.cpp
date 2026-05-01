#include "gdb_stub.h"
#include "emulator.h"

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "tusb.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "RP2350.h"

// GDB-Stub für 17 Register: r0..r12, sp, lr, pc, xpsr.

namespace gdb_stub {
namespace {

constexpr uint8_t  CDC_GDB_ITF      = 1;     // CDC #0 = stdio, CDC #1 = GDB
constexpr uint32_t MAX_BP           = 16;
constexpr uint32_t MAX_PKT          = 1024;

std::atomic<bool>     g_active{false};
std::atomic<bool>     g_halted{false};
std::atomic<bool>     g_continue{false};
std::atomic<bool>     g_step{false};

uint32_t*             g_frame  = nullptr;     // exception-stacked frame
uint32_t*             g_r4_r11 = nullptr;
char                  g_pkt[MAX_PKT];
size_t                g_pkt_len = 0;

struct BP {
    uint32_t addr;
    uint16_t saved;
    bool     used;
};
BP g_bp[MAX_BP]{};

inline bool cdc_avail() { return tud_cdc_n_connected(CDC_GDB_ITF); }

void cdc_write(const char* s, size_t n) {
    if (!cdc_avail()) return;
    while (n) {
        size_t w = tud_cdc_n_write(CDC_GDB_ITF, s, static_cast<uint32_t>(n));
        s += w; n -= w;
        tud_cdc_n_write_flush(CDC_GDB_ITF);
        tud_task();
    }
}

void put_packet(const char* body, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum = static_cast<uint8_t>(sum + body[i]);
    char hdr[2] = {'$', 0};
    char tail[4]; std::snprintf(tail, sizeof tail, "#%02x", sum);
    cdc_write(hdr, 1);
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

uint32_t parse_hex_le(const char* s, size_t bytes) {
    uint32_t v = 0;
    for (size_t i = 0; i < bytes; ++i) {
        int hi = hexv(s[i*2]); int lo = hexv(s[i*2+1]);
        if (hi < 0 || lo < 0) return v;
        v |= static_cast<uint32_t>(((hi << 4) | lo) & 0xFF) << (i * 8);
    }
    return v;
}

void emit_hex_le(char* out, uint32_t v, size_t bytes) {
    static const char* hex = "0123456789abcdef";
    for (size_t i = 0; i < bytes; ++i) {
        uint8_t b = static_cast<uint8_t>((v >> (i * 8)) & 0xFFu);
        out[i*2]   = hex[b >> 4];
        out[i*2+1] = hex[b & 0xF];
    }
}

uint32_t* reg_ptr(unsigned idx) {
    if (!g_frame) return nullptr;
    // Frame: r0,r1,r2,r3,r12,lr,pc,xpsr (Indizes 0..7)
    // Wir bilden ab: 0..3=r0..r3, 4..11=r4..r11, 12=r12, 13=sp, 14=lr, 15=pc, 16=xpsr
    static thread_local uint32_t sp_cache;
    switch (idx) {
        case 0: case 1: case 2: case 3: return &g_frame[idx];
        case 4: case 5: case 6: case 7: case 8: case 9: case 10: case 11:
            return g_r4_r11 ? &g_r4_r11[idx - 4] : nullptr;
        case 12: return &g_frame[4];
        case 13: sp_cache = reinterpret_cast<uint32_t>(g_frame) + 0x20; return &sp_cache;
        case 14: return &g_frame[5];
        case 15: return &g_frame[6];
        case 16: return &g_frame[7];
    }
    return nullptr;
}

bool addr_in_guest_ram(uint32_t a) {
    return a >= emulator::LPC_LOAD_BASE
        && a <  emulator::LPC_LOAD_BASE + emulator::LPC_LOAD_MAX_SIZE;
}

bool insert_breakpoint(uint32_t addr) {
    if (!addr_in_guest_ram(addr)) return false;
    for (auto& b : g_bp) if (b.used && b.addr == addr) return true;
    for (auto& b : g_bp) if (!b.used) {
        auto* p = reinterpret_cast<uint16_t*>(addr);
        b.addr = addr; b.saved = *p; b.used = true;
        *p = 0xBE00;                      // BKPT #0
        __DSB(); __ISB();
        return true;
    }
    return false;
}

bool remove_breakpoint(uint32_t addr) {
    for (auto& b : g_bp) if (b.used && b.addr == addr) {
        *reinterpret_cast<uint16_t*>(addr) = b.saved;
        b.used = false;
        __DSB(); __ISB();
        return true;
    }
    return false;
}

void handle_packet(const char* p, size_t n) {
    if (n == 0) { put_str(""); return; }
    char cmd = p[0];
    char rsp[256];

    switch (cmd) {
        case '?':
            put_str("S05");                     // SIGTRAP
            return;
        case 'g': {                             // read all regs
            char* w = rsp;
            for (unsigned i = 0; i < 17; ++i) {
                uint32_t v = 0;
                if (auto* r = reg_ptr(i)) v = *r;
                emit_hex_le(w, v, 4); w += 8;
            }
            put_packet(rsp, static_cast<size_t>(w - rsp));
            return;
        }
        case 'G': {                             // write all regs
            for (unsigned i = 0; i < 17 && (1 + i*8 + 8) <= n; ++i) {
                uint32_t v = parse_hex_le(p + 1 + i*8, 4);
                if (auto* r = reg_ptr(i)) *r = v;
            }
            put_str("OK");
            return;
        }
        case 'p': {                             // read single reg
            unsigned idx = 0;
            for (size_t i = 1; i < n; ++i) {
                int v = hexv(p[i]); if (v < 0) break;
                idx = (idx << 4) | static_cast<unsigned>(v);
            }
            uint32_t v = 0;
            if (auto* r = reg_ptr(idx)) v = *r;
            char buf[9]; emit_hex_le(buf, v, 4); put_packet(buf, 8);
            return;
        }
        case 'P': {                             // write single reg
            unsigned idx = 0; size_t i = 1;
            for (; i < n && p[i] != '='; ++i) {
                int v = hexv(p[i]); if (v < 0) break;
                idx = (idx << 4) | static_cast<unsigned>(v);
            }
            if (i < n && p[i] == '=' && i + 8 < n) {
                uint32_t val = parse_hex_le(p + i + 1, 4);
                if (auto* r = reg_ptr(idx)) *r = val;
                put_str("OK");
            } else put_str("E01");
            return;
        }
        case 'm': {                             // read mem: m<addr>,<len>
            uint32_t addr = 0, len = 0;
            size_t i = 1;
            for (; i < n && p[i] != ','; ++i) {
                int v = hexv(p[i]); if (v < 0) break;
                addr = (addr << 4) | static_cast<uint32_t>(v);
            }
            if (i < n && p[i] == ',') {
                for (++i; i < n; ++i) {
                    int v = hexv(p[i]); if (v < 0) break;
                    len = (len << 4) | static_cast<uint32_t>(v);
                }
            }
            if (len > sizeof(rsp)/2) len = sizeof(rsp)/2;
            const auto* src = reinterpret_cast<volatile uint8_t*>(addr);
            for (uint32_t k = 0; k < len; ++k)
                emit_hex_le(rsp + k*2, src[k], 1);
            put_packet(rsp, len * 2);
            return;
        }
        case 'M': {                             // write mem
            uint32_t addr = 0, len = 0;
            size_t i = 1;
            for (; i < n && p[i] != ','; ++i) {
                int v = hexv(p[i]); if (v < 0) break;
                addr = (addr << 4) | static_cast<uint32_t>(v);
            }
            if (i < n && p[i] == ',') {
                for (++i; i < n && p[i] != ':'; ++i) {
                    int v = hexv(p[i]); if (v < 0) break;
                    len = (len << 4) | static_cast<uint32_t>(v);
                }
            }
            if (i < n && p[i] == ':') {
                ++i;
                auto* dst = reinterpret_cast<uint8_t*>(addr);
                for (uint32_t k = 0; k < len && i + 1 < n; ++k, i += 2) {
                    int hi = hexv(p[i]); int lo = hexv(p[i+1]);
                    if (hi < 0 || lo < 0) break;
                    dst[k] = static_cast<uint8_t>((hi << 4) | lo);
                }
                __DSB(); __ISB();
                put_str("OK");
            } else put_str("E01");
            return;
        }
        case 'c':
            g_continue.store(true);
            // Antwort kommt erst beim nächsten Halt
            return;
        case 's':
            g_step.store(true);
            g_continue.store(true);
            return;
        case 'Z':
        case 'z': {
            // Z/z<type>,<addr>,<kind>
            if (n >= 4 && p[1] == '0') {
                uint32_t addr = 0;
                size_t i = 3;
                for (; i < n && p[i] != ','; ++i) {
                    int v = hexv(p[i]); if (v < 0) break;
                    addr = (addr << 4) | static_cast<uint32_t>(v);
                }
                bool ok = (cmd == 'Z') ? insert_breakpoint(addr)
                                       : remove_breakpoint(addr);
                put_str(ok ? "OK" : "E01");
            } else put_str("");
            return;
        }
        case 'q': {
            if (n >= 10 && std::memcmp(p, "qSupported", 10) == 0) {
                std::snprintf(rsp, sizeof rsp, "PacketSize=%x;swbreak+", MAX_PKT);
                put_str(rsp);
            } else if (n >= 9 && std::memcmp(p, "qAttached", 9) == 0) {
                put_str("1");
            } else if (n >= 2 && std::memcmp(p, "qC", 2) == 0) {
                put_str("QC1");
            } else if (n >= 11 && std::memcmp(p, "qfThreadInfo", 11) == 0) {
                put_str("m1");
            } else if (n >= 11 && std::memcmp(p, "qsThreadInfo", 11) == 0) {
                put_str("l");
            } else {
                put_str("");
            }
            return;
        }
        case 'v': {
            if (n >= 6 && std::memcmp(p, "vCont?", 6) == 0) {
                put_str("vCont;c;s");
            } else if (n >= 7 && std::memcmp(p, "vCont;c", 7) == 0) {
                g_continue.store(true);
            } else if (n >= 7 && std::memcmp(p, "vCont;s", 7) == 0) {
                g_step.store(true); g_continue.store(true);
            } else {
                put_str("");
            }
            return;
        }
    }
    put_str("");
}

void rx_byte(char c) {
    static enum { Idle, InPkt, CkHi, CkLo } st = Idle;
    static uint8_t calc_sum = 0;
    static uint8_t want_sum = 0;

    switch (st) {
        case Idle:
            if (c == '$') { g_pkt_len = 0; calc_sum = 0; st = InPkt; }
            else if (c == 0x03) {                   // Ctrl-C async halt
                SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; // PendSV → Halt-Pfad
            }
            break;
        case InPkt:
            if (c == '#') { st = CkHi; }
            else if (g_pkt_len < MAX_PKT - 1) {
                g_pkt[g_pkt_len++] = c;
                calc_sum = static_cast<uint8_t>(calc_sum + c);
            }
            break;
        case CkHi: want_sum = static_cast<uint8_t>(hexv(c) << 4); st = CkLo; break;
        case CkLo: {
            want_sum = static_cast<uint8_t>(want_sum | hexv(c));
            const char ack = (want_sum == calc_sum) ? '+' : '-';
            cdc_write(&ack, 1);
            if (ack == '+') handle_packet(g_pkt, g_pkt_len);
            st = Idle;
            break;
        }
    }
}

} // namespace

void init() { /* USB-CDC #1 wird vom TinyUSB-Stack mitkonfiguriert */ }

void poll() {
    tud_task();
    if (!g_active.load()) return;
    if (!cdc_avail())     return;
    while (tud_cdc_n_available(CDC_GDB_ITF)) {
        char c;
        if (tud_cdc_n_read(CDC_GDB_ITF, &c, 1) == 1) rx_byte(c);
    }
}

void on_breakpoint(uint32_t* exc_frame, uint32_t* r4_r11) {
    if (!g_active.load()) return;
    g_frame  = exc_frame;
    g_r4_r11 = r4_r11;
    g_halted.store(true);
    g_continue.store(false);
    g_step.store(false);

    // Stop-Reason an GDB schicken
    put_str("S05");

    // Spinnen bis GDB continue / step kommandiert
    while (!g_continue.load()) {
        poll();
        sleep_us(100);
    }
    g_continue.store(false);
    g_halted.store(false);

    // Bei Step: DEMCR.MON_EN+MON_STEP setzen — der Cortex-M33 löst nach
    // genau einer ausgeführten Instruktion DebugMonitor aus, der wieder
    // bei uns landet (sieht in fault.cpp::debugmon_c).
    if (g_step.load()) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk
                         |  CoreDebug_DEMCR_MON_STEP_Msk;
        g_step.store(false);
    }

    g_frame = nullptr; g_r4_r11 = nullptr;
}

void start() { g_active.store(true);  std::printf("[GDB] aktiviert (CDC #%u)\n", CDC_GDB_ITF); }
void stop()  { g_active.store(false); std::printf("[GDB] deaktiviert\n"); }
bool active() { return g_active.load(); }
uint16_t port_index() { return CDC_GDB_ITF; }

} // namespace gdb_stub
