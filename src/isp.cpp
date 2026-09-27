#include "isp.h"

#include "config.h"
#include "emulator.h"
#include "iap.h"
#include "peripherals.h"
#include "storage.h"
#include "usb_descriptors.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "tusb.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

namespace isp {

namespace {

// --- LPC1115 ISP-Konstanten (UM10398 Kap. 26) -----------------------------
constexpr uint32_t PART_ID        = 0x0005'0080u;   // LPC1115/303
constexpr unsigned BOOT_VER_MAJOR = 7;
constexpr unsigned BOOT_VER_MINOR = 1;
constexpr uint32_t UNLOCK_CODE    = 23130;
constexpr uint32_t FLASH_BYTES    = 64u * 1024u;
constexpr uint32_t SECTOR_BYTES   = 4096;
constexpr uint32_t NUM_SECTORS    = FLASH_BYTES / SECTOR_BYTES;
constexpr uint32_t RAM_BASE       = 0x1000'0000u;
constexpr uint32_t RAM_BYTES      = 8u * 1024u;
constexpr uint32_t CRP_ADDR       = 0x2FC;
constexpr uint32_t CRP1 = 0x1234'5678u, CRP2 = 0x8765'4321u,
                   CRP3 = 0x4321'8765u, NO_ISP = 0x4E69'7370u;

// ISP-Rueckgabecodes.
enum : unsigned {
    CMD_SUCCESS = 0, INVALID_COMMAND = 1, SRC_ADDR_ERROR = 2, DST_ADDR_ERROR = 3,
    SRC_ADDR_NOT_MAPPED = 4, DST_ADDR_NOT_MAPPED = 5, COUNT_ERROR = 6,
    INVALID_SECTOR = 7, SECTOR_NOT_BLANK = 8, SECTOR_NOT_PREPARED = 9,
    COMPARE_ERROR = 10, PARAM_ERROR = 12, ADDR_ERROR = 13, ADDR_NOT_MAPPED = 14,
    CMD_LOCKED = 15, INVALID_CODE = 16, INVALID_BAUD_RATE = 17,
    INVALID_STOP_BIT = 18, CODE_READ_PROTECTION = 19,
};

constexpr uint8_t LPC_PIN_RESET = 0;   // P0_0 (RESET/PIO0_0) in der Pinmap
constexpr uint8_t LPC_PIN_ISP   = 1;   // P0_1 (ISP-Entry)

enum class State : uint8_t { Sync, SyncReply, Freq, Cmd, WriteData, ReadWait };

// --- Sitzungszustand ------------------------------------------------------
bool      g_active   = false;
Transport g_tp       = Transport::None;
State     g_state    = State::Sync;
bool      g_echo     = true;
bool      g_unlocked = false;
uint16_t  g_prepared = 0;
char      g_line[160];
uint32_t  g_len      = 0;
bool      g_cr_pending = false;   // CR empfangen, Zeile wartet auf LF
uint32_t  g_cr_us      = 0;

// ISP-Sicht des LPC-RAM (W/C/M/R) = der Gast-RAM-Puffer. Der Gast ist waehrend
// des ISP gestoppt; wie auf dem echten Chip teilen sich ISP und Anwendung den RAM.
uint8_t* ram() { return reinterpret_cast<uint8_t*>(emulator::guest_ram_base()); }

// W (Write to RAM): laufender 20-Zeilen-Block, fuer RESEND zurueckspulbar.
uint32_t g_w_off = 0, g_w_left = 0, g_w_blk_off = 0, g_w_blk_left = 0;
uint32_t g_w_lines = 0, g_w_sum = 0;
// R (Read memory): aktueller Block.
uint32_t g_r_addr = 0, g_r_left = 0, g_r_blk_len = 0;

// Flash-Aenderungen werden nach kurzer Ruhe bzw. beim Verlassen festgeschrieben.
bool     g_flash_dirty = false;
uint32_t g_last_flash_ms = 0;

// UART-Transport.
uart_inst_t* g_uart    = nullptr;
int          g_rx_gpio = -1;
uint32_t     g_baud    = 0;
bool         g_autobaud = false;

// RESET-/ISP-Steuerung.
bool     g_pin_reset_held = false;   // RESET-Pin (P0_0) low
bool     g_cdc_reset_held = false;   // DTR aktiv auf der ISP-CDC
int      g_reset_gpio_cfg = -1;      // aktuell als RESET-Eingang konfigurierter GPIO
bool     g_reset_level    = true;    // entprellter Pegel
uint32_t g_reset_change_ms = 0;
volatile bool g_ls_pending = false;  // Line-State-Aenderung aus dem TinyUSB-Callback
volatile bool g_ls_dtr = false, g_ls_rts = false;

uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

// --- Transport-I/O --------------------------------------------------------
void tx(const char* s, std::size_t n) {
    if (g_tp == Transport::Uart && g_uart) {
        uart_write_blocking(g_uart, reinterpret_cast<const uint8_t*>(s), n);
        return;
    }
    const int itf_i = usb_desc_cdc_isp();
    if (g_tp != Transport::Cdc || itf_i < 0) return;
    const uint8_t itf = static_cast<uint8_t>(itf_i);
    const absolute_time_t deadline = make_timeout_time_ms(500);
    while (n > 0) {
        uint32_t avail = tud_cdc_n_write_available(itf);
        if (avail == 0) {
            tud_cdc_n_write_flush(itf);
            tud_task();
            if (time_reached(deadline)) return;   // Host liest nicht -> verwerfen
            continue;
        }
        uint32_t chunk = n < avail ? static_cast<uint32_t>(n) : avail;
        tud_cdc_n_write(itf, s, chunk);
        s += chunk; n -= chunk;
    }
    tud_cdc_n_write_flush(itf);
}

void txs(const char* s) { tx(s, std::strlen(s)); }

void txf(const char* fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) tx(buf, static_cast<std::size_t>(n < (int)sizeof buf ? n : (int)sizeof buf - 1));
}

void reply(unsigned code) { txf("%u\r\n", code); }

int rx_byte() {
    if (g_tp == Transport::Uart) {
        if (g_uart && uart_is_readable(g_uart)) return uart_getc(g_uart);
        return -1;
    }
    const int itf = usb_desc_cdc_isp();
    if (itf < 0 || !tud_cdc_n_available(static_cast<uint8_t>(itf))) return -1;
    uint8_t c;
    return tud_cdc_n_read(static_cast<uint8_t>(itf), &c, 1) ? c : -1;
}

// --- Speicherzugriff ------------------------------------------------------
bool in_flash(uint32_t a, uint32_t n) { return a <= FLASH_BYTES && n <= FLASH_BYTES - a; }
bool in_ram(uint32_t a, uint32_t n) {
    return a >= RAM_BASE && a - RAM_BASE <= RAM_BYTES && n <= RAM_BYTES - (a - RAM_BASE);
}

bool read_mem(uint32_t a, uint8_t* dst, uint32_t n) {
    if (in_flash(a, n)) return storage::firmware_read(a, dst, n);
    if (in_ram(a, n))   { std::memcpy(dst, ram() + (a - RAM_BASE), n); return true; }
    return false;
}

uint32_t crp_level() {
    uint32_t w = 0xFFFF'FFFFu;
    storage::firmware_read(CRP_ADDR, &w, 4);
    switch (w) {
        case CRP1: return 1;
        case CRP2: return 2;
        case CRP3: return 3;
        default:   return 0;
    }
}
bool crp_no_isp_entry() {
    uint32_t w = 0xFFFF'FFFFu;
    storage::firmware_read(CRP_ADDR, &w, 4);
    return w == CRP3 || w == NO_ISP;
}

void mark_flash_dirty() { g_flash_dirty = true; g_last_flash_ms = now_ms(); }

void commit_flash() {
    if (!g_flash_dirty) return;
    storage::firmware_commit();
    g_flash_dirty = false;
}

bool sectors_prepared(uint32_t s, uint32_t e) {
    for (uint32_t i = s; i <= e; ++i) if (!(g_prepared & (1u << i))) return false;
    return true;
}
void unprepare(uint32_t s, uint32_t e) {
    for (uint32_t i = s; i <= e; ++i) g_prepared &= static_cast<uint16_t>(~(1u << i));
}

// --- UU-Kodierung ---------------------------------------------------------
char uu_char(uint32_t v) { v &= 0x3Fu; return v ? static_cast<char>(0x20 + v) : '`'; }

// Dekodiert eine UU-Zeile; liefert die Anzahl Bytes (Laengenzeichen).
uint32_t uu_decode(const char* line, uint8_t* out, uint32_t max) {
    if (!line[0]) return 0;
    uint32_t n = (static_cast<uint8_t>(line[0]) - 0x20u) & 0x3Fu;
    if (n > max) n = max;
    const char* p = line + 1;
    for (uint32_t i = 0; i < n; i += 3) {
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k) {
            uint32_t c = *p ? ((static_cast<uint8_t>(*p++) - 0x20u) & 0x3Fu) : 0u;
            v = (v << 6) | c;
        }
        for (uint32_t k = 0; k < 3 && i + k < n; ++k)
            out[i + k] = static_cast<uint8_t>(v >> (16 - 8 * k));
    }
    return n;
}

void uu_send_line(const uint8_t* d, uint32_t n) {
    char buf[2 + 60 + 2];
    uint32_t p = 0;
    buf[p++] = uu_char(n);
    for (uint32_t i = 0; i < n; i += 3) {
        uint32_t v = static_cast<uint32_t>(d[i]) << 16;
        if (i + 1 < n) v |= static_cast<uint32_t>(d[i + 1]) << 8;
        if (i + 2 < n) v |= d[i + 2];
        buf[p++] = uu_char(v >> 18); buf[p++] = uu_char(v >> 12);
        buf[p++] = uu_char(v >> 6);  buf[p++] = uu_char(v);
    }
    buf[p++] = '\r'; buf[p++] = '\n';
    tx(buf, p);
}

// Sendet den naechsten R-Block (max. 20 Zeilen a 45 Byte) + Pruefsumme.
void r_send_block() {
    uint32_t sum = 0, sent = 0;
    uint8_t  buf[45];
    for (int line = 0; line < 20 && sent < g_r_left; ++line) {
        uint32_t n = g_r_left - sent < 45u ? g_r_left - sent : 45u;
        read_mem(g_r_addr + sent, buf, n);
        for (uint32_t i = 0; i < n; ++i) sum += buf[i];
        uu_send_line(buf, n);
        sent += n;
    }
    g_r_blk_len = sent;
    txf("%lu\r\n", static_cast<unsigned long>(sum));
    g_state = State::ReadWait;
}

// --- Kommandos ------------------------------------------------------------
void exec(char* line) {
    char* argv[5] = {};
    int argc = 0;
    for (char* t = std::strtok(line, " "); t && argc < 5; t = std::strtok(nullptr, " "))
        argv[argc++] = t;
    if (argc == 0) return;
    if (argv[0][1] != '\0') { reply(INVALID_COMMAND); return; }
    uint32_t a[4] = {};
    for (int i = 1; i < argc && i < 5; ++i)
        a[i - 1] = static_cast<uint32_t>(std::strtoul(argv[i], nullptr, 10));
    const int n = argc - 1;
    const uint32_t crp = crp_level();

    switch (argv[0][0]) {
    case 'U':
        if (n != 1) { reply(PARAM_ERROR); return; }
        if (a[0] != UNLOCK_CODE) { reply(INVALID_CODE); return; }
        g_unlocked = true; reply(CMD_SUCCESS); return;

    case 'A':
        if (n != 1 || a[0] > 1) { reply(PARAM_ERROR); return; }
        g_echo = a[0] != 0; reply(CMD_SUCCESS); return;

    case 'B': {
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (a[0] < 1200 || a[0] > 1'000'000) { reply(INVALID_BAUD_RATE); return; }
        if (a[1] != 1 && a[1] != 2) { reply(INVALID_STOP_BIT); return; }
        reply(CMD_SUCCESS);
        if (g_tp == Transport::Uart && g_uart) {
            uart_tx_wait_blocking(g_uart);
            g_baud = uart_set_baudrate(g_uart, a[0]);
            uart_set_format(g_uart, 8, a[1], UART_PARITY_NONE);
        }
        return;
    }

    case 'J': txf("%u\r\n%lu\r\n", CMD_SUCCESS, static_cast<unsigned long>(PART_ID)); return;
    case 'K': txf("%u\r\n%u\r\n%u\r\n", CMD_SUCCESS, BOOT_VER_MINOR, BOOT_VER_MAJOR); return;
    case 'N': {
        uint32_t w[4];
        iap::read_uid(w);
        txf("%u\r\n%lu\r\n%lu\r\n", CMD_SUCCESS,
            static_cast<unsigned long>(w[0]), static_cast<unsigned long>(w[1]));
        txf("%lu\r\n%lu\r\n", static_cast<unsigned long>(w[2]), static_cast<unsigned long>(w[3]));
        return;
    }

    case 'P':
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (a[0] > a[1] || a[1] >= NUM_SECTORS) { reply(INVALID_SECTOR); return; }
        for (uint32_t s = a[0]; s <= a[1]; ++s) g_prepared |= static_cast<uint16_t>(1u << s);
        reply(CMD_SUCCESS); return;

    case 'W':
        if (!g_unlocked) { reply(CMD_LOCKED); return; }
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (a[0] & 3u) { reply(ADDR_ERROR); return; }
        if (a[1] & 3u) { reply(COUNT_ERROR); return; }
        if (!in_ram(a[0], a[1])) { reply(ADDR_NOT_MAPPED); return; }
        if (crp && a[0] < RAM_BASE + 0x200) { reply(CODE_READ_PROTECTION); return; }
        reply(CMD_SUCCESS);
        if (a[1] == 0) return;
        g_w_off = g_w_blk_off = a[0] - RAM_BASE;
        g_w_left = g_w_blk_left = a[1];
        g_w_lines = g_w_sum = 0;
        g_state = State::WriteData;
        return;

    case 'R':
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (crp) { reply(CODE_READ_PROTECTION); return; }
        if (a[0] & 3u) { reply(ADDR_ERROR); return; }
        if (a[1] & 3u) { reply(COUNT_ERROR); return; }
        if (!in_flash(a[0], a[1]) && !in_ram(a[0], a[1])) { reply(ADDR_NOT_MAPPED); return; }
        reply(CMD_SUCCESS);
        if (a[1] == 0) return;
        g_r_addr = a[0]; g_r_left = a[1];
        r_send_block();
        return;

    case 'C': {
        if (!g_unlocked) { reply(CMD_LOCKED); return; }
        if (n != 3) { reply(PARAM_ERROR); return; }
        const uint32_t dst = a[0], src = a[1], cnt = a[2];
        if (crp >= 2 || (crp == 1 && dst < SECTOR_BYTES)) { reply(CODE_READ_PROTECTION); return; }
        if (dst & 0xFFu) { reply(DST_ADDR_ERROR); return; }
        if (src & 3u)    { reply(SRC_ADDR_ERROR); return; }
        if (cnt != 256 && cnt != 512 && cnt != 1024 && cnt != 4096) { reply(COUNT_ERROR); return; }
        if (!in_flash(dst, cnt)) { reply(DST_ADDR_NOT_MAPPED); return; }
        if (!in_ram(src, cnt))   { reply(SRC_ADDR_NOT_MAPPED); return; }
        const uint32_t s0 = dst / SECTOR_BYTES, s1 = (dst + cnt - 1) / SECTOR_BYTES;
        if (!sectors_prepared(s0, s1)) { reply(SECTOR_NOT_PREPARED); return; }
        // Flash-Semantik: Programmieren kann nur 1->0 (UND mit dem Altinhalt).
        const uint8_t* r = ram() + (src - RAM_BASE);
        for (uint32_t off = 0; off < cnt; off += 256) {
            uint8_t buf[256];
            storage::firmware_read(dst + off, buf, sizeof buf);
            for (uint32_t i = 0; i < sizeof buf; ++i) buf[i] &= r[off + i];
            storage::firmware_write(dst + off, buf, sizeof buf);
        }
        unprepare(s0, s1);
        mark_flash_dirty();
        reply(CMD_SUCCESS);
        return;
    }

    case 'E': {
        if (!g_unlocked) { reply(CMD_LOCKED); return; }
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (a[0] > a[1] || a[1] >= NUM_SECTORS) { reply(INVALID_SECTOR); return; }
        if (crp >= 2 && !(a[0] == 0 && a[1] == NUM_SECTORS - 1)) { reply(CODE_READ_PROTECTION); return; }
        if (!sectors_prepared(a[0], a[1])) { reply(SECTOR_NOT_PREPARED); return; }
        uint8_t ff[256];
        std::memset(ff, 0xFF, sizeof ff);
        for (uint32_t off = a[0] * SECTOR_BYTES; off < (a[1] + 1) * SECTOR_BYTES; off += sizeof ff)
            storage::firmware_write(off, ff, sizeof ff);
        unprepare(a[0], a[1]);
        mark_flash_dirty();
        reply(CMD_SUCCESS);
        return;
    }

    case 'I': {
        if (n != 2) { reply(PARAM_ERROR); return; }
        if (a[0] > a[1] || a[1] >= NUM_SECTORS) { reply(INVALID_SECTOR); return; }
        for (uint32_t off = a[0] * SECTOR_BYTES; off < (a[1] + 1) * SECTOR_BYTES; off += 4) {
            uint32_t w;
            storage::firmware_read(off, &w, 4);
            if (w != 0xFFFF'FFFFu) {
                if (crp) { off = 0; w = 0; }
                txf("%u\r\n%lu\r\n%lu\r\n", SECTOR_NOT_BLANK,
                    static_cast<unsigned long>(off), static_cast<unsigned long>(w));
                return;
            }
        }
        reply(CMD_SUCCESS);
        return;
    }

    case 'M': {
        if (n != 3) { reply(PARAM_ERROR); return; }
        if (crp) { reply(CODE_READ_PROTECTION); return; }
        if (a[0] & 3u) { reply(SRC_ADDR_ERROR); return; }
        if (a[1] & 3u) { reply(DST_ADDR_ERROR); return; }
        if (a[2] & 3u) { reply(COUNT_ERROR); return; }
        if (!in_flash(a[0], a[2]) && !in_ram(a[0], a[2])) { reply(SRC_ADDR_NOT_MAPPED); return; }
        if (!in_flash(a[1], a[2]) && !in_ram(a[1], a[2])) { reply(DST_ADDR_NOT_MAPPED); return; }
        uint8_t x[64], y[64];
        for (uint32_t off = 0; off < a[2]; off += sizeof x) {
            uint32_t c = a[2] - off < sizeof x ? a[2] - off : static_cast<uint32_t>(sizeof x);
            read_mem(a[0] + off, x, c);
            read_mem(a[1] + off, y, c);
            for (uint32_t i = 0; i < c; ++i) {
                if (x[i] != y[i]) {
                    txf("%u\r\n%lu\r\n", COMPARE_ERROR, static_cast<unsigned long>((off + i) & ~3u));
                    return;
                }
            }
        }
        reply(CMD_SUCCESS);
        return;
    }

    case 'G':
        if (!g_unlocked) { reply(CMD_LOCKED); return; }
        if (n != 2 || (argv[2][0] != 'T' && argv[2][0] != 'A')) { reply(PARAM_ERROR); return; }
        reply(CMD_SUCCESS);
        // Der Emulator startet den Gast regulaer ueber den Reset-Vektor; ein
        // Sprung an eine beliebige Adresse ist nicht vorgesehen.
        std::printf("[ISP] Go 0x%lx -> Gast wird gestartet\n", static_cast<unsigned long>(a[0]));
        leave(true);
        return;

    default:
        reply(INVALID_COMMAND);
        return;
    }
}

void write_data_line(const char* line) {
    const bool expect_sum = (g_w_lines == 20) || (g_w_left == 0);
    if (expect_sum) {
        const uint32_t sum = static_cast<uint32_t>(std::strtoul(line, nullptr, 10));
        if (sum == g_w_sum) {
            txs("OK\r\n");
            g_w_blk_off = g_w_off; g_w_blk_left = g_w_left;
            if (g_w_left == 0) g_state = State::Cmd;
        } else {
            txs("RESEND\r\n");
            g_w_off = g_w_blk_off; g_w_left = g_w_blk_left;
        }
        g_w_lines = 0; g_w_sum = 0;
        return;
    }
    uint8_t buf[64];
    uint32_t n = uu_decode(line, buf, 45);
    if (n > g_w_left) n = g_w_left;
    std::memcpy(ram() + g_w_off, buf, n);
    for (uint32_t i = 0; i < n; ++i) g_w_sum += buf[i];
    g_w_off += n; g_w_left -= n;
    ++g_w_lines;
}

void read_ack_line(const char* line) {
    if (std::strcmp(line, "RESEND") == 0) { r_send_block(); return; }
    // "OK" (oder Unbekanntes): naechster Block.
    g_r_addr += g_r_blk_len;
    g_r_left -= g_r_blk_len;
    if (g_r_left == 0) g_state = State::Cmd;
    else               r_send_block();
}

void line_done() {
    g_line[g_len] = '\0';
    g_len = 0;
    switch (g_state) {
    case State::SyncReply:
        if (std::strcmp(g_line, "Synchronized") == 0) { txs("OK\r\n"); g_state = State::Freq; }
        else g_state = State::Sync;
        break;
    case State::Freq:      txs("OK\r\n"); g_state = State::Cmd; break;   // Quarzfrequenz (kHz)
    case State::Cmd:       exec(g_line); break;
    case State::WriteData: write_data_line(g_line); break;
    case State::ReadWait:  read_ack_line(g_line); break;
    case State::Sync:      break;
    }
}

void feed(int c) {
    if (g_state == State::Sync) {
        if (c == '?') {
            txs("Synchronized\r\n");
            g_state = State::SyncReply;
            g_len = 0; g_cr_pending = false;
        }
        return;
    }
    // Zeilenende: CR und/oder LF (UM10398). Ausgefuehrt wird erst nach dem LF,
    // damit das Echo von "\r\n" VOR der Antwort steht (lpc21isp/FlashMagic
    // erwarten z. B. "Synchronized\r\nOK\r\n"). Kommt nach einem CR kein LF,
    // fuehrt poll() die Zeile nach kurzer Wartezeit aus.
    if (g_cr_pending && c != '\n') { g_cr_pending = false; line_done(); }
    if (g_echo) { char ch = static_cast<char>(c); tx(&ch, 1); }
    if (c == '\r') { g_cr_pending = true; g_cr_us = time_us_32(); return; }
    if (c == '\n') {
        const bool had_cr = g_cr_pending;
        g_cr_pending = false;
        if (had_cr || g_len > 0) line_done();
        return;
    }
    if (g_len + 1 < sizeof g_line) g_line[g_len++] = static_cast<char>(c);
}

// --- Autobaud (UART) --------------------------------------------------------
// '?' = 0x3F, LSB zuerst: Start(0) 1 1 1 1 1 1 0 0 Stop(1). Gemessen wird von der
// fallenden Flanke des Startbits bis zur steigenden Flanke des Stopbits (9 Bit).
// Das Warten auf die Flanke laeuft in 1-ms-Fenstern mit gesperrten Interrupts,
// damit keine USB-IRQ die Zeitmessung verfaelscht.
bool autobaud_try(uint32_t& baud) {
    const uint rx = static_cast<uint>(g_rx_gpio);
    if (!gpio_get(rx)) return false;              // mitten in einem Zeichen
    const uint32_t irq = save_and_disable_interrupts();
    const uint32_t t_start = time_us_32();
    bool edge = false;
    while (time_us_32() - t_start < 1000u) if (!gpio_get(rx)) { edge = true; break; }
    const uint32_t t0 = time_us_32();
    uint32_t t[3] = {};
    bool ok = edge;
    const bool lv[3] = {true, false, true};
    for (int i = 0; ok && i < 3; ++i) {
        while (gpio_get(rx) != lv[i]) if (time_us_32() - t0 > 20'000u) { ok = false; break; }
        t[i] = time_us_32();
    }
    restore_interrupts(irq);
    if (!ok) return false;
    const float total = static_cast<float>(t[2] - t0);
    if (total < 9.0f) return false;
    const float bit = total / 9.0f;
    const float b1 = static_cast<float>(t[0] - t0), b6 = static_cast<float>(t[1] - t[0]),
                b2 = static_cast<float>(t[2] - t[1]);
    if (b1 < 0.4f * bit || b1 > 1.7f * bit || b6 < 4.5f * bit || b6 > 7.5f * bit ||
        b2 < 1.3f * bit || b2 > 2.7f * bit) return false;
    uint32_t measured = static_cast<uint32_t>(1'000'000.0f / bit + 0.5f);
    static const uint32_t STD[] = {1200, 2400, 4800, 9600, 14400, 19200, 38400,
                                   57600, 115200, 230400, 460800};
    baud = measured;
    for (uint32_t s : STD) {
        const float d = static_cast<float>(measured) / static_cast<float>(s);
        if (d > 0.93f && d < 1.07f) { baud = s; break; }
    }
    return true;
}

void uart_drain() {
    if (!g_uart) return;
    while (uart_is_readable(g_uart)) (void) uart_getc(g_uart);
}

// --- RESET-/ISP-Pins ---------------------------------------------------------
int pin_gpio(uint8_t lpc) { return config::pin_map().lpc_to_rp[lpc]; }

void configure_input_pullup(int g) {
    gpio_init(static_cast<uint>(g));
    gpio_set_dir(static_cast<uint>(g), false);
    gpio_pull_up(static_cast<uint>(g));
}

bool reset_function_active() {
    if (!config::reset_in() || pin_gpio(LPC_PIN_RESET) < 0) return false;
    // Nutzt der laufende Gast P0_0 als GPIO (IOCON.FUNC != 0), ist RESET aus.
    if (emulator::state() == emulator::State::Running &&
        !peripherals::reset_pin_is_reset_function()) return false;
    return true;
}

void hold_guest_in_reset(const char* why) {
    if (g_active) leave(false);
    else if (emulator::state() == emulator::State::Running ||
             emulator::state() == emulator::State::Faulted) emulator::stop();
    std::printf("[RESET] %s -> Gast im Reset\n", why);
}

void poll_reset_pin() {
    const int g = pin_gpio(LPC_PIN_RESET);
    if (!config::reset_in() || g < 0) {
        if (g_pin_reset_held) { g_pin_reset_held = false; }
        g_reset_gpio_cfg = -1;
        return;
    }
    if (g != g_reset_gpio_cfg) {                 // (Neu)konfiguration
        if (emulator::state() != emulator::State::Running) configure_input_pullup(g);
        else gpio_pull_up(static_cast<uint>(g));
        g_reset_gpio_cfg = g;
        g_reset_level = true;
        g_reset_change_ms = now_ms();
    }
    // Entprellen: neuer Pegel muss 5 ms stabil anliegen.
    const bool lvl = gpio_get(static_cast<uint>(g));
    static bool raw_last = true;
    if (lvl != raw_last) { raw_last = lvl; g_reset_change_ms = now_ms(); }
    if (lvl != g_reset_level && now_ms() - g_reset_change_ms >= 5u) g_reset_level = lvl;

    if (!g_pin_reset_held && !g_reset_level && reset_function_active()) {
        g_pin_reset_held = true;
        hold_guest_in_reset("RESET-Pin (P0_0) low");
    } else if (g_pin_reset_held && g_reset_level) {
        g_pin_reset_held = false;
        std::printf("[RESET] RESET-Pin freigegeben\n");
        start_guest_if_possible();
    }
}

void poll_line_state() {
    if (!g_ls_pending) return;
    g_ls_pending = false;
    if (!config::isp_dtr_rts()) return;
    const bool dtr = g_ls_dtr, rts = g_ls_rts;
    if (dtr && !g_cdc_reset_held) {
        g_cdc_reset_held = true;
        hold_guest_in_reset("ISP-CDC DTR aktiv");
    } else if (!dtr && g_cdc_reset_held) {
        g_cdc_reset_held = false;
        if (rts) enter(Transport::Cdc, "DTR-Reset mit RTS (ISP) aktiv");
        else     start_guest_if_possible();
    }
}

// Ohne aktiven ISP: '?' auf der ISP-CDC startet ihn (isp_autosync), sonst werden
// eingehende Bytes verworfen, damit die FIFO nicht voll laeuft.
void poll_idle_cdc() {
    const int itf = usb_desc_cdc_isp();
    if (itf < 0) return;
    while (tud_cdc_n_available(static_cast<uint8_t>(itf))) {
        uint8_t c;
        if (!tud_cdc_n_read(static_cast<uint8_t>(itf), &c, 1)) break;
        if (c == '?' && config::isp_autosync() && !g_pin_reset_held) {
            if (enter(Transport::Cdc, "'?' auf der ISP-CDC")) feed('?');
            return;
        }
    }
}

} // namespace

// TinyUSB-Callback (Core0, aus tud_task): nur vormerken, Auswertung in poll().
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    if (static_cast<int>(itf) != usb_desc_cdc_isp()) return;
    g_ls_dtr = dtr;
    g_ls_rts = rts;
    g_ls_pending = true;
}

void init() {
    g_active = false;
    g_tp = Transport::None;
    // RESET-Pin schon beim Booten auswerten, damit ein Autostart bei gehaltenem
    // RESET unterbleibt (Start erfolgt dann mit der Freigabe).
    const int g = pin_gpio(LPC_PIN_RESET);
    if (config::reset_in() && g >= 0) {
        configure_input_pullup(g);
        busy_wait_us(50);
        g_reset_gpio_cfg = g;
        g_reset_level    = gpio_get(static_cast<uint>(g));
        g_pin_reset_held = !g_reset_level;
        g_reset_change_ms = now_ms();
        if (g_pin_reset_held) std::puts("[RESET] RESET-Pin (P0_0) beim Start low -> Gast im Reset");
    }
}

bool      active()    { return g_active; }
Transport transport() { return g_tp; }

bool available() {
    if (usb_desc_cdc_isp() >= 0) return true;
    return config::isp_pins() && config::uart0_tx_gpio() >= 0 && config::uart0_rx_gpio() >= 0;
}

bool enter(Transport t, const char* reason) {
    if (g_active) return g_tp == t;
    if (t == Transport::Cdc && usb_desc_cdc_isp() < 0) {
        std::puts("[ISP] ISP-CDC nicht vorhanden (isp_enable=off)");
        return false;
    }
    // Gast stoppen (Core1 in die sichere Warteschleife), bevor die UART0-Pads
    // oder der Flash-Slot angefasst werden.
    if (emulator::state() == emulator::State::Running ||
        emulator::state() == emulator::State::Faulted) emulator::stop();
    if (t == Transport::Uart) {
        if (!peripherals::uart0_isp_pads(g_uart, g_rx_gpio)) {
            std::puts("[ISP] keine gueltigen UART0-Pads (uart pins <tx> <rx>) -> kein ISP ueber Pins");
            return false;
        }
        g_autobaud = (config::isp_baud() == 0);
        g_baud = uart_set_baudrate(g_uart, g_autobaud ? 115200u : config::isp_baud());
        uart_set_format(g_uart, 8, 1, UART_PARITY_NONE);
        uart_drain();
    }
    storage::firmware_flush();
    g_tp = t;
    g_active = true;
    g_state = State::Sync;
    g_echo = true;
    g_unlocked = false;
    g_prepared = 0;
    g_len = 0;
    g_cr_pending = false;
    g_flash_dirty = false;
    std::printf("[ISP] aktiv ueber %s (%s)%s\n",
                t == Transport::Cdc ? "ISP-CDC" : "UART0-Pads", reason,
                (t == Transport::Uart && g_autobaud) ? " - Autobaud" : "");
    return true;
}

void leave(bool start_guest) {
    if (!g_active) return;
    commit_flash();
    g_active = false;
    g_tp = Transport::None;
    g_uart = nullptr;
    std::puts("[ISP] beendet");
    if (start_guest) start_guest_if_possible();
}

void start_guest_if_possible() {
    if (g_pin_reset_held || g_cdc_reset_held || g_active) return;
    if (intercept_boot()) return;
    if (storage::firmware_size() == 0) { std::puts("[RESET] keine Firmware -> Gast bleibt aus"); return; }
    emulator::load_and_start();
}

bool intercept_boot() {
    if (g_pin_reset_held || g_cdc_reset_held) return true;
    if (g_active) return true;
    if (!config::isp_pins()) return false;
    const int g = pin_gpio(LPC_PIN_ISP);
    if (g < 0) return false;
    configure_input_pullup(g);
    busy_wait_us(50);
    if (gpio_get(static_cast<uint>(g))) return false;
    if (crp_no_isp_entry()) {
        std::puts("[ISP] P0_1 low, aber CRP3/NO_ISP gesetzt -> ISP-Eintritt gesperrt");
        return false;
    }
    return enter(Transport::Uart, "P0_1 low beim Reset");
}

void on_guest_reinvoke() {
    Transport t = (config::isp_pins() && config::uart0_tx_gpio() >= 0 &&
                   config::uart0_rx_gpio() >= 0) ? Transport::Uart : Transport::Cdc;
    if (!enter(t, "IAP Reinvoke ISP") &&
        !(t == Transport::Uart && enter(Transport::Cdc, "IAP Reinvoke ISP"))) {
        // Kein ISP moeglich: Gast (parkt auf Core1) neu starten.
        emulator::stop();
        start_guest_if_possible();
    }
}

void poll() {
    poll_line_state();
    poll_reset_pin();
    if (!g_active) { poll_idle_cdc(); return; }

    if (g_tp == Transport::Uart && g_state == State::Sync && g_autobaud) {
        uint32_t baud;
        if (autobaud_try(baud)) {
            busy_wait_us(20'000'000u / baud + 100u);   // Rest des Zeichens abwarten
            g_baud = uart_set_baudrate(g_uart, baud);
            uart_drain();
            std::printf("[ISP] Autobaud: %lu Baud\n", static_cast<unsigned long>(baud));
            feed('?');
        } else {
            uart_drain();
        }
    } else {
        for (int i = 0; i < 512 && g_active; ++i) {
            int c = rx_byte();
            if (c < 0) break;
            feed(c);
        }
        if (g_active && g_cr_pending && time_us_32() - g_cr_us > 3000u) {
            g_cr_pending = false;   // nur CR als Zeilenende
            line_done();
        }
    }
    if (g_flash_dirty && now_ms() - g_last_flash_ms > 1000u) commit_flash();
}

void print_status() {
    static const char* st[] = {"Sync", "SyncReply", "Freq", "Cmd", "WriteData", "ReadWait"};
    if (g_active)
        std::printf("isp: AKTIV ueber %s, Zustand=%s, %s, echo=%s, Baud=%lu\n",
                    g_tp == Transport::Cdc ? "ISP-CDC" : "UART0-Pads",
                    st[static_cast<int>(g_state)], g_unlocked ? "entsperrt" : "gesperrt",
                    g_echo ? "an" : "aus", static_cast<unsigned long>(g_baud));
    else
        std::puts("isp: inaktiv");
    std::printf("  ISP-CDC: %s (isp_enable)  dtr_rts=%s  autosync=%s\n",
                usb_desc_cdc_isp() >= 0 ? "vorhanden" : "AUS",
                config::isp_dtr_rts() ? "on" : "off", config::isp_autosync() ? "on" : "off");
    const int rg = pin_gpio(LPC_PIN_RESET), ig = pin_gpio(LPC_PIN_ISP);
    std::printf("  RESET-Eingang (reset_in=%s): P0_0 -> %s%d%s\n",
                config::reset_in() ? "on" : "off", rg >= 0 ? "GP" : "", rg,
                g_pin_reset_held ? "  [GEHALTEN]" : "");
    std::printf("  ISP-Pins (isp_pins=%s): P0_1 -> %s%d, UART0 TX=GP%d RX=GP%d, isp_baud=%lu%s\n",
                config::isp_pins() ? "on" : "off", ig >= 0 ? "GP" : "", ig,
                config::uart0_tx_gpio(), config::uart0_rx_gpio(),
                static_cast<unsigned long>(config::isp_baud()),
                config::isp_baud() ? "" : " (Autobaud)");
    if (g_cdc_reset_held) std::puts("  Gast durch DTR der ISP-CDC im Reset gehalten");
}

} // namespace isp
