#include "iap.h"
#include "emulator.h"
#include "storage.h"
#include "isp.h"

#include <cstdio>
#include <cstring>
#include "pico/unique_id.h"
#include "pico/critical_section.h"
#include "fast_time.h"

namespace iap {

void read_uid(uint32_t w[4]);

namespace {

constexpr uint32_t LPC_FLASH_BYTES   = 64 * 1024;
constexpr uint32_t LPC_SECTOR_BYTES  = 4096;
constexpr uint32_t LPC_NUM_SECTORS   = LPC_FLASH_BYTES / LPC_SECTOR_BYTES;  // 16

Stats g_stats{};
uint16_t g_prepared_mask = 0;       // Bit n => Sektor n vorbereitet (cmd 50)
bool g_dirty = false;

uint8_t* flash_image() {
    return reinterpret_cast<uint8_t*>(emulator::load_base());
}

bool sector_valid(uint32_t s) { return s < LPC_NUM_SECTORS; }

bool sectors_prepared(uint32_t start, uint32_t end) {
    for (uint32_t s = start; s <= end; ++s)
        if ((g_prepared_mask & (1u << s)) == 0) return false;
    return true;
}

bool dst_in_flash(uint32_t addr, uint32_t bytes) {
    return addr + bytes <= LPC_FLASH_BYTES;
}

// Schreibt einen Teilbereich des RAM-Flash-Image zurück in den Storage-Slot,
// damit IAP-induzierte Änderungen einen Power-Cycle überleben. firmware_write()
// puffert nur einen Sektor im RAM; erst finalize() flusht ihn und erneuert den
// Laengen-/CRC-Marker. Ohne finalize ging der zuletzt beschriebene Sektor (z. B.
// sblib-EEPROM) beim Power-Cycle verloren, und ein geflushter Sektor passte
// nicht mehr zur Marker-CRC -> "keine Firmware" nach dem naechsten Boot.
// Gast-Schreibzugriffe aendern sofort das RAM-Abbild; ins RP2350-Flash
// geschrieben wird verzoegert (flush_pending). Frueher lief jedes IAP-Kommando
// synchron: Sektor loeschen + programmieren, CRC ueber 64 KiB, Header neu -
// beim Speichern des User-EEPROMs (sblib: Erase + 3x Copy) ~8 Loeschvorgaenge,
// der Gast stand Hunderte ms. Ein T_Connect direkt nach IndividualAddressWrite
// ging dabei verloren (ETS/SB-Project: "send data connected failed").
// Geschrieben wird nur auf Core1 (SysTick-Shim bzw. vor dem Gast-Reload):
// Core0 kann Core1 bei laufendem Gast nicht per Lockout anhalten.
critical_section_t g_pend_cs;
uint32_t g_pend_lo = 0, g_pend_hi = 0;     // [lo, hi), hi == 0: nichts offen
uint64_t g_pend_t  = 0;                    // Zeit des letzten Gast-Schreibzugriffs
uint32_t g_pend_seq = 0;                   // storage::firmware_seq() beim Eintrag

void persist(uint32_t offset, uint32_t bytes) {
    critical_section_enter_blocking(&g_pend_cs);
    if (g_pend_hi == 0) { g_pend_lo = offset; g_pend_hi = offset + bytes; g_pend_seq = storage::firmware_seq(); }
    else {
        if (offset < g_pend_lo) g_pend_lo = offset;
        if (offset + bytes > g_pend_hi) g_pend_hi = offset + bytes;
    }
    g_pend_t = fast_time_us();
    critical_section_exit(&g_pend_cs);
    g_dirty = true;
}

const uint8_t* guest_ptr(uint32_t addr, uint32_t bytes, bool allow_flash = true) {
    const uint32_t ram  = emulator::guest_ram_base();
    const uint32_t img  = emulator::load_base();
    constexpr uint32_t LPC_RAM = 0x1000'0000u;
    constexpr uint32_t RAM_SZ  = emulator::LPC_GUEST_RAM_SIZE;
    auto inside = [&](uint32_t base, uint32_t size) {
        return addr >= base && bytes <= size && addr - base <= size - bytes;
    };
    if (inside(LPC_RAM, RAM_SZ))       return reinterpret_cast<const uint8_t*>(ram + (addr - LPC_RAM));
    if (inside(ram, RAM_SZ))           return reinterpret_cast<const uint8_t*>(addr);
    if (!allow_flash)                  return nullptr;
    if (inside(0, LPC_FLASH_BYTES))    return flash_image() + addr;
    if (inside(img, LPC_FLASH_BYTES))  return reinterpret_cast<const uint8_t*>(addr);
    return nullptr;
}

uint32_t cmd_prepare(uint32_t* p) {
    uint32_t s_start = p[1], s_end = p[2];
    if (!sector_valid(s_start) || !sector_valid(s_end) || s_start > s_end)
        return CMD_INVALID_SECTOR;
    for (uint32_t s = s_start; s <= s_end; ++s)
        g_prepared_mask |= (1u << s);
    return CMD_SUCCESS;
}

uint32_t cmd_copy_ram_to_flash(uint32_t* p) {
    uint32_t dst   = p[1];
    uint32_t src   = p[2];
    uint32_t bytes = p[3];
    // p[4] = CCLK in kHz — ignoriert
    if (bytes != 256 && bytes != 512 && bytes != 1024 && bytes != 4096)
        return CMD_COUNT_ERROR;
    if ((dst & 0xFFu) != 0)
        return CMD_DST_ADDR_ERROR;
    if (!dst_in_flash(dst, bytes))
        return CMD_DST_ADDR_NOT_MAPPED;
    uint32_t s_start = dst / LPC_SECTOR_BYTES;
    uint32_t s_end   = (dst + bytes - 1) / LPC_SECTOR_BYTES;
    if (!sectors_prepared(s_start, s_end))
        return CMD_SECTOR_NOT_PREPARED;
    // Quelle muss im Gast-RAM liegen (UM10398: RAM-Adresse, word-aligned).
    const uint8_t* srcp = guest_ptr(src, bytes, /*allow_flash=*/false);
    if ((src & 3u) != 0 || !srcp) return CMD_SRC_ADDR_ERROR;
    // Echtes Flash kann beim Programmieren nur Bits 1->0 setzen (Loeschen
    // setzt auf 0xFF). Programmieren eines nicht geloeschten Bereichs ergibt
    // daher das UND aus altem und neuem Inhalt — wie auf dem LPC1115.
    uint8_t* d = flash_image() + dst;
    for (uint32_t i = 0; i < bytes; ++i) d[i] &= srcp[i];
    persist(dst, bytes);
    g_prepared_mask &= ~static_cast<uint16_t>(((1u << (s_end + 1)) - 1u) &
                                              ~((1u << s_start) - 1u));
    ++g_stats.writes;
    return CMD_SUCCESS;
}

uint32_t cmd_erase(uint32_t* p) {
    uint32_t s_start = p[1], s_end = p[2];
    if (!sector_valid(s_start) || !sector_valid(s_end) || s_start > s_end)
        return CMD_INVALID_SECTOR;
    if (!sectors_prepared(s_start, s_end))
        return CMD_SECTOR_NOT_PREPARED;
    uint32_t off = s_start * LPC_SECTOR_BYTES;
    uint32_t len = (s_end - s_start + 1) * LPC_SECTOR_BYTES;
    std::memset(flash_image() + off, 0xFF, len);
    persist(off, len);
    g_prepared_mask &= ~static_cast<uint16_t>(((1u << (s_end + 1)) - 1u) &
                                              ~((1u << s_start) - 1u));
    ++g_stats.erases;
    return CMD_SUCCESS;
}

// Erase page (cmd 59): 256-Byte-granular. Frueher auf den ganzen 4-KiB-Sektor
// aufgerundet -> Nachbar-Pages (andere Daten) wurden mitgeloescht.
uint32_t cmd_erase_page(uint32_t* p) {
    constexpr uint32_t PAGE = 256;
    constexpr uint32_t NUM_PAGES = LPC_FLASH_BYTES / PAGE;
    uint32_t p_start = p[1], p_end = p[2];
    if (p_start >= NUM_PAGES || p_end >= NUM_PAGES || p_start > p_end)
        return CMD_INVALID_SECTOR;
    uint32_t s_start = (p_start * PAGE) / LPC_SECTOR_BYTES;
    uint32_t s_end   = (p_end   * PAGE) / LPC_SECTOR_BYTES;
    if (!sectors_prepared(s_start, s_end))
        return CMD_SECTOR_NOT_PREPARED;
    uint32_t off = p_start * PAGE;
    uint32_t len = (p_end - p_start + 1) * PAGE;
    std::memset(flash_image() + off, 0xFF, len);
    persist(off, len);
    g_prepared_mask &= ~static_cast<uint16_t>(((1u << (s_end + 1)) - 1u) &
                                              ~((1u << s_start) - 1u));
    ++g_stats.erases;
    return CMD_SUCCESS;
}

uint32_t cmd_blank_check(uint32_t* p, uint32_t* r) {
    uint32_t s_start = p[1], s_end = p[2];
    if (!sector_valid(s_start) || !sector_valid(s_end) || s_start > s_end)
        return CMD_INVALID_SECTOR;
    for (uint32_t s = s_start; s <= s_end; ++s) {
        const uint8_t* p0 = flash_image() + s * LPC_SECTOR_BYTES;
        for (uint32_t i = 0; i < LPC_SECTOR_BYTES; ++i) {
            if (p0[i] != 0xFF) {
                r[1] = s * LPC_SECTOR_BYTES + i;
                r[2] = p0[i];
                return CMD_SECTOR_NOT_BLANK;
            }
        }
    }
    return CMD_SUCCESS;
}

uint32_t cmd_compare(uint32_t* p, uint32_t* r) {
    uint32_t dst = p[1], src = p[2], bytes = p[3];
    if ((dst & 3u) || (src & 3u)) return (dst & 3u) ? CMD_DST_ADDR_ERROR : CMD_SRC_ADDR_ERROR;
    if (bytes & 3u) return CMD_COUNT_ERROR;
    const uint8_t* a = guest_ptr(dst, bytes);
    const uint8_t* b = guest_ptr(src, bytes);
    if (!a) return CMD_DST_ADDR_NOT_MAPPED;
    if (!b) return CMD_SRC_ADDR_NOT_MAPPED;
    for (uint32_t i = 0; i < bytes; ++i) {
        if (a[i] != b[i]) { r[1] = dst + i; return CMD_COMPARE_ERROR; }
    }
    return CMD_SUCCESS;
}

void put_uid(uint32_t* r) { read_uid(r + 1); }

} // namespace

void read_uid(uint32_t w[4]) {
    pico_unique_board_id_t id{};
    pico_get_unique_board_id(&id);
    // 8 RP2350-Bytes -> 4x32-Bit (LPC liefert 4 Words).
    for (int i = 0; i < 4; ++i) {
        w[i] = static_cast<uint32_t>(id.id[(i * 2) % 8]) |
               (static_cast<uint32_t>(id.id[(i * 2 + 1) % 8]) << 8) |
               (static_cast<uint32_t>(id.id[(i + 4) % 8]) << 16) |
               (static_cast<uint32_t>(id.id[(i + 1) % 8]) << 24);
    }
}

void init() {
    if (!critical_section_is_initialized(&g_pend_cs)) critical_section_init(&g_pend_cs);
    g_stats = {};
    g_prepared_mask = 0;
    g_dirty = false;
}

void dispatch_guest(uint32_t param_addr, uint32_t result_addr) {
    // Parameter-/Ergebnistabelle muessen im Gast-RAM liegen (5 Worte). Ein
    // roher LPC-RAM-Zeiger (0x10000000+) wird uebersetzt; alles andere (z. B.
    // ein Zeiger in Host-Speicher) wird abgewiesen.
    auto* param  = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(
        guest_ptr(param_addr, 5 * 4, /*allow_flash=*/false)));
    auto* result = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(
        guest_ptr(result_addr, 5 * 4, /*allow_flash=*/false)));
    if ((param_addr | result_addr) & 3u) { param = nullptr; }
    dispatch(param, result);
}

void dispatch(uint32_t* param, uint32_t* result) {
    if (!param || !result) {
        if (result) result[0] = CMD_SRC_ADDR_NOT_MAPPED;
        ++g_stats.errors;
        return;
    }
    ++g_stats.calls;
    uint32_t cmd = param[0];
    // Default-Result clearen
    for (int i = 0; i < 5; ++i) result[i] = 0;

    switch (cmd) {
        case 50: result[0] = cmd_prepare(param);                   break;
        case 51: result[0] = cmd_copy_ram_to_flash(param);         break;
        case 52: result[0] = cmd_erase(param);                     break;
        case 53: result[0] = cmd_blank_check(param, result);       break;
        case 54:
            // Part-ID LPC1115FBD48/303
            result[0] = CMD_SUCCESS;
            result[1] = 0x00050080;
            break;
        case 55:
            result[0] = CMD_SUCCESS;
            result[1] = 0x00010000;     // Boot-Code-Version
            break;
        case 56: result[0] = cmd_compare(param, result);           break;
        case 57:
            // Reinvoke ISP: wie auf dem LPC kehrt der Aufruf nicht zurueck, wenn
            // ein ISP-Transport verfuegbar ist (Core1 parkt, Core0 startet den
            // ISP). Sonst: Erfolg melden und weiterlaufen.
            if (isp::available()) emulator::request_isp_from_guest();
            result[0] = CMD_SUCCESS;
            break;
        case 58:
            result[0] = CMD_SUCCESS;
            put_uid(result);
            break;
        case 59: result[0] = cmd_erase_page(param);                break;
        default:
            result[0] = CMD_INVALID_COMMAND;
            ++g_stats.errors;
            break;
    }
}

Stats stats() { return g_stats; }

void __not_in_flash_func(core1_poll)() {
    if (g_pend_hi != 0 && fast_time_us() - g_pend_t >= 1'000'000u) flush_pending(1000);
}

bool flush_pending(uint32_t min_age_ms) {
    critical_section_enter_blocking(&g_pend_cs);
    const uint32_t lo = g_pend_lo, hi = g_pend_hi, seq = g_pend_seq;
    const bool due = hi != 0 && fast_time_us() - g_pend_t >= static_cast<uint64_t>(min_age_ms) * 1000u;
    if (due) g_pend_hi = 0;
    critical_section_exit(&g_pend_cs);
    if (!due) return false;
    // Inzwischen per Upload/ISP/Erase neu geschrieben: altes Abbild verwerfen.
    if (storage::firmware_seq() != seq) return false;
    storage::firmware_write(lo, flash_image() + lo, hi - lo);
    storage::firmware_finalize(hi);
    return true;
}

} // namespace iap
