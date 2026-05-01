#include "iap.h"
#include "emulator.h"
#include "storage.h"

#include <cstdio>
#include <cstring>
#include "pico/unique_id.h"

namespace iap {

namespace {

constexpr uint32_t LPC_FLASH_BYTES   = 64 * 1024;
constexpr uint32_t LPC_SECTOR_BYTES  = 4096;
constexpr uint32_t LPC_NUM_SECTORS   = LPC_FLASH_BYTES / LPC_SECTOR_BYTES;  // 16

Stats g_stats{};
uint16_t g_prepared_mask = 0;       // Bit n => Sektor n vorbereitet (cmd 50)
bool g_dirty = false;

uint8_t* flash_image() {
    return reinterpret_cast<uint8_t*>(emulator::LPC_LOAD_BASE);
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
// damit IAP-induzierte Änderungen einen Power-Cycle überleben.
void persist(uint32_t offset, uint32_t bytes) {
    storage::firmware_write(offset, flash_image() + offset, bytes);
    g_dirty = true;
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
    // Quell-Adresse ist Guest-RAM (0x10000000-0x10001FFF) → bei uns identisch
    // erreichbar (Privileg-Mode), oder auch Code-Image. Wir nutzen den
    // Adressraum direkt.
    const uint8_t* srcp = reinterpret_cast<const uint8_t*>(src);
    std::memcpy(flash_image() + dst, srcp, bytes);
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
    const uint8_t* a = flash_image() + dst;
    const uint8_t* b = reinterpret_cast<const uint8_t*>(src);
    if (!dst_in_flash(dst, bytes)) return CMD_DST_ADDR_NOT_MAPPED;
    for (uint32_t i = 0; i < bytes; ++i) {
        if (a[i] != b[i]) { r[1] = dst + i; return CMD_COMPARE_ERROR; }
    }
    return CMD_SUCCESS;
}

void put_uid(uint32_t* r) {
    pico_unique_board_id_t id{};
    pico_get_unique_board_id(&id);
    // 8 RP2350-Bytes -> 4×32-Bit (LPC liefert 4 Words).
    for (int i = 0; i < 4; ++i) {
        r[1 + i] = static_cast<uint32_t>(id.id[(i * 2) % 8]) |
                   (static_cast<uint32_t>(id.id[(i * 2 + 1) % 8]) << 8) |
                   (static_cast<uint32_t>(id.id[(i + 4) % 8]) << 16) |
                   (static_cast<uint32_t>(id.id[(i + 1) % 8]) << 24);
    }
}

} // namespace

void init() {
    g_stats = {};
    g_prepared_mask = 0;
    g_dirty = false;
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
            // Reinvoke ISP — im Emulator kein UART-Bootloader. Ignorieren.
            result[0] = CMD_SUCCESS;
            break;
        case 58:
            result[0] = CMD_SUCCESS;
            put_uid(result);
            break;
        case 59: {
            // Erase page — wie cmd 52, aber 256 Byte granular. Wir runden
            // auf den umschließenden Sektor.
            uint32_t p_start = param[1], p_end = param[2];
            uint32_t s_start = (p_start * 256u) / LPC_SECTOR_BYTES;
            uint32_t s_end   = (p_end   * 256u) / LPC_SECTOR_BYTES;
            uint32_t fake[5] = { 59, s_start, s_end, 0, 0 };
            result[0] = cmd_erase(fake);
            break;
        }
        default:
            result[0] = CMD_INVALID_COMMAND;
            ++g_stats.errors;
            break;
    }
}

Stats stats() { return g_stats; }

} // namespace iap
