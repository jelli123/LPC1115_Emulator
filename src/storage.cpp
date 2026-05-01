#include "storage.h"

#include <cstring>
#include <cstdio>
#include <algorithm>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// Adressrechnung: PICO_FLASH_SIZE_BYTES wird vom SDK / Board-Header geliefert.
// XIP_BASE ist der memory-mapped Lesebereich.
extern "C" {
#include "hardware/regs/addressmap.h"
}

namespace storage {

namespace {

constexpr uint32_t MAGIC_CONFIG   = 0xC0FE'1115u;
constexpr uint32_t MAGIC_FIRMWARE = 0xF00D'1115u;

struct ConfigHeader {
    uint32_t magic;
    uint32_t sequence;     // monoton steigend, höchster wert = aktuell
    uint32_t payload_len;  // Länge der Key/Value-Daten dahinter
    uint32_t crc32;        // CRC der payload
};

struct FirmwareHeader {
    uint32_t magic;
    uint32_t sequence;
    uint32_t length;       // tatsächliche Firmware-Länge in Bytes
    uint32_t crc32;        // CRC über die Firmware
};

// Offsets relativ zum Flash-Start. Ende des Flash = (PICO_FLASH_SIZE_BYTES).
// Wir reservieren am Ende:
//   [...firmware...][...config...]
constexpr std::size_t flash_size_bytes = PICO_FLASH_SIZE_BYTES;
constexpr std::size_t config_region_size   = CONFIG_SLOT_SECTORS * SECTOR_SIZE;
constexpr std::size_t firmware_region_size =
    ((FIRMWARE_SLOT_BYTES + SECTOR_SIZE - 1) / SECTOR_SIZE) * SECTOR_SIZE;

static_assert(config_region_size + firmware_region_size <= flash_size_bytes,
              "Storage-Reservierung passt nicht in den verfügbaren Flash");
static_assert(config_region_size + firmware_region_size < flash_size_bytes / 4,
              "Storage darf maximal das letzte Viertel des Flash belegen");

constexpr std::size_t config_region_offset =
    flash_size_bytes - config_region_size;
constexpr std::size_t firmware_region_offset =
    config_region_offset - firmware_region_size;

// In-RAM Konfig-Snapshot. Sehr klein gehalten.
struct KV {
    char key[32];
    char value[96];
};
constexpr std::size_t MAX_KV = 32;

KV       kv_table[MAX_KV];
uint32_t kv_count       = 0;
uint32_t config_sequence = 0;
uint32_t config_active_idx = 0; // welcher Sektor zuletzt benutzt wurde

uint32_t firmware_sequence = 0;
uint32_t firmware_length   = 0;

// CRC32 (poly 0xEDB88320). Tabelle on-the-fly berechnet wäre kürzer, aber
// die Inline-Variante ist einfacher zu auditieren.
uint32_t crc32(const uint8_t* data, std::size_t len, uint32_t seed = 0xFFFF'FFFFu) {
    uint32_t c = seed;
    for (std::size_t i = 0; i < len; ++i) {
        c ^= data[i];
        for (int k = 0; k < 8; ++k) {
            uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(c & 1u));
            c = (c >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~c;
}

// Lock-safe Flash-Schreiben: Core1 muss vorher pausiert werden, sonst
// wird XIP gestört, während Code von dort läuft.
struct FlashGuard {
    uint32_t ints;
    bool     core1_was_running;
    FlashGuard() {
        // Multicore-Lockout: blockiert Core1 nur, wenn er aktiv ist.
        core1_was_running = multicore_lockout_victim_is_initialized(1);
        if (core1_was_running) multicore_lockout_start_blocking();
        ints = save_and_disable_interrupts();
    }
    ~FlashGuard() {
        restore_interrupts(ints);
        if (core1_was_running) multicore_lockout_end_blocking();
    }
    FlashGuard(const FlashGuard&)            = delete;
    FlashGuard& operator=(const FlashGuard&) = delete;
};

const uint8_t* xip_ptr(std::size_t flash_offset) {
    return reinterpret_cast<const uint8_t*>(XIP_BASE + flash_offset);
}

bool sector_erase_and_write(std::size_t flash_offset,
                            const void* data, std::size_t len) {
    if (flash_offset % SECTOR_SIZE != 0) return false;
    if (len > SECTOR_SIZE) return false;

    // Aufgepuffertes 256-Byte-Page-aligned Schreiben.
    alignas(4) static uint8_t page[SECTOR_SIZE];
    std::memset(page, 0xFF, sizeof page);
    std::memcpy(page, data, len);

    FlashGuard guard;
    flash_range_erase(flash_offset, SECTOR_SIZE);
    flash_range_program(flash_offset, page, SECTOR_SIZE);
    return true;
}

const ConfigHeader* read_config_header(uint32_t idx) {
    return reinterpret_cast<const ConfigHeader*>(
        xip_ptr(config_region_offset + idx * SECTOR_SIZE));
}

bool parse_config_payload(const uint8_t* payload, uint32_t len) {
    kv_count = 0;
    uint32_t i = 0;
    while (i + sizeof(KV) <= len && kv_count < MAX_KV) {
        std::memcpy(&kv_table[kv_count], payload + i, sizeof(KV));
        // Sicherheits-Terminator
        kv_table[kv_count].key  [sizeof(kv_table[0].key  ) - 1] = '\0';
        kv_table[kv_count].value[sizeof(kv_table[0].value) - 1] = '\0';
        if (kv_table[kv_count].key[0] == '\0') break;
        ++kv_count;
        i += sizeof(KV);
    }
    return true;
}

} // namespace

bool init() {
    return config_load();
}

bool config_load() {
    const ConfigHeader* best = nullptr;
    uint32_t best_idx = 0;
    uint32_t best_seq = 0;

    for (uint32_t i = 0; i < CONFIG_SLOT_SECTORS; ++i) {
        const ConfigHeader* h = read_config_header(i);
        if (h->magic != MAGIC_CONFIG) continue;
        if (h->payload_len > SECTOR_SIZE - sizeof(ConfigHeader)) continue;
        const uint8_t* payload = reinterpret_cast<const uint8_t*>(h + 1);
        if (crc32(payload, h->payload_len) != h->crc32) continue;
        if (!best || h->sequence > best_seq) {
            best = h; best_seq = h->sequence; best_idx = i;
        }
    }
    if (!best) {
        kv_count = 0;
        config_sequence = 0;
        config_active_idx = CONFIG_SLOT_SECTORS - 1; // nächster wird 0
        return false;
    }
    parse_config_payload(reinterpret_cast<const uint8_t*>(best + 1),
                         best->payload_len);
    config_sequence   = best_seq;
    config_active_idx = best_idx;
    return true;
}

bool config_get(const char* key, char* out, std::size_t out_size) {
    if (!key || !out || out_size == 0) return false;
    for (uint32_t i = 0; i < kv_count; ++i) {
        if (std::strncmp(kv_table[i].key, key, sizeof(kv_table[0].key)) == 0) {
            std::strncpy(out, kv_table[i].value, out_size - 1);
            out[out_size - 1] = '\0';
            return true;
        }
    }
    return false;
}

bool config_set(const char* key, const char* value) {
    if (!key || !value) return false;
    if (std::strlen(key)   >= sizeof(kv_table[0].key))   return false;
    if (std::strlen(value) >= sizeof(kv_table[0].value)) return false;
    for (uint32_t i = 0; i < kv_count; ++i) {
        if (std::strncmp(kv_table[i].key, key, sizeof(kv_table[0].key)) == 0) {
            std::strncpy(kv_table[i].value, value,
                         sizeof(kv_table[0].value) - 1);
            kv_table[i].value[sizeof(kv_table[0].value) - 1] = '\0';
            return true;
        }
    }
    if (kv_count >= MAX_KV) return false;
    std::strncpy(kv_table[kv_count].key, key,
                 sizeof(kv_table[0].key) - 1);
    kv_table[kv_count].key[sizeof(kv_table[0].key) - 1] = '\0';
    std::strncpy(kv_table[kv_count].value, value,
                 sizeof(kv_table[0].value) - 1);
    kv_table[kv_count].value[sizeof(kv_table[0].value) - 1] = '\0';
    ++kv_count;
    return true;
}

bool config_commit() {
    alignas(4) uint8_t buf[SECTOR_SIZE];
    std::memset(buf, 0xFF, sizeof buf);

    ConfigHeader hdr{};
    hdr.magic       = MAGIC_CONFIG;
    hdr.sequence    = config_sequence + 1;
    hdr.payload_len = static_cast<uint32_t>(kv_count * sizeof(KV));

    if (sizeof(ConfigHeader) + hdr.payload_len > SECTOR_SIZE) return false;

    std::memcpy(buf + sizeof(ConfigHeader), kv_table, hdr.payload_len);
    hdr.crc32 = crc32(buf + sizeof(ConfigHeader), hdr.payload_len);
    std::memcpy(buf, &hdr, sizeof(ConfigHeader));

    uint32_t next_idx = (config_active_idx + 1) % CONFIG_SLOT_SECTORS;
    if (!sector_erase_and_write(
            config_region_offset + next_idx * SECTOR_SIZE,
            buf, sizeof(ConfigHeader) + hdr.payload_len)) {
        return false;
    }
    config_active_idx = next_idx;
    config_sequence   = hdr.sequence;
    return true;
}

void config_dump(void (*emit)(const char*)) {
    char line[160];
    for (uint32_t i = 0; i < kv_count; ++i) {
        std::snprintf(line, sizeof line, "%s=%s",
                      kv_table[i].key, kv_table[i].value);
        emit(line);
    }
}

// --- Firmware-Slot ---------------------------------------------------------

bool firmware_erase() {
    FlashGuard guard;
    flash_range_erase(firmware_region_offset, firmware_region_size);
    firmware_length   = 0;
    firmware_sequence = 0;
    return true;
}

bool firmware_write(std::size_t offset, const void* data, std::size_t len) {
    if (offset + len > FIRMWARE_SLOT_BYTES) return false;
    if (offset % FLASH_PAGE_SIZE != 0)      return false;

    // Auf 256-Byte-Pages aufrunden, mit 0xFF füllen.
    std::size_t padded = ((len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE)
                         * FLASH_PAGE_SIZE;
    static alignas(4) uint8_t page[FLASH_PAGE_SIZE * 8];
    if (padded > sizeof page) return false;
    std::memset(page, 0xFF, padded);
    std::memcpy(page, data, len);

    FlashGuard guard;
    flash_range_program(firmware_region_offset + offset, page, padded);
    return true;
}

bool firmware_finalize(std::size_t total_len) {
    if (total_len == 0 || total_len > FIRMWARE_SLOT_BYTES - sizeof(FirmwareHeader)) {
        return false;
    }
    // Marker-Sektor liegt am Ende des Firmware-Slots (letzter Sektor).
    FirmwareHeader hdr{};
    hdr.magic    = MAGIC_FIRMWARE;
    hdr.sequence = firmware_sequence + 1;
    hdr.length   = static_cast<uint32_t>(total_len);
    hdr.crc32    = crc32(xip_ptr(firmware_region_offset),
                         static_cast<std::size_t>(total_len));

    std::size_t marker_offset =
        firmware_region_offset + firmware_region_size - SECTOR_SIZE;
    if (!sector_erase_and_write(marker_offset, &hdr, sizeof hdr)) return false;

    firmware_length   = hdr.length;
    firmware_sequence = hdr.sequence;
    return true;
}

const uint8_t* firmware_data() {
    return xip_ptr(firmware_region_offset);
}

std::size_t firmware_size() {
    if (firmware_length != 0) return firmware_length;
    // Lazy-Read aus Marker
    std::size_t marker_offset = firmware_region_offset + firmware_region_size - SECTOR_SIZE;
    const FirmwareHeader* h = reinterpret_cast<const FirmwareHeader*>(
        xip_ptr(marker_offset));
    if (h->magic != MAGIC_FIRMWARE) return 0;
    if (h->length > FIRMWARE_SLOT_BYTES - SECTOR_SIZE) return 0;
    if (crc32(xip_ptr(firmware_region_offset), h->length) != h->crc32) return 0;
    firmware_length   = h->length;
    firmware_sequence = h->sequence;
    return firmware_length;
}

} // namespace storage
