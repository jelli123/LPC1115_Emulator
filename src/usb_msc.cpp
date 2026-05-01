#include "usb_msc.h"
#include "config.h"
#include "storage.h"
#include "hex_parser.h"

#include "tusb.h"
#include "pico/stdlib.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace usb_msc {

namespace {

constexpr uint16_t SECTORS_PER_FAT      = 1;
constexpr uint16_t RESERVED_SECTORS     = 1;
constexpr uint16_t ROOT_DIR_ENTRIES     = 16;
constexpr uint16_t ROOT_DIR_SECTORS     = (ROOT_DIR_ENTRIES * 32) / SECTOR_SIZE;
constexpr uint16_t FIRST_DATA_SECTOR    = RESERVED_SECTORS + SECTORS_PER_FAT + ROOT_DIR_SECTORS;
constexpr uint16_t SECTORS_PER_CLUSTER  = 1;
constexpr uint16_t TOTAL_CLUSTERS       = SECTOR_COUNT - FIRST_DATA_SECTOR;

// 128 KiB Volume – gehalten in RAM. Nach Host-Eject wird der Bereich nach
// BOOT.HEX und CONFIG.INI durchsucht und in Storage persistiert.
alignas(4) uint8_t g_disk[VOLUME_BYTES];

std::atomic<bool> g_dirty{false};
std::atomic<bool> g_boot_pending{false};
Stats             g_stats{};

const char VOL_LABEL[11]  = { 'L','P','C','1','1','1','5','E','M','U',' ' };
const char OEM_NAME[8]    = { 'M','S','D','O','S','5','.','0' };

// Helper: 8.3-Filename normalisieren ("BOOT.HEX" → "BOOT    HEX").
void to_83(const char* src, char out[11]) {
    std::memset(out, ' ', 11);
    int i = 0, j = 0;
    while (src[i] && src[i] != '.' && j < 8) out[j++] = static_cast<char>(toupper(src[i++]));
    while (src[i] && src[i] != '.') ++i;
    if (src[i] == '.') ++i;
    j = 8;
    while (src[i] && j < 11) out[j++] = static_cast<char>(toupper(src[i++]));
}

// Format-Helper: legt ein leeres FAT12-Volume in g_disk an.
void format_blank() {
    std::memset(g_disk, 0, sizeof g_disk);

    // ---- Boot-Sector (Sektor 0) ----
    uint8_t* bs = g_disk;
    bs[0] = 0xEB; bs[1] = 0x3C; bs[2] = 0x90;          // jmp short
    std::memcpy(bs + 3, OEM_NAME, 8);
    bs[11] = SECTOR_SIZE & 0xFF;  bs[12] = (SECTOR_SIZE >> 8) & 0xFF;
    bs[13] = SECTORS_PER_CLUSTER;
    bs[14] = RESERVED_SECTORS & 0xFF; bs[15] = (RESERVED_SECTORS >> 8) & 0xFF;
    bs[16] = 1;                                        // FAT count
    bs[17] = ROOT_DIR_ENTRIES & 0xFF; bs[18] = (ROOT_DIR_ENTRIES >> 8) & 0xFF;
    bs[19] = SECTOR_COUNT & 0xFF; bs[20] = (SECTOR_COUNT >> 8) & 0xFF;
    bs[21] = 0xF8;                                     // Media Descriptor
    bs[22] = SECTORS_PER_FAT & 0xFF; bs[23] = (SECTORS_PER_FAT >> 8) & 0xFF;
    bs[24] = 1; bs[25] = 0;                            // sec/track
    bs[26] = 1; bs[27] = 0;                            // heads
    // Extended Boot Record
    bs[38] = 0x29;                                     // signature
    bs[39] = 0x12; bs[40] = 0x34; bs[41] = 0x56; bs[42] = 0x78;
    std::memcpy(bs + 43, VOL_LABEL, 11);
    std::memcpy(bs + 54, "FAT12   ", 8);
    bs[510] = 0x55; bs[511] = 0xAA;

    // ---- FAT (Sektor 1) ----
    uint8_t* fat = g_disk + RESERVED_SECTORS * SECTOR_SIZE;
    fat[0] = 0xF8; fat[1] = 0xFF; fat[2] = 0xFF;       // Reserved entries

    // ---- Root-Dir (Sektor 2) — Volume-Label-Eintrag ----
    uint8_t* root = g_disk + (RESERVED_SECTORS + SECTORS_PER_FAT) * SECTOR_SIZE;
    std::memcpy(root, VOL_LABEL, 11);
    root[11] = 0x08;                                   // Volume-ID-Attribut

    g_dirty.store(true);
}

// Liest die Cluster-Kette einer Datei aus FAT12 und kopiert in einen
// linearen Puffer (max. cap Bytes). Liefert Bytes geschrieben.
uint32_t read_cluster_chain(uint16_t first_cluster, uint32_t size,
                            uint8_t* dst, uint32_t cap) {
    uint8_t* fat  = g_disk + RESERVED_SECTORS * SECTOR_SIZE;
    uint16_t cur  = first_cluster;
    uint32_t copied = 0;
    while (cur >= 2 && cur < 0xFF8 && copied < size && copied < cap) {
        uint32_t sec = FIRST_DATA_SECTOR + (cur - 2) * SECTORS_PER_CLUSTER;
        uint32_t to_copy = SECTOR_SIZE * SECTORS_PER_CLUSTER;
        if (size - copied < to_copy) to_copy = size - copied;
        if (cap  - copied < to_copy) to_copy = cap  - copied;
        std::memcpy(dst + copied, g_disk + sec * SECTOR_SIZE, to_copy);
        copied += to_copy;

        // FAT12-Entry für `cur` lesen.
        uint32_t off = (cur * 3) / 2;
        uint16_t e   = static_cast<uint16_t>(fat[off]) |
                       (static_cast<uint16_t>(fat[off + 1]) << 8);
        cur = (cur & 1) ? (e >> 4) : (e & 0x0FFF);
    }
    return copied;
}

bool find_dir_entry(const char* name83, uint16_t& cluster, uint32_t& size) {
    const uint8_t* root = g_disk +
        (RESERVED_SECTORS + SECTORS_PER_FAT) * SECTOR_SIZE;
    char want[11];
    to_83(name83, want);
    for (uint32_t i = 0; i < ROOT_DIR_ENTRIES; ++i) {
        const uint8_t* e = root + i * 32;
        if (e[0] == 0x00) break;
        if (e[0] == 0xE5) continue;
        if ((e[11] & 0x18) != 0) continue;             // Vol/Dir
        if (std::memcmp(e, want, 11) != 0) continue;
        cluster = static_cast<uint16_t>(e[26] | (e[27] << 8));
        size    = static_cast<uint32_t>(e[28]) |
                  (static_cast<uint32_t>(e[29]) << 8) |
                  (static_cast<uint32_t>(e[30]) << 16) |
                  (static_cast<uint32_t>(e[31]) << 24);
        return true;
    }
    return false;
}

// CONFIG.INI parsen: simple key=value, eine Zeile pro Eintrag.
//   pin.<lpc-port_pin>=<rp-gpio>      (z. B. pin.0_3=14)
//   autostart=on|off
//   gdb=on|off
void parse_config(const char* buf, uint32_t len) {
    char line[96];
    uint32_t i = 0;
    while (i < len) {
        uint32_t j = 0;
        while (i < len && j < sizeof(line) - 1 && buf[i] != '\n') {
            if (buf[i] != '\r') line[j++] = buf[i];
            ++i;
        }
        line[j] = 0;
        if (i < len) ++i;
        if (line[0] == '#' || line[0] == ';' || line[0] == 0) continue;
        char* eq = std::strchr(line, '=');
        if (!eq) continue;
        *eq++ = 0;
        // Whitespace trimmen
        while (*line == ' ' || *line == '\t') {
            char* p = line; do { p[0] = p[1]; } while (*p++);
        }
        char* end = line + std::strlen(line);
        while (end > line && (end[-1] == ' ' || end[-1] == '\t')) *--end = 0;

        ++g_stats.parsed_lines;
        if (std::strncmp(line, "pin.", 4) == 0) {
            // pin.<port>_<pin>=<rp-gpio>
            int port, pin, rp;
            if (std::sscanf(line + 4, "%d_%d", &port, &pin) == 2 &&
                std::sscanf(eq, "%d", &rp) == 1) {
                config::set_pin_map(static_cast<uint8_t>(port * 12 + pin), rp);
            }
        } else if (std::strcmp(line, "autostart") == 0) {
            config::set_autostart(std::strcmp(eq, "on") == 0 ||
                                  std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "freq_hz") == 0) {
            config::set_target_frequency_hz(static_cast<uint32_t>(std::atol(eq)));
        }
    }
}

// Trigger nach Eject vom Hauptloop aufgerufen.
void on_volume_ready() {
    uint16_t cl; uint32_t sz;

    // CONFIG.INI zuerst (damit Pinmap vor dem Boot wirkt).
    if (find_dir_entry("CONFIG.INI", cl, sz) && sz > 0 && sz < 4096) {
        char buf[4096];
        uint32_t n = read_cluster_chain(cl, sz, reinterpret_cast<uint8_t*>(buf),
                                        sizeof buf);
        parse_config(buf, n);
        std::printf("[MSC] CONFIG.INI: %lu Bytes, %u Zeilen\n",
                    static_cast<unsigned long>(n),
                    static_cast<unsigned>(g_stats.parsed_lines));
    }

    // BOOT.HEX → in firmware-Slot persistieren.
    if (find_dir_entry("BOOT.HEX", cl, sz) && sz > 0) {
        static uint8_t hex_buf[64 * 1024 + 1024];
        uint32_t n = read_cluster_chain(cl, sz, hex_buf, sizeof hex_buf);
        // Stream-Parser mit Writer auf storage::firmware_write.
        storage::firmware_erase();
        struct Ctx {
            uint32_t total;
            bool     overflow;
        } ctx{0, false};
        static Ctx* g_ctx;
        g_ctx = &ctx;
        auto writer = [](uint32_t address, const uint8_t* data,
                         std::size_t len) -> bool {
            if (address + len > 64u * 1024u) { g_ctx->overflow = true; return false; }
            storage::firmware_write(address, data, len);
            uint32_t end = address + static_cast<uint32_t>(len);
            if (end > g_ctx->total) g_ctx->total = end;
            return true;
        };
        hex::Parser p(writer, 0, 64u * 1024u);
        bool any_err = false;
        for (uint32_t i = 0; i < n; ++i) {
            hex::Result r = p.feed(static_cast<char>(hex_buf[i]));
            if (r == hex::Result::EndOfFile) break;
            if (r != hex::Result::Ok && r != hex::Result::InProgress) {
                any_err = true; break;
            }
        }
        if (any_err || ctx.overflow) {
            std::printf("[MSC] BOOT.HEX parsefehler\n");
        } else {
            storage::firmware_finalize(ctx.total);
            ++g_stats.boot_requests;
            g_boot_pending.store(true);
            std::printf("[MSC] BOOT.HEX %lu B → flash\n",
                        static_cast<unsigned long>(ctx.total));
        }
    }
}

} // namespace

void init() {
    std::memset(&g_stats, 0, sizeof g_stats);
    // Persistente Wiederherstellung wäre möglich (storage::msc_load_volume).
    // Fürs Erste: bei jedem Power-Cycle ein leeres FAT12 anlegen und auf
    // die nächsten Host-Schreibvorgänge warten.
    format_blank();
}

bool consume_pending_boot_request() {
    return g_boot_pending.exchange(false);
}

void poll() {
    // Periodisches Persistieren wäre hier möglich. Aktuell flush nur
    // beim Eject (siehe tud_msc_start_stop_cb).
}

bool find_file(const char* name83, File& out) {
    uint16_t cl; uint32_t sz;
    if (!find_dir_entry(name83, cl, sz)) return false;
    static uint8_t lin[64 * 1024];
    uint32_t n = read_cluster_chain(cl, sz, lin, sizeof lin);
    out.data = lin;
    out.size = n;
    return true;
}

bool read_text_config(const char* /*n*/) { return false; /* ungenutzt */ }

Stats stats() { return g_stats; }

} // namespace usb_msc

// =============================================================================
// TinyUSB MSC-Callbacks
// =============================================================================
extern "C" {

void tud_msc_inquiry_cb(uint8_t /*lun*/, uint8_t vendor[8],
                        uint8_t product[16], uint8_t rev[4]) {
    const char v[] = "Selfbus ";
    const char p[] = "LPC1115EmuVolume";
    const char r[] = "1.0 ";
    std::memcpy(vendor,  v, 8);
    std::memcpy(product, p, 16);
    std::memcpy(rev,     r, 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t /*lun*/) { return true; }

void tud_msc_capacity_cb(uint8_t /*lun*/, uint32_t* block_count,
                         uint16_t* block_size) {
    *block_count = usb_msc::SECTOR_COUNT;
    *block_size  = usb_msc::SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t /*lun*/, uint8_t /*power_condition*/,
                           bool start, bool load_eject) {
    if (load_eject && !start) {
        // Host hat das Volume ausgeworfen → unsere Trigger-Stelle.
        usb_msc::on_volume_ready();
    }
    return true;
}

int32_t tud_msc_read10_cb(uint8_t /*lun*/, uint32_t lba, uint32_t offset,
                          void* buffer, uint32_t bufsize) {
    if (lba >= usb_msc::SECTOR_COUNT) return -1;
    uint32_t addr = lba * usb_msc::SECTOR_SIZE + offset;
    if (addr + bufsize > usb_msc::VOLUME_BYTES) return -1;
    std::memcpy(buffer, usb_msc::g_disk + addr, bufsize);
    ++usb_msc::g_stats.reads;
    return static_cast<int32_t>(bufsize);
}

int32_t tud_msc_write10_cb(uint8_t /*lun*/, uint32_t lba, uint32_t offset,
                           uint8_t* buffer, uint32_t bufsize) {
    if (lba >= usb_msc::SECTOR_COUNT) return -1;
    uint32_t addr = lba * usb_msc::SECTOR_SIZE + offset;
    if (addr + bufsize > usb_msc::VOLUME_BYTES) return -1;
    std::memcpy(usb_msc::g_disk + addr, buffer, bufsize);
    usb_msc::g_dirty.store(true);
    ++usb_msc::g_stats.writes;
    return static_cast<int32_t>(bufsize);
}

int32_t tud_msc_scsi_cb(uint8_t /*lun*/, const uint8_t /*scsi_cmd*/[16],
                        void* /*buffer*/, uint16_t /*bufsize*/) {
    return -1;     // ungehandhabte SCSI-Kommandos → INVALID
}

bool tud_msc_is_writable_cb(uint8_t /*lun*/) { return true; }

} // extern "C"
