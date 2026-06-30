#include "usb_msc.h"
#include "config.h"
#include "storage.h"
#include "hex_parser.h"
#include "peripherals.h"
#include "uart_bridge.h"
#include "emulator.h"

#include "tusb.h"
#include "pico/stdlib.h"

#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "pico/time.h"

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
std::atomic<bool> g_volume_processed{false};
std::atomic<bool> g_eject_pending{false};
Stats             g_stats{};

// Wird beim Parsen von CONFIG.INI (flash_erase=on) gesetzt und in
// on_volume_ready ausgewertet, bevor eine evtl. BOOT.HEX angewendet wird.
bool              g_erase_request = false;

// Dirty-Timeout: wenn der Host N ms nicht mehr schreibt, gilt das Volume
// als fertig (für Linux-Systeme, die kein Eject senden).
constexpr uint32_t WRITE_IDLE_TIMEOUT_MS = 2000;
volatile uint32_t  g_last_write_ms = 0;

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
        // LFN-Einträge überspringen (Attribut = 0x0F)
        if (e[11] == 0x0F) continue;
        if ((e[11] & 0x18) != 0) continue;             // Vol/Dir
        // Case-insensitiver Vergleich: Linux kann 8.3-Einträge klein schreiben
        // (NTRes-Flag in byte 12 gesetzt).
        bool match = true;
        for (int j = 0; j < 11; ++j) {
            if (toupper(static_cast<unsigned char>(e[j])) !=
                toupper(static_cast<unsigned char>(want[j]))) {
                match = false; break;
            }
        }
        if (!match) continue;
        cluster = static_cast<uint16_t>(e[26] | (e[27] << 8));
        size    = static_cast<uint32_t>(e[28]) |
                  (static_cast<uint32_t>(e[29]) << 8) |
                  (static_cast<uint32_t>(e[30]) << 16) |
                  (static_cast<uint32_t>(e[31]) << 24);
        return true;
    }
    return false;
}

// Sucht eine ladbare HEX-Datei im Root-Verzeichnis. Bevorzugt BOOT.HEX,
// faellt sonst auf die erste beliebige Datei mit Endung .HEX zurueck. So
// kann der Anwender die Datei unter ihrem Originalnamen ablegen, ohne sie
// vorher in BOOT.HEX umbenennen zu muessen. 'out_name' (>=13 Byte) erhaelt
// den gefundenen 8.3-Namen ("NAME.HEX") fuer die Log-Ausgabe.
bool find_hex_entry(uint16_t& cluster, uint32_t& size, char* out_name) {
    // 1) Bevorzugt BOOT.HEX (eindeutiges, bewusst gewaehltes Ziel).
    if (find_dir_entry("BOOT.HEX", cluster, size) && size > 0) {
        if (out_name) std::strcpy(out_name, "BOOT.HEX");
        return true;
    }
    // 2) Erste beliebige *.HEX-Datei im Root-Verzeichnis.
    const uint8_t* root = g_disk +
        (RESERVED_SECTORS + SECTORS_PER_FAT) * SECTOR_SIZE;
    for (uint32_t i = 0; i < ROOT_DIR_ENTRIES; ++i) {
        const uint8_t* e = root + i * 32;
        if (e[0] == 0x00) break;
        if (e[0] == 0xE5) continue;
        if (e[11] == 0x0F) continue;                   // LFN
        if ((e[11] & 0x18) != 0) continue;             // Vol/Dir
        if (toupper(static_cast<unsigned char>(e[8]))  != 'H' ||
            toupper(static_cast<unsigned char>(e[9]))  != 'E' ||
            toupper(static_cast<unsigned char>(e[10])) != 'X') continue;
        cluster = static_cast<uint16_t>(e[26] | (e[27] << 8));
        size    = static_cast<uint32_t>(e[28]) |
                  (static_cast<uint32_t>(e[29]) << 8) |
                  (static_cast<uint32_t>(e[30]) << 16) |
                  (static_cast<uint32_t>(e[31]) << 24);
        if (size == 0) continue;
        if (out_name) {
            int p = 0;
            for (int j = 0; j < 8 && e[j] != ' '; ++j)
                out_name[p++] = static_cast<char>(
                    toupper(static_cast<unsigned char>(e[j])));
            out_name[p++] = '.';
            out_name[p++] = 'H'; out_name[p++] = 'E'; out_name[p++] = 'X';
            out_name[p]   = '\0';
        }
        return true;
    }
    return false;
}

// CONFIG.INI parsen: simple key=value, eine Zeile pro Eintrag.
//   pin.<lpc-port_pin>=<rp-gpio>      (z. B. pin.0_3=14)
//   autostart=on|off
//   freq_hz=<Hz>
//   uart_bridge_en=on|off
//   uart_bridge_tx=<rp-gpio>   uart_bridge_rx=<rp-gpio>
//   i2c_bridge_en=on|off
//   i2c_bridge_inst=0|1   i2c_bridge_sda=<rp-gpio>
//   i2c_bridge_scl=<rp-gpio>   i2c_bridge_hz=<Hz>
//   spi_bridge_en=on|off
//   spi_bridge_inst=0|1   spi_bridge_lpc=0|1
//   spi_bridge_sck=<rp-gpio>   spi_bridge_mosi=<rp-gpio>
//   spi_bridge_miso=<rp-gpio>  spi_bridge_hz=<Hz>
//   adc_bridge_en=on|off
//   tcap.<t>=<rp-gpio>        Timer-Capture-Eingang (KNX-RX), t=0..3
//   tmat.<t>.<m>=<rp-gpio>    Timer-Match-Ausgang (KNX-TX), t=0..3, m=0..3
//   tcap_pio=on|off          Capture flankengenau per PIO (opt-in)
//   tmatch_pio=on|off        Match/PWM-Ausgangspuls hardware-getaktet per PIO (opt-in)
//   wfi_pin_wakeup=on|off   (opt-in)
//   primask_shadow=on|off   (opt-in)
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
        } else if (std::strncmp(line, "tcap.", 5) == 0) {
            // tcap.<t>=<rp-gpio>
            int t, rp;
            if (std::sscanf(line + 5, "%d", &t) == 1 &&
                std::sscanf(eq, "%d", &rp) == 1) {
                config::set_ct_capture_pin(t, rp);
            }
        } else if (std::strncmp(line, "tmat.", 5) == 0) {
            // tmat.<t>.<m>=<rp-gpio>
            int t, m, rp;
            if (std::sscanf(line + 5, "%d.%d", &t, &m) == 2 &&
                std::sscanf(eq, "%d", &rp) == 1) {
                config::set_ct_match_pin(t, m, rp);
            }
        } else if (std::strcmp(line, "autostart") == 0) {
            config::set_autostart(std::strcmp(eq, "on") == 0 ||
                                  std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "freq_hz") == 0) {
            config::set_target_frequency_hz(static_cast<uint32_t>(std::atol(eq)));
        } else if (std::strcmp(line, "uart_bridge_en") == 0) {
            config::set_uart_bridge_enabled(std::strcmp(eq, "on") == 0 ||
                                            std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "uart_bridge_tx") == 0) {
            config::set_uart_bridge_tx_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "uart_bridge_rx") == 0) {
            config::set_uart_bridge_rx_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "i2c_bridge_en") == 0) {
            config::set_i2c_bridge_enabled(std::strcmp(eq, "on") == 0 ||
                                           std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "i2c_bridge_inst") == 0) {
            config::set_i2c_bridge_instance(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "i2c_bridge_sda") == 0) {
            config::set_i2c_bridge_sda_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "i2c_bridge_scl") == 0) {
            config::set_i2c_bridge_scl_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "i2c_bridge_hz") == 0) {
            config::set_i2c_bridge_hz(static_cast<uint32_t>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_en") == 0) {
            config::set_spi_bridge_enabled(std::strcmp(eq, "on") == 0 ||
                                           std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "spi_bridge_inst") == 0) {
            config::set_spi_bridge_instance(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_lpc") == 0) {
            config::set_spi_bridge_lpc(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_sck") == 0) {
            config::set_spi_bridge_sck_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_mosi") == 0) {
            config::set_spi_bridge_mosi_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_miso") == 0) {
            config::set_spi_bridge_miso_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "spi_bridge_hz") == 0) {
            config::set_spi_bridge_hz(static_cast<uint32_t>(std::atol(eq)));
        } else if (std::strcmp(line, "adc_bridge_en") == 0) {
            config::set_adc_bridge_enabled(std::strcmp(eq, "on") == 0 ||
                                           std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "tcap_pio") == 0) {
            config::set_tcap_pio(std::strcmp(eq, "on") == 0 ||
                                 std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "tmatch_pio") == 0) {
            config::set_tmatch_pio(std::strcmp(eq, "on") == 0 ||
                                   std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "wfi_pin_wakeup") == 0) {
            config::set_wfi_pin_wakeup(std::strcmp(eq, "on") == 0 ||
                                       std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "primask_shadow") == 0) {
            config::set_primask_shadow(std::strcmp(eq, "on") == 0 ||
                                       std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "app_start") == 0) {
            config::set_app_start_addr(
                static_cast<uint32_t>(std::strtoul(eq, nullptr, 0)));
        } else if (std::strcmp(line, "desc_addr") == 0) {
            config::set_descriptor_addr(
                static_cast<uint32_t>(std::strtoul(eq, nullptr, 0)));
        } else if (std::strcmp(line, "autodesc") == 0) {
            config::set_autodesc(std::strcmp(eq, "on") == 0 ||
                                 std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "flash_erase") == 0 ||
                   std::strcmp(line, "erase") == 0) {
            // Loescht den gesamten Firmware-Slot, bevor eine evtl. ebenfalls
            // abgelegte BOOT.HEX angewendet wird. Ohne diesen Schluessel wird
            // BOOT.HEX additiv (mergend) in den Slot geschrieben.
            g_erase_request = (std::strcmp(eq, "on") == 0 ||
                               std::strcmp(eq, "1")  == 0);
        }
    }
}

// Trigger nach Eject vom Hauptloop aufgerufen.
void on_volume_ready() {
    uint16_t cl; uint32_t sz;
    g_erase_request = false;

    // Lauft ein Gast nativ auf Core1, vor den Flash-Schreibzugriffen pausieren
    // (sonst crasht Core1 waehrend des XIP-Stalls). Der Guard setzt den Gast am
    // Funktionsende fort: bei neuer BOOT.HEX startet er die NEUE Firmware, bei
    // reiner CONFIG.INI die bisherige weiter. Lief kein Gast, ist es ein No-op.
    emulator::FlashPauseGuard flash_pause;

    // CONFIG.INI zuerst (damit Pinmap vor dem Boot wirkt).
    if (find_dir_entry("CONFIG.INI", cl, sz) && sz > 0 && sz < 4096) {
        char buf[4096];
        uint32_t n = read_cluster_chain(cl, sz, reinterpret_cast<uint8_t*>(buf),
                                        sizeof buf);
        parse_config(buf, n);
        // Persistieren, damit die Einstellungen den nächsten Power-Cycle
        // überleben (config::load() liest sie beim Boot wieder ein).
        config::save();
        // Bridges sofort anwenden, damit der direkt folgende Autorun sie
        // bereits nutzen kann (ohne Power-Cycle).
        peripherals::i2c_bridge_reinit();
        peripherals::spi_bridge_reinit();
        peripherals::adc_bridge_reinit();
        peripherals::ct_bridge_reinit();
        if (config::uart_bridge_enabled()) {
            uart_bridge::set_tx_pin(config::uart_bridge_tx_pin());
            uart_bridge::set_rx_pin(config::uart_bridge_rx_pin());
            uart_bridge::start();
        } else {
            uart_bridge::stop();
        }
        std::printf("[MSC] CONFIG.INI: %lu Bytes, %u Zeilen\n",
                    static_cast<unsigned long>(n),
                    static_cast<unsigned>(g_stats.parsed_lines));
    }

    // Optionales vollstaendiges Loeschen des Firmware-Slots (flash_erase=on).
    if (g_erase_request) {
        storage::firmware_erase();
        std::printf("[MSC] flash_erase=on -> Firmware-Slot geloescht\n");
    }

    // BOOT.HEX -> in firmware-Slot persistieren (additiv/mergend; zum
    // vollstaendigen Ersetzen flash_erase=on in CONFIG.INI verwenden).
    // Akzeptiert auch jede andere *.HEX-Datei (Originalname zulaessig).
    char hex_name[16];
    if (find_hex_entry(cl, sz, hex_name) && sz > 0) {
        static uint8_t hex_buf[64 * 1024 + 1024];
        uint32_t n = read_cluster_chain(cl, sz, hex_buf, sizeof hex_buf);
        // Stream-Parser mit Writer auf storage::firmware_write.
        struct Ctx {
            uint32_t total;
            bool     overflow;
        } ctx{0, false};
        static Ctx* g_ctx;
        g_ctx = &ctx;
        auto writer = [](uint32_t address, const uint8_t* data,
                         std::size_t len) -> bool {
            if (address + len > 64u * 1024u) { g_ctx->overflow = true; return false; }
            if (!storage::firmware_write(address, data, len)) {
                g_ctx->overflow = true; return false;
            }
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
            std::printf("[MSC] %s parsefehler\n", hex_name);
        } else {
            storage::firmware_finalize(ctx.total);
            ++g_stats.boot_requests;
            g_boot_pending.store(true);
            std::printf("[MSC] %s %lu B → flash\n", hex_name,
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
    // Eject-Flag: wurde im USB-Callback gesetzt, jetzt im Hauptloop sicher
    // verarbeiten (Flash-Operationen brauchen lange + deaktivieren IRQs).
    if (g_eject_pending.exchange(false)) {
        g_dirty.store(false);
        g_volume_processed.store(true);
        on_volume_ready();
        return;
    }

    // Dirty-Timeout: Falls der Host nach dem letzten Schreibzugriff
    // WRITE_IDLE_TIMEOUT_MS nicht mehr geschrieben hat, gilt das Volume
    // als fertig geschrieben. Deckt Linux-umount ohne explizites Eject ab.
    if (g_dirty.load(std::memory_order_relaxed) &&
        !g_volume_processed.load(std::memory_order_relaxed)) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t last = g_last_write_ms;
        if (last != 0 && (now - last) >= WRITE_IDLE_TIMEOUT_MS) {
            g_dirty.store(false);
            g_volume_processed.store(true);
            on_volume_ready();
        }
    }
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
        // Host hat das Volume ausgeworfen. NICHT hier Flash-Operationen
        // ausführen (wir sind im TinyUSB-Callback-Kontext, Interrupts
        // dürfen nicht lange blockiert werden). Nur Flag setzen.
        usb_msc::g_eject_pending.store(true);
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
    usb_msc::g_volume_processed.store(false);
    usb_msc::g_last_write_ms = to_ms_since_boot(get_absolute_time());
    ++usb_msc::g_stats.writes;
    return static_cast<int32_t>(bufsize);
}

int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16],
                        void* buffer, uint16_t bufsize) {
    (void)lun; (void)buffer; (void)bufsize;
    // PREVENT_ALLOW_MEDIUM_REMOVAL (0x1E): Linux sendet dies vor dem Eject.
    // Ohne Success-Antwort bricht Linux den Eject ab und on_volume_ready()
    // wird nie aufgerufen.
    if (scsi_cmd[0] == 0x1E) return 0;   // Success
    // SYNCHRONIZE_CACHE (0x35): Linux sendet dies beim umount/sync.
    if (scsi_cmd[0] == 0x35) return 0;   // Success
    return -1;
}

bool tud_msc_is_writable_cb(uint8_t /*lun*/) { return true; }

} // extern "C"
