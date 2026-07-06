#include "usb_msc.h"
#include "config.h"
#include "storage.h"
#include "hex_parser.h"
#include "peripherals.h"
#include "uart_bridge.h"
#include "emulator.h"
#include "fault.h"
#include "help_html.h"
#include "debug_bridge.h"

#include "tusb.h"
#include "pico/stdlib.h"

#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>

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

// Auto-Flush der DEBUG.TXT: baut die RAM-Disk periodisch neu auf (frische
// DEBUG.TXT aus der Debug-Bridge) und laesst den Host neu einlesen, sobald neue
// Gast-Debug-Bytes vorliegen. Default 0 = AUS: ein periodischer Medienwechsel
// laesst das Laufwerk am Host staendig neu einbinden ("Remount-Churn"), was das
// Bearbeiten der CONFIG.INI stoert. Der Nutzer aktiviert es bei Bedarf per
// 'dbg auto <sek>'; manuelles 'dbg save' aktualisiert DEBUG.TXT jederzeit.
uint32_t g_dbg_autoflush_ms  = 0;
uint32_t g_last_dbg_flush_ms = 0;
uint32_t g_last_dbg_total    = 0;

// Hash des zuletzt VERARBEITETEN Volume-Zustands (siehe volume_hash). Verhindert,
// dass ein inhaltlich unveraenderter Re-Trigger (z. B. Host-Re-Mount nach Eject,
// der nur FAT-Metadaten neu schreibt) dieselbe BOOT.HEX ein zweites Mal flasht
// und den Gast unnoetig neu startet.
uint32_t g_last_volume_hash = 0;
bool     g_have_volume_hash = false;

// Inhalts-Hashes der zuletzt VERARBEITETEN Nutzdateien (nicht des ganzen
// Volumes). Der Host schreibt eine Datei oft in mehreren Bursts mit Pausen
// (> WRITE_IDLE_TIMEOUT_MS); jeder Zwischenstand hat einen anderen volume_hash
// und wuerde sonst on_volume_ready() (inkl. Gast-Stop/-Neustart) erneut
// ausloesen -> unregelmaessiges Blinken + Revert einer per CLI gesetzten
// Pinmap. Durch den separaten Inhalts-Hash von CONFIG.INI bzw. der HEX wird nur
// dann wirklich verarbeitet (und der Gast nur dann neu gestartet), wenn sich der
// DATEI-Inhalt geaendert hat.
uint32_t g_last_config_hash = 0;
bool     g_have_config_hash = false;
uint32_t g_last_hex_hash    = 0;
bool     g_have_hex_hash    = false;

// FNV-1a ueber einen Byte-Bereich (fuer die Datei-Inhalts-Dedup).
uint32_t content_hash(const uint8_t* p, uint32_t n) {
    uint32_t h = 2166136261u;
    for (uint32_t i = 0; i < n; ++i) h = (h ^ p[i]) * 16777619u;
    return h;
}

// Naechster freier FAT12-Cluster fuer device-seitig injizierte Dateien
// (HELP.HTM, CONFIG.INI, FAULT.TXT). Wird von format_blank auf 2 zurueckgesetzt
// und von fat_write_file pro Datei weitergezaehlt, sodass mehrere Dateien
// nacheinander (nicht ueberlappend) abgelegt werden koennen.
uint16_t g_next_free_cluster = 2;

// --- Media-Change (fuer FAULT.TXT sichtbar machen) ------------------------
// Um eine device-seitig injizierte Datei (FAULT.TXT) fuer den Host sichtbar zu
// machen, simulieren wir Auswurf+Wiedereinlegen des Mediums: waehrend eines
// kurzen Fensters meldet test_unit_ready "medium not present", danach einmalig
// UNIT ATTENTION ("medium may have changed"). Der Host verwirft dann seinen
// FAT-/Directory-Cache und liest das Volume frisch ein. Alles auf Core0
// (TinyUSB-Task + poll laufen kooperativ auf demselben Core) -> keine Atomics.
volatile uint32_t g_media_absent_until = 0;   // ms-Zeitpunkt, bis Medium "weg"
bool              g_media_absent      = false;
bool              g_media_attention   = false; // one-shot nach Wiederkehr

void trigger_media_change() {
    g_media_absent_until = to_ms_since_boot(get_absolute_time()) + 700u;
    g_media_absent = true;
}

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

    g_next_free_cluster = 2;                           // Datenbereich wieder frei
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

bool find_dir_entry(const char* name83, uint16_t& cluster, uint32_t& size,
                    uint32_t* out_idx = nullptr) {
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
        if (out_idx) *out_idx = i;
        return true;
    }
    return false;
}

// VFAT-Pruefsumme des 8.3-Namens (11 Byte). Verbindet die LFN-Teileintraege mit
// ihrem Short-Entry (Byte[13] jedes LFN-Eintrags traegt genau diese Summe).
uint8_t lfn_checksum(const uint8_t* name11) {
    uint8_t sum = 0;
    for (int i = 0; i < 11; ++i)
        sum = static_cast<uint8_t>((((sum & 1u) ? 0x80u : 0u) +
                                    (sum >> 1) + name11[i]) & 0xFFu);
    return sum;
}

// Rekonstruiert den VFAT-Langnamen (LFN) zum 8.3-Eintrag an Index `short_idx`.
// Die LFN-Teileintraege stehen unmittelbar VOR dem Short-Entry (kleinere
// Indizes), jeweils Attribut 0x0F; Byte[0]&0x3F = 1-basierte Teilnummer, Bit
// 0x40 = letzter logischer Teil; Byte[13] = 8.3-Pruefsumme (Zugehoerigkeit).
// Jeder Teil traegt 13 UTF-16-Zeichen (Offsets 1..10, 14..25, 28..31). Fuer die
// Log-Ausgabe auf ASCII reduziert (Nicht-ASCII -> '?'). Liefert false, wenn kein
// gueltiger LFN vorliegt -> der Aufrufer nutzt dann den 8.3-Namen.
bool read_long_name(uint32_t short_idx, char* out, std::size_t cap) {
    if (!out || cap < 2u || short_idx == 0u) return false;
    const uint8_t* root = g_disk +
        (RESERVED_SECTORS + SECTORS_PER_FAT) * SECTOR_SIZE;
    const uint8_t sum = lfn_checksum(root + short_idx * 32);
    static const uint8_t POS[13] = {1,3,5,7,9, 14,16,18,20,22,24, 28,30};

    std::size_t written = 0;
    bool done = false;
    for (uint32_t n = 1; n <= 20u && short_idx >= n && !done; ++n) {
        const uint8_t* le = root + (short_idx - n) * 32;
        if (le[11] != 0x0F)       return false;   // kein LFN -> kein Langname
        if (le[13] != sum)        return false;   // gehoert nicht zu diesem Short
        if ((le[0] & 0x3Fu) != n) return false;   // Reihenfolge gestoert
        std::size_t base = static_cast<std::size_t>(n - 1u) * 13u;
        for (int k = 0; k < 13; ++k) {
            uint16_t ch = static_cast<uint16_t>(le[POS[k]] | (le[POS[k] + 1] << 8));
            if (ch == 0x0000) { done = true; break; }   // Namensende
            if (ch == 0xFFFF) continue;                  // Padding
            std::size_t idx = base + static_cast<std::size_t>(k);
            if (idx < cap - 1u) {
                out[idx] = (ch >= 0x20 && ch < 0x80) ? static_cast<char>(ch) : '?';
                if (idx + 1u > written) written = idx + 1u;
            }
        }
        if (le[0] & 0x40u) done = true;              // letzter logischer Teil
    }
    if (written == 0u) return false;
    out[written] = '\0';
    return true;
}

// Sucht eine ladbare HEX-Datei im Root-Verzeichnis. Bevorzugt BOOT.HEX,
// faellt sonst auf die erste beliebige Datei mit Endung .HEX zurueck. So
// kann der Anwender die Datei unter ihrem Originalnamen ablegen, ohne sie
// vorher in BOOT.HEX umbenennen zu muessen. 'out_name' (>=13 Byte) erhaelt
// den gefundenen 8.3-Namen ("NAME.HEX") fuer die Log-Ausgabe.
bool find_hex_entry(uint16_t& cluster, uint32_t& size,
                    char* out_name, std::size_t out_cap) {
    // 1) Bevorzugt BOOT.HEX (eindeutiges, bewusst gewaehltes Ziel).
    uint32_t bidx = 0;
    if (find_dir_entry("BOOT.HEX", cluster, size, &bidx) && size > 0) {
        if (out_name && out_cap > 0) {
            // Langnamen bevorzugen (falls per LFN abgelegt), sonst 8.3.
            if (!read_long_name(bidx, out_name, out_cap))
                std::snprintf(out_name, out_cap, "BOOT.HEX");
        }
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
        if (out_name && out_cap > 0) {
            // Zuerst den 8.3-Namen als Fallback bauen ...
            int p = 0;
            for (int j = 0; j < 8 && e[j] != ' ' &&
                            static_cast<std::size_t>(p) + 5u < out_cap; ++j)
                out_name[p++] = static_cast<char>(
                    toupper(static_cast<unsigned char>(e[j])));
            out_name[p++] = '.';
            out_name[p++] = 'H'; out_name[p++] = 'E'; out_name[p++] = 'X';
            out_name[p]   = '\0';
            // ... dann, falls vorhanden, den VFAT-Langnamen bevorzugen.
            read_long_name(i, out_name, out_cap);
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
//
// Hinweis: Zahlen werden per strtol geparst (NICHT sscanf). sscanf ist auf
// newlib-nano stark stack-hungrig und kann in tiefen Aufrufkontexten den
// knappen Core0-Stack sprengen (gleiche Klasse wie der fruehere CLI-Hang).

// Eine vorzeichenlose Dezimalzahl am Anfang von s parsen. Liefert false, wenn
// dort keine Ziffer steht; setzt *endp hinter die Zahl (falls endp != nullptr).
bool cfg_parse_int(const char* s, int& out, const char** endp = nullptr) {
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (end == s) return false;
    out = static_cast<int>(v);
    if (endp) *endp = end;
    return true;
}

void parse_config(const char* buf, uint32_t len) {
    // CONFIG.INI ist fuer die Pin-Zuordnungen (Pinmap + Timer-Capture/Match)
    // AUTORITATIV: vor dem Einlesen auf Default zuruecksetzen, damit ein aus der
    // Datei ENTFERNTER (oder mit '#' auskommentierter) Eintrag wirklich
    // verschwindet. Sonst bliebe eine alte Zuordnung (z. B. tmat.2.1) aus dem
    // vorherigen Zustand haengen. Die vom Geraet generierte CONFIG.INI enthaelt
    // stets alle aktiven Zuordnungen -> beim Editieren gehen keine verloren.
    // Skalare Keys (autostart/freq/Bridges) bleiben Merge (eindeutig, kein
    // Loesch-Problem).
    config::reset_pin_mappings();

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
            int port = 0, pin = 0, rp = 0;
            const char* p = nullptr;
            if (cfg_parse_int(line + 4, port, &p) && *p == '_' &&
                cfg_parse_int(p + 1, pin) && cfg_parse_int(eq, rp)) {
                config::set_pin_map(static_cast<uint8_t>(port * 12 + pin), rp);
            }
        } else if (std::strncmp(line, "tcap.", 5) == 0) {
            // tcap.<t>=<rp-gpio>
            int t = 0, rp = 0;
            if (cfg_parse_int(line + 5, t) && cfg_parse_int(eq, rp)) {
                config::set_ct_capture_pin(t, rp);
            }
        } else if (std::strncmp(line, "tmat.", 5) == 0) {
            // tmat.<t>.<m>=<rp-gpio>
            int t = 0, m = 0, rp = 0;
            const char* p = nullptr;
            if (cfg_parse_int(line + 5, t, &p) && *p == '.' &&
                cfg_parse_int(p + 1, m) && cfg_parse_int(eq, rp)) {
                config::set_ct_match_pin(t, m, rp);
            }
        } else if (std::strcmp(line, "autostart") == 0) {
            config::set_autostart(std::strcmp(eq, "on") == 0 ||
                                  std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "freq_hz") == 0) {
            config::set_target_frequency_hz(static_cast<uint32_t>(std::atol(eq)));
        } else if (std::strcmp(line, "cli_enable") == 0) {
            config::set_cli_enabled(std::strcmp(eq, "on") == 0 ||
                                    std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "gdb_enable") == 0) {
            config::set_gdb_enabled(std::strcmp(eq, "on") == 0 ||
                                    std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "serial_enable") == 0) {
            config::set_serial_cdc_enabled(std::strcmp(eq, "on") == 0 ||
                                           std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "uart_bridge_en") == 0) {
            config::set_uart_bridge_enabled(std::strcmp(eq, "on") == 0 ||
                                            std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "uart_bridge_tx") == 0) {
            config::set_uart_bridge_tx_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "uart_bridge_rx") == 0) {
            config::set_uart_bridge_rx_pin(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "uart0_cdc") == 0) {
            config::set_uart0_cdc_enabled(std::strcmp(eq, "on") == 0 ||
                                          std::strcmp(eq, "1")  == 0);
        } else if (std::strcmp(line, "uart0_tx") == 0) {
            config::set_uart0_tx_gpio(static_cast<int>(std::atol(eq)));
        } else if (std::strcmp(line, "uart0_rx") == 0) {
            config::set_uart0_rx_gpio(static_cast<int>(std::atol(eq)));
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

// FNV-1a-Hash ueber den gesamten Volume-Inhalt. Dient als Aenderungs-Detektor:
// nur wenn sich der Hash seit der letzten Verarbeitung geaendert hat, wird das
// Volume erneut nach BOOT.HEX/CONFIG.INI durchsucht + geflasht. g_disk ist
// alignas(4) und VOLUME_BYTES durch 4 teilbar -> wortweise Iteration sicher.
uint32_t volume_hash() {
    uint32_t h = 2166136261u;                    // FNV-1a offset basis
    const uint32_t* p = reinterpret_cast<const uint32_t*>(g_disk);
    for (uint32_t i = 0; i < VOLUME_BYTES / 4u; ++i) {
        h = (h ^ p[i]) * 16777619u;              // FNV-1a prime
    }
    return h;
}

// Schreibt eine Datei frisch in die (zuvor per format_blank geleerte) RAM-Disk:
// freier Root-Directory-Eintrag + FAT12-Cluster-Kette + Datensektoren. Cluster
// werden sequentiell aus g_next_free_cluster vergeben, sodass MEHRERE Dateien
// (HELP.HTM, CONFIG.INI, FAULT.TXT) nacheinander abgelegt werden koennen. name
// im "NAME.EXT"-Format (wird via to_83 auf 8.3 normalisiert).
void fat_write_file(const char* name, const uint8_t* data, uint32_t len) {
    uint8_t* fat  = g_disk + RESERVED_SECTORS * SECTOR_SIZE;
    uint8_t* root = g_disk + (RESERVED_SECTORS + SECTORS_PER_FAT) * SECTOR_SIZE;

    // Freien Directory-Eintrag suchen (0x00 = Ende, 0xE5 = geloescht).
    uint8_t* de = nullptr;
    for (uint32_t i = 0; i < ROOT_DIR_ENTRIES; ++i) {
        uint8_t* e = root + i * 32;
        if (e[0] == 0x00 || e[0] == 0xE5) { de = e; break; }
    }
    if (!de) return;

    uint32_t clusters = (len + SECTOR_SIZE - 1u) / SECTOR_SIZE;
    if (clusters == 0) clusters = 1;
    const uint16_t first = g_next_free_cluster;
    // FAT12 (1 Sektor) fasst ~341 Cluster-Eintraege; defensiv begrenzen.
    if (first + clusters > 340u) return;

    char name83[11];
    to_83(name, name83);
    std::memset(de, 0, 32);
    std::memcpy(de, name83, 11);
    de[11] = 0x01;                               // Attribut: read-only

    // FAT12-Kette first -> first+1 -> ... -> EOF(0xFFF).
    for (uint32_t k = 0; k < clusters; ++k) {
        uint16_t cur  = static_cast<uint16_t>(first + k);
        uint16_t next = (k + 1u < clusters) ? static_cast<uint16_t>(cur + 1) : 0x0FFFu;
        uint32_t off  = (cur * 3u) / 2u;
        if (cur & 1u) {
            fat[off]     = static_cast<uint8_t>((fat[off] & 0x0F) | ((next << 4) & 0xF0));
            fat[off + 1] = static_cast<uint8_t>((next >> 4) & 0xFF);
        } else {
            fat[off]     = static_cast<uint8_t>(next & 0xFF);
            fat[off + 1] = static_cast<uint8_t>((fat[off + 1] & 0xF0) | ((next >> 8) & 0x0F));
        }
    }

    // Datensektoren fuellen (am Cluster-Offset des ersten Clusters).
    uint8_t* region = g_disk +
        (FIRST_DATA_SECTOR + (first - 2u) * SECTORS_PER_CLUSTER) * SECTOR_SIZE;
    uint32_t cap = clusters * SECTORS_PER_CLUSTER * SECTOR_SIZE;
    uint32_t n   = (len < cap) ? len : cap;
    std::memcpy(region, data, n);

    g_next_free_cluster = static_cast<uint16_t>(first + clusters);

    de[26] = static_cast<uint8_t>(first & 0xFF);
    de[27] = static_cast<uint8_t>((first >> 8) & 0xFF);
    de[28] = static_cast<uint8_t>(n & 0xFF);
    de[29] = static_cast<uint8_t>((n >> 8) & 0xFF);
    de[30] = static_cast<uint8_t>((n >> 16) & 0xFF);
    de[31] = static_cast<uint8_t>((n >> 24) & 0xFF);
}

// ------------------------------------------------------------------------
// Die HTML-Kurzanleitung (HELP.HTM) liegt in einer eigenen Quelldatei
// (help_html.cpp) als reiner HTML-Raw-String und wird ueber help_html.h
// (HELP_HTML / HELP_HTML_LEN) eingebunden. So kann der HTML-Inhalt unabhaengig
// von dieser MSC-Logik direkt editiert werden.
// ------------------------------------------------------------------------

// snprintf-Append-Helfer fuer den CONFIG.INI-Generator.
void cfg_append(char* buf, uint32_t cap, uint32_t& pos, const char* fmt, ...) {
    if (pos >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    int n = std::vsnprintf(buf + pos, cap - pos, fmt, ap);
    va_end(ap);
    if (n > 0) pos += static_cast<uint32_t>(n);
    if (pos > cap) pos = cap;
}

// Erzeugt eine CONFIG.INI aus dem AKTUELL geladenen Zustand: aktive Zeilen fuer
// die gesetzten Werte (inkl. GPIO-Mapping), auskommentierte Vorlagen fuer die
// uebrigen Optionen, jeweils mit kurzer Erklaerung. So dient die Datei zugleich
// als lesbare Referenz und als Ausgangspunkt fuer eigene Anpassungen.
void build_config_ini(char* buf, uint32_t cap, uint32_t& out_len) {
    uint32_t p = 0;
    #define A(...) cfg_append(buf, cap, p, __VA_ARGS__)
    A("# ================================================================\n");
    A("# LPC1115-Emulator auf RP2350 - CONFIG.INI\n");
    A("# Aktive Zeilen = aktuelle Einstellungen. Mit '#' beginnende Zeilen\n");
    A("# sind auskommentierte Optionen/Vorlagen. Wird beim Auswerfen des\n");
    A("# Laufwerks uebernommen. Werte: on/off oder Zahl (dez/0x-hex).\n");
    A("# ================================================================\n\n");

    A("# --- USB-Schnittstellen (Aenderung wirkt erst nach RESET!) ------\n");
    A("# Jede aktive CDC erscheint als eigener COM-Port am Host. off = die CDC\n");
    A("# faellt komplett aus dem USB-Deskriptor (ein COM-Port weniger).\n");
    A("# cli_enable:    Kommandozeile (CLI/Konsole) auf eigener USB-CDC.\n");
    A("cli_enable=%s\n", config::cli_enabled() ? "on" : "off");
    A("# gdb_enable:    GDB-Remote-Serial-Protokoll (arm-none-eabi-gdb).\n");
    A("gdb_enable=%s\n", config::gdb_enabled() ? "on" : "off");
    A("# serial_enable: Serial-Adapter-CDC (uart_bridge bzw. uart0_cdc).\n");
    A("serial_enable=%s\n", config::serial_cdc_enabled() ? "on" : "off");
    A("# Hinweis: Das USB-MSC-Laufwerk (diese CONFIG.INI) bleibt IMMER aktiv\n");
    A("#          (Wiederherstellungs-Pfad, falls alle CDCs deaktiviert sind).\n\n");

    A("# --- Allgemein --------------------------------------------------\n");
    A("# autostart: nach Reset automatisch die geladene Firmware starten.\n");
    A("autostart=%s\n", config::autostart() ? "on" : "off");
    A("# freq_hz: LPC-Soll-Takt (Zeitbasis der emulierten Timer/UART-Baud).\n");
    A("#          Der reale RP2350-Takt bleibt bei 150 MHz.\n");
    A("freq_hz=%lu\n", static_cast<unsigned long>(config::target_frequency_hz()));
    A("# flash_erase: on = Firmware-Slot VOR dem Laden von BOOT.HEX komplett\n");
    A("#              loeschen (sonst additiver Merge). Aktion, kein Zustand.\n");
    A("#flash_erase=off\n\n");

    A("# --- GPIO-Mapping  LPC-Pin -> RP2350-GPIO -----------------------\n");
    A("# Format: pin.<port>_<pin>=<gpio>   (z. B. pin.1_8=1)\n");
    A("# GP25 = Status-LED (reserviert); GP26..29 fuer ADC-Bridge frei.\n");
    {
        const auto& m = config::pin_map();
        for (std::size_t i = 0; i < config::LPC_PIN_COUNT; ++i) {
            if (m.lpc_to_rp[i] >= 0) {
                A("pin.%u_%u=%d\n", static_cast<unsigned>(i / 12),
                  static_cast<unsigned>(i % 12),
                  static_cast<int>(m.lpc_to_rp[i]));
            }
        }
    }
    A("# Beispiel weitere Pins:  #pin.2_0=27   #pin.2_1=28\n\n");

    A("# --- Zweistufiger Boot (Bootloader + Applikation) ---------------\n");
    A("# app_start: Flash-Adresse der Applikation (= applicationFirstAddress\n");
    A("#            des Bootloaders; Selfbus-Standard 0x3000).\n");
    A("app_start=0x%lx\n", static_cast<unsigned long>(config::app_start_addr()));
    A("# desc_addr: Adresse des Boot-Descriptors (0 = automatisch app_start-0x100).\n");
    A("desc_addr=0x%lx\n", static_cast<unsigned long>(config::descriptor_addr()));
    A("# autodesc: gueltigen Boot-Descriptor beim Laden automatisch erzeugen.\n");
    A("autodesc=%s\n\n", config::autodesc() ? "on" : "off");

    A("# --- Serielle Schnittstellen -----------------------------------\n");
    A("# uart_bridge: eigenstaendige USB-Serial-Bruecke auf der Serial-CDC (PIO, JEDER GPIO).\n");
    A("# uart_bridge_en/tx/rx: Serial-CDC <-> PIO-UART-Pins (NICHT der Gast-UART0!).\n");
    if (config::uart_bridge_enabled()) {
        A("uart_bridge_en=on\n");
        A("uart_bridge_tx=%d\n", config::uart_bridge_tx_pin());
        A("uart_bridge_rx=%d\n", config::uart_bridge_rx_pin());
    } else {
        A("#uart_bridge_en=on\n#uart_bridge_tx=4\n#uart_bridge_rx=5\n");
    }
    A("# uart0_cdc: LPC-UART0 des Gasts virtuell direkt an die Serial-CDC (kein Draht/Pin).\n");
    A("# Schliesst uart_bridge auf der Serial-CDC aus. Ideal fuer printf/Serial des Gasts.\n");
    A("uart0_cdc=%s\n", config::uart0_cdc_enabled() ? "on" : "off");
    A("# uart0_tx/uart0_rx: LPC-UART0 auf echte RP-UART-Pads (Hardwareentwurf).\n");
    A("# uart0 TX GP0/12/16 RX GP1/13/17 | uart1 TX GP4/8/20/24 RX GP5/9/21/25. -1=aus.\n");
    if (config::uart0_tx_gpio() >= 0 || config::uart0_rx_gpio() >= 0) {
        A("uart0_tx=%d\n", config::uart0_tx_gpio());
        A("uart0_rx=%d\n", config::uart0_rx_gpio());
    } else {
        A("#uart0_tx=0\n#uart0_rx=1\n");
    }
    A("\n");

    A("# --- I2C-Bridge (LPC-I2C-Master -> echte RP2350-Hardware) -------\n");
    A("# inst: 0=i2c0,1=i2c1. sda/scl: GPIOs (externe Pull-ups noetig). hz: Bustakt.\n");
    if (config::i2c_bridge_enabled()) {
        A("i2c_bridge_en=on\n");
        A("i2c_bridge_inst=%d\n", config::i2c_bridge_instance());
        A("i2c_bridge_sda=%d\n", config::i2c_bridge_sda_pin());
        A("i2c_bridge_scl=%d\n", config::i2c_bridge_scl_pin());
        A("i2c_bridge_hz=%lu\n", static_cast<unsigned long>(config::i2c_bridge_hz()));
    } else {
        A("#i2c_bridge_en=on\n#i2c_bridge_inst=0\n#i2c_bridge_sda=6\n");
        A("#i2c_bridge_scl=7\n#i2c_bridge_hz=100000\n");
    }
    A("\n");

    A("# --- SPI-Bridge (LPC-SSP-Master -> echte RP2350-Hardware) ------\n");
    A("# inst: 0=spi0,1=spi1. lpc: 0=SSP0,1=SSP1. sck/mosi/miso: GPIOs. hz: Bustakt.\n");
    if (config::spi_bridge_enabled()) {
        A("spi_bridge_en=on\n");
        A("spi_bridge_inst=%d\n", config::spi_bridge_instance());
        A("spi_bridge_lpc=%d\n", config::spi_bridge_lpc());
        A("spi_bridge_sck=%d\n", config::spi_bridge_sck_pin());
        A("spi_bridge_mosi=%d\n", config::spi_bridge_mosi_pin());
        A("spi_bridge_miso=%d\n", config::spi_bridge_miso_pin());
        A("spi_bridge_hz=%lu\n", static_cast<unsigned long>(config::spi_bridge_hz()));
    } else {
        A("#spi_bridge_en=on\n#spi_bridge_inst=0\n#spi_bridge_lpc=0\n");
        A("#spi_bridge_sck=18\n#spi_bridge_mosi=19\n#spi_bridge_miso=16\n#spi_bridge_hz=1000000\n");
    }
    A("\n");

    A("# --- ADC-Bridge (LPC-ADC 0..3 -> RP2350 ADC an GP26..29) -------\n");
    A("adc_bridge_en=%s\n\n", config::adc_bridge_enabled() ? "on" : "off");

    A("# --- KNX-Bus / Timer-Capture+Match (CT16/CT32 auf GPIOs) -------\n");
    A("# tcap.<t>=<gpio>   Capture-Eingang (Bus-RX), t: 0=CT16B0 1=CT16B1 2=CT32B0 3=CT32B1\n");
    A("# tmat.<t>.<m>=<gpio> Match-Ausgang (Bus-TX), m=0..3\n");
    for (int t = 0; t < 4; ++t) {
        if (config::ct_capture_pin(t) >= 0)
            A("tcap.%d=%d\n", t, config::ct_capture_pin(t));
        for (int mm = 0; mm < 4; ++mm)
            if (config::ct_match_pin(t, mm) >= 0)
                A("tmat.%d.%d=%d\n", t, mm, config::ct_match_pin(t, mm));
    }
    A("# Beispiel KNX auf CT16B1:  #tcap.1=15   #tmat.1.0=14\n");
    A("# tcap_pio / tmatch_pio: flankengenaue Capture/PWM per PIO (opt-in).\n");
    A("%stcap_pio=%s\n", config::tcap_pio() ? "" : "#", config::tcap_pio() ? "on" : "off");
    A("%stmatch_pio=%s\n\n", config::tmatch_pio() ? "" : "#", config::tmatch_pio() ? "on" : "off");

    A("# --- Experimentell (opt-in) ------------------------------------\n");
    A("# wfi_pin_wakeup: WFI der Firmware auf Pin-/Timer-Wakeup patchen.\n");
    A("%swfi_pin_wakeup=%s\n", config::wfi_pin_wakeup() ? "" : "#",
      config::wfi_pin_wakeup() ? "on" : "off");
    A("# primask_shadow: __disable_irq()/__enable_irq() der Firmware nachbilden.\n");
    A("%sprimask_shadow=%s\n", config::primask_shadow() ? "" : "#",
      config::primask_shadow() ? "on" : "off");
    #undef A
    out_len = p;
}

// Legt HELP.HTM und eine aus dem aktuellen Zustand generierte CONFIG.INI auf das
// Laufwerk. Nach format_blank() aufzurufen (frische FAT), damit der Anwender die
// Referenz stets vorfindet (mbed-DAPLink-Stil: Geraet bringt Doku selbst mit).
void build_info_files() {
    fat_write_file("HELP.HTM",
                   reinterpret_cast<const uint8_t*>(HELP_HTML),
                   HELP_HTML_LEN);
    // Reichlich bemessen: die generierte CONFIG.INI (Kommentare + alle mapped
    // Pins + USB-Schnittstellen) liegt bei ~4.5 KB; 8 KiB laesst Raum fuer volle
    // Pinmaps/Bridges (deckungsgleich mit den 8192-Lesepuffern in parse/mark).
    static char cfg[8192];
    uint32_t len = 0;
    build_config_ini(cfg, sizeof cfg, len);
    fat_write_file("CONFIG.INI", reinterpret_cast<const uint8_t*>(cfg), len);

    // DEBUG.TXT: aktueller Inhalt der Guest->Host Debug-Bridge (Momentaufnahme
    // zum Zeitpunkt des Volume-Aufbaus). Fuer Live-Ausgabe ist die CLI ('dbg')
    // besser geeignet; die Datei ist der Offline-/ohne-Serial-Weg.
    static char dbg[4096];
    uint32_t dbg_len = debug_bridge::snapshot(dbg, sizeof dbg);
    if (dbg_len == 0) {
        static const char empty[] = "(keine Gast-Debug-Ausgabe)\n";
        fat_write_file("DEBUG.TXT",
                       reinterpret_cast<const uint8_t*>(empty),
                       sizeof empty - 1u);
    } else {
        fat_write_file("DEBUG.TXT",
                       reinterpret_cast<const uint8_t*>(dbg), dbg_len);
    }
}

// Baut ein frisches Volume, das nur FAULT.TXT enthaelt (mbed-DAPLink-Stil).
// Die RAM-Disk ist nur ein Transfer-Puffer (echte Firmware liegt im Flash-Slot),
// daher ist das Ueberschreiben unkritisch. Dedup-/Dirty-State wird so gesetzt,
// dass poll() das injizierte Volume NICHT als neuen Upload interpretiert.
void build_fault_volume(const char* text, uint32_t len) {
    format_blank();                              // frisches FAT12 (nur Volume-Label)
    if (len > 0) {
        fat_write_file("FAULT.TXT",
                       reinterpret_cast<const uint8_t*>(text), len);
    }
    build_info_files();                          // HELP.HTM + CONFIG.INI beibehalten
    g_dirty.store(false);
    g_volume_processed.store(true);
    g_last_volume_hash = volume_hash();
    g_have_volume_hash = true;
}

// Merkt den Inhalts-Hash der aktuell in der RAM-Disk liegenden CONFIG.INI als
// "verarbeitet". Wird nach init() und refresh_config_volume() aufgerufen, damit
// on_volume_ready() die (device-seitig generierte, unveraenderte) CONFIG.INI
// nicht erneut parst/persistiert — ein Re-Parse wuerde config::save() (Flash-
// Schreibzugriff) ausloesen und dazu den laufenden Gast pausieren.
void mark_config_processed() {
    uint16_t cl; uint32_t sz;
    static char tmp[8192];
    if (find_dir_entry("CONFIG.INI", cl, sz) && sz > 0 && sz < sizeof tmp) {
        uint32_t n = read_cluster_chain(cl, sz, reinterpret_cast<uint8_t*>(tmp),
                                        sizeof tmp);
        g_last_config_hash = content_hash(reinterpret_cast<uint8_t*>(tmp), n);
        g_have_config_hash = true;
    }
}

// Trigger nach Eject vom Hauptloop aufgerufen.
void on_volume_ready() {
    uint16_t cl; uint32_t sz;

    // Re-Processing nur bei ECHTER Aenderung: Inhalts-Hash des Volumes bilden.
    // Manche Hosts schreiben nach dem Eject beim Re-Mount erneut FAT-/Meta-
    // daten -> g_dirty + Dirty-Timeout -> on_volume_ready() erneut, OBWOHL sich
    // der Inhalt nicht geaendert hat. Ist der Hash identisch zum zuletzt
    // verarbeiteten, hier fruehzeitig raus — VOR jedem Gast-Stop.
    const uint32_t vh = volume_hash();
    if (g_have_volume_hash && vh == g_last_volume_hash) {
        return;
    }

    // --- Nutzdateien LESEN (nur aus g_disk; KEIN Flash, KEIN Gast-Stop) ------
    // Erst danach wird anhand der Inhalts-Hashes entschieden, ob ueberhaupt ein
    // Flash-Schreibzugriff (und damit ein Gast-Stop) noetig ist. So stoppt ein
    // reiner Host-Metadaten-Write (gleiche Dateien) den laufenden Gast NICHT.
    static char cfg_buf[8192];
    uint32_t cfg_len  = 0;
    bool     have_cfg = false;
    if (find_dir_entry("CONFIG.INI", cl, sz) && sz > 0 && sz < sizeof cfg_buf) {
        cfg_len  = read_cluster_chain(cl, sz, reinterpret_cast<uint8_t*>(cfg_buf),
                                      sizeof cfg_buf);
        have_cfg = true;
    }

    static uint8_t hex_buf[64 * 1024 + 1024];
    uint32_t hex_len  = 0;
    bool     have_hex = false;
    char     hex_name[64] = {0};
    {
        uint16_t hcl; uint32_t hsz;
        if (find_hex_entry(hcl, hsz, hex_name, sizeof hex_name) && hsz > 0) {
            hex_len  = read_cluster_chain(hcl, hsz, hex_buf, sizeof hex_buf);
            have_hex = true;
        }
    }

    // --- Aenderungen anhand der Datei-Inhalts-Hashes bestimmen ---------------
    const uint32_t cfg_hash = have_cfg ? content_hash(reinterpret_cast<uint8_t*>(cfg_buf), cfg_len) : 0u;
    const uint32_t hex_hash = have_hex ? content_hash(hex_buf, hex_len) : 0u;
    const bool cfg_changed = have_cfg && !(g_have_config_hash && cfg_hash == g_last_config_hash);
    const bool hex_changed = have_hex && !(g_have_hex_hash    && hex_hash == g_last_hex_hash);

    if (!cfg_changed && !hex_changed) {
        // Nur Host-Metadaten/FAT geaendert, keine Nutzdatei -> Gast NICHT
        // anfassen (kein Stop/Neustart). Volume-Hash aktualisieren und raus.
        g_last_volume_hash = volume_hash();
        g_have_volume_hash = true;
        return;
    }

    g_erase_request = false;

    // Ab hier ist ein Flash-Schreibzugriff noetig. Laeuft ein Gast nativ auf
    // Core1, vor den Flash-Schreibzugriffen pausieren (sonst crasht Core1
    // waehrend des XIP-Stalls). Der Guard setzt den Gast am Funktionsende fort;
    // ein Neustart auf NEUE Firmware erfolgt nur, wenn die HEX sich geaendert
    // hat (g_boot_pending unten). Lief kein Gast, ist es ein No-op.
    emulator::FlashPauseGuard flash_pause;

    // CONFIG.INI nur bei geaendertem Inhalt neu parsen/persistieren (sonst
    // wuerde ein Re-Parse einer stale CONFIG.INI eine per CLI gesetzte Pinmap
    // ueberschreiben). parse_config kann g_erase_request setzen (flash_erase=on).
    if (cfg_changed) {
        g_stats.parsed_lines = 0;
        parse_config(cfg_buf, cfg_len);
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
        g_last_config_hash = cfg_hash;
        g_have_config_hash = true;
        std::printf("[MSC] CONFIG.INI: %lu Bytes, %u Zeilen\n",
                    static_cast<unsigned long>(cfg_len),
                    static_cast<unsigned>(g_stats.parsed_lines));
    }

    // Optionales vollstaendiges Loeschen des Firmware-Slots (flash_erase=on).
    if (g_erase_request) {
        storage::firmware_erase();
        g_have_hex_hash = false;   // erzwingt Neu-Flash der HEX
        std::printf("[MSC] flash_erase=on -> Firmware-Slot geloescht\n");
    }

    // BOOT.HEX -> in firmware-Slot persistieren, aber NUR wenn sich der HEX-
    // Inhalt geaendert hat. So loest ein wiederholtes on_volume_ready() mit
    // identischer HEX (Host-Metadaten, Re-Mount) KEINEN Gast-Neustart aus —
    // das war die Ursache des unregelmaessigen Blinkens. Akzeptiert auch jede
    // andere *.HEX-Datei (Originalname zulaessig).
    const bool hex_needs_flash =
        have_hex && (hex_changed || (g_erase_request && !g_have_hex_hash));
    bool hex_flashed = false;
    if (hex_needs_flash) {
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
        for (uint32_t i = 0; i < hex_len; ++i) {
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
            g_boot_pending.store(true);   // nur bei geaenderter HEX -> Neustart
            g_last_hex_hash = hex_hash;
            g_have_hex_hash = true;
            hex_flashed = true;           // -> Laufwerk ohne HEX neu aufbauen
            std::printf("[MSC] %s %lu B → flash\n", hex_name,
                        static_cast<unsigned long>(ctx.total));
        }
    }

    // Wie beim RP2350-Bootloader (UF2): nach dem Flashen die erkannte HEX-Datei
    // vom Laufwerk entfernen und das Medium neu einhaengen. CONFIG.INI wurde
    // oben bereits verarbeitet; build_info_files() schreibt nur HELP.HTM +
    // CONFIG.INI + DEBUG.TXT (KEINE HEX) -> die HEX verschwindet, der Host sieht
    // ein frisches Volume. Der eigentliche Gast-(Neu)start passiert danach ueber
    // g_boot_pending im Hauptloop.
    if (hex_flashed) {
        format_blank();
        build_info_files();
        g_dirty.store(false);
        g_volume_processed.store(true);
        mark_config_processed();
        g_last_volume_hash = volume_hash();
        g_have_volume_hash = true;
        trigger_media_change();
        return;
    }

    // Verarbeiteten Volume-Zustand merken -> inhaltlich identische Re-Trigger
    // (Host-Re-Mount ohne echte Aenderung) werden ab jetzt uebersprungen. Der
    // Hash wird HIER frisch berechnet (nicht der am Funktionsanfang gebildete
    // vh): on_volume_ready selbst schreibt zwar nicht in g_disk, aber ein Host
    // kann waehrend der langen Flash-Operationen weitere identische Sektoren
    // geschrieben haben -> der aktuelle g_disk-Stand ist die verlaessliche
    // Referenz fuer den naechsten Dedup-Vergleich.
    g_last_volume_hash = volume_hash();
    g_have_volume_hash = true;
}

} // namespace

void refresh_config_volume() {
    // Definiert im oeffentlichen Namespace, ruft aber die internen Volume-
    // Helfer (format_blank/build_info_files/volume_hash, g_dirty ...) — diese
    // liegen im anonymen Namespace derselben Uebersetzungseinheit und sind hier
    // sichtbar. Baut die RAM-Disk aus dem aktuellen Live-Config-Zustand neu auf
    // und aktualisiert den Dedup-Hash, damit ein spaeteres on_volume_ready()
    // (durch Host-Metadaten ausgeloest) die gerade per CLI gesetzte Zuordnung
    // NICHT aus einer veralteten CONFIG.INI zuruecksetzt.
    format_blank();
    build_info_files();
    g_dirty.store(false);
    g_volume_processed.store(true);
    // Die frisch generierte CONFIG.INI (spiegelt den Live-Zustand) als bereits
    // verarbeitet registrieren -> ein spaeteres on_volume_ready() (durch Host-
    // Metadaten) parst sie nicht erneut und pausiert den Gast nicht.
    mark_config_processed();
    g_last_volume_hash = volume_hash();
    g_have_volume_hash = true;
    // KRITISCH: Medienwechsel ausloesen. Ein gemounteter Host hat die ALTE
    // CONFIG.INI gecached und wuerde sie sonst (a) weiter anzeigen und (b) beim
    // naechsten Schreibzugriff in die RAM-Disk zurueckschreiben -> das
    // ueberschriebe die gerade per CLI gesetzte Zuordnung und startete den Gast
    // neu (unregelmaessiges Blinken). Der Medienwechsel zwingt den Host, die
    // frische CONFIG.INI neu zu lesen und seinen Cache zu verwerfen -> Anzeige
    // aktuell UND kein Revert mehr.
    trigger_media_change();
}

void flush_debug_volume() {
    // DEBUG.TXT (und CONFIG.INI) frisch aufbauen + Host neu einlesen lassen.
    // Deckungsgleich mit refresh_config_volume() (das jetzt selbst einen
    // Medienwechsel ausloest); zusaetzlich die Auto-Flush-Tracker nachziehen,
    // damit ein direkt folgender Auto-Flush nicht sofort erneut ausloest.
    refresh_config_volume();
    g_last_dbg_total    = debug_bridge::total_bytes();
    g_last_dbg_flush_ms = to_ms_since_boot(get_absolute_time());
}

void set_debug_autoflush_ms(uint32_t ms) { g_dbg_autoflush_ms = ms; }
uint32_t debug_autoflush_ms()            { return g_dbg_autoflush_ms; }

void init() {
    std::memset(&g_stats, 0, sizeof g_stats);
    // Persistente Wiederherstellung waere moeglich (storage::msc_load_volume).
    // Fuers Erste: bei jedem Power-Cycle ein leeres FAT12 anlegen und HELP.HTM
    // + eine aus dem aktuellen Zustand generierte CONFIG.INI ablegen, dann auf
    // die naechsten Host-Schreibvorgaenge warten.
    format_blank();
    build_info_files();
    // Ausgangszustand als "verarbeitet" registrieren: Ein Auswerfen OHNE Host-
    // Aenderung (gleicher Inhalt) wird dann per volume_hash dedupliziert und
    // loest KEIN erneutes Parsen der (ggf. stale) CONFIG.INI aus -> per CLI
    // vorgenommene Aenderungen bleiben erhalten. Erst echte Host-Schreibzugriffe
    // (neue BOOT.HEX / editierte CONFIG.INI) aendern den Hash und werden verarbeitet.
    mark_config_processed();
    g_last_volume_hash = volume_hash();
    g_have_volume_hash = true;
}

bool consume_pending_boot_request() {
    return g_boot_pending.exchange(false);
}

void poll() {
    // Fatal-Fault vom Gast (Core1): kompletten [FAULT]-Report als FAULT.TXT auf
    // das Laufwerk legen und den Host zum Neueinlesen zwingen, damit ein Fehler
    // auch ohne CLI/Serial analysierbar ist. Hat Vorrang; einmal pro Fault.
    if (faultsys::report_ready()) {
        build_fault_volume(faultsys::report_data(), faultsys::report_length());
        faultsys::clear_report();
        trigger_media_change();
        return;
    }

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
        return;
    }

    // Auto-Flush der DEBUG.TXT: sobald der Gast neue Debug-Bytes geschickt hat
    // und das Intervall verstrichen ist, die RAM-Disk mit frischer DEBUG.TXT neu
    // aufbauen und den Host neu einlesen lassen. So fuellt sich DEBUG.TXT von
    // selbst, ohne manuelles 'dbg save'. Gehemmt, solange der Host schreibt
    // (g_dirty) — dann hat die Host-Verarbeitung Vorrang und ein Medienwechsel
    // wuerde eine laufende Host-Bearbeitung stoeren.
    if (g_dbg_autoflush_ms != 0 &&
        !g_dirty.load(std::memory_order_relaxed)) {
        uint32_t now   = to_ms_since_boot(get_absolute_time());
        uint32_t total = debug_bridge::total_bytes();
        if (total != g_last_dbg_total &&
            (g_last_dbg_flush_ms == 0 ||
             (now - g_last_dbg_flush_ms) >= g_dbg_autoflush_ms)) {
            flush_debug_volume();   // aktualisiert g_last_dbg_total/-flush_ms
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

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    // Media-Change-Sequenz (macht device-seitig injizierte FAULT.TXT sichtbar):
    // waehrend des Absent-Fensters "medium not present", danach einmalig
    // UNIT ATTENTION -> Host liest das Volume frisch ein.
    if (usb_msc::g_media_absent) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (static_cast<int32_t>(now - usb_msc::g_media_absent_until) < 0) {
            tud_msc_set_sense(lun, 0x02, 0x3A, 0x00);  // NOT READY, medium not present
            return false;
        }
        usb_msc::g_media_absent    = false;
        usb_msc::g_media_attention = true;             // Wiederkehr signalisieren
    }
    if (usb_msc::g_media_attention) {
        usb_msc::g_media_attention = false;
        tud_msc_set_sense(lun, 0x06, 0x28, 0x00);      // UNIT ATTENTION, medium may have changed
        return false;
    }
    return true;
}

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
