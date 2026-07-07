#pragma once
//
// USB-Mass-Storage-Class auf einem RAM/Flash-Backed FAT12-Volume.
//
// Layout:
//   * LUN 0, 512-Byte-Sektoren, 512 Sektoren = 256 KiB.
//   * FAT12 mit 1 FAT-Kopie, 1 reserviertem Sektor, 16 Root-Directory-
//     Einträgen.
//   * Daten persistieren in der Storage-Schicht (eigener Slot am Ende
//     des QSPI-Flash, separat vom Firmware- und Konfig-Slot).
//
// Userworkflow:
//   1. RP2354 anstecken → erscheint als Wechseldatenträger "LPC1115EMU".
//   2. Datei `BOOT.HEX`  drauf kopieren  → Firmware
//   3. Datei `CONFIG.INI` drauf kopieren  → Pinmap + Optionen
//   4. Wechseldatenträger auswerfen → Emulator parst beide Dateien und
//      startet die Firmware autonom (kein CLI nötig).
//
// API ist absichtlich klein. tinyusb-Callbacks (tud_msc_*) liegen in
// usb_msc.cpp.

#include <cstddef>
#include <cstdint>

namespace usb_msc {

constexpr uint32_t SECTOR_SIZE   = 512;
constexpr uint32_t SECTOR_COUNT  = 512;          // 256 KiB Volume (>= 64-KB-Hex)
constexpr uint32_t VOLUME_BYTES  = SECTOR_SIZE * SECTOR_COUNT;

// Beim Boot aus Storage-Slot laden (oder Default-FS erzeugen, falls leer).
void init();

// Soll der Emulator nach Eject-Erkennung BOOT.HEX laden und CONFIG.INI
// anwenden? Wird vom Hauptloop gepollt.
bool consume_pending_boot_request();

// Fordert ein DEFERRED Persistieren des Live-Config-Zustands an (Flash-save).
// Fuer CLI-Aenderungen (z. B. 'pinmap set'): die Aenderung ist sofort LIVE
// wirksam; das schwere, einen laufenden Gast neu startende config::save() laeuft
// erst, wenn der Gast ohnehin steht (poll) -> kein Neustall/Byte-Verlust. 'cfg
// save' erzwingt es sofort (mit einmaligem Neustart).
void request_config_persist();

// Meldet, dass der Live-Config-Zustand bereits (per explizitem 'cfg save')
// persistiert wurde -> loescht eine noch vorgemerkte deferred-Persistenz.
void note_config_persisted();

// Vom Hauptloop aufgerufen, wenn das Volume gerade nicht beschrieben
// wird; persistiert Dirty-Sektoren in Storage.
void poll();

// Baut die RAM-Disk (HELP.HTM + CONFIG.INI) aus dem aktuellen Live-Config-
// Zustand neu auf. Nach einer CLI-Konfig-Aenderung (z. B. 'pinmap set')
// aufzurufen, damit die CONFIG.INI den Live-Zustand widerspiegelt und ein
// spaeteres on_volume_ready() (durch Host-Metadaten ausgeloest) die gerade
// gesetzte Zuordnung NICHT aus einer veralteten CONFIG.INI zuruecksetzt.
//   trigger_host_reread=true  : zusaetzlich Medienwechsel ausloesen (gemounteter
//     Host verwirft Cache + liest frisch). Noetig fuer den Datei-Workflow.
//   trigger_host_reread=false : KEIN Medienwechsel. Fuer CLI-Aenderungen, wo der
//     700-ms-Medienwechsel den interaktiven CDC-Eingabestrom stoert (dropte das
//     zweite 'pinmap set ... -1' auf '-'). Anti-Revert-Schutz (Hash/processed)
//     bleibt erhalten; der Host sieht die frische CONFIG.INI beim naechsten
//     regulaeren Mount/Refresh.
void refresh_config_volume(bool trigger_host_reread = true);

// Baut die RAM-Disk neu auf (inkl. frischer DEBUG.TXT aus der Debug-Bridge) und
// zwingt den Host per Medienwechsel zum Neu-Einlesen. Fuer 'dbg save' — macht
// die bis dahin gesammelte Gast-Debug-Ausgabe als DEBUG.TXT sichtbar (der Host
// cached Dateiinhalte sonst; ohne Medienwechsel bliebe die Boot-Version stehen).
void flush_debug_volume();

// Auto-Flush-Intervall der DEBUG.TXT (ms; 0 = aus). Nach so vielen ms mit neuen
// Gast-Debug-Bytes baut poll() die RAM-Disk mit frischer DEBUG.TXT neu auf und
// laesst den Host neu einlesen. Per CLI 'dbg auto <sek|off>' steuerbar.
void     set_debug_autoflush_ms(uint32_t ms);
uint32_t debug_autoflush_ms();

struct Stats {
    uint32_t reads;
    uint32_t writes;
    uint32_t boot_requests;
    uint32_t parsed_lines;
};
Stats stats();

// Datei-Lookups (Root-Directory).
//
// Liefert Pointer auf den ersten Daten-Sektor und Länge in Bytes; Datei
// wird hier als zusammenhängend angenommen (FAT12-Cluster-Chain wird
// auf-gelegt im Speicher gehalten).
struct File {
    const uint8_t* data;
    uint32_t       size;
};

bool find_file(const char* name83, File& out);   // "BOOT    HEX"
bool read_text_config(const char* name83);

// Langname der zuletzt via MSC geflashten HEX-Datei ("" = keine seit Boot, z. B.
// Autostart aus dem persistierten Flash-Slot). Fuer 'stats'/'info'.
const char* loaded_hex_name();

} // namespace usb_msc
