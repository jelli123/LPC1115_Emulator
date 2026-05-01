#pragma once
//
// USB-Mass-Storage-Class auf einem RAM/Flash-Backed FAT12-Volume.
//
// Layout:
//   * LUN 0, 512-Byte-Sektoren, 256 Sektoren = 128 KiB.
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
constexpr uint32_t SECTOR_COUNT  = 256;          // 128 KiB Volume
constexpr uint32_t VOLUME_BYTES  = SECTOR_SIZE * SECTOR_COUNT;

// Beim Boot aus Storage-Slot laden (oder Default-FS erzeugen, falls leer).
void init();

// Soll der Emulator nach Eject-Erkennung BOOT.HEX laden und CONFIG.INI
// anwenden? Wird vom Hauptloop gepollt.
bool consume_pending_boot_request();

// Vom Hauptloop aufgerufen, wenn das Volume gerade nicht beschrieben
// wird; persistiert Dirty-Sektoren in Storage.
void poll();

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

} // namespace usb_msc
