#pragma once
//
// Persistente Ablage im internen QSPI-Flash des RP2350 mit einfachem
// Round-Robin Wear-Leveling. Wird genutzt für:
//   * Konfiguration (1 Slot)
//   * LPC-Firmware-Image (mehrere Sektoren am Stück, eigener Slot)
//
// Hinweis: Dies ist kein vollständiges Filesystem (LittleFS-Port wäre
// die nächste Ausbaustufe). Die API ist bewusst minimal, damit sie
// austauschbar bleibt.
//

#include <cstddef>
#include <cstdint>

namespace storage {

// Reservierte Bereiche am Ende des on-board Flash. Werte sind 4-KB-aligned.
// Werden in storage.cpp gegen die im Linker-Skript definierte Flash-Größe
// validiert (statisch).
constexpr std::size_t SECTOR_SIZE          = 4096;
constexpr std::size_t CONFIG_SLOT_SECTORS  = 8;     //  32 KiB, 8-fach Wear-Leveling
constexpr std::size_t FIRMWARE_SLOT_BYTES  = 64 * 1024; // identisch LPC1115-Flash

// Schlüsselbasierte Konfig (key/value, key max 31 Bytes, value max 95 Bytes).
// Speichert immer das gesamte aktuelle KV-Set in den nächsten Sektor;
// Lesen sucht den Sektor mit der höchsten Sequenz.
bool init();

bool config_load();                          // lädt Snapshot in RAM
bool config_get(const char* key, char* out, std::size_t out_size);
bool config_set(const char* key, const char* value);
bool config_commit();                        // schreibt RAM-Snapshot in nächsten Sektor
void config_dump(void (*emit)(const char* line));

// Firmware-Slot
bool firmware_erase();
bool firmware_write(std::size_t offset, const void* data, std::size_t len);
const uint8_t* firmware_data();              // Pointer in den memory-mapped XIP-Bereich
std::size_t   firmware_size();               // letzte beschriebene Länge (Marker)
bool firmware_finalize(std::size_t total_len); // schreibt Längen-Marker

} // namespace storage
