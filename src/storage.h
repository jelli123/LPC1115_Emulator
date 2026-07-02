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

// Persistenz-Diagnose. Zeigt, ob beim letzten config_load() ein gueltiger
// Config-Slot im Flash gefunden wurde, plus Flash-Groesse/Region-Offset.
// Nutzen: einen Flash-Groessen-Mismatch (Board-Header nimmt z. B. 4 MB an,
// der reale Baustein hat 2 MB) erkennen. Dann liegt die Config-Region jenseits
// des realen Flash -> Schreibzugriffe verpuffen; config_get liefert in der
// Sitzung noch aus dem RAM-Snapshot, aber NICHTS ueberlebt einen Power-Cycle.
struct ConfigPersistInfo {
    uint32_t flash_size_kib;   // angenommene Flash-Groesse (PICO_FLASH_SIZE_BYTES)
    uint32_t region_offset;    // Flash-Offset der Config-Region
    uint32_t slot_sectors;     // Anzahl Wear-Leveling-Sektoren
    bool     loaded_valid;     // beim letzten config_load() gueltiger Slot gefunden?
    uint32_t sequence;         // aktuelle Sequenznummer
    uint32_t key_count;        // aktuelle Anzahl Key/Value-Eintraege im RAM
};
ConfigPersistInfo config_persist_info();

// Firmware-Slot
bool firmware_erase();
bool firmware_write(std::size_t offset, const void* data, std::size_t len);
const uint8_t* firmware_data();              // Pointer in den memory-mapped XIP-Bereich
std::size_t   firmware_size();               // letzte beschriebene Länge (Marker)
bool firmware_finalize(std::size_t total_len); // schreibt Längen-Marker

} // namespace storage
