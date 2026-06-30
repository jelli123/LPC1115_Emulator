#pragma once
//
// API zum Native-Execution-Emulator (läuft auf Core 1).
// Lädt LPC-Firmware in RP2350-SRAM, patcht Vektor-Table, aktiviert MPU,
// wechselt in unprivilegierten PSP-Thread-Mode und startet die Firmware
// mit BX in den Reset-Handler.
//

#include <cstdint>
#include <cstddef>

namespace emulator {

enum class State : uint8_t { Idle, Running, Halted, Faulted };

// Größen der Guest-Speicherbereiche.
constexpr uint32_t LPC_LOAD_MAX_SIZE  = 64 * 1024;
// LPC1115 hat 8 KB SRAM (0x10000000-0x10001FFF).
constexpr uint32_t LPC_GUEST_RAM_SIZE = 8 * 1024;

// Laufzeit-Adressen der Guest-Speicherbereiche im RP2350-SRAM.
// Werden in emulator.cpp als aligned Arrays allokiert.
// VTOR-Anforderung: LPC_LOAD_BASE muss 256-Byte-aligned sein.
uint32_t load_base();       // Adresse des Code-Image-Puffers
uint32_t guest_ram_base();  // Adresse des Guest-RAM-Puffers

// Basis-Adresse der aktuell aktiven Guest-Vektortabelle (absolute RP2350-
// Adresse). Nach dem Laden == load_base() (Vektortabelle am Flash-Anfang).
// Nach einem Bootloader->Applikation-Handover zeigt sie auf die Vektortabelle
// der gestarteten Applikation (load_base() + Flash-Offset). Die IRQ-Injektion
// liest hierueber den passenden LPC-Handler.
uint32_t vtable_base();

// Wird vom SYSCON-Modell aufgerufen, sobald der Gast SYSMEMREMAP in einen
// User-Mode schreibt (LPC-typischer Bootloader->Applikation-Uebergang).
// Lokalisiert die soeben nach Guest-RAM kopierte Applikations-Vektortabelle
// im Flash-Image, reloziert deren Eintraege + System-Handler, setzt VTOR und
// die IRQ-Vektorbasis auf die Applikation um. No-op, wenn kein passendes
// zweites Image gefunden wird (dann bleibt die bisherige Tabelle aktiv).
void activate_bootloader_handover();

// Liefert eine vom Handover vorgemerkte SP-Korrektur (rohe LPC-RAM-StackTop-
// Adresse raw -> relocierte Gast-RAM-Adresse reloc) und konsumiert sie.
// Liefert true, wenn eine Korrektur anstand. Der Fault-Handler patcht damit
// das Register, aus dem der Bootloader unmittelbar danach den SP laedt.
bool take_handover_sp_fixup(uint32_t& raw, uint32_t& reloc);

void boot_core1();
void load_and_start();
void stop();
// Asynchrones „Soft-Reset" nur des Guests (z. B. WDT-Ablauf).
void request_guest_reset();
State    state();
uint64_t mem_traps();
uint32_t pc();

} // namespace emulator
