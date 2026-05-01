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

// Adresse, an die LPC-Firmware in RP2350-SRAM kopiert wird. Muss 32-Byte-
// aligned sein (MPU-Anforderung) und Platz für 64 KB Code lassen.
constexpr uint32_t LPC_LOAD_BASE      = 0x2004'0000;
constexpr uint32_t LPC_LOAD_MAX_SIZE  = 64 * 1024;
// Region, in die LPC-RAM (originaler Bereich 0x10000000-0x10001FFF) gemappt
// wird. Best-effort: Firmware muss neu gelinkt werden (siehe README).
constexpr uint32_t LPC_GUEST_RAM_BASE = 0x2006'0000;
constexpr uint32_t LPC_GUEST_RAM_SIZE = 8 * 1024;

void boot_core1();
void load_and_start();
void stop();
// Asynchrones „Soft-Reset" nur des Guests (z. B. WDT-Ablauf).
// Setzt eine Flag, die der Core-1-Run-Loop pollt; dort werden Vector-
// Init, RAM-Clear und Sprung in den Reset-Vektor wiederholt.
// Der RP2350 läuft weiter, USB/CLI bleiben verbunden.
void request_guest_reset();
State    state();
uint64_t mem_traps();
uint32_t pc();

} // namespace emulator
