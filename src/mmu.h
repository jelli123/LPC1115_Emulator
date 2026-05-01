#pragma once
//
// ARMv8-M MPU-Konfiguration auf dem RP2350 Cortex-M33.
//
// Wir konfigurieren die MPU so, dass jeder Zugriff der nativ ausgeführten
// LPC-Firmware auf
//   * LPC-Peripherie    (0x40000000 - 0x4FFFFFFF)  → BusFault
//   * NVIC/SCB-Bereich  (Teile von 0xE000E000 + )  → BusFault
// einen synchronen Fault auslöst, der vom BusFault-Handler getrappt und
// auf die RP2350-Hardware (peripherals::mmio_*) umgesetzt wird.
//
// XIP-Region des RP2350 (0x10000000-0x1FFFFFFF) wird gegen Schreibzugriffe
// geschützt. Das fängt versehentliche Schreibvorgänge an absoluten LPC-RAM-
// Adressen ab, falls die geladene Firmware *nicht* mit angepasstem
// Linkerscript gebaut wurde.
//

#include <cstdint>
#include <cstddef>

namespace mpu_setup {

// Adressbereiche der LPC-Sicht.
constexpr uint32_t LPC_PERIPH_BASE   = 0x4000'0000;
constexpr uint32_t LPC_PERIPH_END    = 0x5000'0000;
constexpr uint32_t LPC_PRIVPERI_BASE = 0xE000'0000;
constexpr uint32_t LPC_PRIVPERI_END  = 0xE010'0000;

// XIP des RP2350.
constexpr uint32_t RP_XIP_BASE = 0x1000'0000;
constexpr uint32_t RP_XIP_END  = 0x2000'0000;

// Aktiviert die MPU-Regionen für den Gast (LPC-Firmware).
// Muss aufgerufen werden, bevor BX in die Gast-Firmware erfolgt.
void enable_for_guest();

// Deaktiviert die MPU vollständig (für Host-Code auf Core 0).
void disable();

} // namespace mpu_setup
