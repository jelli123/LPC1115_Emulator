#pragma once
//
// LPC1115 IRQ-Nummern (UM10398 Tabelle 51). Diese Nummern entsprechen
// den Bit-Positionen in NVIC ISER0/ICER0/ISPR0/ICPR0, also genau dem
// Index, den die Gast-Firmware benutzt.
//
// Die Gast-Vector-Tabelle hat den Eintrag für IRQ N bei dst[16+N].
//

#include <cstdint>

namespace lpc_irq {

enum : uint8_t {
    // 0..12: Start-Logik PIO0_0..PIO0_11, PIO1_0 (Index = Eingang)
    START_PIO0_0   = 0,
    START_PIO1_0   = 12,
    C_CAN          = 13,
    SSP1           = 14,
    I2C0           = 15,
    CT16B0         = 16,
    CT16B1         = 17,
    CT32B0         = 18,
    CT32B1         = 19,
    SSP0           = 20,
    UART0          = 21,
    ADC            = 24,
    WWDT           = 25,
    BOD            = 26,
    EINT3          = 28,
    EINT2          = 29,
    EINT1          = 30,
    EINT0          = 31,
    COUNT          = 32
};

// Der Vector-Tabellen-Eintrag für IRQ n.
inline uint32_t vector_offset(uint8_t n) {
    return (16u + static_cast<uint32_t>(n)) * 4u;
}

} // namespace lpc_irq
