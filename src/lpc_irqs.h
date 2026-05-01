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
    PIN_INT0       = 0,
    PIN_INT1       = 1,
    PIN_INT2       = 2,
    PIN_INT3       = 3,
    PIN_INT4       = 4,
    PIN_INT5       = 5,
    PIN_INT6       = 6,
    PIN_INT7       = 7,
    GINT0          = 8,
    GINT1          = 9,
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
