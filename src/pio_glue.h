#pragma once
//
// PIO-Skelett für Funktionen, die der LPC1115 hat, der RP2350 aber nicht
// 1:1 in Hardware bietet (z. B. Match/Capture-Timer mit Conditional-Reset).
//
// Aktuell: Stub — Programme/State-Machines werden bei Bedarf aus dem
// peripherals::mmio_*-Handler nachgeladen, sobald die LPC-Firmware sie
// programmiert.
//

#include <cstdint>

namespace pio_glue {

void init();

// Capture-Timer: konfiguriert eine PIO-State-Machine, die einen Pin-
// Übergang zählt und den Wert in eine FIFO schreibt. Pin = RP2350-GPIO.
// Liefert die Programm-Offset-Adresse, sonst 0xFFFF wenn voll.
uint16_t setup_capture(uint8_t rp_gpio, bool rising_edge);

// Liest den letzten erfassten Wert (FIFO-Drain). Liefert false, wenn leer.
bool capture_read(uint16_t handle, uint32_t& out);

} // namespace pio_glue