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

// ---------------------------------------------------------------------------
// Flankengenaues Timestamping (KNX-Empfang): Eine PIO-State-Machine fuehrt
// einen frei laufenden Abwaertszaehler und schiebt bei JEDER Pin-Flanke den
// Zaehlerstand in die FIFO. Die CPU rechnet die Differenzen in TC-Ticks um.
// Vorteil ggue. Software-Polling: 0 % Core-Last, flankengenaue Aufloesung.
// ---------------------------------------------------------------------------

// Richtet eine Timestamp-State-Machine fuer `rp_gpio` ein. `out_rate_hz`
// liefert die tatsaechliche Zaehlrate (Counts/Sekunde), die die CPU zur
// Umrechnung braucht. Rueckgabe: Handle >= 0, oder -1 bei Fehler/voll.
int  ts_setup(uint8_t rp_gpio, float& out_rate_hz);

// Zieht genau einen Flanken-Timestamp aus der FIFO. false = FIFO leer.
bool ts_read(int handle, uint32_t& counter);

// Gibt die State-Machine wieder frei (vor Neukonfiguration).
void ts_teardown(int handle);

} // namespace pio_glue