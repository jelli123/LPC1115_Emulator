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
// Flankengenaues Timestamping (Input-Capture): Eine PIO-State-Machine fuehrt
// einen frei laufenden Abwaertszaehler und schiebt bei JEDER Pin-Flanke den
// Zaehlerstand in die FIFO. Die CPU rechnet die Differenzen in TC-Ticks um.
// Vorteil ggue. Software-Polling: 0 % Core-Last, flankengenaue Aufloesung.
// Generisch fuer jede Capture-Anwendung (Frequenz-/Pulsbreitenmessung,
// Decoder, KNX-Empfang, ...).
// ---------------------------------------------------------------------------

// Richtet eine Timestamp-State-Machine fuer `rp_gpio` ein. `out_rate_hz`
// liefert die tatsaechliche Zaehlrate (Counts/Sekunde), die die CPU zur
// Umrechnung braucht. Rueckgabe: Handle >= 0, oder -1 bei Fehler/voll.
int  ts_setup(uint8_t rp_gpio, float& out_rate_hz);

// Zieht genau einen Flanken-Timestamp aus der FIFO. false = FIFO leer.
bool ts_read(int handle, uint32_t& counter);

// Gibt die State-Machine wieder frei (vor Neukonfiguration).
void ts_teardown(int handle);

// ---------------------------------------------------------------------------
// Flankengenaue Match-Puls-Erzeugung (PWM/Match-Ausgang): Eine PIO-State-
// Machine treibt den Ausgangspin. Pro Puls liefert die CPU { delay_counts,
// width_counts }; die SM wartet `delay`, setzt den Pin high (aktiver Puls),
// wartet `width` und setzt ihn wieder low. Vorteil ggue. Software-Bit-Bang:
// hardware-getaktete Flanken ohne Poll-Jitter. Generisch fuer praezise
// PWM-/Trigger-/Bit-Timing-Ausgaben (z. B. KNX-Senden).
// ---------------------------------------------------------------------------

// Richtet eine TX-State-Machine fuer `rp_gpio` ein. `out_rate_hz` liefert die
// Zaehlrate (Counts/Sekunde) zur Umrechnung TC-Ticks -> Counts. Rueckgabe:
// Handle >= 0, oder -1 bei Fehler/voll.
int  tx_setup(uint8_t rp_gpio, float& out_rate_hz);

// Stellt einen Puls in die FIFO. false, wenn die FIFO keinen Platz fuer
// beide Worte hat (Puls wird dann verworfen).
bool tx_emit(int handle, uint32_t delay_counts, uint32_t width_counts);

// Gibt die State-Machine wieder frei.
void tx_teardown(int handle);

// PIO-Ressourcennutzung ueber ALLE PIO-Bloecke (RP2350: pio0/1/2). Zaehlt
// belegte/freie State-Machines und Instruktions-Slots (das sind die knappen
// PIO-Ressourcen). Fuer die 'stats'-Anzeige. Nur oeffentliche SDK-API
// (pio_sm_is_claimed, pio_can_add_program_at_offset).
void usage(uint32_t& sm_used, uint32_t& sm_total,
           uint32_t& instr_used, uint32_t& instr_total);

} // namespace pio_glue