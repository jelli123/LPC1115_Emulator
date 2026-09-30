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
// claim_pin=false: nur mitlesen (Pin-Funktion/-Richtung bleiben unveraendert).
int  ts_setup(uint8_t rp_gpio, float& out_rate_hz, bool claim_pin = true);

// Zieht genau eine Flanke aus der FIFO: absoluter Zeitpunkt (time_us_64-
// Zeitbasis, ~1 us genau) und Pegel NACH der Flanke (1 = steigend).
// false = FIFO leer.
bool ts_read_edge(int handle, uint64_t& t_us, bool& level);

// Gibt die State-Machine wieder frei (vor Neukonfiguration).
void ts_teardown(int handle);

// Verwirft einen noch nicht ausgegebenen Puls (FIFO + laufende Wartezeit),
// Pin geht auf low. Fuer Neuplanung, wenn sich MR/MCR/TC nachtraeglich aendern.
void tx_cancel(int handle);

// Neuaufbau des Pulsplans OHNE Pegelsprung: SM anhalten, FIFO leeren, an den
// Programmanfang - der Pin behaelt seinen Pegel (ein laufender Puls bleibt
// high). keep_high=false zieht den Pin sofort auf low. Danach tx_emit() fuer
// die (Rest-)Pulse und tx_rebuild_end() zum Starten.
void tx_rebuild_begin(int handle, bool keep_high);
// SM anhalten und den Pin (low, Ausgang) an SIO zurueckgeben, ohne die SM
// freizugeben; tx_unpark() holt ihn wieder. Die Neuanlage (tx_setup) laeuft
// ueber SDK-Funktionen im Flash und kostete ~110 us - zu viel im Trap am
// Telegrammbeginn (sblib verlaesst im Ruhezustand den PWM-Modus).
void tx_park(int handle);
void tx_unpark(int handle);
void tx_rebuild_end(int handle);

// Diagnose Match-Puls-SM: Programmzaehler (relativ), TX-FIFO-Fuellstand,
// PIO-Ausgangspegel und -Treiberfreigabe des Pins.
bool tx_debug(int handle, uint32_t& pc, uint32_t& fifo, uint32_t& pin_out, uint32_t& pin_oe);

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