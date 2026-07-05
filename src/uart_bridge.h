#pragma once
//
// PIO-basierte UART-Bridge auf USB CDC#2.
//
// Kein Hardware-UART belegt — TX/RX laufen über PIO-State-Machines.
// Die Baudrate wird vom Host via CDC SET_LINE_CODING gesetzt.
// Aktivierung und Pin-Konfiguration über CLI oder config.ini.
//

#include <cstdint>

namespace uart_bridge {

void init();

// Muss zyklisch aus dem Hauptloop aufgerufen werden.
// Transportiert Daten bidirektional: CDC#2 ↔ PIO-UART.
void poll();

// Bridge ein/ausschalten. Pins müssen vorher gesetzt sein.
bool start();
void stop();
bool active();

// Pin-Konfiguration (gilt ab nächstem start()).
void set_tx_pin(int gpio);
void set_rx_pin(int gpio);
int  tx_pin();
int  rx_pin();

// Aktuelle Baudrate (vom Host gesetzt, read-only).
uint32_t baud_rate();

// Diagnose-Zaehler des Datenflusses CDC#2 -> PIO-TX -> PIO-RX -> CDC#2.
// Zeigt, an welcher Stelle die Kette bricht (via 'uart status').
void debug_counts(uint32_t& cdc_rx, uint32_t& pio_tx,
                  uint32_t& pio_rx, uint32_t& cdc_tx);

// Core0-Poll der virtuellen LPC-UART0 <-> CDC#2 Kopplung (config uart0_cdc).
// Aus dem Hauptloop aufrufen; No-op, wenn uart0_cdc aus ist.
void uart0_cdc_poll();

// Diagnose-Zaehler der virtuellen LPC-UART0 <-> Serial-CDC Kopplung (uart0_cdc),
// via 'uart status' sichtbar. pc_to_guest = von knxd empfangene Bytes,
// guest_to_pc = vom Gast (FT12) gesendete Bytes.
void uart0_cdc_counts(uint32_t& pc_to_guest, uint32_t& guest_to_pc);

} // namespace uart_bridge
