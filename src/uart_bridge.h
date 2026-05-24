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

} // namespace uart_bridge
