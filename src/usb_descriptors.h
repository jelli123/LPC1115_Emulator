#pragma once
//
// Dynamischer USB-Deskriptor-Aufbau (Composite: 0..3x CDC + MSC).
//
// Welche CDC-Interfaces am USB erscheinen, haengt von der CONFIG.INI ab
// (cli_enable / gdb_enable / serial_enable). Deaktivierte CDCs verschwinden
// komplett aus dem Deskriptor -> der Host sieht einen COM-Port weniger. Das MSC
// (Laufwerk) ist IMMER vorhanden, damit CONFIG.INI zur Wiederherstellung
// erreichbar bleibt.
//
// Weil sich dadurch die CDC-Instanz-Indizes (tud_cdc_n_*) verschieben, wird die
// Zuordnung hier zentral berechnet und von stdio/gdb/uart_bridge abgefragt.
//
// usb_desc_build() MUSS vor tusb_init() aufgerufen werden (nach config::load()).

#include <cstdint>

// Baut den Konfigurations-Deskriptor + die CDC-Index-Zuordnung aus der aktuell
// geladenen config:: neu auf. Idempotent.
void usb_desc_build();

// CDC-Instanz-Index (fuer tud_cdc_n_*) der jeweiligen Rolle, oder -1 wenn per
// Config deaktiviert (Interface nicht im Deskriptor).
int  usb_desc_cdc_cli();      // CLI/stdio
int  usb_desc_cdc_gdb();      // GDB-RSP
int  usb_desc_cdc_serial();   // Serial-Adapter / LPC-UART0<->CDC

// Anzahl aktiver CDC-Interfaces (0..3).
int  usb_desc_cdc_count();
