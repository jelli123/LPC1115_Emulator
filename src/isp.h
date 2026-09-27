#pragma once
//
// Virtueller ISP-Bootloader (UART-ISP-Protokoll des LPC1115, UM10398 Kap. 26),
// damit der Emulator wie ein echter LPC1115 z. B. mit FlashMagic oder lpc21isp
// programmiert werden kann. Eigene Implementierung des Protokolls (kein NXP-ROM).
//
// Transporte:
//   * Cdc  - eigene USB-CDC "LPC-Emu ISP" (isp_enable). DTR = RESET, RTS = ISP
//            (isp_dtr_rts), wie die FlashMagic-Option "Use DTR and RTS to control
//            RST and ISP pin". Optional startet ein '?' den ISP direkt
//            (isp_autosync).
//   * Uart - die UART0-Pads (uart0_tx/uart0_rx) wie beim echten LPC. Eintritt,
//            wenn beim Reset LPC-P0_1 (pin.0_1) low ist (isp_pins). Autobaud per
//            '?' oder feste Baudrate (isp_baud).
//
// Unabhaengig vom ISP kann LPC-P0_0 (pin.0_0) als RESET-Eingang dienen
// (reset_in): low haelt den Gast im Reset, die steigende Flanke startet ihn neu
// (bzw. den ISP, wenn P0_1 low ist). Nutzt der Gast P0_0 per IOCON als GPIO,
// ist die RESET-Funktion - wie am echten Chip - abgeschaltet.
//
// Alles laeuft auf Core0 aus dem Hauptloop (poll()); der Gast ist waehrend des
// ISP-Betriebs gestoppt.

#include <cstdint>

namespace isp {

enum class Transport : uint8_t { None, Cdc, Uart };

void init();
void poll();

bool      active();
Transport transport();

// Stoppt den Gast und startet den ISP auf dem angegebenen Transport.
bool enter(Transport t, const char* reason);
// Beendet den ISP (Flash-Stand wird festgeschrieben); optional Gast starten.
void leave(bool start_guest);

// Vor jedem (Neu)start des Gasts aufrufen (Autostart, 'reset', WDT/SYSRESETREQ).
// true = Gast NICHT starten: RESET wird gehalten oder P0_1 war low und der ISP
// wurde gestartet.
bool intercept_boot();

// IAP "Reinvoke ISP" (Kommando 57) - von Core1 abfragbar.
bool available();
// Core0: Gast hat per IAP 57 den ISP angefordert (Core1 parkt bereits).
void on_guest_reinvoke();

// Start des Gasts, sofern kein RESET gehalten wird und Firmware vorhanden ist.
void start_guest_if_possible();

void print_status();

} // namespace isp
