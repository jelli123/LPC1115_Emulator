#pragma once
//
// GDB Remote Serial Protocol Stub für den emulierten LPC1115-Gast.
//
// HONESTY DISCLAIMER: Dies ist *kein* CMSIS-DAP. Echtes CMSIS-DAP setzt
// einen externen SWD-Target voraus; wir wollen aber den Gast debuggen, der
// als Code auf demselben M33 läuft. Stattdessen exponieren wir das
// GDB-Remote-Serial-Protocol über einen zweiten USB-CDC-Endpoint, das
// direkt von `arm-none-eabi-gdb` und der VSCode-Erweiterung
// `cortex-debug` (mit servertype: external) konsumiert werden kann.
//
// Unterstützte Pakete:
//   ?   g  G  m  M  p  P  c  s  vCont?  vCont;c  vCont;s
//   Z0  z0   (software breakpoints via BKPT-Insertion im Gast-RAM)
//   qSupported  qC  qAttached  qfThreadInfo  qsThreadInfo
//
// Halten/Fortsetzen funktioniert über einen kooperativen Mechanismus:
//   - Bei BKPT-Hit landet der Gast im UsageFault-Handler (M33 trapt
//     unprivileged BKPT). Der Handler ruft gdb_stub::on_breakpoint() auf
//     und spinnt, bis GDB Continue schickt.
//   - Bei einem ^C aus GDB setzen wir SCB->ICSR.PENDSVSET, der PendSV-
//     Handler setzt Stop-Flag und wartet ebenfalls.
//

#include <cstdint>
#include <cstddef>

namespace gdb_stub {

void init();                    // Setzt USB-CDC #1 auf, registriert Hooks
void poll();                    // Im Host-Loop aufrufen (TinyUSB-Pump)

// Vom Trap-Handler aufgerufen, wenn der Gast einen BKPT trifft oder
// asynchroner Halt angefordert wurde.
void on_breakpoint(uint32_t* exc_frame, uint32_t* r4_r11);

// Vom CLI aufrufbar:
void start();                   // GDB-Server aktivieren
void stop();                    // deaktivieren
bool active();
uint16_t  port_index();         // Index des USB-CDC-Endpoints (0 oder 1)

} // namespace gdb_stub
