#pragma once
//
// GDB Remote Serial Protocol Stub für den emulierten LPC1115-Gast.
//
// Dies ist *kein* CMSIS-DAP (das setzt ein externes SWD-Target voraus).
// Stattdessen spricht der Stub das GDB-Remote-Serial-Protocol über die
// GDB-USB-CDC, direkt nutzbar von `arm-none-eabi-gdb` und `cortex-debug`
// (servertype: external).
//
// Unterstützte Pakete:
//   ?  g  G  p  P  m  M  c  s  D  k  H  ^C  Z0/z0 (Software-Breakpoints)
//   qSupported  qC  qAttached  qfThreadInfo  qsThreadInfo  (sonst leer)
//
// Adressen und Register in LPC-Sicht: Flash 0x00000000.., RAM 0x10000000..,
// PC/LR/SP (und Register, die ins Image/RAM zeigen) werden zwischen der
// relocierten RP2350-Position und dem LPC-Adressraum umgerechnet, damit GDB
// mit der ELF-Datei der Firmware zusammenpasst.
//
// Anhalten/Schritt/Breakpoints laufen ueber target_halt (Halt im PendSV bzw.
// DebugMonitor auf Core1). Der Stub selbst laeuft ausschliesslich auf Core0.
//

#include <cstdint>
#include <cstddef>

namespace gdb_stub {

void init();
void poll();                    // Im Host-Loop (Core0) aufrufen

void start();                   // GDB-Server aktivieren
void stop();                    // deaktivieren
bool active();
uint16_t  port_index();         // Index der GDB-USB-CDC

} // namespace gdb_stub
