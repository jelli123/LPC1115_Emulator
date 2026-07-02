#pragma once
//
// Guest -> Host Debug-Bridge.
//
// Die Gast-Firmware schreibt Zeichen auf eine feste MMIO-Adresse
// (DEBUG_BRIDGE_PORT). Diese liegt im LPC-Peripheriefenster (0x40000000-
// 0x4FFFFFFF), das fuer den unprivilegierten Gast nicht gemappt ist und daher
// trappt -> peripherals::mmio_write8 leitet die Bytes hierher. Der Host sammelt
// sie in einem Ringpuffer (SPSC: Core1 = Gast schreibt, Core0 = CLI/USB liest)
// und stellt sie ueber die CLI ('dbg') sowie als DEBUG.TXT auf dem USB-MSC-
// Laufwerk bereit.
//
// Verwendung im Gast: examples/lpc_debug/lpc_debug.h einbinden und
// dbg_puts()/dbg_kv()/... aufrufen. Die dortige Port-Adresse MUSS mit
// DEBUG_BRIDGE_PORT hier uebereinstimmen.

#include <cstdint>
#include <cstddef>

namespace debug_bridge {

// MMIO-Port im (getrappten) LPC-Peripherieraum. +0x00 = DATA (Byte schreiben ->
// an den Ringpuffer anhaengen). Bewusst hoch im 0x4Fxx-Bereich, wo kein reales
// LPC1115-Peripheral liegt -> keine Kollision mit echter Firmware.
constexpr uint32_t DEBUG_BRIDGE_PORT = 0x4FFF'0000u;

void init();

// Ein Byte vom Gast anhaengen (aus dem MMIO-Trap auf Core1 aufgerufen).
void put_byte(uint8_t b);

// Aktuellen Pufferinhalt (aelteste .. neueste Bytes, in Reihenfolge) nach 'out'
// kopieren. Gibt die Anzahl kopierter Bytes zurueck (<= cap). NUL-terminiert,
// falls Platz ist.
uint32_t snapshot(char* out, uint32_t cap);

// Anzahl seit dem letzten clear() insgesamt empfangener Bytes (auch die bereits
// aus dem Ring verdraengten). Fuer Diagnose/Statuszeile.
uint32_t total_bytes();

void clear();

} // namespace debug_bridge
