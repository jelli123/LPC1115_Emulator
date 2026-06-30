#pragma once
//
// XMODEM-Empfaenger (CRC-16-CCITT, inkl. 1K/STX-Bloecke).
//
// Robuste Alternative zum reinen Zeilen-Streaming der Intel-HEX-Datei ueber die
// CLI: durch Block-Pruefsumme und ACK/NAK-Handshake gehen bei schnellem
// Einfuegen ("paste") keine Zeichen mehr verloren. Die Nutzdaten werden
// blockweise an eine Callback-Senke geliefert (z. B. den HEX-Parser).
//

#include <cstddef>
#include <cstdint>

namespace xmodem {

enum class Result : uint8_t {
    Ok,          // EOT empfangen, Transfer komplett
    Canceled,    // Sender/Empfaenger hat abgebrochen (CAN) oder Senke-Fehler
    SyncFailed,  // Sender hat nicht innerhalb der Timeouts begonnen
};

// Callback fuer die Nutzdaten eines akzeptierten Blocks. Rueckgabe false
// bricht den Transfer kontrolliert ab (CAN an den Sender).
using DataSink = bool (*)(const uint8_t* data, std::size_t len, void* ctx);

// Pump-Callback: wird waehrend des (blockierenden) Empfangs regelmaessig
// aufgerufen, damit der USB-Stack (tud_task) weiter bedient wird. Darf null
// sein, wenn stdin/stdout ohne periodischen Service auskommt.
using Pump = void (*)();

// Empfaengt einen XMODEM(-CRC/1K)-Transfer ueber stdin/stdout.
// bytes_received zaehlt die ausgelieferten Roh-Bytes (inkl. evtl. Padding des
// letzten Blocks).
Result receive(DataSink sink, void* ctx, Pump pump, uint32_t& bytes_received);

} // namespace xmodem
