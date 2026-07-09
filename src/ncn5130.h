#pragma once
//
// Virtueller NCN5130 – KNX-Sekundaerinterface am LPC-SSP (SPI).
//
// Spezifikation: docs/NCN5130_EMULATION_SPEC.md
// Schnittstelle:  docs/NCN5130_INTERFACE.md
//
// Der emulierte LPC-Gast spricht ueber seine SSP/SPI-Schnittstelle mit einem
// voll virtuellen NCN5130 (ON-Semi KNX-Transceiver). Es entsteht KEIN echter
// KNX-Chip im Emulator; das NCN5130-Kommandoprotokoll wird byte-transparent
// nachgebildet. Der KNX-L2-Stack (Adressierung/Laenge/Routing) liegt im Gast
// (SBLib) und wird hier NICHT dupliziert.
//
// Kernanbindung: peripherals.cpp ssp_write_byte() ruft im NCN-Modus je
// SPI-Byteaustausch spi_exchange(mosi) -> miso auf (statt Loopback/HW-Bridge).
//

#include <cstdint>

namespace ncn5130 {

// Einmalige Initialisierung (Reset des Modellzustands).
void init();

// Aktivierung je LPC-SSP-Instanz (0=SSP0, 1=SSP1). Nur EINE Instanz kann der
// virtuelle NCN5130 sein (er ist SPI-Master, LPC=Slave auf diesem Bus); die
// andere SSP-Instanz bleibt normaler Master fuer weitere SPI-Geraete.
void set_enabled(int lpc_ssp, bool enable);
bool enabled(int lpc_ssp);           // true, wenn DIESE Instanz der NCN ist
bool any_enabled();                  // true, wenn irgendeine Instanz NCN ist

// KERN: ein SPI-Byte-Austausch aus dem SSP-Intercept.
//   mosi = vom Gast geschriebenes Byte (Host -> Device)
//   ret  = an den Gast zu lieferndes Byte (Device -> Host, MISO)
uint8_t spi_exchange(uint8_t mosi);

// Internes KNX-Back-End vorantreiben (Sende-Timing, L_Data.con, Loopback).
// MUSS auf Core1 laufen (IRQ-Injektion + Modell-Zugriff dort), daher aus
// peripherals::poll_timed_sources() aufgerufen. Ohne angebundenen externen
// PHY quittiert dieses Modell gesendete Frames selbst (positiv).
void poll();

// Loopback/Monitor-Selbsttest: gesendete Frames werden zusaetzlich als
// L_Data_*.ind in den RX-Pfad zurueckgespiegelt (Test ohne KNX-Transceiver).
void set_loopback(bool enable);
bool loopback();

// --- Reale KNX-PHY (STKNX/Selfbus diskreter Buskoppler, KEIN UART) ---------
// Der STKNX ist nur die analoge PHY - digital identisch zum Selfbus-KNX-
// Interface (sblib): TX = Idle-Low-Pin, je KNX-"0"-Bit ein 35 us HIGH-Puls;
// RX = Idle-High-Pin, fallende Flanke je "0"-Bit (9600 Bd, 104 us/Bit, 8 Daten
// LSB-first + gerade Paritaet + Stopbit). Bit-Timing laeuft jitterfrei ueber die
// vorhandenen PIO-Primitive (match_pulse TX, timer_edge_ts RX). tx_pin/rx_pin =
// RP2350-GPIOs am STKNX. Beide >=0 -> PHY aktiv; sonst nur Software-Loopback.
// MUSS von Core0 initialisiert werden (PIO/GPIO-Setup).
void phy_init(int tx_pin, int rx_pin);
bool phy_active();

// --- Back-End (KNX-PHY am Sekundaerbus) -----------------------------------
// Ein vom Bus empfangenes KNX-Frame in den RX-Puffer legen -> loest die
// L_Data_*.ind-Auslieferung + DATA_READY aus. extended=true -> Extended-Frame.
void push_rx_frame(const uint8_t* data, uint16_t len, bool extended);

// Naechstes vom Gast zusammengestelltes Sende-Frame abholen (fuer den PHY).
// Liefert false, wenn keins ansteht. Nach dem Senden tx_result() aufrufen.
bool pop_tx_frame(const uint8_t** data, uint16_t* len);

// Ergebnis einer Bus-Sendung zurueckmelden -> erzeugt L_Data.con (pos/neg).
void tx_result(bool positive);

// Meldet, ob gerade Empfangsdaten fuer den Host anstehen (DATA_READY).
bool data_ready();

// --- Diagnose (CLI 'ncn status') ------------------------------------------
struct Debug {
    bool     enabled;        // NCN-Modus aktiv
    int      ssp;            // gekoppelte LPC-SSP-Instanz (-1 = keine)
    uint8_t  state;          // 0=PowerUp 1=Sync 2=Stop 3=Normal
    bool     busmon;         // Bus-Monitor-Modus
    uint8_t  cfg;            // Feature-Bits (auto-ack/poll/CRC/MARKER/busy)
    uint16_t phys_addr;      // gesetzte physikalische Adresse
    uint32_t rx_frames;      // empfangene Frames (Bus -> Host)
    uint32_t tx_frames;      // gesendete Frames (Host -> Bus)
    uint32_t cmd_bytes;      // ueber SPI empfangene Kommando-Bytes
    bool     rx_ready;       // DATA_READY aktiv
    bool     loopback;       // Loopback/Monitor-Selbsttest aktiv
    bool     tx_inflight;    // Sende-Frame wartet auf Bus-/con-Abschluss
    bool     phy_active;     // reale KNX-PHY (STKNX) aktiv
    int      phy_tx_pin;     // TX-GPIO (-1 = keiner)
    int      phy_rx_pin;     // RX-GPIO (-1 = keiner)
    uint32_t rx_ack_ok;      // empfangene positive Quittungen (unsere TX)
    uint32_t rx_ack_err;     // empfangene NACK/BUSY/Timeout (unsere TX)
};
Debug debug();

} // namespace ncn5130
