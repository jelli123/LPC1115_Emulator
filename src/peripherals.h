#pragma once
//
// LPC1115-Peripherie-Modelle. Alle MMIO-Zugriffe der Gast-Firmware werden
// über mmio_read8/mmio_write8 geleitet (vom Trap-Handler aus aufgerufen).
//

#include <cstdint>

namespace peripherals {

void init();
void reset();

// (Re-)Initialisiert die I²C-Hardware-Bridge anhand der aktuellen config.
// Wird nach dem Einlesen von CONFIG.INI aufgerufen, damit die Bridge auch
// ohne Power-Cycle aktiv wird.
void i2c_bridge_reinit();

// (Re-)Initialisiert die SPI- bzw. ADC-Hardware-Bridge anhand der aktuellen
// config (analog zu i2c_bridge_reinit, nach dem Einlesen von CONFIG.INI).
void spi_bridge_reinit();
void adc_bridge_reinit();
void ct_bridge_reinit();

// Vom Trap-Handler nach erfolgter Emulation aufgerufen — Stats/PLL-Folgen.
void on_post_write_hook();

bool mmio_read8 (uint32_t addr, uint8_t&  out);
bool mmio_write8(uint32_t addr, uint8_t   val);

// Aktuelle resultierende CPU-Frequenz, die per SYSCON-Schreibvorgängen
// des Gastes auf dem RP2350 eingestellt wurde (0 = Default 48 MHz IRC).
uint32_t current_cpu_hz();

// Liest echte Eingänge, erkennt Flanken und pendet PINT-/GINT-IRQs.
// Normalerweise aus dem MMIO-Trap aufgerufen; bei aktivem WFI-Pin-Wakeup
// auch aus dem SVC-Warte-Handler (src/emulator.cpp), damit eine reine
// WFI-Warteschleife durch echte Pin-Flanken geweckt werden kann.
void sample_pin_interrupts();

// Treibt die zeitbasierten Modelle (CT16/CT32-Timer, WWDT) lazy weiter und
// pendet fällige IRQs. Wird aus dem WFI-Warte-Handler periodisch gepollt,
// da diese Modelle sonst nur bei MMIO-Zugriffen voranschreiten.
void poll_timed_sources();

// True, sobald mindestens ein Timer-Capture-Kanal scharf ist (CCR-Flanke
// aktiv und Pin gebunden). Der WFI-Warte-Handler pollt dann eng (ohne die
// 50-µs-Pause), damit KNX-Busflanken (~104 µs/Bit) zeitlich aufgelöst werden.
bool capture_armed();

struct Stats {
    uint64_t mmio_writes;
    uint64_t mmio_reads;
    uint64_t gpio_writes;
    uint64_t pll_reconfigs;
    uint64_t nvic_writes;
};
Stats stats();

// SysTick-Diagnose: Trap-Zaehler (Reads/Writes auf 0xE000E010-01F) + aktuelle
// emulierte Registerwerte. Zeigt, ob der unprivilegierte Gast-SysTick-Zugriff
// ueberhaupt getrappt wird und ob CTRL (ENABLE|TICKINT) korrekt gesetzt ist.
void systick_debug(uint32_t& reads, uint32_t& writes,
                   uint32_t& csr, uint32_t& rvr, uint32_t& cvr,
                   uint64_t& ticks);

} // namespace peripherals
