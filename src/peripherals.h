#pragma once
//
// LPC1115-Peripherie-Modelle. Alle MMIO-Zugriffe der Gast-Firmware werden
// über mmio_read8/mmio_write8 geleitet (vom Trap-Handler aus aufgerufen).
//

#include <cstdint>

namespace peripherals {

void init();
void reset();

// Vom Trap-Handler nach erfolgter Emulation aufgerufen — Stats/PLL-Folgen.
void on_post_write_hook();

bool mmio_read8 (uint32_t addr, uint8_t&  out);
bool mmio_write8(uint32_t addr, uint8_t   val);

// Aktuelle resultierende CPU-Frequenz, die per SYSCON-Schreibvorgängen
// des Gastes auf dem RP2350 eingestellt wurde (0 = Default 48 MHz IRC).
uint32_t current_cpu_hz();

struct Stats {
    uint64_t mmio_writes;
    uint64_t mmio_reads;
    uint64_t gpio_writes;
    uint64_t pll_reconfigs;
    uint64_t nvic_writes;
};
Stats stats();

} // namespace peripherals
