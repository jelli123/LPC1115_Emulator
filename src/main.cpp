#include "fault.h"
#include "emulator.h"
#include "peripherals.h"
#include "config.h"
#include "storage.h"
#include "cli.h"
#include "pio_glue.h"
#include "gdb_stub.h"
#include "irq_inject.h"
#include "swd_target.h"
#include "target_halt.h"
#include "iap.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

int main() {
    stdio_init_all();

    // Bis zu 6 s auf USB-Host warten, damit erste Konsolenausgaben sichtbar sind.
    for (int i = 0; i < 60 && !stdio_usb_connected(); ++i) sleep_ms(100);

    setup_fault_handlers();
    storage::init();
    config::load();
    peripherals::init();
    pio_glue::init();
    gdb_stub::init();
    irq_inject::init();
    swd_target::init();
    target_halt::init();
    iap::init();
    emulator::boot_core1();

    cli::init();

    if (config::autostart()) {
        std::printf("[BOOT] autostart aktiv\n");
        emulator::load_and_start();
    }

    cli::run();   // niemals zurück
    return 0;
}
