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
#include "usb_msc.h"
#include "led.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

extern "C" void usb_stdio_init(void);
extern "C" bool usb_stdio_connected(void);
extern "C" void usb_stdio_task(void);

int main() {
    // LED zuerst, damit Lebenszeichen sichtbar ist, auch wenn spätere Init
    // hängt (Heartbeat = bereit). Siehe led.cpp für Statemachine.
    led::init();
    stdio_init_all();
    usb_stdio_init();

    // Bis zu 6 s auf USB-Host warten, damit erste Konsolenausgaben sichtbar sind.
    for (int i = 0; i < 60 && !usb_stdio_connected(); ++i) {
        usb_stdio_task();
        sleep_ms(100);
        led::poll();
    }

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
    usb_msc::init();
    emulator::boot_core1();

    cli::init();

    if (config::autostart()) {
        std::printf("[BOOT] autostart aktiv\n");
        emulator::load_and_start();
    }

    cli::run();   // niemals zurück
    return 0;
}
