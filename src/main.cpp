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
#include "uart_bridge.h"
#include "led.h"
#include "debug_bridge.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "pico/multicore.h"
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
    debug_bridge::init();
    {
        // Persistenz-Diagnose: zeigt, ob eine gueltige Config aus dem Flash
        // geladen wurde (seq/keys) und wo die Config-Region liegt. Faellt ein
        // gespeicherter Wert nach einem Power-Cycle auf "LEER->Defaults" zurueck,
        // deutet das auf einen Flash-Groessen-Mismatch (Board-Header vs. reale
        // Bausteingroesse) hin — die Region liegt dann jenseits des Flash.
        const auto ci = storage::config_persist_info();
        std::printf("[CFG] Flash=%luKiB Region@0x%06lx(%lu Sekt.) %s seq=%lu keys=%lu\n",
                    static_cast<unsigned long>(ci.flash_size_kib),
                    static_cast<unsigned long>(ci.region_offset),
                    static_cast<unsigned long>(ci.slot_sectors),
                    ci.loaded_valid ? "geladen" : "LEER->Defaults",
                    static_cast<unsigned long>(ci.sequence),
                    static_cast<unsigned long>(ci.key_count));
    }
    peripherals::init();
    pio_glue::init();
    gdb_stub::init();
    irq_inject::init();
    swd_target::init();
    target_halt::init();
    iap::init();
    usb_msc::init();
    uart_bridge::init();

    // UART-Bridge aus Config starten, falls aktiviert
    if (config::uart_bridge_enabled()) {
        uart_bridge::set_tx_pin(config::uart_bridge_tx_pin());
        uart_bridge::set_rx_pin(config::uart_bridge_rx_pin());
        uart_bridge::start();
    }

    emulator::boot_core1();

    // Core0 selbst als Multicore-Lockout-Victim registrieren. Damit kann der
    // IAP-Pfad (laeuft im Fault-Handler auf Core1) Core0 vor einem gast-
    // initiierten Flash-Schreibzugriff sicher aussperren. Core0s VTOR bleibt
    // stets die SDK-Tabelle -> sein Lockout-Handler ist immer erreichbar.
    multicore_lockout_victim_init();

    cli::init();

    if (config::autostart()) {
        std::printf("[BOOT] autostart aktiv\n");
        emulator::load_and_start();
    }

    cli::run();   // niemals zurück
    return 0;
}
