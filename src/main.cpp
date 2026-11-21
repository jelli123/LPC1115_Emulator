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
#include "isp.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "hardware/structs/timer.h"

extern "C" void usb_stdio_init(void);
extern "C" bool usb_stdio_connected(void);
extern "C" void usb_stdio_task(void);

int main() {
    // LED zuerst, damit Lebenszeichen sichtbar ist, auch wenn spätere Init
    // hängt (Heartbeat = bereit). Siehe led.cpp für Statemachine.
    led::init();
    stdio_init_all();
    // Systemtimer nicht anhalten, wenn ein Kern per SWD im Debug-Halt steht
    // (RP2350-Default DBGPAUSE: Timer pausiert). Sonst friert ein angehaltener
    // Core1 alle Zeitfunktionen auf Core0 ein (CLI, XMODEM, Lockout-Timeout).
    timer_hw->dbgpause = 0;

    // WICHTIG: Storage + Config VOR usb_stdio_init(), denn der USB-Deskriptor
    // wird beim tusb_init() aus der Config gebaut (welche CDCs — CLI/GDB/Serial —
    // ueberhaupt am USB erscheinen). Aendert man das cli/gdb/serial_enable in der
    // CONFIG.INI, greift es erst nach dem naechsten Reset (Descriptor ist statisch,
    // solange USB laeuft).
    storage::init();
    config::load();
    {
        // INI-Import-Sektor (scripts/mkimage.py, per SWD/UF2 geschrieben): einmalig
        // uebernehmen und persistieren. ini_crc verhindert ein erneutes Anwenden
        // bei jedem Boot (spaetere CLI-/CONFIG.INI-Aenderungen bleiben erhalten).
        // Core1 laeuft noch nicht -> Flash-Schreiben hier unkritisch.
        uint32_t len = 0, crc = 0;
        const char* ini = storage::ini_import(len, crc);
        if (ini && crc != config::ini_import_crc()) {
            const uint32_t n = config::apply_ini(ini, len, nullptr, /*authoritative=*/false);
            config::set_ini_import_crc(crc);
            const bool ok = config::save();
            std::printf("[CFG] INI-Import uebernommen (%lu Zeilen, crc=0x%08lx) %s\n",
                        static_cast<unsigned long>(n), static_cast<unsigned long>(crc),
                        ok ? "und gespeichert" : "- SPEICHERN FEHLGESCHLAGEN");
        }
    }
    usb_stdio_init();

    // Kein Warten auf ein Terminal mehr (frueher bis zu 6 s, die den Autostart
    // verzoegerten): Ausgaben bis zum Verbinden puffert usb_stdio.cpp und
    // liefert sie nach.

    setup_fault_handlers();
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
        std::printf("[CFG] Board=%s%s INI-Import@0x%06lx\n", config::board_name(),
                    config::board_profile_active() ? " (Board-Profil aktiv, nicht gespeichert)" : "",
                    static_cast<unsigned long>(storage::ini_import_flash_offset()));
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
    isp::init();

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

    // Wie beim echten LPC: gehaltener RESET oder P0_1 low (isp_pins) hat Vorrang.
    if (!isp::intercept_boot() && config::autostart()) {
        std::printf("[BOOT] autostart aktiv\n");
        emulator::load_and_start();
    }

    cli::run();   // niemals zurück
    return 0;
}
