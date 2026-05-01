// Eigene TinyUSB-Initialisierung + stdio-Bridge auf CDC-Interface 0.
//
// Hintergrund: pico_stdio_usb der SDK kompiliert TinyUSB mit eigener
// tusb_config.h (CFG_TUD_CDC=1, kein MSC). Mit unserem 2×CDC + MSC
// Descriptor passt das nicht. Daher: pico_stdio_usb ist via
// CMakeLists deaktiviert, wir managen TinyUSB hier selbst.

#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"

#include <cstring>

extern "C" {

// stdio → CDC#0 OUT
static void stdio_cdc_out_chars(const char* buf, int length) {
    if (!tud_cdc_n_connected(0)) return;
    int written = 0;
    while (written < length) {
        uint32_t avail = tud_cdc_n_write_available(0);
        if (avail == 0) {
            tud_task();
            tud_cdc_n_write_flush(0);
            sleep_us(100);
            continue;
        }
        uint32_t chunk = (uint32_t)(length - written);
        if (chunk > avail) chunk = avail;
        tud_cdc_n_write(0, buf + written, chunk);
        written += (int)chunk;
    }
    tud_cdc_n_write_flush(0);
}

// stdio ← CDC#0 IN  (non-blocking; returns PICO_ERROR_NO_DATA on empty)
static int stdio_cdc_in_chars(char* buf, int length) {
    if (!tud_cdc_n_connected(0)) return PICO_ERROR_NO_DATA;
    if (!tud_cdc_n_available(0))  return PICO_ERROR_NO_DATA;
    uint32_t r = tud_cdc_n_read(0, buf, (uint32_t)length);
    return r ? (int)r : PICO_ERROR_NO_DATA;
}

static stdio_driver_t s_stdio_cdc = {
    .out_chars                    = stdio_cdc_out_chars,
    .out_flush                    = nullptr,
    .in_chars                     = stdio_cdc_in_chars,
    .set_chars_available_callback = nullptr,
    .next                         = nullptr,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .last_ended_with_cr           = false,
    .crlf_enabled                 = true,
#endif
};

// Periodischer tud_task() wird aus dem Hauptloop (cli::run) aufgerufen.
// WICHTIG: tud_task() darf NICHT aus IRQ-Kontext laufen (TinyUSB ist nicht
// IRQ-safe; auf RP2350 führt das zu HardFault → Boot-ROM → BOOTSEL).

void usb_stdio_init(void) {
    tusb_init();
    stdio_set_driver_enabled(&s_stdio_cdc, true);
}

void usb_stdio_task(void) {
    tud_task();
}

bool usb_stdio_connected(void) {
    return tud_cdc_n_connected(0);
}

} // extern "C"
