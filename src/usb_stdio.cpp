// Eigene TinyUSB-Initialisierung + stdio-Bridge auf CDC-Interface 0.
//
// Hintergrund: pico_stdio_usb der SDK kompiliert TinyUSB mit eigener
// tusb_config.h (CFG_TUD_CDC=1, kein MSC). Mit unserem 2×CDC + MSC
// Descriptor passt das nicht. Daher: pico_stdio_usb ist via
// CMakeLists deaktiviert, wir managen TinyUSB hier selbst.

#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "usb_descriptors.h"

#include <cstring>

extern "C" {

// CDC-Instanz der CLI (dynamisch; -1 wenn CLI per Config deaktiviert).
static inline int cli_cdc() { return usb_desc_cdc_cli(); }

// stdio -> CLI-CDC OUT
static void stdio_cdc_out_chars(const char* buf, int length) {
    int itf = cli_cdc();
    if (itf < 0 || !tud_cdc_n_connected((uint8_t)itf)) return;
    int written = 0;
    while (written < length) {
        uint32_t avail = tud_cdc_n_write_available((uint8_t)itf);
        if (avail == 0) {
            tud_task();
            tud_cdc_n_write_flush((uint8_t)itf);
            sleep_us(100);
            continue;
        }
        uint32_t chunk = (uint32_t)(length - written);
        if (chunk > avail) chunk = avail;
        tud_cdc_n_write((uint8_t)itf, buf + written, chunk);
        written += (int)chunk;
    }
    tud_cdc_n_write_flush((uint8_t)itf);
}

// stdio <- CLI-CDC IN  (non-blocking; returns PICO_ERROR_NO_DATA on empty)
static int stdio_cdc_in_chars(char* buf, int length) {
    int itf = cli_cdc();
    if (itf < 0 || !tud_cdc_n_connected((uint8_t)itf)) return PICO_ERROR_NO_DATA;
    if (!tud_cdc_n_available((uint8_t)itf))  return PICO_ERROR_NO_DATA;
    uint32_t r = tud_cdc_n_read((uint8_t)itf, buf, (uint32_t)length);
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
    usb_desc_build();        // Deskriptor aus config bauen (VOR tusb_init).
    tusb_init();
    // stdio nur einhaengen, wenn die CLI-CDC ueberhaupt existiert.
    if (usb_desc_cdc_cli() >= 0)
        stdio_set_driver_enabled(&s_stdio_cdc, true);
}

void usb_stdio_task(void) {
    tud_task();
}

bool usb_stdio_connected(void) {
    int itf = usb_desc_cdc_cli();
    return itf >= 0 && tud_cdc_n_connected((uint8_t)itf);
}

} // extern "C"
