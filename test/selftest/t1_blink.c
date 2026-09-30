/*
 * t1_blink.c — Test: GPIO-Ausgang.
 *
 * Schaltet LPC-Pin P0_7 im ~2-Hz-Takt um. Über die Pinmap des Emulators
 * landet P0_7 standardmäßig auf RP2350-GP9 (cli: `pinmap show`).
 *
 * ERWARTUNG: An GP9 (bzw. dem auf P0_7 gemappten Pin) eine LED+Vorwiderstand
 * gegen GND; sie blinkt sichtbar. Alternativ mit Logiktester/Oszilloskop.
 *
 * Deckt ab: GPIO-DIR, maskierter DATA-Schreibzugriff, Pinmap-Routing.
 */
#include "lpc1115.h"

#define LED_PORT 0
#define LED_PIN  7

int main(void) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_GPIO;
    gpio_set_output(LED_PORT, LED_PIN);

    for (;;) {
        gpio_write(LED_PORT, LED_PIN, 1);
        delay_loops(600000);
        gpio_write(LED_PORT, LED_PIN, 0);
        delay_loops(600000);
    }
}
