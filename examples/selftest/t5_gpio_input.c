/*
 * t5_gpio_input.c — Test: GPIO-Eingang (echter RP2350-Pin-Pegel).
 *
 * Liest LPC-Pin P0_1 als Eingang und spiegelt den Pegel auf den Ausgang
 * P0_7. Eingänge lesen im Emulator den ECHTEN Pegel des gemappten
 * RP2350-Pins (Default P0_1 -> GP3, P0_7 -> GP9).
 *
 * ERWARTUNG: GP3 auf 3,3 V oder GND legen (Taster/Jumper); die LED an GP9
 * folgt dem Pegel. Zusätzlich wird jede Änderung über UART gemeldet.
 *
 * Deckt ab: GPIO-DIR (Eingang), Lesen realer Pin-Pegel, maskierter Ausgang.
 */
#include "lpc1115.h"

#define IN_PORT  0
#define IN_PIN   1
#define OUT_PORT 0
#define OUT_PIN  7

int main(void) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_GPIO;
    uart_init(39);
    gpio_set_input(IN_PORT, IN_PIN);
    gpio_set_output(OUT_PORT, OUT_PIN);
    uart_puts("\n[t5] Spiegel P0_1 (GP3) -> P0_7 (GP9)\n");

    int last = -1;
    for (;;) {
        int v = gpio_read(IN_PORT, IN_PIN);
        gpio_write(OUT_PORT, OUT_PIN, v);
        if (v != last) {
            last = v;
            uart_puts("in=");
            uart_put_u32((uint32_t)v);
            uart_putc('\n');
        }
        delay_loops(20000);
    }
}
