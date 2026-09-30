/*
 * t3_timer_irq.c — Test: CT16B0-Match-Interrupt + NVIC-Injektion.
 *
 * Konfiguriert CT16B0 auf einen periodischen Match-Interrupt (~2 Hz) und
 * toggelt im Handler den GPIO-Pin P0_7. Zusätzlich wird über UART pro
 * Interrupt ein Tick-Zähler ausgegeben.
 *
 * ERWARTUNG: P0_7 (Default RP2350-GP9) blinkt im 2-Hz-Takt; im Terminal
 * (19200 8N1, RP2350-uart0) zählt "tick=N" hoch.
 *
 * Deckt ab: Timer-Modell (PR/MR0/MCR/TCR), Match-IRQ-Injektion über den
 * Schatten-NVIC und den synthetischen Cortex-M-Exception-Frame (PendSV).
 *
 * Timing: PR=119 -> Timertakt 100 kHz; MR0=50000 -> 500 ms je Match;
 * Toggle im Handler => 1 Hz sichtbarer Voll-Zyklus (2 Flanken/s).
 */
#include "lpc1115.h"

#define LED_PORT 0
#define LED_PIN  7

static volatile uint32_t g_ticks = 0;
static volatile int      g_level = 0;

void CT16B0_Handler(void) {
    CTx_IR(CT16B0_BASE) = 0x1;            /* MR0-Interrupt-Flag löschen */
    g_level ^= 1;
    gpio_write(LED_PORT, LED_PIN, g_level);
    g_ticks++;
}

int main(void) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_GPIO | SYSAHBCLKCTRL_CT16B0;
    uart_init(39);
    gpio_set_output(LED_PORT, LED_PIN);

    CTx_TCR(CT16B0_BASE) = 0x2;           /* Reset */
    CTx_PR(CT16B0_BASE)  = 119;           /* 12 MHz / 120 = 100 kHz */
    CTx_MR0(CT16B0_BASE) = 50000;         /* 500 ms */
    CTx_MCR(CT16B0_BASE) = 0x3;           /* MR0: Interrupt + Reset */
    nvic_enable(IRQ_CT16B0);
    CTx_TCR(CT16B0_BASE) = 0x1;           /* Enable */

    uart_puts("\n[t3] CT16B0-IRQ laeuft...\n");
    uint32_t last = 0;
    for (;;) {
        uint32_t t = g_ticks;
        if (t != last) {
            last = t;
            uart_puts("tick=");
            uart_put_u32(t);
            uart_putc('\n');
        }
    }
}
