/*
 * t4_adc_uart.c — Test: ADC + UART-Ausgabe.
 *
 * Liest ADC-Kanal 0 zyklisch und gibt den 10-Bit-Wert über UART aus.
 * Im Emulator liefert der ADC bei aktivierter ADC-Bridge (cli:
 * `cfg set adc_bridge_en 1`) den echten RP2350-ADC-Wert von GPIO26;
 * ohne Bridge einen deterministischen Mittenwert (~512).
 *
 * ERWARTUNG (mit ADC-Bridge): "adc0=NNN" folgt der Spannung an GP26
 * (0..3,3 V -> 0..1023). Ohne Bridge: konstant ~512.
 *
 * Deckt ab: ADC-CR (SEL/START), sofortige Wandlung im Modell, GDR-Auswertung.
 */
#include "lpc1115.h"

static uint32_t adc_read_ch0(void) {
    /* SEL=Kanal0, CLKDIV=11, START=001 (Bit24). */
    ADC_CR = (1u << 0) | (11u << 8) | (1u << 24);
    while (!(ADC_GDR & ADC_GDR_DONE)) { }
    return (ADC_GDR >> 6) & 0x3FF;       /* 10-Bit-Resultat */
}

int main(void) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_ADC;
    uart_init(39);
    uart_puts("\n[t4] ADC-Kanal0 (GP26 mit adc_bridge_en=1)\n");

    for (;;) {
        uart_puts("adc0=");
        uart_put_u32(adc_read_ch0());
        uart_putc('\n');
        delay_loops(800000);
    }
}
