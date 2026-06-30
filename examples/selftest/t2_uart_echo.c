/*
 * t2_uart_echo.c — Test: UART0 senden + empfangen.
 *
 * Sendet beim Start eine Begrüßung und spiegelt danach jedes empfangene
 * Zeichen zurück (Echo). UART0 ist im Emulator an die echte RP2350-uart0
 * gebrückt (Default-Pins GP0=TX, GP1=RX).
 *
 * ERWARTUNG: Terminal an RP2350-uart0 (USB-TTL-Adapter), 19200 8N1.
 * Beim Reset erscheint die Begrüßung; Tastatureingaben werden als
 * Großbuchstaben zurückgeschickt.
 *
 * Deckt ab: UART-Init (Divisor/LCR/FCR), TX (THR/LSR), RX (RBR/LSR),
 * Baudraten-Retargeting im Emulator.
 *
 * Baudrate-Hinweis: Emulator-Default 12 MHz IRC, Divisor 39 => ~19230 Baud.
 */
#include "lpc1115.h"

int main(void) {
    uart_init(39);                 /* ~19200 Baud bei 12 MHz */
    uart_puts("\n[t2] UART-Echo bereit. Tippe etwas:\n");

    for (;;) {
        int c = uart_getc_nonblock();
        if (c < 0) continue;
        if (c >= 'a' && c <= 'z') c -= 32;   /* zu Großbuchstabe */
        uart_putc((char)c);
        if (c == '\r') uart_putc('\n');
    }
}
