/*
 * t7_uart_diag.c — Gezielte UART0-Diagnose fuer den FT12-serial.begin()-Hang.
 *
 * HINTERGRUND: FT12 haengt im Emulator in sblib serial.begin() in der finalen
 * RX-Drain-Schleife `while (LSR & RDR) val = RBR;`, weil die emulierte LPC-UART0
 * (auf echte RP2350-uart0 an GP0/GP1 gebrueckt) DAUERHAFT "Empfangsdaten bereit"
 * (LSR.DR) meldet — auch OHNE angeschlossenen Adapter. Dieses Programm grenzt
 * ein, OB das ein Emulator-Bug (uart0 faelschlich readable) oder echte HW ist.
 *
 * ABLAUF (Ausgabe ueber Debug-Bridge 0x4FFF0000 -> CLI 'dbg' / DEBUG.TXT):
 *   1. UART0 mit 8E1 @ ~19200 initialisieren (wie FT12: LCR=0x1B).
 *   2. LSR 16x lesen OHNE zu senden -> zeigt, ob DR faelschlich dauergesetzt ist.
 *      (Auf echter Leitung idle-high MUESSTE DR=0 sein.)
 *   3. Falls DR bereits gesetzt: bis zu 32 RBR-Bytes ausdrainen + hex melden
 *      (zeigt, was der uart0 "empfaengt" — z. B. 0x00 bei idle-low = floating).
 *   4. Ein Byte 'U' (0x55) senden. MIT Bruecke GP0<->GP1 (Loopback) sollte es
 *      auf RX zurueckkommen.
 *   5. Begrenzt (nicht endlos!) auf RX pollen; empfangenes Byte melden.
 *
 * KEIN endloser Drain -> das Programm selbst haengt nie; es meldet immer ein
 * Ergebnis, egal wie sich uart0 verhaelt.
 *
 * CONFIG: uart0_tx=0, uart0_rx=1 (GP0/GP1). Optional Bruecke GP0<->GP1 fuer
 * den Loopback-Teil.
 */
#include "lpc1115.h"

/* --- Debug-Bridge (Emulator) --------------------------------------------- */
#define DBG (*(volatile unsigned char *)0x4FFF0000u)
static void dbg_puts(const char *s) { while (*s) DBG = (unsigned char)*s++; }
static void dbg_hex8(unsigned char b) {
    static const char hx[] = "0123456789ABCDEF";
    DBG = (unsigned char)hx[(b >> 4) & 0xF];
    DBG = (unsigned char)hx[b & 0xF];
}

/* UART0 mit 8E1 initialisieren (wie sblib serial.begin(19200, SERIAL_8E1)).
 * LCR 8E1 = 8 Datenbits (0x03) | PEN (0x08) | EPS (0x10) = 0x1B. */
static void uart_init_8e1(uint16_t divisor) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_UART;
    UART0_LCR = 0x80;                 /* DLAB = 1 */
    UART0_DLL = divisor & 0xFF;
    UART0_DLM = (divisor >> 8) & 0xFF;
    UART0_LCR = 0x1B;                 /* 8E1, DLAB = 0 */
    UART0_FCR = 0x07;                 /* FIFO an + RX/TX-Reset */
}

int main(void) {
    dbg_puts("\n[t7] UART0-Diag start\n");

    uart_init_8e1(39);               /* ~19230 Baud @ 12 MHz */
    dbg_puts("[t7] uart_init_8e1 ok, LCR=0x1B\n");

    /* --- Schritt 2: LSR 16x lesen OHNE Senden ---------------------------- */
    dbg_puts("[t7] LSR x16 (ohne TX):");
    unsigned dr_count = 0;
    for (int i = 0; i < 16; ++i) {
        uint32_t lsr = UART0_LSR;
        DBG = ' ';
        dbg_hex8((unsigned char)(lsr & 0xFF));
        if (lsr & UART0_LSR_RDR) ++dr_count;
    }
    DBG = '\n';
    dbg_puts("[t7] DR-gesetzt in ");
    dbg_hex8((unsigned char)dr_count);
    dbg_puts(" von 16 Reads\n");

    /* --- Schritt 3: Falls DR gesetzt, bis zu 32 Bytes ausdrainen --------- */
    if (dr_count) {
        dbg_puts("[t7] RBR-Drain (max 32):");
        for (int i = 0; i < 32; ++i) {
            if (!(UART0_LSR & UART0_LSR_RDR)) break;
            DBG = ' ';
            dbg_hex8((unsigned char)(UART0_RBR & 0xFF));
        }
        DBG = '\n';
        dbg_puts("[t7] -> uart0 meldet OHNE TX Daten = idle-Pegel/Emulator, NICHT echte Bytes\n");
    } else {
        dbg_puts("[t7] LSR.DR sauber 0 -> RX idle korrekt\n");
    }

    /* --- Schritt 4: 'U' senden (Loopback-Test mit GP0<->GP1-Bruecke) ----- */
    dbg_puts("[t7] sende 'U' (0x55)\n");
    /* THRE abwarten, aber begrenzt (kein Endlos-Hang) */
    for (uint32_t g = 0; g < 200000u; ++g)
        if (UART0_LSR & UART0_LSR_THRE) break;
    UART0_THR = 0x55;

    /* --- Schritt 5: begrenzt auf RX pollen ------------------------------- */
    int got = -1;
    for (uint32_t g = 0; g < 2000000u; ++g) {
        if (UART0_LSR & UART0_LSR_RDR) { got = (int)(UART0_RBR & 0xFF); break; }
    }
    if (got >= 0) {
        dbg_puts("[t7] RX empfangen: ");
        dbg_hex8((unsigned char)got);
        if (got == 0x55) dbg_puts("  == 'U' -> LOOPBACK OK (GP0<->GP1 verbunden)\n");
        else             dbg_puts("  != 'U' -> abweichend (Framing/Bruecke/Emulator)\n");
    } else {
        dbg_puts("[t7] RX nichts empfangen (keine Bruecke / TX->RX getrennt)\n");
    }

    dbg_puts("[t7] fertig\n");
    for (;;) { }
}
