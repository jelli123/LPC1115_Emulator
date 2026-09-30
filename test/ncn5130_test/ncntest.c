/* Bare-Metal-Test des virtuellen NCN5130 am LPC1115-SSP0 (fuer den Emulator).
   Ausgabe ueber die Debug-Bridge (0x4FFF0000) -> CLI 'dbg'. */
#include <stdint.h>
#define REG(a) (*(volatile uint32_t*)(a))
#define DBG    (*(volatile uint8_t*)0x4FFF0000u)
#define SSP0_CR0 REG(0x40040000u)
#define SSP0_CR1 REG(0x40040004u)
#define SSP0_DR  REG(0x40040008u)
#define SSP0_SR  REG(0x4004000Cu)
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)

static void puts_(const char* s) { while (*s) DBG = (uint8_t)*s++; }
static void hex8(uint8_t v) { const char* h = "0123456789ABCDEF"; DBG = h[v >> 4]; DBG = h[v & 15]; }

/* grobe Wartezeit ueber SysTick-Zaehler (12 MHz IRC, Emulator skaliert) */
static void delay_ms(uint32_t ms) {
    SYST_RVR = 12000 - 1; SYST_CVR = 0; SYST_CSR = 5;
    while (ms--) { while (!(SYST_CSR & 0x10000u)) { } }
    SYST_CSR = 0;
}

static uint8_t rx[256]; static int nrx;
static void drain(uint32_t ms) {            /* alle vom NCN getakteten Bytes einsammeln */
    for (uint32_t t = 0; t <= ms; ++t) {
        for (int k = 0; k < 64 && (SSP0_SR & 0x04u); ++k) { uint8_t b = (uint8_t)SSP0_DR; if (nrx < 256) rx[nrx++] = b; }
        if (t < ms) delay_ms(1);
    }
}
static void xfer(uint8_t b) {               /* Host->NCN (TREQ-Transfer); Dummy-RX verwerfen */
    SSP0_DR = b;
    while (SSP0_SR & 0x04u) { uint8_t d = (uint8_t)SSP0_DR; (void)d; break; }
}
static void report(const char* name) {
    puts_(name); puts_(" n="); hex8((uint8_t)nrx); puts_(":");
    if (nrx > 24) nrx = 24;
    for (int i = 0; i < nrx; ++i) { DBG = ' '; hex8(rx[i]); }
    puts_("\n"); nrx = 0;
}
static void send_frame(const uint8_t* f, int n, int bad_cs) {   /* n ohne Checksumme */
    uint8_t cs = 0xFF; for (int i = 0; i < n; ++i) cs ^= f[i];
    if (bad_cs) cs ^= 0x55;
    xfer(0x80); xfer(f[0]);
    for (int i = n - 1; i >= 1; --i) { xfer((uint8_t)(0x80 | i)); xfer(f[i]); }   /* umgekehrte Reihenfolge */
    xfer((uint8_t)(0x40 | n)); xfer(cs);
}

void Reset_Handler(void) {
    SSP0_CR0 = 0x07;                        /* 8 Bit, SPI, Mode 0 */
    SSP0_CR1 = 0x06;                        /* SSE + MS (Slave) */
    puts_("NCNTEST start SR="); hex8((uint8_t)SSP0_SR); puts_("\n");
    drain(5);                 report("T0 unaufgefordert (alt)");
    xfer(0x01); drain(5);     report("T0b U_Reset (03)");
    xfer(0x02); drain(5);     report("T1 U_State (00 07)");
    xfer(0x0D); drain(5);     report("T2 U_SystemState (00 4B FB)");
    xfer(0x19); drain(5);     report("T3 U_Configure MARKER (00 05)");
    xfer(0xF1); xfer(0x11); xfer(0x05); xfer(0x00); drain(5);
                              report("T4 U_SetAddress (00 00 00 00 25)");
    xfer(0xF1); xfer(0x11); xfer(0x06); xfer(0x00); drain(5);
                              report("T5 U_SetAddress 2x (00.. keine ind)");
    static const uint8_t fr[] = { 0xBC, 0x11, 0x05, 0x00, 0x01, 0xE1, 0x00, 0x81 };
    send_frame(fr, 8, 0); drain(40);
                              report("T6 Senden (Echo BC..cs, 13, 8B, CB)");
    static const uint8_t fcb[] = { 0xBC, 0x11, 0x05, 0x00, 0x01, 0xE1, 0x00, 0xCB };
    send_frame(fcb, 8, 0); drain(40);
                              report("T7 Senden mit 0xCB (CB verdoppelt)");
    xfer(0x1A); drain(5);     report("T7b U_Configure CRC (0D)");
    send_frame(fr, 8, 0); drain(40);
                              report("T7c Senden mit CRC (Echo, CRC lo hi, 13, 8B, CB)");
    send_frame(fr, 8, 1); drain(40);
                              report("T8 falsche Checksumme (47)");
    xfer(0x0E); drain(5);     report("T9 StopMode (2B)");
    xfer(0x0F); drain(5);     report("T10 ExitStop (03)");
    xfer(0x28); xfer(0x5A); xfer(0x38); drain(5);
                              report("T11 IntReg wr/rd (5A)");
    puts_("NCNTEST ende\n");
    for (;;) { }
}

__attribute__((section(".vectors"), used))
const void* vectors[48] = { (void*)0x10001FF0u, (void*)Reset_Handler };
