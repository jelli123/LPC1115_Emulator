/*
 * lpc_debug.h - Guest -> Host Debug-Bridge (header-only).
 *
 * Fuer Gast-Firmware, die im LPC1115-Emulator (RP2350) laeuft. Schreibt
 * Debug-Text ueber eine feste MMIO-Adresse an den Emulator-Host, der die
 * Bytes sammelt und ueber die CLI ('dbg') sowie als DEBUG.TXT auf dem
 * USB-MSC-Laufwerk bereitstellt.
 *
 * Einbinden und benutzen:
 *     #include "lpc_debug.h"
 *     dbg_puts("setup done\n");
 *     dbg_kv_u32("systemTime", millis());
 *     dbg_kv_hex("GPIO0", LPC_GPIO0->DATA);
 *
 * Kosten: jedes Zeichen ist EIN MMIO-Byte-Write, der im Emulator trappt und
 * emuliert wird (langsamer als natives RAM, fuer Diagnose aber unkritisch).
 * Auf echter LPC1115-Hardware schreibt es ins Leere (Adresse unbestueckt) —
 * der Code bleibt also portabel.
 *
 * WICHTIG: LPC_DBG_PORT_ADDR MUSS mit debug_bridge::DEBUG_BRIDGE_PORT im
 * Emulator (src/debug_bridge.h) uebereinstimmen.
 */
#ifndef LPC_DEBUG_H
#define LPC_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* MMIO-Datenport: Byte-Schreibzugriff haengt ein Zeichen an. */
#define LPC_DBG_PORT_ADDR 0x4FFF0000u
#define LPC_DBG_DATA (*(volatile unsigned char *)LPC_DBG_PORT_ADDR)

static inline void dbg_putc(char c)
{
    LPC_DBG_DATA = (unsigned char)c;
}

static inline void dbg_puts(const char *s)
{
    if (!s) return;
    while (*s) dbg_putc(*s++);
}

/* Vorzeichenlose Dezimalzahl ausgeben. */
static inline void dbg_u32(unsigned int v)
{
    char tmp[10];
    int i = 0;
    if (v == 0) { dbg_putc('0'); return; }
    while (v && i < (int)sizeof(tmp)) { tmp[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (i > 0) dbg_putc(tmp[--i]);
}

/* 32-bit-Wert als 8-stelliges Hex (0x....) ausgeben. */
static inline void dbg_hex(unsigned int v)
{
    static const char hexd[] = "0123456789abcdef";
    int shift;
    dbg_putc('0'); dbg_putc('x');
    for (shift = 28; shift >= 0; shift -= 4)
        dbg_putc(hexd[(v >> shift) & 0xFu]);
}

/* "name=<dez>\n" */
static inline void dbg_kv_u32(const char *name, unsigned int v)
{
    dbg_puts(name); dbg_putc('='); dbg_u32(v); dbg_putc('\n');
}

/* "name=<hex>\n" */
static inline void dbg_kv_hex(const char *name, unsigned int v)
{
    dbg_puts(name); dbg_putc('='); dbg_hex(v); dbg_putc('\n');
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LPC_DEBUG_H */
