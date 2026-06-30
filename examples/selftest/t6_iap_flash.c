/*
 * t6_iap_flash.c — Test: IAP-ROM (Prepare/Erase/Copy) + Flash-Rücklesen.
 *
 * Schreibt über die IAP-Routine 256 Byte in Flash-Sektor 14 (0xE000) und
 * liest sie anschließend als Flash-Speicher zurück. Das ist exakt der
 * Mechanismus, über den die sblib ihr EEPROM persistiert.
 *
 * ERWARTUNG: Im Terminal (19200 8N1) erscheint "IAP ok" und die ersten
 * Bytes (0x00 0x01 0x02 ...). Nach einem `reset` (ohne neu zu laden) bleibt
 * der Inhalt erhalten -> Persistenz über den Storage-Slot bestätigt.
 *
 * Deckt ab: IAP-Dispatch (50/52/51), Schreibpfad ins Firmware-Image +
 * Storage-Slot, getrappte Low-Flash-Datenlesungen.
 *
 * Hinweis: Sektor 14 (0xE000) ist bewusst gewählt, um den sblib-EEPROM-
 * Sektor (0xF000) nicht zu berühren.
 */
#include "lpc1115.h"

#define TARGET_ADDR   0xE000u
#define TARGET_SECTOR 14u
#define CCLK_KHZ      12000u

static unsigned int cmd[5];
static unsigned int res[5];
static uint8_t      page[256];

static int iap(unsigned int c0, unsigned int p1, unsigned int p2,
               unsigned int p3, unsigned int p4) {
    cmd[0] = c0; cmd[1] = p1; cmd[2] = p2; cmd[3] = p3; cmd[4] = p4;
    res[0] = 0xFFFFFFFFu;
    IAP_CALL(cmd, res);
    return (int)res[0];          /* 0 = CMD_SUCCESS */
}

int main(void) {
    uart_init(39);
    uart_puts("\n[t6] IAP-Flash-Test\n");

    for (int i = 0; i < 256; ++i) page[i] = (uint8_t)i;

    int rc = 0;
    rc |= iap(IAP_PREPARE,          TARGET_SECTOR, TARGET_SECTOR, 0, 0);
    rc |= iap(IAP_ERASE_SECTOR,     TARGET_SECTOR, TARGET_SECTOR, CCLK_KHZ, 0);
    rc |= iap(IAP_PREPARE,          TARGET_SECTOR, TARGET_SECTOR, 0, 0);
    rc |= iap(IAP_COPY_RAM_TO_FLASH, TARGET_ADDR, (unsigned int)(uintptr_t)page,
              256, CCLK_KHZ);

    if (rc != 0) {
        uart_puts("IAP Fehler rc=");
        uart_put_u32((uint32_t)res[0]);
        uart_putc('\n');
        for (;;) { }
    }

    /* Rücklesen direkt aus dem Flash-Adressraum (wird getrappt/umgeleitet). */
    const volatile uint8_t *fp = (const volatile uint8_t *)TARGET_ADDR;
    uart_puts("IAP ok, bytes:");
    for (int i = 0; i < 8; ++i) { uart_putc(' '); uart_put_hex(fp[i]); }
    uart_putc('\n');

    int good = 1;
    for (int i = 0; i < 256; ++i) if (fp[i] != (uint8_t)i) good = 0;
    uart_puts(good ? "VERIFY ok\n" : "VERIFY FAIL\n");

    for (;;) { }
}
