/*
 * lpc1115.h — Minimaler Bare-Metal-Registersatz für die Selbsttest-Programme.
 *
 * Deckt nur die Peripherie ab, die der RP2350-Emulator modelliert
 * (siehe docs/TECHNICAL.md §7). Bewusst schlank gehalten – kein CMSIS,
 * keine Vendor-Header nötig.
 *
 * Wichtig (Emulator-Eigenheiten):
 *  - UART-Baudrate = Kernfrequenz / (16 * Divisor). Per Default läuft der
 *    Emulator mit 12 MHz IRC; mit Divisor 39 ergeben sich ~19230 Baud.
 *    => Terminal auf 19200 8N1 einstellen.
 *  - GPIO ist die LPC11xx-Legacy-Variante mit maskiertem DATA-Zugriff.
 *  - LPC-Pins werden über die Pinmap des Emulators auf echte RP2350-GPIOs
 *    abgebildet (cli: `pinmap show`).
 */
#ifndef LPC1115_SELFTEST_H
#define LPC1115_SELFTEST_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* ---- SYSCON @ 0x40048000 ------------------------------------------------ */
#define SYSCON_BASE        0x40048000u
#define SYSAHBCLKCTRL      REG32(SYSCON_BASE + 0x080)
#define SYSAHBCLKCTRL_GPIO   (1u << 6)
#define SYSAHBCLKCTRL_CT16B0 (1u << 7)
#define SYSAHBCLKCTRL_CT16B1 (1u << 8)
#define SYSAHBCLKCTRL_CT32B0 (1u << 9)
#define SYSAHBCLKCTRL_CT32B1 (1u << 10)
#define SYSAHBCLKCTRL_SSP0   (1u << 11)
#define SYSAHBCLKCTRL_UART   (1u << 12)
#define SYSAHBCLKCTRL_ADC    (1u << 13)
#define SYSAHBCLKCTRL_IOCON  (1u << 16)
#define SYSAHBCLKCTRL_I2C    (1u << 5)

/* ---- GPIO (LPC11xx-Legacy) @ 0x50000000 + port*0x10000 ----------------- */
#define GPIO_PORT(p)        (0x50000000u + (uint32_t)(p) * 0x10000u)
/* Maskierter Schreib-/Lesezugriff: Adress-Bits[11:2] = Pin-Maske.          */
#define GPIO_MASKED(p, msk) REG32(GPIO_PORT(p) + ((uint32_t)(msk) << 2))
/* Alle Pins (Maske 0xFFF) lesen/schreiben.                                 */
#define GPIO_DATA(p)        REG32(GPIO_PORT(p) + 0x3FFCu)
#define GPIO_DIR(p)         REG32(GPIO_PORT(p) + 0x8000u)

static inline void gpio_set_output(int port, int pin) {
    GPIO_DIR(port) |= (1u << pin);
}
static inline void gpio_set_input(int port, int pin) {
    GPIO_DIR(port) &= ~(1u << pin);
}
static inline void gpio_write(int port, int pin, int level) {
    /* Maskiertes Schreiben: nur das adressierte Bit wird verändert. */
    GPIO_MASKED(port, (1u << pin)) = level ? (1u << pin) : 0u;
}
static inline int gpio_read(int port, int pin) {
    return (GPIO_DATA(port) >> pin) & 1u;
}

/* ---- UART0 (16550) @ 0x40008000 ---------------------------------------- */
#define UART0_BASE          0x40008000u
#define UART0_RBR           REG32(UART0_BASE + 0x000)  /* DLAB=0, R   */
#define UART0_THR           REG32(UART0_BASE + 0x000)  /* DLAB=0, W   */
#define UART0_DLL           REG32(UART0_BASE + 0x000)  /* DLAB=1      */
#define UART0_DLM           REG32(UART0_BASE + 0x004)  /* DLAB=1      */
#define UART0_IER           REG32(UART0_BASE + 0x004)  /* DLAB=0      */
#define UART0_FCR           REG32(UART0_BASE + 0x008)  /* W           */
#define UART0_LCR           REG32(UART0_BASE + 0x00C)
#define UART0_LSR           REG32(UART0_BASE + 0x014)
#define UART0_LSR_RDR       (1u << 0)   /* Empfangsdaten bereit       */
#define UART0_LSR_THRE      (1u << 5)   /* Sendepuffer frei           */

/* Divisor 39 @ 12 MHz IRC ≈ 19230 Baud (Terminal: 19200 8N1). */
static inline void uart_init(uint16_t divisor) {
    SYSAHBCLKCTRL |= SYSAHBCLKCTRL_UART;
    UART0_LCR = 0x80;                 /* DLAB = 1 */
    UART0_DLL = divisor & 0xFF;
    UART0_DLM = (divisor >> 8) & 0xFF;
    UART0_LCR = 0x03;                 /* 8N1, DLAB = 0 */
    UART0_FCR = 0x07;                 /* FIFO an + Reset */
}
static inline void uart_putc(char c) {
    while (!(UART0_LSR & UART0_LSR_THRE)) { }
    UART0_THR = (uint8_t)c;
}
static inline void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}
static inline int uart_getc_nonblock(void) {
    if (UART0_LSR & UART0_LSR_RDR) return (int)(UART0_RBR & 0xFF);
    return -1;
}
static inline void uart_put_u32(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i--] = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v && i >= 0) { buf[i--] = (char)('0' + (v % 10)); v /= 10; }
    uart_puts(&buf[i + 1]);
}
static inline void uart_put_hex(uint32_t v) {
    static const char hx[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int s = 28; s >= 0; s -= 4) uart_putc(hx[(v >> s) & 0xF]);
}

/* ---- CT16B0/CT16B1/CT32B0/CT32B1 @ 0x4000C000 +n*0x4000 ---------------- */
#define CT16B0_BASE         0x4000C000u
#define CT16B1_BASE         0x40010000u
#define CT32B0_BASE         0x40014000u
#define CT32B1_BASE         0x40018000u
#define CTx_IR(base)        REG32((base) + 0x00)  /* W1C der Match-Flags */
#define CTx_TCR(base)       REG32((base) + 0x04)  /* Bit0=Enable Bit1=Reset */
#define CTx_TC(base)        REG32((base) + 0x08)
#define CTx_PR(base)        REG32((base) + 0x0C)  /* Prescale */
#define CTx_MCR(base)       REG32((base) + 0x14)  /* je Kanal 3 Bit: I/R/S */
#define CTx_MR0(base)       REG32((base) + 0x18)
#define CTx_MR1(base)       REG32((base) + 0x1C)
#define CTx_EMR(base)       REG32((base) + 0x3C)
#define CTx_PWMC(base)      REG32((base) + 0x74)

/* ---- ADC @ 0x4001C000 -------------------------------------------------- */
#define ADC_BASE            0x4001C000u
#define ADC_CR              REG32(ADC_BASE + 0x000)
#define ADC_GDR             REG32(ADC_BASE + 0x004)
#define ADC_DR(ch)          REG32(ADC_BASE + 0x010 + (ch) * 4)
#define ADC_GDR_DONE        (1u << 31)

/* ---- NVIC (ISER0 @ 0xE000E100) ----------------------------------------- */
#define NVIC_ISER0          REG32(0xE000E100u)
#define NVIC_ICER0          REG32(0xE000E180u)
static inline void nvic_enable(int irq) { NVIC_ISER0 = (1u << irq); }

/* LPC1115-IRQ-Nummern (= NVIC-Bit, = Vektorindex 16+n). */
enum {
    IRQ_CT16B0 = 16, IRQ_CT16B1 = 17, IRQ_CT32B0 = 18, IRQ_CT32B1 = 19,
    IRQ_UART0  = 21, IRQ_ADC    = 24, IRQ_WWDT   = 25
};

/* ---- IAP-ROM-Entry ----------------------------------------------------- */
#define IAP_LOCATION        0x1FFF1FF1u
typedef void (*iap_fn)(unsigned int *cmd, unsigned int *res);
#define IAP_CALL            ((iap_fn)IAP_LOCATION)
enum {
    IAP_PREPARE = 50, IAP_COPY_RAM_TO_FLASH = 51, IAP_ERASE_SECTOR = 52,
    IAP_BLANK_CHECK = 53, IAP_READ_PARTID = 54, IAP_READ_UID = 58
};

static inline void delay_loops(volatile uint32_t n) { while (n--) { } }

#endif /* LPC1115_SELFTEST_H */
