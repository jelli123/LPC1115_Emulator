/*
 * startup_lpc1115.c — Reset-Vektor, Cortex-M0-Vektortabelle und
 * C-Runtime-Init (.data kopieren, .bss nullen) für alle Selbsttests.
 *
 * Die Vektortabelle enthält die System-Exceptions plus 32 IRQ-Slots
 * (LPC1115). Jedes Testprogramm definiert nur die Handler, die es braucht;
 * alle anderen sind als `weak` an Default_Handler gebunden.
 *
 * Der RP2350-Emulator injiziert IRQs, indem er den Handler-Pointer live aus
 * vector[16 + IRQn] liest (siehe docs/TECHNICAL.md §6). Die Tabelle muss
 * daher korrekt befüllt sein.
 */
#include <stdint.h>

extern unsigned int _etext;     /* Ende .text (= LMA von .data)   */
extern unsigned int _sdata;     /* Start .data (RAM)              */
extern unsigned int _edata;     /* Ende  .data (RAM)              */
extern unsigned int _sbss;      /* Start .bss                    */
extern unsigned int _ebss;      /* Ende  .bss                    */
extern unsigned int _estack;    /* Initialer Stackpointer        */

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Schwache Aliase – jedes Testprogramm überschreibt nach Bedarf. */
#define WEAK __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void)        WEAK;
void HardFault_Handler(void)  WEAK;
void SVC_Handler(void)        WEAK;
void PendSV_Handler(void)     WEAK;
void SysTick_Handler(void)    WEAK;

/* LPC1115-IRQ-Handler 0..31. */
void PIN_INT0_Handler(void)   WEAK;  void PIN_INT1_Handler(void)  WEAK;
void PIN_INT2_Handler(void)   WEAK;  void PIN_INT3_Handler(void)  WEAK;
void PIN_INT4_Handler(void)   WEAK;  void PIN_INT5_Handler(void)  WEAK;
void PIN_INT6_Handler(void)   WEAK;  void PIN_INT7_Handler(void)  WEAK;
void GINT0_Handler(void)      WEAK;  void GINT1_Handler(void)     WEAK;
void SSP1_Handler(void)       WEAK;  void I2C0_Handler(void)      WEAK;
void CT16B0_Handler(void)     WEAK;  void CT16B1_Handler(void)    WEAK;
void CT32B0_Handler(void)     WEAK;  void CT32B1_Handler(void)    WEAK;
void SSP0_Handler(void)       WEAK;  void UART0_Handler(void)     WEAK;
void ADC_Handler(void)        WEAK;  void WWDT_Handler(void)      WEAK;
void BOD_Handler(void)        WEAK;

/* Vektortabelle nach 0x00000000. */
__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))(&_estack),  /* 0: Initial-SP                 */
    Reset_Handler,               /* 1: Reset                      */
    NMI_Handler,                 /* 2                             */
    HardFault_Handler,           /* 3                             */
    0, 0, 0, 0, 0, 0, 0,         /* 4..10 (inkl. Checksum-Slot 7) */
    SVC_Handler,                 /* 11                            */
    0, 0,                        /* 12,13                         */
    PendSV_Handler,              /* 14                            */
    SysTick_Handler,             /* 15                            */
    /* ---- IRQ 0..31 (Vektorindex 16..47) ---- */
    PIN_INT0_Handler, PIN_INT1_Handler, PIN_INT2_Handler, PIN_INT3_Handler,
    PIN_INT4_Handler, PIN_INT5_Handler, PIN_INT6_Handler, PIN_INT7_Handler,
    GINT0_Handler,    GINT1_Handler,    0,                0,
    0,                0,                SSP1_Handler,     I2C0_Handler,
    CT16B0_Handler,   CT16B1_Handler,   CT32B0_Handler,   CT32B1_Handler,
    SSP0_Handler,     UART0_Handler,    0,                0,
    ADC_Handler,      WWDT_Handler,     BOD_Handler,      0,
    0,                0,                0,                0
};

void Reset_Handler(void) {
    /* .data aus Flash (LMA) ins RAM kopieren. */
    unsigned int *src = &_etext;
    unsigned int *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    /* .bss nullen. */
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0u;

    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }
