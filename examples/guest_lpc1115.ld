/*
 * Linker-Skript für LPC1115-SBLib-Firmware, die im LPC1115-Emulator auf
 * dem RP2350 nativ ausgeführt werden soll.
 *
 * Wichtig: Die Firmware MUSS mit diesem Skript gelinkt werden, sonst
 * landen absolute Datenpointer (z. B. nach 0x10000000 für Variablen)
 * außerhalb des MPU-konfigurierten Gast-Speichers, was zu nicht
 * decodierbaren Faults führt.
 *
 * FLASH bei 0x20040000 (LPC_LOAD_BASE in src/emulator.h)
 * RAM   bei 0x20060000 (LPC_GUEST_RAM_BASE)
 *
 * Maximale Größen: 64 KiB Flash, 8 KiB RAM (entsprechen LPC1115FBD48/302).
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x20040000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20060000, LENGTH = 8K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .isr_vector : ALIGN(128) {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : ALIGN(4) {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);

    .bss : ALIGN(4) {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Stack belegt das Top-Ende des RAM, vom CRT verwendet. */
    .stack : ALIGN(8) {
        . = . + 1024;
        _estack = .;
    } > RAM
}
