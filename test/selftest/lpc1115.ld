/*
 * lpc1115.ld — Linker-Skript für LPC1115 (64 KiB Flash @ 0x0, 8 KiB SRAM
 * @ 0x10000000). Passt zum Speicherlayout des RP2350-Emulators
 * (docs/TECHNICAL.md §2).
 *
 * Hinweis: Der RP2350-Emulator prüft KEINE LPC-Boot-Checksumme (Vektor 7).
 * Für echte LPC-Hardware müsste diese nachträglich gesetzt werden.
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
}

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);   /* Stacktop (volle 8 KiB) */

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : AT (_etext)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
