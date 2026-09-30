MEMORY { FLASH (rx) : ORIGIN = 0, LENGTH = 64K  RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 8K }
SECTIONS {
  .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) } > FLASH
  .bss (NOLOAD) : { *(.bss*) *(COMMON) } > RAM
}
