# NCN5130-Selbsttest (Gast-Firmware)

Bare-Metal-Testprogramm für den LPC1115-Gast: spricht den virtuellen NCN5130 an SSP0
an und gibt die empfangenen Bytes über die Debug-Bridge aus (CLI `dbg`).

```
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -nostdlib -ffreestanding -T link.ld ncntest.c -o ncntest.elf
arm-none-eabi-objcopy -O ihex ncntest.elf ncntest.hex
```

Emulator: `ncn on 0`, HEX laden (USB-Laufwerk, `upload` oder ISP), dann `dbg`.
Die erwarteten Bytes stehen in den Testbezeichnungen (Datenblatt Fig. 44–55).
Mit `ncn loopback on` erscheint jedes gesendete Frame zusätzlich als Empfangsframe.
