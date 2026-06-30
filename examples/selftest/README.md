# Selbsttest-Programme (Bare-Metal LPC1115)

Kleine, eigenständige Testfirmware zum Prüfen der vom RP2350-Emulator
modellierten LPC1115-Peripherie. Kein Vendor-SDK nötig — nur die
arm-none-eabi-Toolchain.

Die zugehörige Schritt-für-Schritt-Prüfliste steht in
[../docs/TESTPLAN.md](../docs/TESTPLAN.md).

## Bauen

**Windows (PowerShell):**

```powershell
cd examples\selftest
.\build.ps1                 # alle Tests -> *.hex
.\build.ps1 t3_timer_irq    # nur einen
```

`build.ps1` nutzt automatisch die Toolchain der Pico-VS-Code-Erweiterung
(`~/.pico-sdk/toolchain/14_2_Rel1/bin`) oder den PATH.

**Linux/macOS:**

```bash
cd examples/selftest
make
```

Ergebnis ist je Programm eine `*.hex` (Intel-HEX), die per `upload`,
`xmodem` oder als `BOOT.HEX` (USB-Volume) in den Emulator geladen wird.
Vorgebaute `*.hex` liegen bereits im Ordner.

## Gemeinsame Dateien

| Datei                 | Zweck                                                  |
|-----------------------|--------------------------------------------------------|
| `lpc1115.h`           | Minimaler Registersatz + UART-/GPIO-Helfer             |
| `startup_lpc1115.c`   | Vektortabelle (inkl. 32 IRQ-Slots) + C-Runtime-Init    |
| `lpc1115.ld`          | Linker-Skript (64 KiB Flash @0, 8 KiB RAM @0x10000000) |
| `build.ps1` / `Makefile` | Build-Skripte                                       |

## Testprogramme

| HEX                 | Prüft                          | Beobachtung (Erwartung)                                   |
|---------------------|--------------------------------|-----------------------------------------------------------|
| `t1_blink.hex`      | GPIO-Ausgang                   | P0_7 (→GP9) blinkt ~2 Hz                                   |
| `t2_uart_echo.hex`  | UART0 TX/RX                     | Begrüßung + Echo (Großbuchstaben) auf uart0, 19200 8N1    |
| `t3_timer_irq.hex`  | CT16B0-Match-IRQ + NVIC        | GP9 blinkt 1 Hz, `tick=N` zählt im Terminal hoch          |
| `t4_adc_uart.hex`   | ADC                            | `adc0=NNN` folgt GP26 (mit `adc_bridge_en=1`)             |
| `t5_gpio_input.hex` | GPIO-Eingang (echter Pin)      | GP3-Pegel wird auf GP9 gespiegelt, `in=0/1` im Terminal   |
| `t6_iap_flash.hex`  | IAP (Prepare/Erase/Copy)       | `IAP ok` + `VERIFY ok`; bleibt über `reset` erhalten      |

### Wichtige Hinweise

* **Pinmap:** Die genannten RP2350-GPIOs entsprechen der Default-Pinmap
  (`cli: pinmap show`). Bei eigener Pinmap die Zielpins entsprechend anpassen.
* **UART-Baudrate:** Der Emulator startet mit 12 MHz IRC. Mit Divisor 39
  ergeben sich ~19230 Baud → Terminal auf **19200 8N1** stellen. Dieser
  UART hängt an der **RP2350-uart0** (Default GP0=TX/GP1=RX), **nicht** an
  der USB-CLI (CDC#0).
* **ADC-Bridge:** `t4` liefert nur mit aktiver ADC-Bridge echte Werte:
  `cfg set adc_bridge_en 1` + `cfg save`, dann `reset`.
* Diese Programme sind für den **Emulator** gedacht und enthalten **keine**
  LPC-Boot-Checksumme (Vektor 7). Für echte LPC-Hardware müsste diese
  ergänzt werden.
