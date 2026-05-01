# LPC1115 Emulator auf RP2350 (RP2354) — Native Execution

Selfbus-SBLib-Firmware (gebaut für NXP LPC1115, Cortex-M0) läuft **nativ
auf einem Cortex-M33-Kern des RP2350** und wird nur dort getrappt, wo
LPC-spezifische Hardware angesprochen wird. Damit erreichen wir nahezu
native Geschwindigkeit; Interpretation findet nicht statt.

## Architektur

```
+-- Core 0 (privilegiert) ------+    +-- Core 1 (Gast) ----------+
| USB-CDC #0  CLI/stdio         |    | unprivileged Thread Mode  |
| USB-CDC #1  GDB Remote (opt.) |    | PSP-Stack                 |
| Storage (Wear-Leveling, CRC)  |    | Native LPC-Firmware in    |
| peripherals::mmio_*           |    | RP2350-SRAM @ 0x20040000  |
| pio_glue (Edge-Capture)       |    +---------------------------+
+-------------------------------+
              ^                          Trap & Emulate
              |
     +-- ARMv8-M MPU ---------------+ ────► BusFault / MemManage
     | RW: 0x20000000-2008_1FFF     |        ↓
     | RW: 0xE000_0000-E000_E0FF    |        decoded LDR/STR
     | TRAP: 0xE000_E100-E000_E4FF  |        ↓
     | RW: 0xE000_E500-E00F_FFFF    |        peripherals::mmio_* / vnvic
     | TRAP: 0x4000_0000-4FFF_FFFF  |
     +------------------------------+
```

## Wesentliche Eigenschaften

* **Cortex-M33 ist befehlssatzkompatibel** zu Cortex-M0 (ARMv6-M Thumb).
  ALU/Branch/Stack/MUL laufen 1:1 nativ.
* **MPU-Isolation:** Gast (unprivileged) sieht nur Gast-RAM und PPB
  (außer NVIC). Jeder Zugriff auf `0x40000000-0x4FFFFFFF` und auf NVIC
  trapt → wird vom Handler decodet und an
  [peripherals::mmio_*](src/peripherals.cpp) bzw.
  [vnvic](src/vnvic.cpp) weitergereicht.
* **Trap-Decoder:** schmaler Decoder
  in [opcodes.cpp](src/opcodes.cpp) (~150 Zeilen), erkennt LDR/STR-Familie.

## RAM-Adress-Relocation (zwei Wege)

Die LPC1115 hat ihren SRAM bei `0x10000000`. Auf dem RP2350 liegt dort
das XIP. Wir mappen Gast-RAM auf `0x20060000`. Damit absolute SRAM-Pointer
stimmen, gibt es zwei Wege:

### A) Re-Linken (sauber)

Mit dem mitgelieferten Linker-Skript
[examples/guest_lpc1115.ld](examples/guest_lpc1115.ld):

* FLASH bei `0x20040000`
* RAM   bei `0x20060000`

### B) HEX-Patcher (kein Re-Link nötig)

Beim Laden patcht [hex_patcher.cpp](src/hex_patcher.cpp) **alle 4-byte-
aligned Wörter** im Image, deren Wert im Bereich
`[0x10000000, 0x10002000)` liegt, auf `0x20060000 + (val − 0x10000000)`.

* Geeignet für GCC/Keil-erzeugte Literal-Pools (`LDR Rx, =sym`).
* False-Positive-Wahrscheinlichkeit: ~ 8 KB / 2³² ≈ **1/524 288 pro Wort**.
  Bei 64 KB Firmware (16 384 Wörter) erwartet man ~0,03 falsche
  Treffer. Praktisch vernachlässigbar — bei Auffälligkeiten Variante A
  nutzen.
* Vector-Tabellen-Initial-SP wird separat im Loader behandelt
  ([emulator.cpp](src/emulator.cpp)).

## Original-Geschwindigkeit / PLL

Der Gast schreibt seine PLL-Konfiguration in `LPC SYSCON`
(`0x40048000`). Das **wird vom MPU-Trap erfasst**: Jede Schreibanforderung
auf `SYSPLLCTRL`/`SYSPLLCLKSEL`/`MAINCLKSEL`/`SYSAHBCLKDIV` läuft durch
[peripherals::mmio_write8](src/peripherals.cpp). Sobald ein vollständiges
32-bit-Wort beisammen ist, berechnet der Handler die Soll-Frequenz nach
`F_OUT = F_CLKIN × (MSEL+1)` und ruft `set_sys_clock_khz()` auf — der
RP2350 läuft anschließend mit dieser Frequenz.

Vorteil gegenüber HEX-Heuristik: kein Pattern-Matching auf Code; die
echten Werte werden zur Laufzeit übernommen.

## NVIC / IRQs

NVIC-Register (`0xE000_E100 … 0xE000_E4FF`) sind aus der MPU-Whitelist
**ausgenommen**. Schreibvorgänge des Gastes auf `ISER`/`ICER`/`ISPR`/
`ICPR`/`IPR` landen über [vnvic](src/vnvic.cpp) in einem Schatten-NVIC.

## Automatische IRQ-Injektion

Peripherie-Modelle in [peripherals.cpp](src/peripherals.cpp) rufen bei
Events `irq_inject::pend(lpc_irq::UART0)` etc. auf — der LPC-IRQ-Index
ist genau das Bit im Schatten-NVIC. Wenn das zugehörige ISER-Bit gesetzt
ist, setzt [irq_inject.cpp](src/irq_inject.cpp) `SCB->ICSR.PENDSVSET`.
Der naked PendSV-Handler synthetisiert auf dem PSP-Stack des Gastes
einen **echten Cortex-M-Exception-Frame**:

* `return_pc` = Eintrag aus der Gast-Vector-Tabelle bei `vector[16+IRQ]`
* `LR`        = `0xFFFFFFFD` (EXC_RETURN: Thread + PSP, kein FP)
* `xPSR`      = `0x01000000` (Thumb-Bit + ggf. 8-Byte-Padding)

Beim Exception-Return springt der Core in den Original-LPC-Handler. Der
Gast bemerkt nicht, dass die Exception nicht aus echter NVIC-Hardware
kommt. Das funktioniert **automatisch für jeden LPC-IRQ-Index** (0..31)
— der Handler-Pointer wird live aus der Gast-Vector-Tabelle geholt.

Mapping LPC-Peripherie → IRQ-Index ist zentral in
[lpc_irqs.h](src/lpc_irqs.h) (Quelle: UM10398 Tab. 51) hinterlegt.

## Modellierte LPC-Peripherie

| Peripherie       | Status      | IRQ        | Anmerkung                       |
|------------------|-------------|------------|---------------------------------|
| GPIO0            | nutzbar     | -          | Mapping → RP2350-GPIO über `pin`|
| SYSCON / PLL     | nutzbar     | -          | retargeted RP2350-PLL           |
| SysTick          | nativ       | (SysTick)  | M0/M33 register-kompatibel      |
| UART0 (16550)    | nutzbar     | UART0 (21) | Backend = RP2350 `uart0`        |
| CT16B0/CT16B1    | nutzbar     | 16/17      | Match-IRQ via irq_inject        |
| CT32B0/CT32B1    | nutzbar     | 18/19      | Match-IRQ via irq_inject        |
| IOCON            | passive     | -          | nur Schatten-RAM                |
| SSP0/SSP1, I²C   | TODO        | 20/14/15   | Modellgerüst über CT-Pattern    |
| ADC, WWDT, BOD   | TODO        | 24/25/26   |                                 |
| GINT0/GINT1, PINT| TODO        | 8/9/0..7   |                                 |

## GDB-Remote-Stub (statt CMSIS-DAP)

> **Klare Ansage:** Echtes CMSIS-DAP geht hier nicht — DAP setzt einen
> *externen* SWD-Target voraus, wir wollen aber den Gast debuggen, der
> als Code im selben M33 läuft. Funktional gleichwertig, deutlich
> einfacher: ein **GDB Remote Serial Protocol Stub** über einen zweiten
> USB-CDC-Endpoint.

* Implementierung: [gdb_stub.cpp](src/gdb_stub.cpp).
* USB: zweiter CDC-Port via [usb_descriptors.cpp](src/usb_descriptors.cpp)
  und [tusb_config.h](src/tusb_config.h) (`CFG_TUD_CDC = 2`).
* Aktivierung in der CLI: `gdb on` / `gdb off` / `gdb status`.
* Software-Breakpoints: `Z0`/`z0` ersetzt 16-bit-Instruktion durch `BKPT`,
  Hit landet im UsageFault-Handler → `gdb_stub::on_breakpoint()`,
  spinnt bis `c`/`s` von GDB kommt.
* GDB-Anbindung (Beispiel):

  ```
  arm-none-eabi-gdb fw.elf
  (gdb) target remote \\.\COMx     # bzw. /dev/ttyACM1
  ```

  Für VSCode-`cortex-debug` als `servertype: external` mit
  `gdbTarget: "/dev/ttyACM1"` (Linux) bzw. `\\.\COM<n>` (Windows).
* Single-Step nutzt **DEMCR.MON_EN | MON_STEP** — der Cortex-M33 löst
  nach genau einer Instruktion DebugMonitor aus, der wieder im Stub
  landet. Funktioniert sauber für 16- *und* 32-bit-Thumb (`BL`,
  `LDR.W`, ...). Voraussetzung: kein DAP angehängt (`DHCSR.C_DEBUGEN=0`).

## Externer Debugger über SWD-Pins

Mit `swd start <swdio-gpio> <swclk-gpio>` wird auf zwei externen
RP2350-Pins ein **ADIv5 SWD-Target** exponiert. Damit sieht jedes
Standard-Werkzeug (J-Link, ST-Link, OpenOCD-CMSIS-DAP, Keil) den
Emulator als echten LPC1115:

| Konstante                  | Wert         | Quelle               |
|----------------------------|--------------|----------------------|
| DPIDR                      | `0x0BB11477` | LPC1115 Cortex-M0 DAP|
| AHB-AP IDR                 | `0x04770031` | ARM Cortex-M0 AHB-AP |
| CoreSight-ROM-Table-Adresse| `0xE00FF000` | LPC1115              |
| Part-ID (PIDR0)            | `0x411`      | LPC1115              |

**Pinbelegung:** `SWCLK = SWDIO + 1` (RP2350-GPIO-Adjacency, weil das
PIO-Programm `wait pin 1` mit relativer Indizierung benutzt). Beispiel:
`swd start 6 7` ⇒ SWDIO=GP6, SWCLK=GP7.

**Funktionsumfang (vollständig):**

* SWD line reset (50× '1'), JTAG-to-SWD-Selektion (`0xE79E`)
* 8-bit Header-Framing inkl. Parity, ACK (OK/WAIT/FAULT), Turnaround
* DP: `DPIDR`, `CTRL/STAT`, `SELECT`, `RDBUFF`, `ABORT`
* AHB-AP[0]: `CSW`, `TAR`, `DRW`, `IDR` mit Single-/Packed-Auto-Increment
* Memory-AP-Reads/Writes auf Gast-Adressraum mit `0x10000000`→
  `0x20060000`-Mapping
* DHCSR/DCRSR/DCRDR/DEMCR werden auf
  [target_halt](src/target_halt.cpp) abgebildet → `C_HALT`/`C_STEP`/
  `C_DEBUGEN` lösen echten Stopp/Step im Gast aus, Register-Zugriff
  über DCRSR/DCRDR liest/schreibt im gestackten Frame
* CoreSight-ROM-Table mit korrekten LPC1115-PIDR/CIDR-Werten
* FPB (`FP_CTRL`, `FP_COMP[0..7]`) → BKPT-Insertion via
  `target_halt::set_breakpoint()`

**Bewusste Auslassungen** (im Code dokumentiert):

* Multi-Drop-SWD (mehrere Targets an einem Bus, `TARGETSEL`)
* JTAG-Mode (nur SWD)
* Dormant-State-Wakeup-Sequenz
* Banked APs außer AHB-AP[0]

PHY-Hinweis: Die Implementation nutzt PIO-RX (Sample auf SWCLK-Rising)
plus CPU-getriebene SWDIO-Output-Phasen (`wait_swclk_falling()`-Polling).
Das funktioniert robust bis ~2 MHz SWCLK; OpenOCD `adapter speed 1000`
ist passend. Höhere Geschwindigkeiten brauchen ein vollständig
PIO-getriebenes TX (Folge-Inkrement).

**Verwendung mit OpenOCD/GDB:**

```
# OpenOCD verbindet via CMSIS-DAP-Probe an den RP2350-Pins
openocd -f interface/cmsis-dap.cfg -f target/lpc11xx.cfg
arm-none-eabi-gdb fw.elf -ex "target extended-remote :3333"
```

## PIO Edge-Capture

[pio_glue.cpp](src/pio_glue.cpp) lädt ein vollständiges PIO-Programm:

```
.program edge_capture
    .wrap_target
        mov   x, !null            ; X = 0xFFFFFFFF
    loop:
        jmp   pin   capture       ; Pin == 1?
        jmp   x--   loop
    capture:
        mov   isr, x
        push  noblock
    .wrap
```

→ FIFO erhält `0xFFFFFFFF − ticks_until_edge`. Das wird in
`pio_glue::capture_read()` invertiert und kann von einem
LPC-CT16/CT32-Capture-Modell als Zeitstempel verwendet werden.

## Build

```powershell
cmake -S . -B build -G Ninja
cmake --build build
```

Wenn `PICO_SDK_PATH` nicht gesetzt ist, wird das SDK 2.1.0 per
FetchContent gezogen. `emulator.uf2` per BOOTSEL aufs Board kopieren.

## CLI

| Befehl                        | Wirkung                                       |
|-------------------------------|-----------------------------------------------|
| `help`                        | Hilfe                                         |
| `status`                      | State, MMIO-/GPIO-/PLL-/NVIC-Stats, GDB-Status|
| `config get/set/save/dump`    | persistierte Konfiguration                    |
| `pin set <lpc> <rp\|-1>`      | Pin-Mapping setzen                            |
| `pin show`                    | aktuelles Pin-Mapping                         |
| `freq <Hz>`                   | nur Konfig (RP2350-Takt folgt der Gast-PLL)   |
| `flash erase`                 | Firmware-Slot löschen                         |
| `flash hex`                   | Intel-Hex-Upload                              |
| `flash finalize <bytes>`      | CRC-Marker setzen                             |
| `run`                         | Gast starten (Core 1)                         |
| `stop` / `reset`              | Core 1 abschießen + neu booten                |
| `gdb on/off/status`           | GDB-Stub auf USB-CDC #1                       |
| `swd start <dio> <clk>`       | SWD-Target auf RP-GPIOs (clk = dio+1)         |
| `swd stop` / `swd status`     | SWD-Target stoppen / Zustand                  |

## Sicherheits-Eigenschaften

* MPU isoliert den Gast: er kann **nicht** in Host-Speicher schreiben
  (XIP, Konfiguration, Storage, Host-Stack).
* `MPU.PRIVDEFENA = 1` für Host → Host-Code sieht weiterhin den vollen
  Speicher; Gast (unprivileged) **nur** explizit erlaubte Regionen.
* Privileg wechselt beim Sprung in den Gast — Gast kann sich nicht
  selbst re-privilegieren, ohne durch unsere Handler zu laufen.
* Fault-Handler clearn `CFSR`-Sticky-Bits explizit.
* CRC32 schützt Konfig-Snapshot und Firmware-Image.
* Watchdog-Reset bei nicht-decodierbaren Faults und HardFaults.

## Bekannte Grenzen / TODO

* **Peripherie-Modelle**: GPIO0, SYSCON, UART0, CT16B0/B1, CT32B0/B1
  drin. SSP/I²C/ADC/WWDT/PINT/GINT folgen dem CT-Pattern (siehe
  [peripherals.cpp](src/peripherals.cpp)).
* **SWD-PHY** ist polling-basiert auf der TX-Seite — robust bis ~2 MHz
  SWCLK. Voll-PIO-TX für höhere Geschwindigkeiten ist ein optionales
  Folge-Inkrement.
* **set_sys_clock_khz** akzeptiert nicht jede beliebige Frequenz; bei
  Ablehnung bleibt der RP2350 auf der vorherigen Frequenz. Auf der
  Konsole erscheint dann eine Meldung.
* **IRQ-Injektion** geht davon aus, dass der Gast in Thread Mode auf
  PSP läuft und Pico-eigene Host-IRQs (USB, Timer) niedrigere Priorität
  als PendSV nicht haben. PendSV läuft auf Priorität 0xFF (niedrigste),
  damit andere Host-IRQs nicht versehentlich verschoben werden.
