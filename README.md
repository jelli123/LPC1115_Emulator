# LPC1115 Emulator auf RP2350 (RP2354) — Native Execution

Selfbus-SBLib-Firmware (gebaut für NXP LPC1115, Cortex-M0) läuft **nativ
auf einem Cortex-M33-Kern des RP2350** und wird nur dort getrappt, wo
LPC-spezifische Hardware angesprochen wird. Damit erreichen wir nahezu
native Geschwindigkeit; Interpretation findet nicht statt.

## Architektur

```
+-- Core 0 (privilegiert) ------+    +-- Core 1 (Gast) ----------+
| USB-CDC CLI/GDB/Serial (dyn.) |    | unprivileged Thread Mode  |
| USB-MSC-Laufwerk (immer)      |    | PSP-Stack                 |
| Storage (Wear-Leveling, CRC)  |    | Native LPC-Firmware in    |
| peripherals::mmio_*           |    | RP2350-SRAM (aligned Array)|
| pio_glue (Edge-Capture)       |    +---------------------------+
+-------------------------------+
              ^                          Trap & Emulate
              |
     +-- ARMv8-M MPU ---------------+ ────► BusFault / MemManage
     | RW: Gast-Image (64K) + RAM   |        ↓
     | RX: enter_guest-Trampolin    |        decoded LDR/STR
     | RW: 0xE000_0000-E000_E0FF    |        ↓
     | TRAP:0xE000_E100-E000_E4FF   |        peripherals::mmio_* / vnvic
     | RO:  0xE000_ED00-E000_ED1F   |  ◄─ AIRCR-Trap (nur Gast-Reset)
     | RW: uebriger PPB-Bereich     |
     | (uebriges SRAM: gesperrt)    |
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

## RAM-Adress-Relocation

Die LPC1115 hat ihren SRAM bei `0x10000000`. Auf dem RP2350 liegt dort
das XIP. Der Gast-RAM wird deshalb als aligned Array im RP2350-SRAM
allokiert; die Laufzeit-Basis liefert `emulator::guest_ram_base()` (keine
feste Adresse mehr). Damit absolute SRAM-Pointer stimmen, greifen zwei
sich ergaenzende Mechanismen:

### A) HEX-Patcher (Load-Time, schnell)

Beim Laden patcht [hex_patcher.cpp](src/hex_patcher.cpp) **alle 4-byte-
aligned Wörter** im Image, deren Wert im Bereich
`[0x10000000, 0x10002000]` liegt (obere Grenze **inklusive**, deckt
„one-past-end"-Zeiger wie `_estack`/`_ebss` ab), auf
`guest_ram_base() + (val − 0x10000000)`.

* Geeignet für GCC/Keil-erzeugte Literal-Pools (`LDR Rx, =sym`).
* False-Positive-Wahrscheinlichkeit: ~ 8 KB / 2³² ≈ **1/524 288 pro Wort**.
  Bei 64 KB Firmware (16 384 Wörter) erwartet man ~0,03 falsche
  Treffer. Praktisch vernachlässigbar.
* Vector-Tabellen-Initial-SP wird separat im Loader behandelt
  ([emulator.cpp](src/emulator.cpp)).

### B) Fault-Backstop (Run-Time, robust)

Verpasst die Heuristik einen RAM-Pointer (z. B. zur Laufzeit *berechnete*
Adressen), faengt der MemManage-Handler ([fault.cpp](src/fault.cpp)) den
Zugriff auf `[0x10000000, +8 KB)` ab und bedient ihn direkt auf dem
Gast-RAM. Ein verpasster Heuristik-Treffer ist damit **nicht fatal**,
sondern nur etwas langsamer (ein Trap pro Zugriff) — kein Re-Linken nötig.

## Original-Geschwindigkeit / PLL

Der Gast schreibt seine PLL-Konfiguration in `LPC SYSCON`
(`0x40048000`). Das **wird vom MPU-Trap erfasst**: Jede Schreibanforderung
auf `SYSPLLCTRL`/`SYSPLLCLKSEL`/`MAINCLKSEL`/`SYSAHBCLKDIV` läuft durch
[peripherals::mmio_write8](src/peripherals.cpp). Sobald ein vollständiges
32-bit-Wort beisammen ist, berechnet der Handler die Soll-Frequenz nach
`F_OUT = F_CLKIN × (MSEL+1)` und übernimmt sie als **Zeitbasis der emulierten
Peripherie** (Timer-Tick-Skalierung, UART-Baud). Der **reale** RP2350-Takt
bleibt bei 150 MHz (SDK-Default) und wird bewusst **nicht** umgeschaltet:

* Die emulierten Timer skalieren reale Wall-Clock-Zeit (`time_us_64()`) mit
  der Soll-Frequenz — der Silizium-Takt ist für die Genauigkeit irrelevant.
* Trap-and-Emulate braucht die volle 150-MHz-Reserve; ein Herunterregeln
  würde den Emulator selbst ausbremsen.
* Ein `set_sys_clock_khz()` von Core 1 aus würde zudem den USB-/CLI-Betrieb
  auf Core 0 stören.

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

Weil der injizierte Handler im **Thread-Mode** (mit `LR=0xFFFFFFFD`) läuft,
ist sein regulärer Rücksprung (`POP {pc}`/`BX LR`) dort keine gültige
Exception-Rückkehr — der Core faultet beim Anspringen von `0xFFFFFFFC`. Dieser
Fall wird im Fault-Handler ([fault.cpp](src/fault.cpp) `try_injected_irq_return`)
abgefangen: er legt den darunter liegenden Original-Frame frei und setzt den
unterbrochenen Gast-Code nahtlos fort.

Mapping LPC-Peripherie → IRQ-Index ist zentral in
[lpc_irqs.h](src/lpc_irqs.h) (Quelle: UM10398 Tab. 51) hinterlegt.

## Modellierte LPC-Peripherie

| Peripherie       | Status      | IRQ        | Anmerkung                       |
|------------------|-------------|------------|---------------------------------|
| GPIO0            | nutzbar     | -          | Mapping → RP2350-GPIO über `pin`; Eingänge lesen echte Pin-Pegel |
| SYSCON / PLL     | nutzbar     | -          | Soll-Takt als Zeitbasis; RP2350 bleibt 150 MHz |
| SysTick          | nativ       | (SysTick)  | M0/M33 register-kompatibel      |
| UART0 (16550)    | nutzbar     | UART0 (21) | Backend = RP2350 `uart0`        |
| CT16B0/CT16B1    | nutzbar     | 16/17      | Match-IRQ via irq_inject        |
| CT32B0/CT32B1    | nutzbar     | 18/19      | Match-IRQ via irq_inject        |
| IOCON            | passive     | -          | nur Schatten-RAM, kein Pinmux-Effekt |
| SSP0/SSP1        | nutzbar     | 20/14      | Loopback-Modell (RX=TX), RX-IRQ |
| I²C0             | nutzbar¹    | 15         | Bridge auf RP2350-HW-I²C (`i2c on`), sonst Stub |
| ADC              | nutzbar     | 24         | sofortige Wandlung, ADC-IRQ     |
| WWDT             | nutzbar     | 25         | echtes Timeout, Soft-Reset des Guests |
| BOD              | TODO        | 26         | nicht modelliert                |
| PINT (PIN_INT0..7)| nutzbar²   | 0..7       | PINTSEL + Edge/Level, IRQ via irq_inject |
| GINT0/GINT1      | nutzbar²    | 8/9        | Gruppen-Match (AND/OR), IRQ via irq_inject |

¹ Aktivierung per CLI `i2c on <inst> <sda> <scl> [hz]` (wirkt nach Reset).
  Ohne Bridge bleibt das alte Stub-Verhalten. Schreib-Adress-ACK wird
  optimistisch gemeldet; lazy Byte-Reads passen zu Auto-Increment-Slaves.
² Pin-Interrupts werden **synchron** beim nächsten MMIO-Trap des Guests
  abgetastet (`sample_pin_interrupts()` aus `mmio_read8/write8`). Ein
  Programm, das ausschließlich in `WFI` ohne jeden MMIO-Zugriff wartet,
  kann dadurch nicht aufgeweckt werden — Folge des nativen Core-1-Modells.

## GDB-Remote-Stub (statt CMSIS-DAP)

> **Klare Ansage:** Echtes CMSIS-DAP geht hier nicht — DAP setzt einen
> *externen* SWD-Target voraus, wir wollen aber den Gast debuggen, der
> als Code im selben M33 läuft. Funktional gleichwertig, deutlich
> einfacher: ein **GDB Remote Serial Protocol Stub** über einen zweiten
> USB-CDC-Endpoint.

* Implementierung: [gdb_stub.cpp](src/gdb_stub.cpp).
* USB: eigener CDC-Port via [usb_descriptors.cpp](src/usb_descriptors.cpp)
  und [tusb_config.h](src/tusb_config.h) (`CFG_TUD_CDC = 3`). Der
  Konfigurations-Deskriptor wird zur Bootzeit **dynamisch** aus der Config
  gebaut: `cli_enable` / `gdb_enable` / `serial_enable` (CONFIG.INI, Default
  `on`) bestimmen, welche CDCs am USB erscheinen. Deaktivierte CDCs fallen
  komplett weg (ein COM-Port weniger); das MSC-Laufwerk ist immer aktiv.
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

Wenn `PICO_SDK_PATH` nicht gesetzt ist, wird das SDK per FetchContent
gezogen. In dieser Arbeitskopie wird das von der Pico-VS-Code-Erweiterung
verwaltete **SDK 2.2.0** benutzt; das Build-Artefakt ist `build/emulator.uf2`
und wird per BOOTSEL aufs Board kopiert.

## Zweistufiger Boot: Bootloader + Applikation (auto-Descriptor)

Selfbus-Geräte bestehen aus zwei Teilen: einem **Bootloader** ab
`0x0000` und der **Applikation** ab einer höheren Flash-Adresse (Selfbus-
Default `0x3000`). Der Bootloader prüft vor dem Sprung einen
**App-Descriptor** (Start-/End-Adresse + CRC32) in einem Flash-Block kurz
vor der App (Default `app_start − 0x100`).

Der Emulator unterstützt diesen Ablauf direkt:

* **Mergendes Laden:** `upload`/`xmodem`/`BOOT.HEX` schreiben **additiv** in
  den 64-KiB-Slot (Sektor-Read-Modify-Write). Eine zweite Datei (z. B. die
  App) überschreibt den zuvor geladenen Bootloader **nicht**. Vollständiges
  Löschen nur explizit über `erase` (CLI) bzw. `flash_erase=on` (CONFIG.INI).
  Nach dem Flashen einer per USB-Volume abgelegten HEX wird diese \u2014 wie beim
  RP2350-UF2-Bootloader \u2014 automatisch vom Laufwerk entfernt und das Medium
  neu eingeh\u00e4ngt (CONFIG.INI/HELP.HTM/DEBUG.TXT bleiben erhalten).
* **Auto-Descriptor:** Erkennt der Loader beim Start eine gültige
  Applikation ab `app_start` (plausibler Initial-SP im LPC-RAM + Thumb-
  Reset-Vektor im Flash), synthetisiert er bei `autodesc=on` einen
  korrekten Selfbus-Descriptor bei `desc_addr` und protokolliert das auf der
  CLI (`[EMU] Boot-Descriptor erzeugt …`). Die CRC ist bit-identisch zur
  Selfbus-Bootloader-Routine; die App-Bytes bleiben dabei unangetastet
  (pristine), damit Descriptor- und Laufzeit-CRC übereinstimmen.
* **Konfigurierbar** (CLI `cfg set …` oder CONFIG.INI):

  | Key         | Default            | Bedeutung                                  |
  |-------------|--------------------|--------------------------------------------|
  | `app_start` | `0x3000`           | Flash-Adresse der Applikation              |
  | `desc_addr` | `0` (= auto)       | Descriptor-Adresse; `0` ⇒ `app_start−0x100`|
  | `autodesc`  | `on`               | Descriptor automatisch erzeugen            |

  `app_start` muss der `applicationFirstAddress()` des verwendeten
  Bootloaders entsprechen.

Alternativ zum kombinierten Laden kann die App auch per **OTA über den
KNX-Bus** in den emulierten Bootloader geschrieben werden (IAP-Pfad).

## CLI

| Befehl                        | Wirkung                                       |
|-------------------------------|-----------------------------------------------|
| `help`                        | Hilfe                                         |
| `status`                      | State, MMIO-/GPIO-/PLL-/NVIC-Stats, GDB-Status|
| `config get/set/save/dump`    | persistierte Konfiguration                    |
| `pin set <lpc> <rp\|-1>`      | Pin-Mapping setzen                            |
| `pin show`                    | aktuelles Pin-Mapping                         |
| `freq <Hz>`                   | nur Konfig (RP2350-Takt folgt der Gast-PLL)   |
| `upload` / `flash hex`        | Intel-Hex-Upload (**additiv/mergend**)        |
| `xmodem`                      | Intel-Hex per XMODEM-CRC/1K empfangen         |
| `erase` / `flash erase`       | Firmware-Slot komplett löschen                |
| `flash finalize <bytes>`      | CRC-Marker setzen                             |
| `run`                         | Gast starten (Core 1)                         |
| `stop` / `reset`              | Core 1 abschießen + neu booten                |
| `gdb on/off/status`           | GDB-Stub auf USB-CDC #1                       |
| `swd start <dio> <clk>`       | SWD-Target auf RP-GPIOs (clk = dio+1)         |
| `swd stop` / `swd status`     | SWD-Target stoppen / Zustand                  |
| `i2c on <inst> <sda> <scl> [hz]` | I²C-Bridge auf RP2350-HW (wirkt nach Reset)|
| `i2c off` / `i2c status`      | I²C-Bridge deaktivieren / Zustand             |

## Sicherheits-Eigenschaften

* MPU isoliert den Gast: er kann **nicht** in Host-Speicher schreiben
  (XIP, Konfiguration, Storage, Host-Stack). Dem unprivilegierten Gast sind
  **nur** die eigenen SRAM-Bereiche freigegeben (Code-Image 64 KiB, Gast-RAM
  8 KiB, das `enter_guest`-Trampolin) plus PPB — der übrige RP2350-SRAM
  (TinyUSB-Puffer, Core-Stacks) bleibt gesperrt. Ein wilder Pointer/Überlauf
  wird so zu einem sauberen Trap statt zu stiller Korruption.
* Der SCB-Kontrollblock (`0xE000ED00-0xED1F`) ist für den Gast **read-only**:
  ein `AIRCR.SYSRESETREQ` (`NVIC_SystemReset()`) trapt und startet nur den
  **Gast** neu — nicht das echte RP2350-Silizium.
* `MPU.PRIVDEFENA = 1` für Host → Host-Code sieht weiterhin den vollen
  Speicher; Gast (unprivileged) **nur** explizit erlaubte Regionen.
* Privileg wechselt beim Sprung in den Gast — Gast kann sich nicht
  selbst re-privilegieren, ohne durch unsere Handler zu laufen.

## Fehlerbehandlung & Diagnose

Ein nicht emulierbarer Gast-Fehler führt **nicht** zu einem Board-Reset,
sondern hält den Gast an (`State=Faulted`, LED flackert 8 Hz). Core 0 (USB/CLI)
läuft weiter. Der komplette Fault-Bericht (Typ, `CFSR/HFSR` im Klartext,
Register, gefehlerte Instruktion) erscheint

* seriell auf CDC#0 als `[FAULT] …`-Block **und**
* als Datei `FAULT.TXT` auf dem USB-Laufwerk (Analyse ohne CLI möglich).

Das Laufwerk enthält zudem ab Werk `HELP.HTM` (Kurzanleitung im Browser) und
eine aus dem aktuellen Zustand generierte `CONFIG.INI` (aktive Einstellungen +
auskommentierte Optionen). Die Feld- und Abkürzungs-Erklärungen stehen in
[docs/USERGUIDE.md](docs/USERGUIDE.md#3a-diagnose-stats-faulttxt-und-stacktrace).
* Fault-Handler clearn `CFSR`-Sticky-Bits explizit.
* CRC32 schützt Konfig-Snapshot und Firmware-Image.
* Watchdog-Reset bei nicht-decodierbaren Faults und HardFaults.

## Bekannte Grenzen / TODO

* **Peripherie-Modelle**: GPIO0 (inkl. Eingangspegel), SYSCON, UART0,
  CT16B0/B1, CT32B0/B1, SSP0/SSP1, I²C0 (HW-Bridge), ADC, WWDT sowie
  PINT (PIN_INT0..7) und GINT0/GINT1 sind modelliert (siehe
  [peripherals.cpp](src/peripherals.cpp)). Noch offen: BOD.
* **Pin-Interrupts (PINT/GINT)** werden nur abgetastet, wenn der Gast
  einen MMIO-Zugriff auslöst (synchrones Core-1-Modell). Ein reiner
  `WFI`-Wartepunkt ohne MMIO kann nicht durch Pin-IRQs geweckt werden.
* **I²C-Bridge**: Schreib-Transaktionen werden gepuffert und beim STOP/
  Repeated-START geflusht; das Adress-ACK wird währenddessen optimistisch
  gemeldet. Lese-Transaktionen lesen lazy Byte-für-Byte (passt zu
  Auto-Increment-Slaves). Nicht hardwarevalidiert in dieser Session.
* **Unbekannte MMIO-Adressen** sind nicht mehr fatal: Schreibzugriffe
  landen in einem generischen Schatten-RAM (kein Watchdog-Reset mehr),
  Rücklesen bleibt konsistent.
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
