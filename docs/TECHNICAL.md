# LPC1115-Emulator – Technische Beschreibung

Implementierungs-Doku zum Emulator. Bedienung steht in [USERGUIDE.md](USERGUIDE.md).

---

## 1. Architektur in einem Satz

> Die LPC1115-Firmware läuft **nativ** und unprivilegiert auf RP2350-Core 1.
> Eine ARMv8-M-MPU fängt jeden Zugriff auf nicht-vorhandene Bereiche ab.
> Der Host (Core 0 + Privilegierter-Mode) modelliert die LPC1115-Peripherie.

Das ist ein klassischer **Trap-and-Emulate-Hypervisor** ohne Interpreter.

---

## 2. Speicher-Layout

| Region                   | Guest-Sicht (LPC)        | Host-Realität (RP2350)        |
|--------------------------|--------------------------|--------------------------------|
| Code-Flash               | `0x00000000-0x0000FFFF`  | RAM-Image @ `0x20040000`       |
| RAM                      | `0x10000000-0x10001FFF`  | RAM @ `0x20060000`             |
| APB-Peripherie           | `0x40000000-0x4007FFFF`  | **MPU-Trap** → `peripherals::` |
| AHB-GPIO                 | `0x50000000-0x5003FFFF`  | **MPU-Trap** → `peripherals::` |
| PPB / SCB / SysTick      | `0xE000E000-0xE000EFFF`  | **echt** (Cortex-M33)          |
| NVIC                     | `0xE000E100-0xE000E4FF`  | **MPU-Trap** → `vnvic::`       |
| IAP-ROM-Stub             | `0x1FFF1000-0x1FFF1FFF`  | RAM-Stub mit `BKPT`-Trampolin  |

Der Reset-Vector aus dem geladenen Image wird beim `run` in einen
synthetischen Initial-Frame auf der Guest-PSP geschrieben; danach erfolgt
ein `BX` mit `EXC_RETURN = 0xFFFFFFFD` (Thread, PSP, unprivilegiert).

---

## 3. MPU-Konfiguration

ARMv8-M MPU mit acht Regionen:

| #  | Bereich                  | Attribute                        |
|----|--------------------------|----------------------------------|
| 0  | Host-Code (XIP)          | RX, privilegiert + ungelegt      |
| 1  | Host-Stack/RAM           | RW, privilegiert                 |
| 2  | Guest-Code-Image         | RX, alle                         |
| 3  | Guest-RAM                | RW + XN, alle                    |
| 4  | LPC-Peripherie (APB)     | **kein Zugriff** für Guest       |
| 5  | LPC-GPIO                 | **kein Zugriff** für Guest       |
| 6  | NVIC-Range               | **kein Zugriff** für Guest       |
| 7  | IAP-ROM-Stub             | RX, alle                         |

`PRIVDEFENA = 1` lässt den Host (Core 0, privilegiert) alles sehen –
sonst nur, was Region 0/1 erlaubt.

---

## 4. Trap-Pfad

```
guest LDR/STR  ->  MemManage-Fault  ->  fault.cpp:isr_memmanage
                                           |
                                           v
                                    opcodes.cpp:decode()    (LDR/STR/STM/LDM)
                                           |
                                           v
                          peripherals::mmio_read/write[8|16|32]
                                           |
                                           v
                                  Guest-Frame patchen, PC += 2|4
                                           |
                                           v
                                       BX LR (return)
```

Die naked Wrapper in [src/fault.cpp](../src/fault.cpp) leiten den
Stack-Frame an den C-Handler. Decoder unterstützt:

* T1/T2 LDR/STR (immediate, register, PC-relativ)
* LDR/STR/B/H/SB/SH
* LDM/STM (Burst, mit Auto-Increment)
* LDREX/STREX werden als Single-Word emuliert

---

## 5. IAP-ROM-Emulation

LPC1115-IAP-Aufruf:

```c
typedef void (*IAP)(unsigned param[5], unsigned result[5]);
const IAP iap_entry = (IAP)0x1FFF1FF1;
```

Der Emulator legt einen **RAM-Stub** bei `0x1FFF1FF0` ab:

```
0x1FFF1FF0:  BE AB           BKPT #0xAB
0x1FFF1FF2:  47 70           BX   LR
```

`fault.cpp:isr_hardfault` erkennt das Pattern und ruft
`iap::dispatch(r0_param, r1_result)`. Unterstützte Kommandos
(UM10398 Tab. 411):

| Cmd | Name              | Mapping im Emulator                     |
|-----|-------------------|------------------------------------------|
| 50  | Prepare sectors   | No-op, Result = `CMD_SUCCESS`            |
| 51  | Copy RAM→Flash    | Schreibt in Code-Image **und** Storage   |
| 52  | Erase sectors     | Memset 0xFF auf Image + Storage-Sektor   |
| 53  | Blank check       | Vergleich gegen 0xFF                     |
| 54  | Read Part-ID      | Liefert `0x00050080` (LPC1115/303)       |
| 55  | Read Boot-Version | Liefert `0x00010000`                     |
| 56  | Compare           | Memcmp                                   |
| 57  | Reinvoke ISP      | Ignoriert (Result = `CMD_SUCCESS`)       |
| 58  | Read UID          | Hash aus RP2350-UID                      |
| 59  | Erase page        | Wie 52, 256-Byte-granular                |

Damit funktioniert die **Selfbus-EEPROM-Emulation** (`Eeprom::commit()`)
out-of-the-box.

---

## 6. NVIC-Virtualisierung (`vnvic`)

Das echte Cortex-M33-NVIC darf der Guest **nicht** sehen, weil:

* IRQ-Nummern unterscheiden sich (LPC1115 hat 32, RP2350 hat 52).
* Der Guest würde Host-Interrupts anschalten.

`vnvic` schattet `ISER/ICER/ISPR/ICPR/IPR` für 32 virtuelle IRQs.
`peripherals::` setzt bei Modell-Events `vnvic::set_pending(N)`. Das
löst eine **PendSV** auf Core 1 aus.

`irq_inject.cpp:pendsv_inject_c()` synthetisiert daraufhin einen
Cortex-M-Exception-Frame auf der Guest-PSP, lädt `PC` aus
`vector_table[16+N]`, setzt `LR = EXC_RETURN`, und kehrt zurück. Der
Guest landet im richtigen ISR.

IRQ-Tabelle: [src/lpc_irqs.h](../src/lpc_irqs.h) (UM10398 Tab. 51).

---

## 7. Peripherie-Modelle

| Block           | Status | Hinweise                                      |
|-----------------|--------|-----------------------------------------------|
| SYSCON / PLL    | ✅     | `MSEL`-Schreiben → `set_sys_clock_khz()`      |
| GPIO0..GPIO3    | ✅     | echte RP-GPIOs via `pin_map`, masked DATA     |
| IOCON           | ✅     | RAM-Schatten, kein Effekt (RP-Pinmux ist los) |
| UART0 (16550)   | ✅     | Backend = RP2350 `uart0`                      |
| CT16B0/B1       | ✅     | Match-IRQ-Injection                           |
| CT32B0/B1       | ✅     | Match-IRQ-Injection                           |
| SysTick         | ✅     | echt (Cortex-M33-PPB)                         |
| WDT (WWDT)      | ✅     | echtes Timeout, **soft-reset nur des Guests** |
| ADC             | ✅     | sofortige Wandlung, deterministisches Sample  |
| SSP0/SSP1       | ✅     | Loopback-Modell (RX=TX)                       |
| I²C0            | ✅     | Stub (NAK auf Adress-Send) — ohne externen Slave |
| PMU/PCON        | ✅     | siehe [§13 Energiemanagement](#13-energiemanagement) |
| RTC             | ❌     | LPC1115 hat keinen RTC-Block (entfällt)       |

PLL-Modell: `F_OUT = F_CLKIN × (M+1)`, gültig 156-320 MHz CCO.
Re-Targeting wird in einem **Post-Hook** nach dem letzten Byte des
SYSCON-Word-Schreibens ausgeführt, damit kein 4-faches Re-Targeting
entsteht.

---

## 8. GDB-Remote-Stub

Auf CDC#1, RSP-Subset:

* `g` / `G` (Register), `m` / `M` (Memory)
* `c` / `s` (Continue / Step), `?` (Stop-Reason)
* `Z0/z0` (SW-Breakpoint, max. 8 Slots, BKPT-Patching)
* `Z2/z2` (Write-Watchpoint via DWT)
* `qSupported`, `qXfer:features:read:target.xml` (Cortex-M0 Layout)
* `vCont`, `vRun`, `D` (Detach)

**Single-Step** ist über `DEMCR.MON_STEP` realisiert: Statt
DHCSR-Halting wird die DebugMonitor-Exception (`isr_debugmon` in
[src/fault.cpp](../src/fault.cpp)) genutzt, sodass das System
weiterläuft.

---

## 9. SWD-Target (externer Probe)

[src/swd_target.cpp](../src/swd_target.cpp) implementiert einen
**vollständigen ADIv5-Target**:

| Schicht         | Inhalt                                                 |
|-----------------|--------------------------------------------------------|
| PHY (PIO-RX)    | 3-Instr-Programm: `wait 1 pin1 / in pins,1 / wait 0 pin1` |
| PHY (CPU-TX)    | `gpio_get(SWCLK)`-Polling – robust bis ~2 MHz          |
| Wire-Protokoll  | Line-Reset (50 × 1), JTAG→SWD (`0xE79E`), Header+Parität, ACK, Turnaround |
| DP              | DPIDR (`0x0BB11477`), CTRL/STAT, SELECT, RDBUFF, ABORT |
| AHB-AP[0]       | CSW, TAR, DRW, IDR (`0x04770031`); SGL/PCK Auto-Inc    |
| Bridge          | `target_halt::*` für Memory + Register                 |
| DCB             | DHCSR (DBGKEY-validiert), DCRSR/DCRDR, DEMCR, FP_CTRL/COMP |
| ROM-Table       | @ `0xE00FF000`, PIDR0=`0x411` (LPC1115), CIDR class 1  |

**Pin-Constraint**: `SWCLK == SWDIO + 1`, weil das PIO-Programm
`wait pin 1` relativ adressiert.

**Cooperative Halt**: `target_halt.cpp` löst per `SCB->ICSR.PENDSVSET`
eine Halt-Anforderung aus. PendSV erfasst den Guest-Frame
(`stmia/ldmia r4-r11`), spinnt bis `request_resume()`, schreibt
zurück. So wirkt der externe Debugger zwischen zwei Guest-Instruktionen.

---

## 10. Aufruf-Diagramm Boot

```
main.cpp (Core 0, privilegiert)
  +-- pico_stdio_init
  +-- tinyusb init  (CDC#0 + CDC#1)
  +-- storage::init
  +-- config::load
  +-- mmu::init           (MPU 8 Regionen)
  +-- vnvic::init
  +-- peripherals::init
  +-- emulator::load_image_from_storage
  +-- iap::install_stub
  +-- target_halt::init
  +-- swd_target::init
  +-- gdb_stub::init
  +-- multicore_launch_core1(emulator::core1_entry)
  +-- cli::run            (forever)

emulator::core1_entry
  +-- mmu::drop_to_unprivileged   (MSP -> PSP, CONTROL.nPRIV=1)
  +-- "BX" Reset-Vector des Guest-Image
```

---

## 11. Build-Layout

```
CMakeLists.txt                    # FetchContent pico-sdk 2.1.0
linker.ld                         # Reservierte Slots am Flash-Ende
src/
  main.cpp        cli.cpp         # Core 0
  emulator.cpp                    # Core 1 entry, image loader
  mmu.cpp         vnvic.cpp       # MPU + NVIC-Schatten
  fault.cpp       opcodes.cpp     # Trap-and-Emulate
  peripherals.cpp                 # SYSCON/GPIO/UART/CT16/CT32
  irq_inject.cpp  lpc_irqs.h      # PendSV-IRQ-Injection
  iap.cpp         iap.h           # IAP-ROM-Stub-Dispatcher
  target_halt.cpp                 # Halt-Bridge für SWD/GDB
  swd_target.cpp                  # ADIv5-Target
  gdb_stub.cpp                    # RSP-Server auf CDC#1
  pio_glue.cpp                    # Edge-Capture-PIO
  hex_parser.cpp  hex_patcher.cpp # Intel-HEX + Relokation
  storage.cpp                     # Flash-WL
  config.cpp                      # KV-Konfig
  usb_descriptors.cpp tusb_config.h
docs/
  USERGUIDE.md  TECHNICAL.md
```

---

## 12. Bewusste Auslassungen

* **Multi-Drop-SWD**, **JTAG**, **SWD-Dormant-State**, banked APs.
* RTC (auf der LPC1115 nicht vorhanden).
* I²C als reiner Stub (NAK ohne externen Slave).
* SSP nur als Loopback (kein echtes RP-SPI-Backend).
* **LittleFS** für Storage – aktuell ist nur einfaches Round-Robin-WL.

---

## 13. Energiemanagement

LPC1115 → RP2350-Mapping, soweit das RP2350 vergleichbare Modi anbietet:

| LPC-Modus               | LPC-Trigger                       | RP2350-Mapping                                    |
|-------------------------|-----------------------------------|---------------------------------------------------|
| Sleep                   | `WFI` mit `SCR.SLEEPDEEP=0`       | RP2350 Core 1 schläft bis IRQ (echt)              |
| Deep-Sleep              | `WFI` mit `SCR.SLEEPDEEP=1`       | echtes RP2350 Deep-Sleep (Clocks gedrosselt)      |
| Power-Down              | `PMU.PCON.PM=2` + WFI             | `clocks_hw->sleep_en{0,1}=0` + Deep-Sleep         |
| Deep-Power-Down         | `PMU.PCON.PM=3` + WFI             | nicht 1:1 möglich (RP2350 Dormant würde USB+CLI killen) — ignoriert |
| Wake-Up Quellen         | NVIC, BOD, WAKEUP-Pins            | jeder Cortex-M33-NVIC-IRQ + alle GPIO-Wake-Pins   |
| BOD (Brown-Out-Detect)  | LPC SYSCON BOD-Block              | nicht modelliert (RP2350 hat eigenen Mechanismus) |

Konkret:

* `WFI` des Guests blockiert Core 1 nativ — RP2350 schaltet die
  betroffenen Subsysteme automatisch ab. Ein Guest, der den
  Energiesparmodus erwartet, profitiert davon **direkt**.
* PCON-PM-Schatten ([src/peripherals.cpp](../src/peripherals.cpp)
  `g_pmu`) wird gelesen/geschrieben. PM=2 (Power-Down) führt
  beim nächsten Idle-Punkt zu `clocks_hw->sleep_en* = 0`,
  `SCR.SLEEPDEEP=1`, `__wfi()`.
* Deep-Power-Down (PM=3) wird absichtlich nicht in RP2350-Dormant
  übersetzt, weil dann USB/CLI/SWD ebenfalls anhalten und der
  Emulator die Verbindung zum Host verliert. Stattdessen wird der
  Modus wie Power-Down behandelt.
* SYSCON-PDRUNCFG wird als RAM-Schatten geführt; einzelne Power-
  Domain-Bits werden im RP2350 nicht abgebildet.

**Ergebnis**: Der Energieverbrauch des Emulators folgt grob dem
Verhalten des Guests — wenn der Guest schläft, schläft auch das
RP2350. Absoluter Verbrauch liegt naturgemäß über dem einer echten
LPC1115, weil USB-CDC + Core 0 + SWD-Polling weiterlaufen.

---

## 14. WDT — echter Reset (nur Guest)

[src/peripherals.cpp](../src/peripherals.cpp) `WdtModel` führt einen
realen 24-Bit-Counter (`WDT_TC`/`WDT_TV`) mit der WDT-Clock-Frequenz
(default ≈ 500 kHz). Bei jedem MMIO-Zugriff auf den WDT-Block wird
`wdt_advance()` aufgerufen und der Zähler entsprechend der seither
verstrichenen Zeit dekrementiert.

Bei Ablauf:

* `WDT_MOD.WDRESET=1` → `peripherals_wdt_reset_guest()` →
  [src/emulator.cpp](../src/emulator.cpp) `request_guest_reset()`:
  Core 1 wird via `multicore_reset_core1()` zurückgesetzt, MPU
  reinitialisiert, das Image neu in RAM kopiert, Reset-Vector
  ausgeführt. **Der RP2350 selbst bleibt aktiv** — USB/CLI/SWD
  laufen weiter.
* `WDT_MOD.WDRESET=0` → IRQ `WWDT` wird gepended.

Feed-Sequenz `0xAA, 0x55` auf `WDT_FEED` lädt den Counter neu.

---

## 15. Voll-PIO-SWD-TX

[src/swd_target.cpp](../src/swd_target.cpp) belegt **zwei** PIO-State-
Machines am gleichen PIO-Block:

| SM   | Programm                                          | Zweck                            |
|------|---------------------------------------------------|----------------------------------|
| RX   | `wait 1 pin1 / in pins,1 / wait 0 pin1`           | SWDIO sampling auf SWCLK rising  |
| TX   | `wait 0 pin1 / out pins,1 / wait 1 pin1`          | SWDIO drive  auf SWCLK falling   |

Der TX-SM ist permanent geladen; vor jedem Burst wird:

1. PULL_THRESH dynamisch auf `n` Bits gesetzt (FIFO konsumiert genau
   so viele OSR-Bits, bevor sie blockt),
2. SWDIO-Pindir auf Output umgelegt
   (`pio_sm_set_pindirs_with_mask`),
3. Datenwort in die TX-FIFO geschoben,
4. nach Bit `n-1` der Pindir wieder auf Input zurückgesetzt
   (Trn-Bit / Read-Phase).

Damit fällt der bisherige ~2 MHz-Limit der CPU-getriebenen TX weg —
der TX läuft synchron zur PIO-Clock (typ. 150 MHz / Default-Clkdiv =
1.0). Die effektive obere SWCLK-Frequenz wird dann durch die
RX-Sampling-Latenz und den Trn-Pfad bestimmt; konservativ
**≈ 25 MHz** SWCLK.

---

## 16. USB-Mass-Storage-Boot

[src/usb_msc.cpp](../src/usb_msc.cpp) stellt eine virtuelle FAT12-
Diskette über TinyUSB-MSC bereit (LUN 0, 256 KiB). Der Host sieht
das Volume `LPC1115EMU`. Beim **Eject** (`SCSI START_STOP_UNIT
load_eject=1, start=0`) parst der Emulator:

* `CONFIG.INI` → `pin.<p>_<n>=<gpio>`, `autostart`, `freq_hz`.
  Konfig wird in den Storage-Slot persistiert.
* `BOOT.HEX` → Stream-Parser → Firmware-Slot.

Ist `autostart=on` aktiv, wird die Firmware **sofort gestartet**.
Beim nächsten Power-Cycle läuft der Emulator damit autonom — keine
serielle Konsole, kein Host-PC erforderlich.

Persistenz: das FAT12-Volume liegt aktuell als 256-KiB-RAM-Spiegel
in SRAM; der Inhalt geht beim Power-Cycle verloren, **außer** für
die geparsten Targets (Firmware-Slot, Konfig-KV) — die sind
persistent und reichen für Auto-Boot.

---

## 17. Status-LED (Heartbeat)

`led.cpp` blendet `emulator::state()` auf die Onboard-LED des Pi Pico 2
(`PICO_DEFAULT_LED_PIN` = GPIO 25):

| Zustand              | LED-Muster                |
|----------------------|---------------------------|
| `Idle`               | 1 Hz Heartbeat (50 % duty)|
| `Running`            | dauerhaft an              |
| `Halted`             | Doppelblitz alle 2 s      |
| `Faulted`            | 8 Hz Flackern             |

Implementierung: `led::poll()` wird aus dem CLI-Hauptloop aufgerufen,
bestimmt den Zustand zeitbasiert über `to_ms_since_boot(...)` und
schaltet GPIO 25 mit `gpio_put`. Keine Timer/IRQs, daher unkritisch
gegenüber TinyUSB-Polling.

`led::init()` wird **als allererstes** in `main()` ausgeführt, noch vor
`stdio_init_all()` und `usb_stdio_init()` — so ist auch dann ein
Lebenszeichen sichtbar, wenn die USB-Initialisierung später hängen
sollte.

---

## 18. USB-Stack — eigene tusb_config.h, kein `pico_stdio_usb`

Die SDK-Library `pico_stdio_usb` bringt eine eigene `tusb_config.h`
mit (`CFG_TUD_CDC=1`, kein MSC), die mit unseren 2×CDC + MSC-
Descriptoren kollidieren würde — Folge: das Device enumeriert gar
nicht und der Host sieht VID:PID `cafe:4011` nicht.

Daher:
* `pico_enable_stdio_usb(emulator 0)` in [CMakeLists.txt](CMakeLists.txt) — die Bridge ist aus.
* Eigene Datei [src/usb_stdio.cpp](src/usb_stdio.cpp): ruft `tusb_init()` auf, registriert einen
  `stdio_driver_t`, der stdout/stdin auf CDC-Interface 0 (CLI) leitet.
* `tud_task()` wird **aus dem Hauptloop** in `cli::run()` aufgerufen —
  niemals aus IRQ-Kontext (TinyUSB ist nicht IRQ-safe; ein
  Timer-Callback würde auf RP2350 zu HardFault → Boot-ROM →
  BOOTSEL-Modus führen).

VID:PID = `0xCAFE:0x4011` (TinyUSB-Test-VID, in [src/usb_descriptors.cpp](src/usb_descriptors.cpp)).
Mit `lsusb -d cafe:4011 -v` zu prüfen.
