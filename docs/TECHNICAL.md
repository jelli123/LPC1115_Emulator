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

| Region                   | Guest-Sicht (LPC)        | Host-Realität (RP2350)         |
|--------------------------|--------------------------|--------------------------------|
| Code-Flash               | `0x00000000-0x0000FFFF`  | aligned SRAM-Array (`load_base()`) |
| RAM                      | `0x10000000-0x10001FFF`  | aligned SRAM-Array (`guest_ram_base()`) |
| APB-Peripherie           | `0x40000000-0x4007FFFF`  | **MPU-Trap** → `peripherals::` |
| AHB-GPIO                 | `0x50000000-0x5003FFFF`  | **MPU-Trap** → `peripherals::` |
| PPB / SCB / SysTick      | `0xE000E000-0xE000EFFF`  | **echt** (Cortex-M33), außer NVIC + SCB-Ctrl |
| NVIC                     | `0xE000E100-0xE000E4FF`  | **MPU-Trap** → `vnvic::`       |
| SCB-Ctrl (inkl. AIRCR)   | `0xE000ED00-0xE000ED1F`  | **read-only** → Trap (`NVIC_SystemReset` → nur Gast) |
| IAP-ROM-Einsprung        | `0x1FFF1FF1`             | Prefetch-Trap → `iap::dispatch` |

Die SRAM-Adressen sind **nicht** fest, sondern werden als `alignas`-Arrays
allokiert (`g_firmware_image` 256-aligned für VTOR, `g_guest_ram` 32-aligned);
die Laufzeit-Basis liefern `emulator::load_base()` / `guest_ram_base()`.
Der Reset-Vector aus dem geladenen Image wird beim `run` in einen
synthetischen Initial-Frame auf der Guest-PSP geschrieben; danach erfolgt
ein `BX` mit `EXC_RETURN = 0xFFFFFFFD` (Thread, PSP, unprivilegiert).

---

## 3. MPU-Konfiguration

ARMv8-M MPU, eng gefasst — dem unprivilegierten Gast werden **nur die
eigenen** SRAM-Bereiche freigegeben ([src/mmu.cpp](../src/mmu.cpp)):

| #  | Bereich                       | Attribute                         |
|----|-------------------------------|-----------------------------------|
| 0  | Gast-Code-Image (64 KiB)      | RWX, Normal                       |
| 1  | Gast-RAM (8 KiB)              | RWX, Normal                       |
| 2  | `enter_guest`-Trampolin (32 B)| RX, Normal                        |
| 3  | PPB Teil A `0xE0000000-E0FF`  | RW, Device, XN                    |
| 4  | PPB Teil B1 `0xE500-ECFF`     | RW, Device, XN                    |
| 5  | SCB-Ctrl `0xED00-ED1F`        | **RO**, Device, XN → AIRCR-Trap   |
| 6  | PPB Teil B2 `0xED20-0xE00FFFFF`| RW, Device, XN                   |

* `PRIVDEFENA = 1`: der **Host** (Core 0, privilegiert) sieht via Default-Map
  weiterhin den vollen Speicher (Flash-Code, RP2350-HW, TinyUSB). Der **Gast**
  (unprivilegiert) sieht nur die Regionen oben.
* Der übrige RP2350-SRAM (TinyUSB-Puffer, Core-Stacks, Emulator-Daten) ist für
  den Gast **nicht** freigegeben — ein wilder Pointer/Heap-/Stack-Überlauf trapt
  sauber, statt Host-Speicher still zu korrumpieren.
* Die **NVIC-Range** `0xE000E100-0xE000E4FF` ist bewusst *nicht* whitelistet →
  jeder Gast-Zugriff trapt nach [vnvic](../src/vnvic.cpp).
* Der **SCB-Kontrollblock** `0xE000ED00-0xED1F` ist read-only: Gast-Schreib-
  zugriffe (v. a. `AIRCR.SYSRESETREQ` via `NVIC_SystemReset()`) trappen und
  werden emuliert — ein `SYSRESETREQ` startet nur den **Gast** neu, nicht das
  echte RP2350-Silizium. Lesezugriffe (CPUID/ICSR-Status) laufen dank RO nativ.
* LPC-Peripherie `0x40000000-0x4FFFFFFF` liegt in **keiner** Region → für den
  unprivilegierten Gast gesperrt → Trap nach `peripherals::mmio_*`.
* Das `enter_guest`-Trampolin muss im SRAM liegen (`__not_in_flash_func`,
  32-Byte-aligned): nach dem Privilegienwechsel (`msr control`) erfolgt der
  nächste Instruktions-Fetch unprivilegiert — läge er im XIP-Flash, würde die
  MPU ihn als `IACCVIOL` abweisen und der Gast könnte nie starten.

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

Da der injizierte Handler im **Thread-Mode** läuft (`LR=0xFFFFFFFD`), ist sein
regulärer Rücksprung dort keine gültige Exception-Rückkehr; der Core faultet
beim Anspringen von `0xFFFFFFFC`. `fault.cpp:try_injected_irq_return()` erkennt
den `EXC_RETURN`-artigen PC im Thread-Mode, hebt den PSP um den 32-Byte-Fault-
Frame an und legt so den darunter liegenden Original-Frame frei — der Gast setzt
nahtlos fort. `pendsv_inject_c()` läuft nur auf Core 1 (Guard `get_core_num()`),
da das überschriebene PendSV-Symbol auch in Core 0s Vektortabelle steht.

IRQ-Tabelle: [src/lpc_irqs.h](../src/lpc_irqs.h) (UM10398 Tab. 51).

---

## 7. Peripherie-Modelle

| Block           | Status | Hinweise                                      |
|-----------------|--------|-----------------------------------------------|
| SYSCON / PLL    | ✅     | `MSEL`-Schreiben → Soll-Takt als Zeitbasis (RP2350-Takt unverändert) |
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
| BOD (`BODCTRL`) | ✅     | Register modelliert; `BODRSTENA` → echte RP2350-POWMAN-BOD aktiv |
| RTC             | ❌     | LPC1115 hat keinen RTC-Block (entfällt)       |

PLL-Modell: `F_OUT = F_CLKIN × (M+1)`, gültig 156-320 MHz CCO.
Die berechnete Soll-Frequenz wird in einem **Post-Hook** nach dem letzten
Byte des SYSCON-Word-Schreibens übernommen (kein 4-faches Re-Targeting) —
als **Zeitbasis** der emulierten Timer/UART-Baud (`g_current_hz`). Der reale
RP2350-`clk_sys` bleibt bewusst bei 150 MHz (SDK-Default): die Timer skalieren
reale Wall-Clock-Zeit, und ein `set_sys_clock_khz()` von Core 1 aus würde
USB/CLI auf Core 0 stören. Einbuße: gast-eigene SysTick/Busy-Loops laufen mit
150 MHz statt der LPC-Soll-Frequenz; das Bit-Timing (KNX) läuft über die
real-zeit-skalierten CT16/CT32-Capture/Match-Modelle und bleibt korrekt.

---

## 8. GDB-Remote-Stub

Auf der GDB-CDC (Instanz-Index dynamisch, siehe §16a), RSP-Subset:

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
  +-- storage::init            # VOR USB: Config bestimmt den Deskriptor
  +-- config::load
  +-- usb_stdio_init
  |     +-- usb_desc_build     # Konfig-Deskriptor dynamisch aus config bauen
  |     +-- tinyusb init       # aktive CDCs (CLI/GDB/Serial) + MSC (immer)
  +-- mmu::init           (MPU 8 Regionen)
  +-- vnvic::init
  +-- peripherals::init
  +-- emulator::load_image_from_storage
  +-- iap::install_stub
  +-- target_halt::init
  +-- swd_target::init
  +-- gdb_stub::init
  +-- multicore_launch_core1(emulator::core1_entry)
  +-- cli::run            (forever; Eingabe/Prompt nur bei cli_enable=on)

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
  gdb_stub.cpp                    # RSP-Server auf der GDB-CDC
  pio_glue.cpp                    # Edge-Capture-PIO
  hex_parser.cpp  hex_patcher.cpp # Intel-HEX + Relokation
  xmodem.cpp                      # XMODEM-CRC/1K-Empfänger (CLI-Upload)
  storage.cpp                     # Flash-WL + 64-KiB-Firmware-Slot (RMW)
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
| BOD (Brown-Out-Detect)  | LPC SYSCON `BODCTRL`              | Register modelliert; bei `BODRSTENA` echte RP2350-POWMAN-BOD aktiv |

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
* **BOD (`BODCTRL` @ `0x40048048`):** Das LPC-Register wird modelliert
  ([src/peripherals.cpp](../src/peripherals.cpp) `g_bodctrl`,
  `bod_apply()`), damit die Firmware konsistent zurücklesen kann. Setzt
  der Gast `BODRSTENA`, stellen wir sicher, dass die **echte**
  RP2350-POWMAN-BOD aktiv ist (`powman_set_bits(&powman_hw->bod,
  POWMAN_BOD_EN_BITS)`), sodass reale Unterspannung einen Hardware-Reset
  auslöst.
  **Hardware-Grenze:** Die LPC-BOD überwacht die 3,3-V-VDD-Versorgung,
  die RP2350-POWMAN-BOD dagegen die *Core-Rail* (~1,1 V, VSEL
  0,473–1,204 V). Eine wörtliche Übersetzung der 3,3-V-Schwellen ist
  physikalisch sinnlos; `VSEL` bleibt daher auf RP2350-Default (sichere
  Core-Schwelle) — ein Anheben würde im Feldgerät spurious Resets
  riskieren. Die LPC-Schwellenbits werden gespeichert, aber nicht auf
  eine Core-Spannung gemappt.

### Pin-IRQ-Wakeup aus reiner WFI-Schleife (opt-in)

Da der Gast nativ ohne Host-Loop läuft, ist der MMIO-Trap normalerweise
der einzige synchrone Injektionspunkt für emulierte IRQs — eine reine
`__WFI()`-Warteschleife ohne MMIO-Zugriff lässt sich so nicht wecken.

Mit `wfi_pin_wakeup=on` (CONFIG.INI, Default **aus**) werden beim Laden
alle `WFI` (Thumb `0xBF30`) auf `SVC #0` (`0xDF00`) gepatcht
([src/hex_patcher.cpp](../src/hex_patcher.cpp) `patch_wfi_to_svc`) und der
SVC-Vektor (Slot 11) auf `isr_svc_wfi`
([src/emulator.cpp](../src/emulator.cpp)) umgebogen. Der Handler pollt
`peripherals::sample_pin_interrupts()` (echte PINT/GINT-Flanken) und
`peripherals::poll_timed_sources()` (CT16/CT32/WWDT) und kehrt mit
gesetztem PendSV zurück, sobald ein vom Gast aktivierter IRQ pending wird.

**Einschränkungen:**
* Aktives Pollen (`busy_wait_us(50)`), funktional korrekt, aber **nicht
  stromsparend**.
* `WFI` mit aktivierten Interrupts (`PRIMASK=0`) läuft über den SVC-Handler.
  Das Idiom `__disable_irq(); __WFI();` (`PRIMASK=1`) eskaliert den SVC zu
  HardFault und wird dort gesondert behandelt
  ([src/fault.cpp](../src/fault.cpp) `hardfault_c`): gleiches Polling, PendSV
  wird gesetzt und feuert, sobald der Gast die Interrupts wieder freigibt.
* Das Patchen ersetzt das 16-Bit-Muster `0xBF30`; ein gleich aussehendes
  Datenwort würde fälschlich getroffen → Feature nur für getestete
  Firmware, daher Default aus.
* Auf Hardware nicht validiert.

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

## 16. Fehler-Handling & `FAULT.TXT`

Ein nicht emulierbarer Gast-Fault führt **nicht** mehr zu einem Watchdog-/
Board-Reset (der früher die Diagnose und die USB-Sitzung vernichtete), sondern
zu einem sauberen **Halt** ([src/fault.cpp](../src/fault.cpp) `enter_fatal_halt`):

* `emulator::notify_guest_faulted()` setzt `State::Faulted` (LED flackert 8 Hz).
* Core 1 parkt in `for(;;) __wfe()`; **Core 0 (USB/CLI) läuft weiter**.
* Recovery per CLI `reset`/`run` oder neues Image.

**Diagnose (dual-sink).** `femit()` schreibt jede `[FAULT]`-Zeile gleichzeitig
seriell und in einen RAM-Puffer. `print_fault_cause()` schlüsselt `CFSR/HFSR`
in Klartext auf (`IACCVIOL`, `UNDEFINSTR`, `FORCED`, …); `print_exc_diag()`
ergänzt Register, `instr@PC` und `MMFAR/BFAR`.

**`FAULT.TXT` auf dem USB-MSC** (mbed-DAPLink-Stil). Am Ende der Fault-Sequenz
gibt `report_commit()` den Puffer frei; `usb_msc::poll()` (Core 0) baut ein
frisches Volume mit `FAULT.TXT` (+ `HELP.HTM`/`CONFIG.INI`) und erzwingt per
simuliertem Medienwechsel (`test_unit_ready` meldet kurz *not present*, dann
*UNIT ATTENTION*) ein Neueinlesen des Hosts. So ist die Analyse ohne CLI/Serial
möglich. Der Report liegt nur im RAM (nach Power-Cycle weg).

**Eskalierte Faults unter `PRIMASK`.** Läuft der Gast in einer kritischen
Sektion (`__disable_irq()`, `PRIMASK=1`) und trapt dort (Flash-Funktionspointer-
Sprung oder MMIO), ist die MemManage/BusFault-Priorität maskiert und der Trap
eskaliert zu **HardFault** (`HFSR.FORCED`). `hardfault_c()` behandelt das
gestuft: (1) rohe Code-Fetch-Adresse relozieren, (2) WFI-Wakeup, (3) den vollen
Memory-Fault-Emulator (`handle_memfault_c`) delegieren. Damit laufen auch
`PRIMASK`-nutzende Firmwares (z. B. bim112/BCU2) transparent.

**AIRCR-Trap.** `NVIC_SystemReset()` schreibt `SCB->AIRCR` mit `SYSRESETREQ`.
Da der SCB-Kontrollblock read-only gemappt ist, trapt der Schreibzugriff und
wird in `handle_memfault_c` erkannt (VECTKEY `0x05FA` + `SYSRESETREQ`): statt
das Silizium zu rebooten, löst er `emulator::request_guest_reset()` aus — nur
der Gast startet neu, USB/CLI bleiben.

---

## 17. Flash-Schreiben bei laufendem Gast (Multicore-Lockout)

Ein Flash-Erase/Program stallt XIP; läuft währenddessen Code aus dem Flash,
crasht der betroffene Core. Betroffen sind zwei Pfade:

* **Core 0 schreibt** (CLI `upload`/`erase`/`finalize`, `xmodem`, `cfg save`,
  USB-MSC-Eject): Vor dem Schreiben wird der Gast pausiert
  (`emulator::FlashPauseGuard` → `pause_for_flash()` → `stop()`), sodass Core 1
  in der SDK-Spin-Schleife steht (SDK-VTOR). `stop()` legt zudem Core 0s
  `SIO_IRQ_FIFO` während Reset+Relaunch still, damit der Lockout-Handler nicht
  die Multicore-Handshake-Bytes wegdrainiert.
* **Core 1 schreibt** (IAP `persist()` im Fault-Handler, z. B. Selfbus-EEPROM/
  OTA): sperrt Core 0 per `multicore_lockout_start_blocking()` aus. Dafür ist
  Core 0 in [main.cpp](../src/main.cpp) als `multicore_lockout_victim_init()`
  registriert; sein VTOR bleibt stets die SDK-Tabelle, der Lockout-Handler ist
  also immer erreichbar.

`storage::FlashGuard` ist core-aware und sperrt jeweils den *anderen* Core aus,
sofern dieser als Victim initialisiert ist. Da ein Core-0-Flash immer zuerst den
Gast pausiert, können sich die beiden Lockout-Pfade nicht gegenseitig
verklemmen. Kosten: Ein Core-0-Flash bei laufendem Gast startet den Gast neu
(beim Reflash gewollt; bei `cfg save` kurzer Neustart) — kein Hang, keine stille
Korruption.

**Cross-Core-Halt-Grenze.** Ein frei laufender Gast (Core 1, Gast-VTOR) lässt
sich von Core 0 aus **nicht** asynchron per PendSV anhalten (ICSR ist per-Core
gebankt). CLI `stop`/`halt` versucht daher kurz einen kooperativen Halt (falls
der Gast gerade trapt) und fällt sonst auf einen erzwungenen Core-Reset zurück
(`halted (forced core reset)`).
---

## 16. USB-Mass-Storage-Boot

[src/usb_msc.cpp](../src/usb_msc.cpp) stellt eine virtuelle FAT12-
Diskette über TinyUSB-MSC bereit (LUN 0, 256 KiB). Der Host sieht
das Volume `LPC1115EMU`. Beim **Eject** (`SCSI START_STOP_UNIT
load_eject=1, start=0`) parst der Emulator:

* `CONFIG.INI` → `pin.<p>_<n>=<gpio>`, `autostart`, `freq_hz`, die
  USB-Schnittstellen-Schalter `cli_enable`/`gdb_enable`/`serial_enable` (§16a)
  sowie alle Bridge-Schlüssel. Konfig wird in den Storage-Slot persistiert.
* `BOOT.HEX` (oder jede `*.HEX`) → Stream-Parser → Firmware-Slot. Nach
  erfolgreichem Flashen wird die HEX-Datei — wie beim RP2350-UF2-Bootloader —
  automatisch vom Volume entfernt und ein Medienwechsel ausgelöst
  (`format_blank` + `build_info_files` ohne HEX + `trigger_media_change`);
  CONFIG.INI/HELP.HTM/DEBUG.TXT bleiben erhalten.

Ist `autostart=on` aktiv, wird die Firmware **sofort gestartet**.
Beim nächsten Power-Cycle läuft der Emulator damit autonom — keine
serielle Konsole, kein Host-PC erforderlich.

Persistenz: das FAT12-Volume liegt aktuell als 256-KiB-RAM-Spiegel
in SRAM; der Inhalt geht beim Power-Cycle verloren, **außer** für
die geparsten Targets (Firmware-Slot, Konfig-KV) — die sind
persistent und reichen für Auto-Boot.

Weitere CONFIG.INI-Schlüssel (Auszug, Phase 3): `app_start`, `desc_addr`,
`autodesc` (siehe §18) sowie `flash_erase=on`/`erase=on`, das den
Firmware-Slot **vor** dem Anwenden einer ggf. mitgelieferten `BOOT.HEX`
komplett löscht. Ohne diesen Schlüssel wird `BOOT.HEX` additiv gemergt.

---

## 16a. Dynamische USB-Composite-Deskriptoren

[src/usb_descriptors.cpp](../src/usb_descriptors.cpp) baut den
Konfigurations-Deskriptor **zur Bootzeit** (`usb_desc_build()`, aufgerufen von
`usb_stdio_init()` **vor** `tusb_init()`). `tusb_config.h` reserviert
`CFG_TUD_CDC = 3` und `CFG_TUD_MSC = 1`; welche der drei CDCs tatsächlich im
Deskriptor erscheinen, entscheiden die CONFIG.INI-Schalter (Default alle `on`):

| Schlüssel        | Rolle / TU-Instanz            | CLI-Nutzung                    |
|------------------|-------------------------------|--------------------------------|
| `cli_enable`     | CLI / stdio (`usb_stdio.cpp`) | Kommandozeile, `printf`        |
| `gdb_enable`     | GDB-RSP (`gdb_stub.cpp`)      | `gdb on/off`                   |
| `serial_enable`  | Serial-Adapter (`uart_bridge.cpp`) | `cdc …`, `uart cdc on`    |

Mechanik:

* Die aktiven CDCs werden in **fester Reihenfolge** CLI → GDB → Serial
  gezählt und erhalten fortlaufende TinyUSB-Instanz-Indizes (0..n-1). Diese
  Reihenfolge deckt sich mit der Interface-Reihenfolge im Deskriptor, sodass
  `tud_cdc_n_*` mit dem berechneten Index den richtigen Endpunkt trifft.
* Endpoint-Nummern werden über einen laufenden Zähler vergeben (CDC: notif
  `0x80|n`, out `n`, in `0x80|n`); das **MSC** liegt immer zuletzt und ist
  **immer** präsent (Recovery-Pfad über CONFIG.INI, selbst wenn alle CDCs aus).
* Interface- und Endpoint-Blöcke werden mit den TinyUSB-Makros in lokale
  Puffer erzeugt und in einen statischen Deskriptor-Puffer kopiert;
  `tud_descriptor_configuration_cb()` liefert diesen zurück.
* Die Zuordnung wird zentral über `usb_desc_cdc_cli()` / `_gdb()` / `_serial()`
  abgefragt (Rückgabe: Instanz-Index oder **-1** = deaktiviert). `usb_stdio.cpp`,
  `gdb_stub.cpp` und `uart_bridge.cpp` nutzen diese Accessoren statt fester
  Konstanten; bei `-1` werden die zugehörigen Pfade übersprungen.

Weil der Deskriptor statisch ist, solange USB läuft, wirkt eine Änderung der
`*_enable`-Schalter erst nach einem **Reset**. `cli::run()` läuft bei
`cli_enable=off` weiter (Housekeeping, GDB/MSC/UART-Poll), überspringt aber
Prompt und Tastatureingabe. `status` gibt die aktuelle Zuordnung aus
(`USB-CDC: CDC#0=CLI …`, plus `| aus:` für deaktivierte Rollen).

---

## 17. Firmware-Slot: additives Laden (Sektor-RMW)

[src/storage.cpp](../src/storage.cpp) verwaltet den 64-KiB-Firmware-Slot.
Uploads (`upload`, `xmodem`, `BOOT.HEX`) schreiben über
`storage::firmware_write(offset, data, len)` **additiv** — es wird **nicht**
mehr implizit vorgelöscht.

Mechanik (Read-Modify-Write je 4-KiB-Sektor):

* Ein `fw_sector_buf[4096]` puffert den aktuell beschriebenen Sektor.
* Beim Sektorwechsel wird der bestehende Inhalt zuerst aus dem XIP-Flash
  (`xip_ptr(region + sector_start)`) in den Puffer geladen, dann werden die
  neuen Bytes darüber gelegt (Overlay).
* `fw_flush_sector()` führt — nur wenn der Puffer „dirty" ist — innerhalb
  einer `FlashGuard` ein `flash_range_erase` + `flash_range_program` aus.
* Nicht beschriebene Sektoren (z. B. ein zuvor geladener Bootloader)
  bleiben dadurch unverändert erhalten.
* `firmware_finalize(total_len)` flusht den letzten Sektor, setzt
  `total_len = max(vorhandene_Größe, total_len)` (Längen-Merge), berechnet
  die CRC32 über `[region, total_len)` und schreibt anschließend den
  Firmware-Marker.

`firmware_erase()` (CLI `erase`/`flash erase`, CONFIG.INI `flash_erase=on`)
löscht den gesamten Slot explizit und setzt den Sektor-Cache zurück.

Derselbe RMW-Pfad macht auch die **IAP-Persistenz** (Cmd 51, sblib-EEPROM,
OTA) robuster: Programmierung erfolgt jetzt als echtes Erase+Program statt
Program-only.

---

## 18. Zweistufiger Boot: Bootloader → Applikation + Auto-Descriptor

Selfbus-Geräte koppeln einen **Bootloader** (ab `0x0000`) mit einer
**Applikation** (ab `app_start`, Default `0x3000`). Der Bootloader springt
nur, wenn ein **App-Descriptor** bei `desc_addr` (Default `app_start−0x100`)
gültig ist. Der Descriptor entspricht der Selfbus-`AppDescriptionBlock`:

```c
struct AppDescriptor {            // 16 Byte
    uint32_t startAddress;        // = app_start (LPC-Flash-Adresse)
    uint32_t endAddress;          // letztes belegtes App-Byte
    uint32_t crc;                 // CRC32 über [start..end]
    uint32_t appVersionAddress;   // "!AVP!@:"-Magic oder app_start
};
```

### Erkennung (Load-Time)

[src/emulator.cpp](../src/emulator.cpp) `core1_main` prüft via
`has_valid_app_vectors()`, ob bei `app_start` eine plausible Vektortabelle
liegt: Wort[0] (Initial-SP) im LPC-RAM-Bereich `[0x10000000, +RAM)` und
Wort[1] (Reset-Vektor) mit gesetztem Thumb-Bit im Flash `≥ app_start`. Ist
das der Fall, läuft der Modus „zweistufig".

### Pristine-App-Grenze (CRC-Korrektheit)

Im zweistufigen Modus werden die Load-Time-Patches (`relocate_ram_refs`,
WFI-/CPS-Patches) **nur auf den Bootloader-Bereich `[0, app_start)`**
angewandt (`reloc_len = app_start`). Die App-Bytes bleiben damit **exakt
so wie geladen** — sonst wiche die Laufzeit-CRC von der Descriptor-CRC ab
und der Bootloader bliebe im Updater-Modus. Die App läuft trotzdem korrekt,
weil Flash-Zugriffe ohnehin getrappt und auf das relozierte Image
umgeleitet werden (gleicher Pfad wie beim OTA-Update).

### Descriptor-Synthese

Bei `autodesc=on` ruft der Loader `ensure_boot_descriptor()`:

* End-Adresse = letztes Byte ≠ `0xFF` im App-Bereich.
* `crc32_selfbus()` (Polynom `0xEDB88320`, Seed `0xFFFFFFFF`, finales `~`) —
  **bit-identisch** zur Bootloader-Routine — über `[app_start..end]`.
* Schreibt den Descriptor mit **LPC-Adressen** (z. B. `0x3000`), weil der
  Bootloader sie zur Laufzeit dereferenziert und der Flash-Trap sie aufs
  Image umleitet.
* **Schutz:** geschrieben wird nur, wenn der Descriptor-Bereich entweder
  blank (`0xFF`) ist **oder** bereits descriptor-artig aussieht
  (`startAddress == app_start`). Ein bereits gültiger Descriptor (CRC passt)
  bleibt unangetastet; Fremddaten werden geschont und nur geloggt.

Log-Ausgabe: `[EMU] Boot-Descriptor erzeugt @0x… start=… end=… crc=… ver=…`.

### Handover

Der Sprung BL→App läuft über den bestehenden SYSMEMREMAP-Hook
(`activate_bootloader_handover()`, siehe Boot-Diagramm): App-Vektoren werden
**erst nach** der Bootloader-CRC-Prüfung relociert, VTOR auf `app_start`
gesetzt; ein SP-Fixup sorgt dafür, dass der rohe LPC-StackTop auf gültigen
RP2350-SRAM zeigt.

---

## 19. XMODEM-Upload (CLI)

[src/xmodem.cpp](../src/xmodem.cpp) implementiert einen **XMODEM-CRC/1K-
Empfänger** als robuste Alternative zum reinen HEX-Paste auf CDC#0 (das bei
fehlender Flusskontrolle Zeichen verlieren kann).

* Blockgrößen: `SOH`=128 Byte, `STX`=1024 Byte; Sync per `'C'` (CRC-Mode).
* Integritätsprüfung: CRC16-CCITT (Poly `0x1021`) je Block, `ACK`/`NAK`.
* `xmodem::receive(sink, ctx, pump, bytes)` ist blockierend, ruft aber je
  Poll-Iteration `pump()` (= `usb_stdio_task` → `tud_task`) auf, damit der
  USB-CDC-Stack während des Empfangs bedient wird.
* Die empfangenen Bytes werden in den Intel-HEX-Parser gespeist; nach dem
  EOF-Record wird `storage::firmware_finalize()` aufgerufen. XMODEM-Padding
  (`0x1A`) nach EOF wird verworfen.

CLI: `xmodem` → „Empfang bereit", dann die `.hex`-Datei mit einem XMODEM-
fähigen Terminal (Tera Term, `sx`/`lrzsz`) senden.

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
mit (`CFG_TUD_CDC=1`, kein MSC), die mit unseren 3×CDC + MSC-
Descriptoren kollidieren würde — Folge: das Device enumeriert gar
nicht und der Host sieht VID:PID `cafe:4012` nicht.

Daher:
* `pico_enable_stdio_usb(emulator 0)` in [CMakeLists.txt](CMakeLists.txt) — die Bridge ist aus.
* Eigene Datei [src/usb_stdio.cpp](src/usb_stdio.cpp): ruft `tusb_init()` auf, registriert einen
  `stdio_driver_t`, der stdout/stdin auf CDC-Interface 0 (CLI) leitet.
* `tud_task()` wird **aus dem Hauptloop** in `cli::run()` aufgerufen —
  niemals aus IRQ-Kontext (TinyUSB ist nicht IRQ-safe; ein
  Timer-Callback würde auf RP2350 zu HardFault → Boot-ROM →
  BOOTSEL-Modus führen).

VID:PID = `0xCAFE:0x4012` (TinyUSB-Test-VID, in [src/usb_descriptors.cpp](src/usb_descriptors.cpp)).
Mit `lsusb -d cafe:4012 -v` zu prüfen.

---

## 19. UART-Bridge (CDC#2 ↔ PIO-UART)

[src/uart_bridge.cpp](../src/uart_bridge.cpp) stellt eine transparente
serielle Bridge bereit: USB CDC#2 wird bidirektional auf zwei frei
wählbare RP2350-GPIOs gemappt — **ohne die Hardware-UARTs zu belegen**.

### Architektur

```
Host (PC)                    RP2350 (Core 0)
   │                              │
   │  USB CDC#2                   │
   │  SET_LINE_CODING(baud)  ───► │  tud_cdc_line_coding_cb()
   │  Bulk OUT (TX-Daten)    ───► │  poll(): tud_cdc_n_read(2)
   │  Bulk IN  (RX-Daten)   ◄─── │  poll(): tud_cdc_n_write(2)
   │                              │
                                  │  PIO1 (oder PIO2)
                                  │  ┌─────────────┐
                     TX-GPIO ◄────┤  │ SM-TX: 8N1  │ ◄── TX-FIFO
                                  │  └─────────────┘
                                  │  ┌─────────────┐
                     RX-GPIO ────►┤  │ SM-RX: 8N1  │ ──► RX-FIFO
                                  │  └─────────────┘
```

### PIO-Programme

**TX** (4 Instruktionen, `side_set 1 opt`):
```
.wrap_target
    pull       side 1 [7]   ; Idle-High / Stop-Bit, auf FIFO warten
    set  x, 7 side 0 [7]   ; Start-Bit (low), 8 Bits laden
bitloop:
    out  pins, 1            ; 1 Bit ausgeben
    jmp  x-- bitloop  [6]  ; 8 Zyklen/Bit
.wrap
```

**RX** (5 Instruktionen, kein side-set):
```
.wrap_target
    wait 0 pin 0            ; Start-Bit abwarten
    set  x, 7         [10] ; Mitte erstes Datenbit (12 Zyklen nach Flanke)
bitloop:
    in   pins, 1            ; Bit sampeln
    jmp  x-- bitloop  [6]  ; 8 Zyklen/Bit
    push                    ; 8-Bit-Wort in RX-FIFO
.wrap
```

Clock-Divider: `clk_sys / (baud × 8)`.

### Baudraten-Steuerung

Der Host setzt die Baudrate über CDC `SET_LINE_CODING` (Standard-
Mechanismus, z. B. `stty -F /dev/ttyACM2 19200` oder Terminal-App).
TinyUSB ruft `tud_cdc_line_coding_cb(itf=2, ...)` auf → der PIO-
Clock-Divider wird live aktualisiert, ohne die Bridge neu zu starten.

### PIO-Block-Wahl

Die Bridge versucht **PIO1**, dann **PIO2**. PIO0 bleibt für
`pio_glue` (Edge-Capture) und SWD-Target reserviert.

### Konfiguration

| Key in config.ini        | Wirkung                              |
|--------------------------|--------------------------------------|
| `uart_bridge_en=1`       | Bridge beim Boot automatisch starten |
| `uart_bridge_tx=<gpio>`  | TX-Pin (Datenrichtung: RP→extern)    |
| `uart_bridge_rx=<gpio>`  | RX-Pin (Datenrichtung: extern→RP)    |

CLI: `uart start <tx> <rx>`, `uart stop`, `uart status`.
