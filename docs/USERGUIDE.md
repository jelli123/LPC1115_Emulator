# LPC1115-Emulator auf RP2350 – User-Guide

Dieses Dokument beschreibt **Bedienung** und **Workflow** des Emulators.
Die Implementierungs-Interna stehen in [TECHNICAL.md](TECHNICAL.md).

---

## 1. Was die Hardware mitbringen muss

| Funktion                 | RP2350-Pins (Default, änderbar)             |
|--------------------------|----------------------------------------------|
| USB CDC#0 (CLI)          | USB-Port (TinyUSB)                           |
| USB CDC#1 (GDB-RSP)      | USB-Port (TinyUSB, zweite CDC-Schnittstelle) |
| LPC-GPIO0 → RP-GPIO      | konfigurierbar, siehe Abschnitt 5            |
| LPC-UART0 (TX/RX)        | RP2350 `uart0` (Default GP0/GP1)             |
| KNX-Bus (TPUART o. ä.)   | frei wählbar, siehe Abschnitt 7              |
| SWD-Target-Pins          | 2 freie GPIO, **SWCLK = SWDIO + 1**          |

Eine Pico-2- oder Pico-2-W-Platine genügt.

---

## 2. Erst-Inbetriebnahme

1. Firmware bauen (Linux/macOS mit `arm-none-eabi-gcc`):

   ```bash
   cmake -S . -B build -DPICO_BOARD=pico2
   cmake --build build -j
   ```

   Ergebnis: `build/lpc1115_emulator.uf2`.

2. RP2350 in BOOTSEL halten, einstecken, UF2 kopieren.
3. Zwei serielle Ports erscheinen:
   * **CDC#0** – CLI (z. B. `COM7` / `/dev/ttyACM0`).
   * **CDC#1** – GDB-Remote-Stub (z. B. `COM8` / `/dev/ttyACM1`).
4. Außerdem erscheint ein **Wechseldatenträger** namens `LPC1115EMU`
   (FAT12, 256 KiB).
5. Mit Terminal an CDC#0 verbinden (115200 8N1, Line-Ending CRLF).

Es erscheint:

```
LPC1115-Emu  rev <git-sha>  (RP2350)
emu> 
```

### Status-LED (Onboard-LED, GPIO 25)

Die grüne LED auf dem Pi Pico 2 zeigt jederzeit den Zustand des Emulators
an, ohne dass die CLI verbunden sein muss:

| Muster                    | Bedeutung                                  |
|---------------------------|--------------------------------------------|
| 1 Hz Heartbeat (50 % duty)| Idle / bereit, keine Gast-Firmware aktiv   |
| dauerhaft an              | Gast-Firmware läuft (`emulator running`)   |
| Doppelblitz alle 2 s      | Gast-Firmware angehalten (`halt`/Breakpoint) |
| schnelles Flackern (8 Hz) | Hard-/Bus-/Memmanage-Fault im Gast         |
| dunkel                    | Emulator selbst ist gecrasht oder nicht geflasht |

So kann man am USB-Stecker schon vor dem Öffnen des Terminals erkennen,
ob die Firmware bereit ist.

---

## 3. CLI-Übersicht (CDC#0)

Eingabe mit Enter. Befehle sind nicht case-sensitive, Argumente whitespace-getrennt.

### Allgemein

| Befehl              | Wirkung                                              |
|---------------------|------------------------------------------------------|
| `help`              | Liste aller Befehle                                  |
| `version`           | Build-Info, Pico-SDK, Emulator-Rev                   |
| `stats`             | MMIO-/Fault-/IRQ-Counter                             |
| `reset`             | Emulator-Core neu starten                            |

### Firmware

| Befehl              | Wirkung                                              |
|---------------------|------------------------------------------------------|
| `upload`            | wartet auf Intel-HEX-Stream auf CDC#0                |
| `info`              | zeigt Reset-Vector, Stack, Größe, CRC                |
| `erase`             | Firmware-Slot leeren                                 |
| `run`               | Guest starten                                        |
| `halt`              | Guest anhalten (kooperatives Halt via PendSV)        |
| `step`              | ein Befehl ausführen, dann halten                    |
| `autostart on/off`  | nach Reset automatisch starten                       |

### Konfiguration

| Befehl                    | Wirkung                                          |
|---------------------------|--------------------------------------------------|
| `cfg list`                | alle KV-Paare ausgeben                           |
| `cfg get <key>`           | einen Wert lesen                                 |
| `cfg set <key> <value>`   | Wert setzen (RAM)                                |
| `cfg save`                | RAM-Snapshot in nächsten Sektor schreiben        |
| `pinmap show`             | Tabelle LPC-Pin → RP2350-GPIO                    |
| `pinmap set <lpc> <rp>`   | Pin zuweisen, danach `cfg save`                  |
| `pinmap reset`            | Default-Tabelle wiederherstellen                 |

### Debugger / Trace

| Befehl                        | Wirkung                                          |
|-------------------------------|--------------------------------------------------|
| `gdb on/off`                  | RSP-Server auf CDC#1 ein/aus                     |
| `bp <addr>`                   | SW-Breakpoint setzen (max. 8)                    |
| `bp clr <addr>`               | Breakpoint löschen                               |
| `regs`                        | r0-r15, xPSR, MSP/PSP, CONTROL                   |
| `mem <addr> <len>`            | Hex-Dump aus dem Guest-Adressraum                |
| `swd start <swdio> <swclk>`   | externer Debug-Probe an RP2350 (clk = dio+1)     |
| `swd stop`                    | SWD-Target deaktivieren                          |
| `pio capture <pin> <count>`   | Edge-Capture-Trace (Mikrosekunden)               |

---

## 4. Firmware aufspielen

### Variante A: USB-Wechseldatenträger (empfohlen, kein CLI nötig)

Der Emulator stellt sich auch als **USB-Mass-Storage-Volume** dar
(LUN0, FAT12, 256 KiB, Label `LPC1115EMU`).

1. RP2350 anstecken → Volume erscheint im Datei-Manager / Finder.
2. Datei `BOOT.HEX` (Intel-HEX, max. 64 KiB) hineinkopieren.
3. Optional: Datei `CONFIG.INI` mit Pinmap und Optionen hineinkopieren.
4. **Volume auswerfen** ("sicheres Entfernen" / "Eject"). Beim Eject:
   * `CONFIG.INI` wird geparst (siehe unten),
   * `BOOT.HEX` wird in den Firmware-Slot geschrieben,
   * `autostart on` ist Default → Guest läuft sofort weiter
     (kein CLI-Eingriff nötig).

`CONFIG.INI` Format (UTF-8, eine Direktive pro Zeile):

```
# Kommentare beginnen mit # oder ;
autostart=on            # nach Reset automatisch starten
freq_hz=48000000        # Wunsch-Coreclock (PLL-Soll)

# Pinmap: pin.<port>_<pin>=<rp2350-gpio>
pin.0_3=14
pin.1_8=17              # KNX-RX -> uart0-RX
pin.1_9=16              # KNX-TX -> uart0-TX
pin.2_0=25              # Status-LED
```

> Wenn `BOOT.HEX` und `autostart=on` gesetzt sind, läuft der Emulator
> nach jedem Power-Cycle **vollständig autonom** ohne USB-Konsole.

### Variante B: Intel-HEX über CLI

```
emu> upload
... <jetzt HEX-Datei senden> ...
[upload] 16234 bytes, 12 records, CRC32=0xA8B3F210
emu> info
Reset:  0x00000A85    Stack: 0x10001FF8    Size: 16234    CRC32: 0xA8B3F210
emu> run
[guest] running...
```

In *PuTTY*/*Tera Term*: „Send file" → Datei `.hex` wählen, Protocol: **plain**.

### Variante C: GDB-Load über CDC#1

```
arm-none-eabi-gdb fw.elf
(gdb) target extended-remote /dev/ttyACM1
(gdb) load
(gdb) continue
```

Die geladene Firmware landet sowohl im RAM-View des Emulators als auch im
Wear-Leveling-Slot des QSPI-Flash (überlebt Power-Cycle wenn `autostart on`).

---

## 5. Pinmap konfigurieren

Selfbus-Apps wie *bim112* benutzen meist:

| sblib-Konstante  | LPC-Pin   | typische Funktion              |
|------------------|-----------|--------------------------------|
| `PIN_INFO`       | PIO2_0    | Status-LED                     |
| `PIN_RUN`        | PIO3_3    | Run-LED                        |
| `PIN_PROG`       | PIO2_0    | Prog-Taster                    |
| `PIN_EIB_RX`     | PIO1_8    | KNX-Bus RX (UART/TPUART)       |
| `PIN_EIB_TX`     | PIO1_9    | KNX-Bus TX                     |
| `PIN_IO1..IO16`  | PIO0_x / PIO1_x / PIO2_x | Eingänge          |

Mapping anpassen:

```
emu> pinmap set 0_0 14         # LPC P0_0 → RP-GP14
emu> pinmap set 1_8 16
emu> pinmap set 1_9 17
emu> cfg save
```

Adressformat: `port_pin` (z. B. `0_3`, `2_11`).

---

## 6. Debugging

### 6.1 Über USB-CDC#1 (eingebauter GDB-Stub, einfach)

```
arm-none-eabi-gdb 16in_bim112.elf
(gdb) target extended-remote /dev/ttyACM1
(gdb) monitor reset halt
(gdb) b setup
(gdb) c
```

Funktioniert mit allen Standard-GDB-Kommandos (`b`, `c`, `s`, `n`, `p`,
`x/`, Watchpoints via Cortex-M-DWT).

Single-Step nutzt **DEMCR.MON_STEP** im DebugMonitor – läuft im Run-Mode
ohne Halting-Debug zu benötigen.

### 6.2 Über externen Debug-Probe (SWD-Target)

Damit „sieht" J-Link, ST-Link oder OpenOCD/CMSIS-DAP den RP2350 wie eine
echte LPC1115:

```
emu> swd start 14 15            # SWDIO=GP14, SWCLK=GP15  (clk = dio+1!)
[swd] active. DPIDR=0x0BB11477  AHB-AP IDR=0x04770031
```

Auf der Probe-Seite:

```
openocd -f interface/cmsis-dap.cfg -f target/lpc11xx.cfg
arm-none-eabi-gdb 16in_bim112.elf -ex "target extended-remote :3333"
```

Das Ziel meldet:
* DPIDR `0x0BB11477` (ARM JEDEC-Designer 0x23B)
* AHB-AP[0] IDR `0x04770031`
* CoreSight-ROM-Table @ `0xE00FF000` mit Part-ID `0x411` (LPC1115)

> **PHY-Limit**: SWCLK robust bis ~2 MHz (CPU-getriebene TX). Für
> höhere Geschwindigkeit ist eine optionale Voll-PIO-TX-Implementation
> vorgesehen.

---

## 7. KNX-Bus anschließen (Selfbus-Apps)

Selfbus-Apps reden über die **sblib** mit dem KNX-Bus. sblib unterstützt
zwei PHYs:

* **TP-UART** (Siemens) – serielle Schnittstelle 19 200 Bd, einfache
  Verkabelung. Empfohlen.
* **bcu1/bcu2** Direktansteuerung – timing-genaues GPIO-Bit-Banging,
  dafür ist im Emulator das **PIO-Edge-Capture-Programm** in
  [src/pio_glue.cpp](../src/pio_glue.cpp) vorgesehen.

### Beispiel TP-UART

1. TP-UART-Modul (z. B. Siemens-5WG1) an RP2350-`uart0`-Pins anschließen.
2. Pinmap setzen:

   ```
   emu> pinmap set 1_8 17     # LPC PIO1_8 (RX) -> RP GP17 (uart0-RX)
   emu> pinmap set 1_9 16     # LPC PIO1_9 (TX) -> RP GP16 (uart0-TX)
   emu> cfg save
   ```

3. Firmware mit sblib-Default `BCU::begin(0x004C, 0x6049, 1)` läuft
   ohne Anpassung.

---

## 8. Persistenz

* **Konfig** (`cfg`) → 8-Sektor-Wear-Leveling am Flash-Ende.
* **Firmware-Slot** → ein zusammenhängender 64-KiB-Slot (entspricht
  LPC1115-Flash).
* **Selfbus-EEPROM** → wird per **IAP-Stub** auf den Firmware-Slot
  abgebildet (siehe [TECHNICAL.md §5](TECHNICAL.md#5-iap-rom-emulation)).
  D. h. von sblib geschriebene Parameter überleben Reset und Power-Cycle.

---

## 9. Häufige Probleme

| Symptom                                   | Ursache / Abhilfe                                  |
|-------------------------------------------|----------------------------------------------------|
| LED dauerhaft dunkel nach Reset           | Firmware crasht früh / nicht geflasht – UF2 erneut kopieren |
| LED blinkt 1 Hz, kein USB-Gerät sichtbar  | Host-USB-Kabel defekt / nur Ladekabel              |
| `info` zeigt Reset = 0x0                  | HEX-Datei unvollständig empfangen → erneut         |
| `run` führt sofort zu `HardFault`         | Stack-Top liegt außerhalb 0x10000000-0x10001FFF    |
| GDB sagt „Remote connection closed"       | TinyUSB hat Re-Enum gemacht – Port neu öffnen      |
| `swd start` → „SWCLK must be SWDIO+1"     | PIO-Programm nutzt relatives `wait pin 1`          |
| OpenOCD findet kein Target                | Pull-Ups (10 kΩ) auf SWDIO, GND verbinden!         |
| sblib-EEPROM-Write „failed"               | Firmware-Slot voll – `erase` und neu laden         |
| KNX-RX leer                               | Pinmap und sblib-Konstanten gegenchecken           |

---

## 10. Quick-Reference

```
upload          # HEX laden
run / halt      # Guest steuern
gdb on          # GDB-Stub auf CDC#1
swd start 14 15 # SWD-Target auf GP14(DIO)/GP15(CLK)
cfg save        # Persistieren
stats           # Diagnose
```
