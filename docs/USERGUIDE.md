# LPC1115-Emulator auf RP2350 – User-Guide

Dieses Dokument beschreibt **Bedienung** und **Workflow** des Emulators.
Die Implementierungs-Interna stehen in [TECHNICAL.md](TECHNICAL.md).

---

## 1. Was die Hardware mitbringen muss

| Funktion                 | RP2350-Pins (Default, änderbar)             |
|--------------------------|----------------------------------------------|
| USB CDC „CLI"            | USB-Port (TinyUSB) – abschaltbar (`cli_enable`) |
| USB CDC „GDB-RSP"        | USB-Port (TinyUSB) – abschaltbar (`gdb_enable`) |
| USB CDC „Serial-Adapter" | USB-Port (TinyUSB) – abschaltbar (`serial_enable`) |
| USB-MSC-Laufwerk         | USB-Port (TinyUSB) – **immer aktiv** (Recovery) |
| LPC-GPIO0 → RP-GPIO      | konfigurierbar, siehe Abschnitt 5            |
| LPC-UART0 (TX/RX)        | RP2350 `uart0` (Default GP0/GP1)             |
| UART-Bridge TX/RX        | frei wählbare GPIOs (PIO-basiert)            |
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

   Ergebnis: `build/emulator.uf2`.

2. RP2350 in BOOTSEL halten, einstecken, UF2 kopieren.
3. Bis zu **drei serielle Ports** erscheinen (je nach `*_enable` in der
   CONFIG.INI, Default alle an — deaktivierte CDCs verschwinden komplett vom USB):
   * **CLI** – Kommandozeile (z. B. `COM7` / `/dev/ttyACM0`).
   * **GDB-RSP** – GDB-Remote-Stub.
   * **Serial-Adapter** – USB-Serial-Bridge bzw. virtueller LPC-UART0.

   Die Reihenfolge der COM-Ports folgt genau dieser Liste; fehlt eine CDC,
   rücken die nachfolgenden auf. `status` zeigt die aktuelle Zuordnung an
   (`USB-CDC: CDC#0=CLI …`). Die Auswahl wirkt erst nach einem **Reset**
   (der USB-Deskriptor wird beim Boot aus der Config gebaut).
4. Außerdem erscheint ein **Wechseldatenträger** namens `LPC1115EMU`
   (FAT12, 256 KiB). Dieses MSC-Laufwerk ist **immer** vorhanden – auch wenn
   alle CDCs deaktiviert sind – und dient als Wiederherstellungs-Pfad.
5. Mit Terminal an die **CLI-CDC** verbinden (115200 8N1, Line-Ending CRLF).

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

> **Fehler halten an statt zu rebooten:** Ein nicht emulierbarer Gast-Fehler
> führt **nicht** mehr zu einem Reset des RP2350. Der Gast geht in den Zustand
> `Faulted` (LED flackert), die CLI bleibt bedienbar, und der komplette
> Fehlerbericht erscheint seriell **und** als Datei `FAULT.TXT` auf dem
> USB-Laufwerk (siehe Abschnitt 3a). Auch ein `NVIC_SystemReset()` der Firmware
> (z. B. Selfbus-Neustart) startet nur den **Gast** neu, nicht das ganze Board.

---

## 3. CLI-Übersicht (CLI-CDC)

Eingabe mit Enter. Befehle sind nicht case-sensitive, Argumente whitespace-getrennt.

> **CLI abschaltbar:** Mit `cli_enable=off` in der CONFIG.INI entfällt die
> CLI-CDC komplett (ein COM-Port weniger). Der Emulator läuft dann vollständig
> autonom (Autostart, MSC, GDB, Serial-Adapter bleiben nutzbar); Bedienung/
> Diagnose erfolgt über das USB-Laufwerk (CONFIG.INI, FAULT.TXT, DEBUG.TXT).

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
| `upload`            | wartet auf Intel-HEX-Stream auf der CLI-CDC (**additiv**) |
| `xmodem`            | Intel-HEX per XMODEM-CRC/1K empfangen (robust)       |
| `info`              | zeigt Reset-Vector, Stack, Größe, CRC                |
| `erase`             | Firmware-Slot komplett leeren                        |
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
| `gdb on/off`                  | RSP-Server auf der GDB-CDC ein/aus               |
| `bp <addr>`                   | SW-Breakpoint setzen (max. 8)                    |
| `bp clr <addr>`               | Breakpoint löschen                               |
| `regs`                        | r0-r15, xPSR, MSP/PSP, CONTROL                   |
| `mem <addr> <len>`            | Hex-Dump aus dem Guest-Adressraum                |
| `swd start <swdio> <swclk>`   | externer Debug-Probe an RP2350 (clk = dio+1)     |
| `swd stop`                    | SWD-Target deaktivieren                          |
| `pio capture <pin> <count>`   | Edge-Capture-Trace (Mikrosekunden)               |

### Serielle Schnittstellen

Der Emulator trennt sauber zwischen **zwei unabhängigen** Dingen:

* **`cdc …`** – ein eigenständiger **USB↔UART-Adapter** (PIO-basiert, auf
  beliebigen GPIOs). Hat mit dem Gast **nichts** zu tun; verwendet die
  Serial-Adapter-CDC.
* **`uart …`** – betrifft **nur den LPC-UART0 des Gasts**: entweder auf echte
  RP2350-UART-Pads routen (`uart pins`) oder virtuell direkt an die
  Serial-Adapter-CDC koppeln (`uart cdc on`).

| Befehl                        | Wirkung                                          |
|-------------------------------|--------------------------------------------------|
| `cdc start <tx> <rx>`         | USB-Serial-Konverter starten (Serial-CDC ↔ PIO-UART) |
| `cdc stop`                    | Konverter stoppen                                |
| `cdc status`                  | Pins, Baudrate, Datenfluss-Zähler                |
| `uart pins <tx> <rx>` \| `off` | LPC-UART0 auf RP2350-UART-Pads routen (Hardware) |
| `uart cdc on` \| `off`         | LPC-UART0 virtuell an die Serial-CDC koppeln     |
| `uart status`                 | LPC-UART0-Routing (HW-Pads + virtuell) anzeigen  |

> Die frühere Kommandogruppe `uart start/stop` heißt jetzt `cdc start/stop`
> und wurde ersatzlos aus dem `uart`-Namespace entfernt.
>
> **UART0-Pad-Zuordnung (F2-Primärfunktion):** `uart0` TX GP0/12/16, RX
> GP1/13/17 · `uart1` TX GP4/8/20/24, RX GP5/9/21/25. TX und RX müssen zum
> selben RP-Peripheral gehören. `uart cdc on` schließt die `cdc`-Bridge auf
> der Serial-CDC aus (beide können sie nicht gleichzeitig nutzen).
>
> Ist die Serial-CDC per `serial_enable=off` deaktiviert, sind `cdc …` und
> `uart cdc on` wirkungslos (kein USB-Endpunkt).

---

## 3a. Diagnose: `stats`, `FAULT.TXT` und Stacktrace

### `stats` / `status` – Bedeutung der Felder

```
State=Running PC=0x20002fdd mmio-traps=1031 MMIO R=268 W=320  GPIO=4  PLL-cfg=1  NVIC-W=8
CPU-target=48000000 Hz  GDB=off
USB-CDC: CDC#0=CLI CDC#1=GDB CDC#2=Serial-Adapter
SysTick: trapR=12 trapW=3 CSR=0x7 RVR=0xbb7f CVR=0x51a2 ticks=48213
Config: Flash=4096KiB seq=5 keys=18 (persistent)
PIO used: SM 2, Instr 9 | free: SM 10, Instr 87
```

| Feld          | Bedeutung                                                                 |
|---------------|---------------------------------------------------------------------------|
| `State`       | `Idle` bereit · `Running` Gast läuft · `Halted` angehalten (Debugger) · `Faulted` Gast-Fehler |
| `PC`          | Reset-PC, **nur beim Start gesetzt** – kein Live-Programmzähler. Ändert sich während der Ausführung nicht. |
| `mmio-traps`  | Summe aller erfolgreich emulierten Trap-Zugriffe (MMIO + RAM-Fallback + Flash-Lesevorgänge). Steigt, solange der Gast mit LPC-Peripherie arbeitet. |
| `MMIO R / W`  | Lese-/Schreibzugriffe auf modellierte LPC-Peripherie                      |
| `GPIO`        | GPIO-Schreibzugriffe (Pin-Ausgaben)                                       |
| `PLL-cfg`     | Anzahl PLL-/Takt-Rekonfigurationen des Gastes                             |
| `NVIC-W`      | Schreibzugriffe auf den Schatten-NVIC (`ISER`/`ICER`/`ISPR`/`ICPR`/`IPR`) |
| `CPU-target`  | übernommene LPC-Soll-Frequenz (Zeitbasis der Timer/UART-Baud). Der **reale** RP2350-Takt bleibt bei 150 MHz. |
| `GDB`         | GDB-Stub aktiv (`on`/`off`)                                               |
| `USB-CDC`     | dynamische CDC-Zuordnung (Rolle je COM-Port); `\| aus:` listet per Config deaktivierte Rollen |
| `SysTick`     | Trap-Zähler + aktuelle SysTick-Register des Gasts (Timer-Diagnose)        |
| `Config`      | Flash-Größe, Persistenz-Sequenz/Key-Anzahl, ob aus Flash geladen         |
| `PIO used`    | belegte/freie PIO-State-Machines + Instruktions-Slots (CDC-Bridge, Timer-Capture/Match) |

> **Eingefrorene Zähler sind normal:** Bleiben `mmio-traps` bei erneutem
> `stats` konstant, dreht der Gast meist eine Warteschleife (z. B. KNX-Bus-
> Idle ohne Telegramme). Der Gast läuft trotzdem.

### `FAULT.TXT` / Stacktrace lesen

Bei einem nicht emulierbaren Fehler hält der Gast an (`State=Faulted`) und der
vollständige Bericht wird seriell ausgegeben **und** als `FAULT.TXT` auf das
USB-Laufwerk gelegt – so ist die Analyse **auch ohne CLI/Serial** möglich.
Kopfzeile:

```
[FAULT] HardFault @PC=0x00004e4c (LPC 0xffffffff) CFSR=0x00000000 HFSR=0x40000000
```

* **Typ** – `HardFault`, `UsageFault`, `non-decodable` (unbekannte Instruktion
  am Trap) oder `mmio rejected` (Zugriff auf nicht modellierte Peripherie).
* **PC** – Programmzähler beim Fehler. `LPC 0x…` = Offset im LPC-Flash-Abbild;
  `0xffffffff` = die Adresse liegt nicht im Flash-Abbild.
* **CFSR/HFSR** – ARM-Fault-Statusregister; die gesetzten Bits werden darunter
  im Klartext aufgeschlüsselt.

Weitere Zeilen: `r0`–`r12`, `LR` (+ LPC-Offset), `xPSR`, `SP`, `r4`–`r7`, die
gefehlerte Instruktion `instr@PC` (zwei 16-Bit-Halbworte) und ggf. `MMFAR`/`BFAR`
(Fehleradresse).

**Abkürzungen der Fault-Bits:**

| Bit            | Bedeutung                                                          |
|----------------|--------------------------------------------------------------------|
| `IACCVIOL`     | Code-Ausführung an gesperrter Adresse (wilder Sprung/Funktionspointer) |
| `DACCVIOL`     | Datenzugriff an gesperrter Adresse (MPU)                           |
| `MSTKERR` / `MUNSTKERR` | Fehler beim Exception-Stacking / -Unstacking              |
| `IBUSERR`      | Bus-Fehler beim Instruktions-Fetch                                 |
| `PRECISERR` / `IMPRECISERR` | präziser / verzögerter Datenbus-Fehler                |
| `UNDEFINSTR`   | illegale/unbekannte Instruktion                                    |
| `INVSTATE`     | ungültiger Thumb-/EPSR-Zustand (z. B. Sprung ohne Thumb-Bit)       |
| `INVPC`        | ungültiger `EXC_RETURN`/PC bei Exception-Rückkehr                   |
| `NOCP`         | Coprozessor/FPU angesprochen, aber nicht verfügbar                 |
| `STKOF`        | Stack-Overflow (Stack-Limit verletzt)                              |
| `UNALIGNED`    | nicht ausgerichteter Speicherzugriff                               |
| `DIVBYZERO`    | Division durch Null                                                |
| `FORCED`       | eskalierter Fault (häufig: Fehler in kritischer Sektion mit gesperrten IRQs) |
| `VECTTBL`      | Fehler beim Lesen der Vektortabelle                                |

Weiterlaufen: CLI `reset` bzw. `run`, oder neue Firmware aufspielen. `FAULT.TXT`
wird beim nächsten Fehler überschrieben (nur im RAM – nach Power-Cycle weg).

---

## 4. Firmware aufspielen

> **Wichtig – additives Laden:** Alle drei Upload-Wege schreiben seit
> Phase 3 **mergend** in den 64-KiB-Slot. Eine zweite Datei überschreibt
> die zuvor geladene **nicht** komplett, sondern nur die tatsächlich
> belegten Sektoren. Das ist genau das, was man für „Bootloader laden,
> danach Applikation laden" braucht. Zum **vollständigen Ersetzen** vorher
> `erase` (CLI) ausführen bzw. `flash_erase=on` in die CONFIG.INI schreiben.

### Variante A: USB-Wechseldatenträger (empfohlen, kein CLI nötig)

Der Emulator stellt sich auch als **USB-Mass-Storage-Volume** dar
(LUN0, FAT12, 256 KiB, Label `LPC1115EMU`).

Auf dem Laufwerk liegen ab Werk zwei generierte Dateien:

| Datei         | Inhalt                                                              |
|---------------|--------------------------------------------------------------------|
| `HELP.HTM`    | Kurzanleitung (im Browser lesbar) – CLI, CONFIG.INI, `stats`, FAULT-Abkürzungen, Pinmap. |
| `CONFIG.INI`  | **Aktuelle** Einstellungen als aktive Zeilen (inkl. GPIO-Mapping) + auskommentierte Optionen mit Kurzerklärung. Editierbar. |
| `FAULT.TXT`   | erscheint nur nach einem Gast-Fehler (kompletter Fehlerbericht, siehe Abschnitt 3a). |

Die `CONFIG.INI` wird bei jedem Boot aus dem aktuell geladenen Zustand neu
erzeugt und ist damit stets eine korrekte Referenz. Ein bloßes Auswerfen ohne
Änderung löst **kein** erneutes Anwenden aus (Inhalts-Hash-Erkennung) – per CLI
vorgenommene Änderungen bleiben also erhalten.

1. RP2350 anstecken → Volume erscheint im Datei-Manager / Finder.
2. Datei `BOOT.HEX` (Intel-HEX, max. 64 KiB) hineinkopieren.
3. Optional: Datei `CONFIG.INI` mit Pinmap und Optionen hineinkopieren.
4. **Volume auswerfen** ("sicheres Entfernen" / "Eject"). Beim Eject:
   * `CONFIG.INI` wird geparst (siehe unten),
   * `BOOT.HEX` wird in den Firmware-Slot geschrieben,
   * die erkannte HEX-Datei wird anschließend **automatisch vom Laufwerk
     entfernt** und das Medium neu eingehängt (wie beim RP2350-UF2-Bootloader);
     alle anderen Dateien (CONFIG.INI, HELP.HTM, DEBUG.TXT) bleiben,
   * `autostart on` ist Default → Guest läuft sofort weiter
     (kein CLI-Eingriff nötig).

`CONFIG.INI` Format (UTF-8, eine Direktive pro Zeile):

```
# Kommentare beginnen mit # oder ;

# USB-Schnittstellen (wirken erst nach RESET!). off = die CDC fällt komplett
# aus dem USB-Deskriptor -> ein COM-Port weniger. MSC-Laufwerk bleibt immer aktiv.
cli_enable=on           # Kommandozeile (CLI/Konsole) auf eigener USB-CDC
gdb_enable=on           # GDB-Remote-Serial-Protokoll (arm-none-eabi-gdb)
serial_enable=on        # Serial-Adapter-CDC (uart_bridge bzw. uart0_cdc)

autostart=on            # nach Reset automatisch starten
freq_hz=48000000        # Wunsch-Coreclock (PLL-Soll)

# Pinmap: pin.<port>_<pin>=<rp2350-gpio>
pin.0_3=14
pin.1_8=17              # KNX-RX -> uart0-RX
pin.1_9=16              # KNX-TX -> uart0-TX
pin.2_0=25              # Status-LED

# Eigenständiger USB-Serial-Konverter (Serial-CDC <-> PIO-UART, beliebige GPIOs)
uart_bridge_en=1        # Bridge beim Boot starten
uart_bridge_tx=4        # TX-Pin (RP2350 → extern)
uart_bridge_rx=5        # RX-Pin (extern → RP2350)

# LPC-UART0 des Gasts (getrennt vom Konverter oben)
uart0_cdc=off           # on = LPC-UART0 virtuell direkt an die Serial-CDC
#uart0_tx=0             # LPC-UART0 auf echte RP-UART-Pads (Hardwareentwurf)
#uart0_rx=1

# I²C-Bridge (LPC-I²C-Master → echte RP2350-Hardware)
i2c_bridge_en=1         # Bridge aktivieren
i2c_bridge_inst=0       # 0 = i2c0, 1 = i2c1
i2c_bridge_sda=6        # SDA-Pin (RP2350-GPIO, externer Pull-up nötig)
i2c_bridge_scl=7        # SCL-Pin (RP2350-GPIO, externer Pull-up nötig)
i2c_bridge_hz=100000    # Bus-Takt (Standard 100 kHz)

# Zweistufiger Boot: Bootloader + Applikation (siehe Abschnitt 4d)
app_start=0x3000        # Flash-Adresse der Applikation
desc_addr=0x2F00        # Boot-Descriptor (0 oder weglassen = app_start-0x100)
autodesc=on             # Descriptor beim Laden automatisch erzeugen
flash_erase=off         # on = Firmware-Slot vor BOOT.HEX komplett löschen

# WFI-Pin-Wakeup (opt-in, Default aus) — siehe Hinweis unten
wfi_pin_wakeup=off      # on = WFI der Firmware auf Pin-IRQ-Wakeup patchen
```

> **WFI-Pin-Wakeup (experimentell, opt-in):** Der LPC-Gast läuft nativ ohne
> Host-Loop; eine reine `__WFI()`-Warteschleife der Firmware wird sonst nur
> durch einen MMIO-Zugriff wieder „geweckt". Mit `wfi_pin_wakeup=on` werden
> beim Laden alle `WFI`-Instruktionen auf einen SVC-Trap gepatcht. Der Host
> pollt dann echte RP2350-Pin-Flanken (PINT/GINT) sowie die Timer-/WWDT-
> Modelle und injiziert fällige LPC-IRQs.
> Einschränkungen: (1) Es wird aktiv gepollt — funktional korrekt, aber
> **nicht stromsparend**. (2) `WFI` mit aktivierten Interrupts läuft über
> einen SVC-Handler; das Idiom `__disable_irq(); __WFI();` eskaliert intern
> zu einem HardFault und wird dort äquivalent behandelt.
> (3) Der Patch ersetzt das 16-Bit-Muster `0xBF30`; ein gleich aussehendes
> Datenwort würde fälschlich getroffen — daher Default **aus** und nur für
> getestete Firmware. Auf Hardware nicht validiert.


> Wenn `BOOT.HEX` und `autostart=on` gesetzt sind, läuft der Emulator
> nach jedem Power-Cycle **vollständig autonom** ohne USB-Konsole.
> CONFIG.INI-Werte werden beim Einlesen dauerhaft gespeichert; UART-/I²C-
> Bridge werden sofort (ohne Power-Cycle) angewandt.

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

### Variante B2: XMODEM-CRC/1K über CLI (robust gegen Zeichenverlust)

Der reine HEX-Stream (Variante B) kann bei schnellem „Einfügen" ohne
Flusskontrolle einzelne Zeichen verlieren. `xmodem` überträgt blockweise
mit CRC-Prüfung und ACK/NAK und ist dadurch zuverlässig:

```
emu> xmodem
xmodem: Empfang (CRC/1K, additiv) bereit - Datei jetzt senden...
... <jetzt .hex per XMODEM senden> ...
[xmodem] 16234 hex-bytes (16896 roh empfangen), CRC ok
```

Sender-Seite:

* **Tera Term:** *Datei → Transfer → XMODEM → Send…*, Option **1K** wählen,
  die `.hex`-Datei auswählen.
* **Linux/macOS:** `sx -k -X firmware.hex < /dev/ttyACM0 > /dev/ttyACM0`
  (aus `lrzsz`; `-k` = 1K-Blöcke).

Auch der XMODEM-Upload ist **additiv** — für Vollersatz vorher `erase`.

### Variante C: GDB-Load über die GDB-CDC

```
arm-none-eabi-gdb fw.elf
(gdb) target extended-remote /dev/ttyACM1
(gdb) load
(gdb) continue
```

> Der Port `/dev/ttyACM1` ist nur ein Beispiel – die genaue Nummer hängt davon
> ab, welche CDCs aktiv sind (`gdb_enable` muss `on` sein). `status` zeigt die
> Zuordnung; unter Windows den zur „LPC-Emu GDB"-CDC gehörenden COM-Port wählen.

Die geladene Firmware landet sowohl im RAM-View des Emulators als auch im
Wear-Leveling-Slot des QSPI-Flash (überlebt Power-Cycle wenn `autostart on`).

### Variante D: Bootloader + Applikation (zweistufig, mit Auto-Descriptor)

Selfbus-Geräte bestehen aus **Bootloader** (ab `0x0000`) und **Applikation**
(ab `app_start`, Default `0x3000`). Der Emulator kann beides nacheinander in
denselben Slot laden und den vom Bootloader benötigten **App-Descriptor**
selbst erzeugen:

```
emu> erase                         # einmalig: Slot komplett leeren
emu> cfg set app_start 0x3000      # Applikations-Startadresse (= BL applicationFirstAddress)
emu> cfg set desc_addr 0           # 0 = automatisch app_start-0x100
emu> cfg set autodesc on
emu> cfg save
emu> upload                        # 1) Bootloader-HEX senden
...
emu> upload                        # 2) Applikations-HEX (@0x3000) senden – additiv!
...
emu> run
[EMU] Boot-Descriptor erzeugt @0x2F00 start=0x3000 end=0x... crc=0x... ver=0x3000
[guest] running...
```

Wichtig:

* `app_start` muss exakt der `applicationFirstAddress()` des verwendeten
  Bootloaders entsprechen (beim Selfbus-Bootloader `0x3000`).
* Der Descriptor wird nur erzeugt, wenn bei `app_start` eine plausible
  Vektortabelle erkannt wird (`autodesc=on`). Schon vorhandene gültige
  Descriptoren bleiben unangetastet.
* Die fertigen Beispiel-HEX-Dateien liegen unter `examples/`
  (`bootloader_…hex` + `in16-bim112_flashstart_0x3000_…hex`).
* Dasselbe geht über das USB-Volume: erst `CONFIG.INI` mit `app_start`/
  `autodesc`/`flash_erase=on` + Bootloader als `BOOT.HEX` ablegen, auswerfen,
  dann ein zweites Mal die App als `BOOT.HEX` ohne `flash_erase` ablegen.
* Alternativ: nur den Bootloader laden und die App per **OTA über den
  KNX-Bus** in den emulierten Bootloader programmieren (Abschnitt 7).

---

## 5. Pinmap konfigurieren

### Default-Pinmap (ohne CONFIG.INI)

Wird keine `config.ini` gefunden (erster Start, nach Factory-Reset via
`pinmap reset`), werden folgende Zuordnungen aktiv. Die Belegung orientiert
sich an der physischen Lage am LPC1115-LQFP48-Gehäuse und der Pin-Seite
des Pico 2:

| LPC-Pin | Funktion (sblib / Peripherie) | RP2350-GPIO | Pico-2-Pad |
|---------|-------------------------------|-------------|------------|
| P0_0    | GPIO / SPI0-MISO              | GP2         | Pin 4      |
| P0_1    | GPIO / SPI0-MOSI              | GP3         | Pin 5      |
| P0_2    | GPIO / SPI0-SCK               | GP4         | Pin 6      |
| P0_3    | GPIO / SPI0-CS                | GP5         | Pin 7      |
| P0_4    | GPIO / I2C-SCL                | GP6         | Pin 9      |
| P0_5    | GPIO / I2C-SDA                | GP7         | Pin 10     |
| P0_6    | GPIO                          | GP8         | Pin 11     |
| P0_7    | GPIO                          | GP9         | Pin 12     |
| P0_8    | GPIO                          | GP10        | Pin 14     |
| P0_9    | GPIO                          | GP11        | Pin 15     |
| P0_10   | GPIO                          | GP12        | Pin 16     |
| P0_11   | GPIO                          | GP13        | Pin 17     |
| P1_0    | GPIO                          | GP14        | Pin 19     |
| P1_1    | GPIO                          | GP15        | Pin 20     |
| P1_2    | GPIO                          | GP16        | Pin 21     |
| P1_3    | GPIO                          | GP17        | Pin 22     |
| P1_4    | GPIO                          | GP18        | Pin 24     |
| P1_5    | GPIO                          | GP19        | Pin 25     |
| P1_6    | GPIO                          | GP20        | Pin 26     |
| P1_7    | GPIO                          | GP21        | Pin 27     |
| P1_8    | UART RX (KNX-Bus / TPUART)   | GP1         | Pin 2      |
| P1_9    | UART TX (KNX-Bus / TPUART)   | GP0         | Pin 1      |
| P1_10   | GPIO                          | GP22        | Pin 29     |
| P1_11   | GPIO / ADC                    | GP26        | Pin 31     |
| P2_0    | GPIO (Status-LED / Prog)      | GP27        | Pin 32     |
| P2_1    | GPIO / ADC                    | GP28        | Pin 34     |

> **GP25** (Onboard-LED) ist für die Status-Anzeige reserviert und steht
> nicht als LPC-Pin zur Verfügung. Port 3 ist im Default nicht gemappt
> (zu wenige freie GPIOs).

### Selfbus-Apps

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

### 6.1 Über die USB-GDB-CDC (eingebauter GDB-Stub, einfach)

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

## 7a. USB-Serial-Konverter (`cdc`, Serial-CDC ↔ PIO-UART)

Der Emulator stellt einen eigenständigen **USB↔UART-Adapter** bereit, der
transparent auf zwei frei wählbare RP2350-GPIOs gemappt wird. Er nutzt die
**Serial-Adapter-CDC** (nur vorhanden, wenn `serial_enable=on`). Die
Datenübertragung läuft über **PIO-basiertes UART** — die Hardware-UARTs des
RP2350 werden **nicht** belegt und stehen dem Emulator (z. B. für die
LPC1115-UART0-Emulation) weiterhin zur Verfügung.

> **`cdc` vs. `uart`:** `cdc …` ist dieser gast-**unabhängige** Adapter.
> Der **LPC-UART0 des Gasts** wird stattdessen mit `uart pins` (echte Pads)
> bzw. `uart cdc on` (virtuell an dieselbe Serial-CDC) bedient — siehe
> Abschnitt 3 und 7b.

### Anwendungsfälle

* Direkte Kommunikation mit einem TP-UART-Modul vom PC aus (Debugging)
* Transparenter serieller Durchgriff zu beliebiger externer Hardware
* Log-Ausgabe der Gast-Firmware über einen zweiten UART-Kanal

### Aktivierung

**Variante A — über CLI:**

```
emu> cdc start 4 5            # TX=GP4, RX=GP5
ok
```

Zum Persistieren (startet dann automatisch beim nächsten Boot):

```
emu> cfg set uart_bridge_en 1
emu> cfg set uart_bridge_tx 4
emu> cfg set uart_bridge_rx 5
emu> cfg save
```

**Variante B — über CONFIG.INI (USB-Volume):**

```ini
uart_bridge_en=1
uart_bridge_tx=4
uart_bridge_rx=5
```

### Baudrate setzen

Die Baudrate wird **vom Host-PC** über den Standard-CDC-Mechanismus
gesetzt (`SET_LINE_CODING`). Jedes Terminal-Programm, das die Baudrate
konfigurieren kann, funktioniert (Port = die Serial-Adapter-CDC):

```bash
# Linux (Beispiel-Port; genaue Nummer je nach aktiven CDCs)
stty -F /dev/ttyACM2 19200
picocom -b 19200 /dev/ttyACM2

# Windows (PowerShell)
mode COM9: BAUD=19200 PARITY=N DATA=8 STOP=1
```

Die PIO-Clock wird **live** angepasst — ein Neustart der Bridge ist
nicht nötig.

### Status abfragen

```
emu> cdc status
cdc-bridge=active TX=GP4 RX=GP5 baud=19200
  Fluss: CDC-RX=0 -> PIO-TX=0 -> PIO-RX=0 -> CDC-TX=0
```

### Stoppen

```
emu> cdc stop
```

---

## 7b. LPC-UART0 des Gasts (`uart pins` / `uart cdc`)

Der **LPC-UART0 des emulierten Gasts** kann auf zwei Wegen nach außen geführt
werden (unabhängig vom `cdc`-Adapter oben):

* **Auf echte RP2350-UART-Pads** (Hardwareentwurf, echte TX/RX-Leitungen):

  ```
  emu> uart pins 0 1          # TX=GP0, RX=GP1 (uart0)
  emu> uart pins off          # Routing entfernen
  ```

  Zulässige Pads (F2-Primärfunktion): `uart0` TX GP0/12/16, RX GP1/13/17 ·
  `uart1` TX GP4/8/20/24, RX GP5/9/21/25. TX und RX müssen zum selben
  RP-Peripheral gehören.

* **Virtuell direkt an die Serial-Adapter-CDC** (kein Draht, ideal für
  `printf`/Serial des Gasts am PC):

  ```
  emu> uart cdc on            # LPC-UART0 <-> Serial-CDC
  emu> uart cdc off
  ```

  `uart cdc on` schließt den `cdc`-Adapter auf der Serial-CDC aus (beide
  können sie nicht gleichzeitig nutzen). Ist `serial_enable=off`, ist die
  Kopplung wirkungslos (kein USB-Endpunkt).

```
emu> uart status
LPC-UART0: HW-Pads TX=GP0 RX=GP1 | Serial-CDC-virtuell=off
```

Entsprechende CONFIG.INI-Schlüssel: `uart0_cdc`, `uart0_tx`, `uart0_rx`.

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
| LED flackert (8 Hz), `stats` zeigt `Faulted` | Gast-Fehler – `FAULT.TXT` auf dem Laufwerk bzw. seriellen `[FAULT]`-Block lesen (Abschnitt 3a) |
| GDB sagt „Remote connection closed"       | TinyUSB hat Re-Enum gemacht – Port neu öffnen      |
| `swd start` → „SWCLK must be SWDIO+1"     | PIO-Programm nutzt relatives `wait pin 1`          |
| OpenOCD findet kein Target                | Pull-Ups (10 kΩ) auf SWDIO, GND verbinden!         |
| sblib-EEPROM-Write „failed"               | Firmware-Slot voll – `erase` und neu laden         |
| KNX-RX leer                               | Pinmap und sblib-Konstanten gegenchecken           |
| Zweite App ersetzt Bootloader nicht / alte Reste | Laden ist **additiv** – vor Vollersatz `erase`     |
| Bootloader springt nicht in App           | `app_start` ≠ `applicationFirstAddress`, oder `autodesc=off` – Log `[EMU] Boot-Descriptor …` prüfen |
| `xmodem` „kein Sender erkannt"            | Sender nutzt kein CRC/1K, oder falscher COM-Port    |

---

## 10. Quick-Reference

```
upload          # HEX laden (additiv)
xmodem          # HEX per XMODEM-CRC/1K laden (robust)
erase           # Firmware-Slot komplett löschen
run / halt      # Guest steuern
gdb on          # GDB-Stub auf der GDB-CDC
swd start 14 15 # SWD-Target auf GP14(DIO)/GP15(CLK)
cfg save        # Persistieren
stats           # Diagnose
```
