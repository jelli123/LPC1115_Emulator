# Prüfliste – LPC1115-Emulator auf RP2350

Strukturierte Test-Checkliste zum Verifizieren **aller** Emulator-Funktionen
auf echter Hardware (Pi Pico 2 / RP2350). Abhaken, was funktioniert; Notizen
in die letzte Spalte.

> **Wichtiger Hinweis:** Stand der Implementierung ist **build-/statisch
> verifiziert, aber nicht auf Hardware getestet**. Diese Liste ist genau dazu
> da, das nachzuholen. Bei Abweichungen die jeweilige Quelle prüfen.

**Voraussetzungen / Material**

- Pi Pico 2 (RP2350A) mit aufgespieltem `build/emulator.uf2`.
- Terminalprogramm (Tera Term / PuTTY / `picocom`), 115200 8N1 für die CLI.
- USB-TTL-Adapter für die emulierte LPC-uart0 (Default GP0=TX/GP1=RX),
  eingestellt auf **19200 8N1** (für die Selbsttests).
- LED + Vorwiderstand, ein paar Jumperkabel, ggf. Poti für den ADC-Test.
- Selbsttest-HEX aus [../examples/selftest](../examples/selftest) gebaut.
- Optional: KNX-/SWD-Hardware für die fortgeschrittenen Abschnitte.

Legende: ⬜ offen · ✅ ok · ❌ Fehler

---

## A — Inbetriebnahme & USB

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| A1 | UF2-Flash | Board im BOOTSEL anstecken, `emulator.uf2` kopieren | Board re-enumeriert | ⬜ |
| A2 | CDC-Ports | Geräte-Manager / `ls /dev/ttyACM*` | 3 serielle Ports (CLI, GDB, UART-Bridge) | ⬜ |
| A3 | MSC-Volume | Datei-Explorer | Laufwerk `LPC1115EMU` (FAT12) mit `HELP.HTM` + `CONFIG.INI` sichtbar | ⬜ |
| A4 | CLI-Prompt | Terminal an CDC#0, Enter | `emu>`-Prompt | ⬜ |
| A5 | Status-LED | ohne geladene Firmware | Onboard-LED (GP25) ~1 Hz Heartbeat | ⬜ |

## B — CLI-Grundfunktionen

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| B1 | Hilfe | `help` | Befehlsliste inkl. `xmodem`, `erase`, `cfg …` | ⬜ |
| B2 | Version | `version` | `LPC1115-Emu  RP2350  SDK=2.2.0` | ⬜ |
| B3 | Status | `status` (oder `stats`) | Zustand, Zähler, GDB-Status | ⬜ |
| B5 | HELP.HTM | `HELP.HTM` vom Laufwerk im Browser öffnen | Kurzanleitung wird angezeigt | ⬜ |
| B6 | CONFIG.INI-Referenz | `CONFIG.INI` vom Laufwerk öffnen | aktive Zeilen = aktueller Zustand (inkl. `pin.*`), Optionen auskommentiert | ⬜ |
| B4 | Reset | `reset` | Neustart, erneuter Prompt | ⬜ |

## C — Firmware-Upload (alle Wege)

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| C1 | USB-MSC | `t1_blink.hex` als `BOOT.HEX` aufs Volume, auswerfen | LED an GP9 blinkt (Autostart) | ⬜ |
| C2 | CLI-Upload | `upload`, dann `t2_uart_echo.hex` als Plain-Text senden | `[upload] … bytes … CRC` | ⬜ |
| C3 | `info` | nach C2 `info` | Reset-Vektor/Stack/Size plausibel | ⬜ |
| C4 | XMODEM | `xmodem`, dann `.hex` per XMODEM-1K senden (Tera Term/`sx -k`) | `[xmodem] … CRC ok` | ⬜ |
| C5 | GDB-Load | `arm-none-eabi-gdb`, `target extended-remote <CDC#1>`, `load` | Übertragung ohne Fehler | ⬜ |
| C6 | Run/Halt | `run` … `halt` … `run` | State wechselt Running/Halted (LED) | ⬜ |

## D — Konfiguration & Pinmap

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| D1 | Pinmap zeigen | `pinmap show` | Tabelle LPC-Pin → RP-GPIO | ⬜ |
| D2 | Wert setzen | `cfg set freq_hz 48000000`, `cfg get freq_hz` | Wert zurückgelesen | ⬜ |
| D3 | Persistenz | `cfg save`, `reset`, `cfg get freq_hz` | Wert überlebt Reset | ⬜ |
| D4 | CONFIG.INI | `CONFIG.INI` mit `pin.0_7=25` aufs Volume, auswerfen, `pinmap show` | Mapping übernommen | ⬜ |
| D5 | Pinmap-Reset | `pinmap reset` | Default-Tabelle wieder aktiv | ⬜ |

## E — Peripherie mit Selbsttest-Firmware

Jeweils HEX laden (Weg aus Abschnitt C) und `run`.

| # | Test | HEX | Erwartung | Status |
|---|------|-----|-----------|:------:|
| E1 | GPIO-Ausgang | `t1_blink.hex` | LED an GP9 blinkt ~2 Hz | ⬜ |
| E2 | UART TX/RX | `t2_uart_echo.hex` | Begrüßung + Echo (Großbuchst.) auf uart0 @19200 | ⬜ |
| E3 | Timer-IRQ + NVIC | `t3_timer_irq.hex` | GP9 1 Hz, `tick=N` zählt hoch | ⬜ |
| E4 | ADC | `t4_adc_uart.hex` (+ `cfg set adc_bridge_en 1`, `cfg save`, `reset`) | `adc0=NNN` folgt Spannung an GP26 | ⬜ |
| E5 | GPIO-Eingang | `t5_gpio_input.hex` | GP3-Pegel erscheint auf GP9, `in=0/1` | ⬜ |
| E6 | IAP/Flash | `t6_iap_flash.hex` | `IAP ok` + `VERIFY ok` | ⬜ |
| E7 | IAP-Persistenz | nach E6 `reset` (NICHT neu laden), `run` | Inhalt bleibt, `VERIFY ok` | ⬜ |

## F — Weitere Peripherie (eigene Firmware / Realapps)

| # | Test | Vorgehen | Erwartung | Status |
|---|------|----------|-----------|:------:|
| F1 | SSP/SPI-Loopback | SSP0 senden, RX lesen | RX == TX (Loopback-Modell) | ⬜ |
| F2 | I²C-Bridge | `i2c on <inst 0\|1> <sda> <scl> [hz]`, `reset`, echten Slave anschließen | Slave antwortet (ACK/Daten) | ⬜ |
| F3 | WWDT-Reset | Firmware mit WDT ohne Feed | Guest-Reset (LED), RP2350 bleibt aktiv | ⬜ |
| F4 | PINT/GINT | Firmware mit Pin-Interrupt, Flanke an gemapptem GPIO | ISR feuert (bei MMIO-Aktivität) | ⬜ |
| F5 | WFI-Wakeup | `cfg set wfi_pin_wakeup on`, Firmware mit `__WFI()` | Pin-Flanke weckt Guest (opt-in) | ⬜ |
| F6 | BOD | Firmware setzt `BODRSTENA` | echte RP2350-POWMAN-BOD aktiv | ⬜ |

## G — Debugging

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| G1 | GDB-Stub aktiv | `gdb on`, `status` | `GDB=on` | ⬜ |
| G2 | GDB verbinden | `gdb` an CDC#1, `target extended-remote …` | Verbindung steht | ⬜ |
| G3 | Halt/Register | `monitor`/`Ctrl-C`, `info registers` | r0-r15/xPSR plausibel | ⬜ |
| G4 | Breakpoint | `b main`/`b <addr>`, `c` | hält am Breakpoint | ⬜ |
| G5 | Single-Step | `si` / `s` | genau ein Schritt | ⬜ |
| G6 | Speicher | `x/16xb 0x10000000` | RAM-Dump | ⬜ |
| G7 | SWD-Target | `swd start 14 15`, OpenOCD `lpc11xx.cfg` | DPIDR `0x0BB11477`, AHB-AP `0x04770031` | ⬜ |
| G8 | SWD-Mem | OpenOCD `mdw 0x10000000` | liest Guest-RAM | ⬜ |

## H — UART-Bridge (CDC#2)

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| H1 | Bridge starten | `uart start 4 5` | `[uart-bridge] TX=GP4 RX=GP5` | ⬜ |
| H2 | Durchgriff | GP4/GP5 an externe Gegenstelle, Terminal an CDC#2 | Daten bidirektional | ⬜ |
| H3 | Baudwechsel | Terminal-Baud ändern, `uart status` | neue Baudrate übernommen | ⬜ |

## I — PIO Edge-Capture

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| I1 | Capture-Trace | Rechtecksignal an Pin, `pio capture <pin> <n>` | plausible Flankenabstände | ⬜ |

## J — Zweistufiger Boot + Auto-Descriptor (Phase 3)

Beispiel-HEX aus [../examples](../examples).

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| J1 | Slot leeren | `erase` | Slot leer (`info` → kein Image) | ⬜ |
| J2 | App-Adresse | `cfg set app_start 0x3000`, `cfg set autodesc on`, `cfg save` | gesetzt | ⬜ |
| J3 | Bootloader laden | `upload` → `bootloader_…hex` | additiv geschrieben | ⬜ |
| J4 | App laden | `upload` → `in16-bim112_flashstart_0x3000_…hex` | additiv, BL bleibt erhalten | ⬜ |
| J5 | Descriptor-Log | `run` | `[EMU] Boot-Descriptor erzeugt @… start=0x3000 …` | ⬜ |
| J6 | App startet | nach J5 | BL springt in App (KNX-Verhalten) | ⬜ |
| J7 | Re-Descriptor-Schutz | gültige App erneut `run` | bestehender Descriptor bleibt (kein Überschreiben) | ⬜ |

## K — Additiver Merge & Erase (Phase 3)

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| K1 | Merge | nach J: zweites `upload` einer Datei in anderem Sektor | beide Bereiche vorhanden | ⬜ |
| K2 | Kein Auto-Erase | App laden ohne vorheriges `erase` | BL **nicht** gelöscht | ⬜ |
| K3 | Explizit löschen | `erase`, `info` | Slot leer | ⬜ |
| K4 | CONFIG.INI-Erase | `flash_erase=on` + neue `BOOT.HEX`, auswerfen | Slot vorher geleert | ⬜ |

## L — KNX / Selfbus-Realfirmware (fortgeschritten)

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| L1 | Realapp laden | `in16-bim112_…hex` laden, KNX-Pins mappen, `run` | App läuft, kein Fault | ⬜ |
| L2 | KNX-RX | TP-UART/Transceiver an Bus-Pins, ETS/knxd | Telegramme werden empfangen | ⬜ |
| L3 | KNX-TX | Schaltbefehl senden | Gerät reagiert / sendet | ⬜ |
| L4 | EEPROM-Persist | Parameter via ETS schreiben, `reset` | Parameter überleben | ⬜ |
| L5 | OTA-Update | App per KNX in emulierten BL programmieren | neue App startet (Handover) | ⬜ |

## M — Fehleranzeige & Robustheit

| # | Test | Schritte | Erwartung | Status |
|---|------|----------|-----------|:------:|
| M1 | Fault hält an | Firmware mit provoziertem Fehler `run` | `State=Faulted`, LED flackert, **kein** Board-Reset, CLI weiter bedienbar | ⬜ |
| M2 | FAULT.TXT | nach M1 Laufwerk öffnen | `FAULT.TXT` mit `[FAULT]`-Block (Typ, CFSR/HFSR, Register, `instr@PC`) | ⬜ |
| M3 | Recovery | nach M1 `reset` bzw. neue `BOOT.HEX` | Gast läuft wieder / neue Firmware startet | ⬜ |
| M4 | NVIC_SystemReset | Firmware ruft `NVIC_SystemReset()` (z. B. Selfbus-Neustart) | nur Gast startet neu, USB/CLI bleiben, kein Board-Reset | ⬜ |
| M5 | Reflash bei Betrieb | während `Running` neue `BOOT.HEX` aufspielen (MSC oder `upload`) | läuft sauber durch (Gast wird gestoppt/neu gestartet), kein Hang | ⬜ |
| M6 | IRQ-Dauerlauf | interruptgetriebene Firmware (Timer/KNX) mehrere Minuten laufen lassen | keine Faults durch IRQ-Rückkehr; Zähler steigen | ⬜ |
| M7 | Re-Eject ohne Änderung | Laufwerk ohne Schreibzugriff erneut auswerfen | **kein** erneutes Laden/Neustart (Inhalts-Hash-Dedup) | ⬜ |
| M8 | CLI-Änderung bleibt | `pinmap set …`, `cfg save`, dann Laufwerk auswerfen | CLI-Änderung wird **nicht** durch stale CONFIG.INI überschrieben | ⬜ |

---

## Fehlerbilder (Kurzreferenz)

| Symptom | Wahrscheinliche Ursache |
|---------|--------------------------|
| Selbsttest-UART stumm | Terminal nicht 19200 8N1 oder falscher Port (uart0 ≠ CDC#0) |
| `run` → HardFault (LED flackert) | Stack-Top außerhalb 0x10000000–0x10001FFF — Details in `FAULT.TXT` / seriellem `[FAULT]`-Block |
| Zweite App ersetzt BL nicht | Laden ist **additiv** – vor Vollersatz `erase` |
| BL springt nicht in App | `app_start` ≠ `applicationFirstAddress`, oder `autodesc=off` |
| `xmodem` „kein Sender erkannt" | Sender ohne CRC/1K, falscher Port |
| ADC konstant ~512 | ADC-Bridge nicht aktiv (`adc_bridge_en=1`) |

Details: [USERGUIDE.md](USERGUIDE.md) · [TECHNICAL.md](TECHNICAL.md) ·
Selbsttests: [../examples/selftest/README.md](../examples/selftest/README.md)
