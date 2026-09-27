# Virtueller NCN5130 – Emulations-Spezifikation (Variante B)

Status: **Entwurf v0.1** · Basis: ON-Semi *NCN5130-D* Datenblatt (`docs/NCN5130-D.PDF`,
extrahierter Text: `docs/ncn5130_text.txt`) · Ziel-Plattform: LPC1115-Emulator auf RP2350.

Dieses Dokument spezifiziert einen **voll virtuellen NCN5130** als KNX-Sekundärinterface,
das der emulierte LPC-Gast über seine **SSP/SPI-Schnittstelle** anspricht. Es entsteht
**kein** zusätzlicher KNX-Chip im Emulator; das KNX-Layer-2/-Timing wird so weit
nachgebildet, wie es der Gast-Treiber (Referenzsoftware, noch zu bauen) tatsächlich sieht.
Alle Byte-Codes und Protokolleigenschaften unten sind aus dem Datenblatt verifiziert.

> Schwesterdokument: **`docs/NCN5130_INTERFACE.md`** — die Schnittstellenbeschreibung
> (Emulator-C++-API + Gast-seitiger Treiber-Kontrakt).

---

## 1. Kernentscheidung: NCN5130 ist SPI-**Master**

Datenblatt (S. 28, Fig. 26 „SPI Master"): Im SPI-Modus ist **der NCN5130 der SPI-Master**,
der **Host-Controller (LPC) der Slave**. `CSB` und `SCK` sind **Ausgänge** des NCN5130.

| Signal (NCN5130) | Richtung | Host-Controller (LPC) |
|---|---|---|
| `SCK/UC2` | Ausgang (Master-Clock) | Slave-Clock-Eingang |
| `CSB/UC1` | Ausgang (Slave-Select) | Slave-Select-Eingang |
| `SDO/TXD` | Ausgang (Device→Host) | MISO-Eingang |
| `SDI/RXD` | Eingang (Host→Device) | MOSI-Ausgang |
| `TREQ` | **Eingang** (Transmit Request) | GPIO-Ausgang des Hosts |

- **SPI-Modus 0** (CPOL = 0, CPHA = 0): Daten werden an der **fallenden** Flanke ausgegeben
  und an der **steigenden** Flanke gesampelt.
- **LSB zuerst** (Datenblatt Fig. 25/30 – im Gegensatz zum LPC-SSP-Default MSB-first!).
- Baudrate 125 kbps oder 500 kbps (für den Emulator irrelevant – Byte-transparent).

### 1.1 TREQ – Transmit Request (Host → Device)

Weil der NCN5130 Master ist, kann der Host **nicht selbst** einen Transfer takten. Ablauf
(Datenblatt Fig. 27):

1. Host will ein Byte senden → zieht **`TREQ` auf negative Flanke** (high→low).
2. NCN5130 erkennt die Flanke und taktet **8 Dummy-Bits** als Master.
3. Während dieser 8 Takte schiebt der Host sein Byte über `SDI` in den NCN5130
   (und empfängt gleichzeitig ein Byte auf `SDO`).

Für empfangene KNX-Daten taktet der NCN5130 **von sich aus** (er ist Master) und schiebt
die Bytes über `SDO` zum Host; parallel signalisiert er per `TREQ`/Timing bzw. der Host
pollt `U_State`.

### 1.2 Konsequenz für das bestehende SSP-Modell

Die vorhandene SSP-Bridge (`peripherals.cpp`, `SspModel g_ssp[2]`) modelliert den LPC-SSP
als **Master** (ein `DR`-Write löst einen Transfer aus). Der virtuelle NCN5130 kehrt die
Rolle **nicht** hardwaregenau um – der Emulator arbeitet **byte-transparent**:

- Jeder vom Gast in `SSP_DR` geschriebene Wert ist ein `MOSI`-Byte (Host→Device).
- Der als `dr_rx` zurückgelieferte Wert ist das zugehörige `MISO`-Byte (Device→Host).
- Das **TREQ-Signal** wird als virtueller GPIO modelliert (siehe §5): Der Gast-Treiber
  toggelt es vor jedem Sende-Byte; das NCN-Modell nutzt es als „Byte folgt"-Trigger.
- **Bit-Reihenfolge (LSB-first):** Der Emulator tauscht Bytes als 8-Bit-Werte – die
  LSB-first-Serialisierung ist auf Byte-Ebene transparent, **solange** der Gast-Treiber
  das SSP-Format konsistent auf LSB-first (bzw. selbst-konsistent) setzt. Der Emulator
  **spiegelt** die vom Gast gewählte Bit-Order nicht auf den (virtuellen) Draht – er
  vergleicht nur Byte-Werte. Ein realer Aufbau erfordert LSB-first; im Emulator ist es
  ohne Wirkung. **Hinweis für die Referenzsoftware:** SSP auf LSB-first + Mode 0 setzen,
  damit derselbe Treiber auf echter Hardware funktioniert.

---

## 2. Digitale Zustandsmaschine

Datenblatt Table 10/Fig. 29. Der `Mode`-Code steht in den **2 niederwertigen Bits** von
`U_SystemStat.ind`:

| Code | Zustand | Bedeutung (Emulator) |
|---|---|---|
| `00` | **Power-Up** | Nach POR/Reset; VBUS/VFILT/XTAL „ok" sofort simuliert. KNX-RX aus. |
| `01` | **Sync** | Wartet auf ≥ 40 Tbit Bus-Ruhe; RX an, aber keine Frames an Host. |
| `10` | **Stop** | Sicheres Setup / RX-Pause. |
| `11` | **Normal** | Voll funktionsfähig; KNX-Kommunikation erlaubt. |

Zusätzlich orthogonal: **Bus-Monitor** (per `U_Busmon.req`) – liegt „über" Normal/Stop/Sync.

Übergänge (vereinfacht, für den Emulator ausreichend):

```
POR ─▶ RESET ─(init, Features aus)─▶ POWER-UP(00)
POWER-UP ─(≥40 Tbit Bus-Ruhe)─▶ SYNC(01) ─▶ NORMAL(11)   [Emulator darf direkt nach NORMAL]
NORMAL ─(U_StopMode.req)─▶ STOP(10) ─(U_ExitStopMode.req)─▶ NORMAL(11)
* ─(U_Reset.req / POR)─▶ RESET ─▶ POWER-UP(00)
```

Beim Eintritt in **Normal** sendet das Modell **`U_Reset.ind` (0x03)** an den Host.

**Emulator-Vereinfachung:** Da es keinen echten Quarz/VBUS gibt, darf das Modell die
Analog-Vorbedingungen (VBUS/VFILT/XTAL) als „sofort gut" behandeln und Power-Up→Sync→Normal
zügig durchlaufen. Der Gast-Treiber sieht denselben Ablauf wie an echter Hardware.

---

## 3. Aufgabenteilung Host ↔ NCN5130 (Datenblatt S. 29)

| Aufgabe | Host (LPC/SBLib) | NCN5130 (virtuell) |
|---|---|---|
| Checksum (FCS) | ✔ berechnet/prüft | ✔ (redundant) |
| Parität | ✔ | ✔ |
| **Adressierung** | ✔ | – |
| **Länge** | ✔ | – |
| **Acknowledge** (IACK/NACK/BUSY) | – | ✔ |
| **Repetition** (Wiederholung) | – | ✔ |
| **Timing** (Bus) | – | ✔ |

**Emulatorrelevanz:** Adressierung/Länge/L2-Logik liegen im Gast (SBLib) → **müssen im
NCN-Modell NICHT nachgebaut werden**. Das Modell muss nur Acknowledge, Repetition-Quittung
(`L_Data.con`) und das Byte-transparente Weiterreichen empfangener Frames leisten.

---

## 4. Kommandosatz (verifiziert aus Table 12/13)

### 4.1 Services **vom Host** (Table 12) – Control-Byte + Folgebytes

| Hex | Service | Folgebytes | Total | Emulator-Verhalten |
|---|---|---|---|---|
| `0x01` | `U_Reset.req` | – | 1 | Reset → Power-Up; sendet `U_Reset.ind` bei Normal. |
| `0x02` | `U_State.req` | – | 1 | Antwort `U_State.ind` (Fehlerflags, sonst 0x07). |
| `0x03` | `U_SetBusy.req` | – | 1 | BUSY-Mode an (nur wirksam bei Auto-Ack). |
| `0x04` | `U_QuitBusy.req` | – | 1 | BUSY-Mode aus. |
| `0x05` | `U_Busmon.req` | – | 1 | Bus-Monitor an (nur per Reset verlassbar). |
| `0xF1` | `U_SetAddress.req` | AddrHigh, AddrLow, X | 4 | Phys. Adresse + Auto-Ack an; Antwort `U_Configure.ind`. |
| `0xF2` | `U_SetRepetition.req` | RepCntrs, X, X | 4 | Max. Wiederholungen setzen. |
| `0x08–0x0C` | `U_L_DataOffset.req` | (iii = MSB-Index 0…4) | 1 | Offset des nächsten Sende-Frames. |
| `0x0D` | `U_SystemState.req` | – | 1 | Antwort `U_SystemStat.ind` + 1 Statusbyte. |
| `0x0E` | `U_StopMode.req` | – | 1 | → Stop; Antwort `U_StopMode.ind` (0x2B). |
| `0x0F` | `U_ExitStopMode.req` | – | 1 | Stop verlassen → Normal. |
| `0x10–0x17` | `U_Ackn.req` | (n=nack, b=busy, a=addr) | 1 | Quittung des empfangenen Frames (Bit0..2). |
| `0x18–0x1F` | `U_Configure.req` | (p=poll, c=CRC, m=MARKER) | 1 | Features **nur setzen** (0-Bits ohne Wirkung); Antwort `U_Configure.ind`. |
| `0x28–0x2F` | `U_IntRegWr.req` | Datenbyte | 2 | Internes Register (aaa) schreiben. |
| `0x38–0x3F` | `U_IntRegRd.req` | – | 1 | Nächstes Byte = Registerinhalt (aaa). |
| `0xE0–0xEE` | `U_PollingState.req` | PollAddrH, PollAddrL, PollState | 4 | Slot s (0…14) für Auto-Polling. |
| `0x80` | `U_L_DataStart.req` | Control-Octet (CTRL) | 2 | Beginn eines Sende-Frames. |
| `0x81–0xBF` | `U_L_DataCont.req` | Daten-Octet | 2 | Frame-Byte i (1…63): CTRLE/SA/DA/AT/NPCI/LG/TPDU. |
| `0x47–0x7F` | `U_L_DataEnd.req` | Check-Octet (FCS) | 2 | l = letzter Index +1 (7…63); löst Bus-TX aus. |

Hinweise:
- **Single-Byte** vs. **Multi-Byte**: Nach einem Control-Byte, das Folgebytes erwartet, sind
  die nächsten `n` Bytes Datenbytes zu diesem Kommando (kein neues Control-Byte).
- **Interne vs. KNX-Kommandos**: `U_L_Data*` starten KNX-Bus-Verkehr; alle anderen sind
  gerätelokal.
- **`U_IntRegWr` über SPI**: Datenblatt empfiehlt, dafür in **Stop-Mode** zu gehen
  (Synchronisation). Das Modell darf den Write jederzeit annehmen, sollte den Hinweis aber
  im Interface dokumentieren.

### 4.2 Services **zum Host** (Table 13)

| Bitmuster | Hex (Basis) | Service | Bedeutung |
|---|---|---|---|
| `10r1 p1p0 00` | – | `L_Data_Standard.ind` | Empf. Standard-Frame; r=nicht-wiederholt(1), p=Prio; danach n Datenbytes. |
| `00r1 p1p0 00` | – | `L_Data_Extended.ind` | Empf. Extended-Frame; danach n Datenbytes. |
| `1111 0000` | `0xF0` | `L_Poll_Data.ind` | Poll-Antwort; danach n Bytes. |
| `xx00 xx00` | – | `L_Ackn.ind` | Acknowledge-Frame (nur im Bus-Monitor sichtbar). |
| `z000 1011` | `0x0B`/`0x8B` | `L_Data.con` | Sende-Quittung; z=positiv(1)/negativ(0). |
| `0000 0011` | `0x03` | `U_Reset.ind` | Reset abgeschlossen (Normal erreicht). |
| `sc re te pe tw 111` | `0x07`+Flags | `U_State.ind` | Kommunikations-Statusflags. |
| `re ce te 1 res 011` | `0x0B`+Flags | `U_FrameState.ind` | Frame-Fehlerflags (Parität/Checksum/Timing). |
| `0b aa ap c m 01` | – | `U_Configure.ind` | Aktueller Feature-Zustand. |
| `1100 1011` | `0xCB` | `U_FrameEnd.ind` | Frame-Ende (nur mit MARKER-Feature). |
| `0010 1011` | `0x2B` | `U_StopMode.ind` | Stop-Mode bestätigt. |
| `0100 1011` | `0x4B` | `U_SystemStat.ind` | + 1 Byte: V20V, VDD2, VBUS, VFILT, XTAL, TW, Mode. |

`U_State.ind`-Flags (Bit7..3): `sc` Slave-Collision, `re` Receive-Error, `te` Transceiver-Error,
`pe` Protocol-Error, `tw` Thermal-Warning. Fehlerfrei = `0x07`.

`U_SystemStat.ind`-Datenbyte: **Mode** in Bit1..0 (siehe §2), obere Bits =
Analog-Statusbits (im Emulator „gut"/fix).

---

## 5. Ablaufmodelle

### 5.1 Reset

```
Host  ── 0x01 (U_Reset.req) ─▶  NCN
NCN   ── (init, Features aus, Power-Up) ─▶
NCN   ── 0x03 (U_Reset.ind) ─▶  Host        (bei Eintritt Normal)
```

### 5.2 Frame senden (Host → KNX)

```
Host ─ 0x80, CTRL              (U_L_DataStart.req)
Host ─ 0x81, byte1             (U_L_DataCont.req, i=1)
Host ─ 0x82, byte2             (U_L_DataCont.req, i=2)
        …                      (SA, DA, AT, NPCI, LG, TPDU …)
Host ─ (0x47+lastIdx), FCS     (U_L_DataEnd.req)   ─▶ NCN sendet auf KNX-Bus
NCN  ─ (Repetition bis IACK / max. Retries) ─▶
NCN ─ 0x8B / 0x0B (L_Data.con) ─▶ Host   (positiv / negativ)
```

Das NCN-Modell reiht die Bytes im **TX-Assembly-Puffer** und übergibt bei `U_L_DataEnd`
den Frame an das **Back-End** (§7). Repetition/ACK-Warten wird zeitlich modelliert; das
Ergebnis kommt als `L_Data.con`.

### 5.3 Frame empfangen (KNX → Host)

```
KNX  ─ Frame ─▶ NCN (Back-End/PHY)
NCN ─ Ctrl-Byte (L_Data_Standard/Extended.ind) ─▶ Host
NCN ─ byte1, byte2, … (transparent) ─▶ Host
[CRC aktiv]  NCN ─ CRC_hi, CRC_lo ─▶ Host
[MARKER aktiv] NCN ─ 0xCB (U_FrameEnd.ind) + U_FrameState.ind ─▶ Host
[sonst] Frame-Ende = 2.6 ms Stille auf SDO
Host ─ 0x1x (U_Ackn.req: n/b/a) ─▶ NCN   (quittiert; NCN sendet IACK/NACK/BUSY auf Bus)
```

**DATA_READY / IRQ:** Sobald empfangene Bytes bereitstehen, signalisiert das Modell dem
Gast einen Interrupt – über das **PINT/`irq_inject`-Muster** wie beim UART0-RX
(inkl. **Level-Halte-Flag**, damit kein IRQ-Sturm entsteht; siehe Lessons #31/#32 im
Repo-Gedächtnis). Pollt der Gast-Treiber nur `U_State`/eine Ready-Leitung, genügt der
GPIO-Read.

---

## 6. Emulator-Modul – interne Struktur

Neues Modul **`src/ncn5130.{h,cpp}`** (Vorschlag). Wird vom SSP-Intercept aufgerufen.

### 6.1 Intercept-Punkt

In `peripherals.cpp`, `ssp_write_byte`, DR-Zweig, existiert heute der Loopback-Fallback:

```cpp
} else {
    s.dr_rx = s.tx;        // Loopback-Fallback  ← HIER
}
```

Ein neuer, je SSP-Instanz wählbarer Modus **NCN** ersetzt Loopback **und** HW-Bridge:

```cpp
if (ssp_is_ncn(idx)) {
    s.dr_rx = ncn5130::spi_exchange(static_cast<uint8_t>(s.tx & 0xFF));
} else if (ssp_is_bridged(idx)) {
    …  // reale SPI-HW (unverändert)
} else {
    s.dr_rx = s.tx;        // Loopback
}
```

Der übrige SSP-Pfad (`SR`, `RIS`, IRQ-Pend) bleibt unverändert.

### 6.2 Zustand (RAM-Budget < 2 KB)

> **RAM-Trennung (wichtig):** Die folgenden Puffer liegen **emulatorseitig** (RP2350,
> 520 KB) und kosten **null LPC-RAM**. Der LPC1115 hat nur 8 KB SRAM und mit SBLib wenig
> frei – deshalb muss der **Gast-Treiber** ein Streaming-Shim ohne eigenen Frame-Puffer
> sein (siehe `NCN5130_INTERFACE.md` §0.1). Und weil der NCN5130 SPI-**Master** ist, gehören
> weitere SPI-Geräte an die **zweite** LPC-SSP-Instanz (§0.2 dort).

```cpp
struct Ncn5130 {
    uint8_t  state;          // Power-Up/Sync/Stop/Normal (+ busmon-Flag)
    uint8_t  cfg;            // Feature-Bits: auto-poll, CRC, MARKER, auto-ack, busy
    uint16_t phys_addr;      // aus U_SetAddress.req
    uint8_t  rep_cnt;        // aus U_SetRepetition.req

    // Empfangs-Pfad (KNX → Host)
    uint8_t  rx_frame[280];  // ein Standard/Extended-Frame (≤ 263 B + Overhead)
    uint16_t rx_len, rx_pos; // gefüllt vom Back-End, ausgeschoben zum Host
    bool     rx_ready;       // treibt DATA_READY/IRQ

    // Sende-Pfad (Host → KNX)
    uint8_t  tx_frame[280];
    uint16_t tx_len;
    uint8_t  cmd_state;      // Parser: Idle/CollectN/…
    uint8_t  cmd_expect;     // erwartete Folgebytes

    // Antwort-FIFO (Device → Host, wird bei nächstem SPI-Exchange ausgeschoben)
    uint8_t  resp[16];
    uint8_t  resp_head, resp_tail;

    // internes Registerfile (device-spezifisch, aaa = 0..7)
    uint8_t  ireg[8];
};
```

### 6.3 `spi_exchange`-Automat (Kern)

```
uint8_t spi_exchange(uint8_t mosi):
    // 1) Wenn ein Antwort-Byte ansteht (resp-FIFO), dieses als MISO liefern.
    // 2) mosi in den Kommando-Parser geben:
    //      Idle:      Control-Byte dekodieren (Tabellen §4.1) → cmd_state/cmd_expect
    //      CollectN:  Datenbytes sammeln; bei Vollständigkeit Aktion ausführen
    //                 (Frame-TX enqueue, Register-Write, Config, …)
    // 3) Ergebnis-Bytes (Antworten/Indications) in resp-FIFO legen.
    // 4) Das zu liefernde MISO-Byte zurückgeben (Default 0x00, wenn nichts ansteht).
```

Nicht implementierte/unnötige Kommandos → **plausible Dummy-Antwort** (ACK / feste
Defaults), nie „hängen". Das erfüllt die Vorgabe „aufs Notwendigste beschränken".

---

## 7. Back-End (KNX-PHY am Sekundär-Bus)

Der virtuelle NCN5130 nutzt für den **realen Sekundär-Bus** den bereits vorhandenen
diskreten PHY-Pfad (Selfbus/STKNX-kompatibel, „dummer PHY"):

- **TX**: Bit-Timing über **CT-Match** bzw. **PIO-TX** auf einen eigenen RP2350-Pin
  (Transceiver des Sekundärinterfaces).
- **RX**: **CT-Capture** bzw. **PIO-RX** von einem eigenen RP2350-Pin.
- **Loopback/Monitor**: Für reine Firmware-Tests ohne Transceiver kann das Back-End die
  gesendeten Frames direkt in den RX-Pfad zurückspiegeln (Selbsttest).

**Autonomie während Flash-Stalls (siehe §8):** Die zeitkritische Bit-Ebene läuft in **PIO**;
optional entkoppelt **DMA** (SRAM ↔ PIO-FIFO) die Bit-Erzeugung vollständig von der CPU.

---

## 8. Robustheit gegen Flash-Stalls (DMA)

`flash_range_erase/program` (Config-Save, MSC) legt XIP still und stoppt CPU-Zugriffe aus
dem Flash (das VTOR-/FlashPauseGuard-Thema). Für KNX-Timing gilt:

- **DMA holt keine Instruktionen** → ein DMA-Kanal **SRAM ↔ PIO-FIFO**, getriggert vom
  **PIO-DREQ**, läuft **ohne CPU/IRQ** durch den Stall weiter. Voraussetzung: Ringpuffer
  **und** DMA-Steuerblöcke liegen im **SRAM**.
- **RX** wird damit voll robust; **TX** eines bereits eingereihten Telegramms läuft zu Ende.
- **Ergänzung:** Config-Persistenz nur in **Bus-Idle-Fenstern** (der Emulator schreibt
  ohnehin deferred – nur bei Gast-Stop / bewusstem `cfg save`). Damit wird ein
  Mid-Telegramm-Flash ausgeschlossen. KNX-ACK/Repetition fängt seltene Verzögerungen ab.

Kein Zwang: Ohne DMA funktioniert alles, solange die PIO-FIFO tief genug ist bzw. der Stall
kürzer als das FIFO-Polster; DMA ist der robuste Ausbau.

---

## 9. Ressourcen (bestätigt)

| Ressource | Verfügbar | Bedarf | Bewertung |
|---|---|---|---|
| PIO | ~12 SM / 96 Instr (FT12: 0 belegt) | 2–4 SM PHY + opt. DMA-Feed | reichlich |
| DMA | 12 Kanäle | 1–2 pro Richtung | reichlich |
| RAM | 520 KB (Image 64K + Gast-RAM 8K + MSC 256K + hex 64K …) | NCN-Modell < 2 KB | vernachlässigbar |
| Flash | ~4 MB | KB-Bereich | reichlich |
| Cores | Core0 Poll-Leerlauf; Core1 = Gast | KNX langsam (9600 bps) | kein neuer Core |
| Wiederverwendung | SSP-Modell, PINT+irq_inject, CT-Capture/Match, PIO | – | großer Vorteil |

---

## 10. Offene Punkte / Annahmen

- **`U_SystemStat.ind`-Datenbyte-Bitlayout** (Reihenfolge V20V/VDD2/VBUS/VFILT/XTAL/TW über
  den Mode-Bits) ist im extrahierten Text nicht bitgenau; für den Emulator „alles gut" +
  Mode-Bits. Vor Hardware-Abgleich am PDF (S. 37) verifizieren.
- **Interne Register (aaa = 0…7)**: konkrete Adressbelegung steht in „Internal
  Device-Specific Registers" (S. 54) – für den Emulator zunächst als RAM-Registerfile mit
  Read-back; reale Semantik nur soweit nötig.
- **CRC-CCITT**: nur relevant, wenn der Gast `c`-Feature aktiviert; dann 2 Byte je RX-Frame
  anhängen (CRC-16-CCITT).
- **Bit-Order**: Emulator ist byte-transparent (§1.2). Reale HW braucht LSB-first.

---

## 11. Umsetzungsschritte (Vorschlag)

1. **Skelett** `src/ncn5130.{h,cpp}` + `ssp_is_ncn`-Modus im SSP-Intercept (Antworten =
   Dummy/ACK) → Gast kann ohne Hänger initialisieren.
2. **Kommando-Parser** (§4.1) + Antwort-FIFO (§4.2): Reset/State/Config/SetAddress echt.
3. **RX-Pfad** + DATA_READY/IRQ (§5.3) über PINT/`irq_inject` (Level-Hold).
4. **TX-Pfad** (§5.2) + `L_Data.con`.
5. **Back-End** an CT-Capture/Match bzw. PIO (§7), zunächst Loopback/Monitor.
6. Optional **DMA-Entkopplung** (§8).
7. CLI: `ncn on <ssp-inst>`/`ncn off`/`ncn status` (analog `i2c`/`cdc`), Config-Persistenz.

Die **Gast-seitige Referenzsoftware** (SBLib-NCN5130-Treiber) wird gegen das
Schwesterdokument **`NCN5130_INTERFACE.md`** gebaut.
