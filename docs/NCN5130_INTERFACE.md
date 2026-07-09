# NCN5130 – Schnittstellenbeschreibung

Status: **Entwurf v0.1** · Gehört zu `docs/NCN5130_EMULATION_SPEC.md` · Locale: de

Dieses Dokument beschreibt **zwei** Schnittstellen:

- **A) Gast ↔ virtueller NCN5130** (das SPI-Protokoll, gegen das die *Referenzsoftware*
  gebaut wird) – identisch zu echter NCN5130-Hardware.
- **B) Emulator-intern** (die C++-Modul-API `ncn5130::*` und die SSP-Anbindung).

Alle Byte-Werte sind aus dem Datenblatt `docs/NCN5130-D.PDF` verifiziert.

---

## 0. Zwei harte Design-Randbedingungen (zuerst lesen)

### 0.1 LPC-RAM knapp → Treiber MUSS ein Streaming-Shim sein

Der LPC1115 hat **8 KB SRAM**; mit geladener SBLib ist davon nur noch wenig frei. Deshalb
gilt für die Referenzsoftware **verbindlich**:

- **Kein eigener Frame-Puffer im Treiber.** Der NCN5130 überträgt byte-transparent; der
  Treiber streamt jedes empfangene Byte **direkt in den vorhandenen SBLib-L2-Telegramm-
  puffer** und sendet aus dem SBLib-Sendepuffer heraus Byte für Byte. Kein Doppelpuffern.
- Eigener RAM-Footprint des Treibers ⇒ nur **State + kleine SPI-Byte-Queue** (zweistellige
  Byte-Zahl), nicht die 263 B eines Extended-Frames.
- Die **schweren Puffer liegen emulatorseitig** (`src/ncn5130.cpp`, RP2350-RAM) und kosten
  **null LPC-RAM**. Der Emulator modelliert `LPC_GUEST_RAM_SIZE = 8 KB` (echte Größe) →
  eine Fehldimensionierung des Gast-Treibers fällt im Emulator genauso auf wie auf HW.

### 0.2 SPI-Bus-Rolle: NCN5130 ist Master → weitere SPI-Geräte an die **andere** SSP-Instanz

Weil der NCN5130 **SPI-Master** ist (er treibt SCK/CSB), ist der LPC auf **diesem** Bus
**Slave** und kann dort **keine** weiteren Geräte als Master ansteuern (nur ein Master pro
Bus). Der LPC1115 hat aber **zwei** SSP-Blöcke – Trennung löst das:

| SSP-Instanz | LPC-Rolle | Nutzung |
|---|---|---|
| z. B. **SSP0** (`0x40040000`) | **Slave** | dediziert für den NCN5130 (Master) |
| z. B. **SSP1** (`0x40058000`) | **Master** | frei für weitere SPI-Geräte (Sensoren, Flash …) |

Im Emulator ist der NCN-Modus **pro Instanz** wählbar (`ncn5130::set_enabled(int lpc_ssp,…)`),
die andere Instanz bleibt normaler Master (auf HW gebrückt oder Loopback). Beide parallel.
⇒ **Ja**, weitere SPI-Geräte funktionieren – nur nicht am NCN5130-Bus, sondern an der
zweiten SSP.

---

## A. Gast-Schnittstelle (Referenzsoftware-Kontrakt)

Die Referenzsoftware (LPC/SBLib-Treiber) redet mit dem virtuellen NCN5130 **exakt wie mit
dem echten Chip**. Wer sich an diesen Kontrakt hält, läuft im Emulator **und** auf realer
Hardware.

### A.1 Physische Anbindung (SPI-Slave am Host)

| Eigenschaft | Wert | Anmerkung |
|---|---|---|
| Rolle | **NCN5130 = Master, LPC = Slave** | `CSB`,`SCK` = NCN-Ausgänge |
| Modus | **SPI-Modus 0** (CPOL=0, CPHA=0) | Ausgabe an fallender, Sampling an steigender Flanke |
| Bit-Order | **LSB zuerst** | ⚠ nicht LPC-SSP-Default (MSB) – **explizit setzen** |
| Wortbreite | 8 Bit | |
| `TREQ` | Host-GPIO-Ausgang → NCN-Eingang | negative Flanke = „Byte folgt" (§A.2) |
| DATA_READY | NCN → Host (GPIO/IRQ) | „Empfangsdaten liegen an" (§A.4) |

> **Referenzsoftware-Pflicht:** SSP auf **LSB-first + Mode 0** konfigurieren. Der Emulator
> vergleicht Byte-Werte transparent, echte Hardware braucht diese Einstellung.

### A.2 Byte senden (Host → Device) via TREQ

```
1. Host schreibt Byte in SSP-DR (Slave-TX-Register).
2. Host erzeugt negative Flanke auf TREQ.
3. NCN taktet 8 Bit → Byte wandert zum Device; Host empfängt gleichzeitig 1 Byte.
```

Der Emulator bildet das byte-transparent ab: **ein `SSP_DR`-Write = ein `spi_exchange`**.
Die TREQ-Flanke ist im Emulator optional (das `DR`-Write genügt), sollte aber vom Treiber
erzeugt werden, damit derselbe Code auf Hardware funktioniert.

### A.3 Kommandosatz (Control-Byte + Folgebytes)

**Vom Host** (Auswahl der für einen TP-Koppler notwendigen Services):

| Hex | Service | Folgebytes | Zweck |
|---|---|---|---|
| `0x01` | `U_Reset.req` | – | Chip zurücksetzen |
| `0x02` | `U_State.req` | – | Kommunikationsstatus abfragen |
| `0x0D` | `U_SystemState.req` | – | Betriebszustand abfragen |
| `0x0E` / `0x0F` | `U_StopMode` / `U_ExitStopMode` | – | Stop betreten/verlassen |
| `0x18–0x1F` | `U_Configure.req` | – | Features: `p`=Auto-Poll, `c`=CRC, `m`=MARKER (Bit2/1/0) |
| `0xF1` | `U_SetAddress.req` | AddrHi, AddrLo, X | Phys. Adresse + Auto-Ack |
| `0xF2` | `U_SetRepetition.req` | RepCntrs, X, X | Wiederholungszähler |
| `0x10–0x17` | `U_Ackn.req` | – | Empf. Frame quittieren: Bit0 `a`=addressed, Bit1 `b`=busy, Bit2 `n`=nack |
| `0x80` | `U_L_DataStart.req` | CTRL | Sende-Frame beginnen |
| `0x81–0xBF` | `U_L_DataCont.req` | Daten-Octet | Frame-Byte i (1…63) |
| `0x47–0x7F` | `U_L_DataEnd.req` | FCS | Frame abschließen + senden (l = letzter Index+1) |
| `0x28–0x2F` | `U_IntRegWr.req` | Data | Internes Register aaa schreiben (SPI: vorher Stop empfohlen) |
| `0x38–0x3F` | `U_IntRegRd.req` | – | Internes Register aaa lesen (Antwort = nächstes Byte) |

**Zum Host** (Antworten/Indications):

| Hex/Muster | Service | Bedeutung |
|---|---|---|
| `0x03` | `U_Reset.ind` | Reset fertig (Normal erreicht) |
| `0x07`+Flags | `U_State.ind` | sc/re/te/pe/tw (fehlerfrei = `0x07`) |
| `0x4B`+Byte | `U_SystemStat.ind` | Statusbyte; Mode in Bit1..0 (`00`PU `01`Sync `10`Stop `11`Normal) |
| `0x2B` | `U_StopMode.ind` | Stop bestätigt |
| `0xCB` | `U_FrameEnd.ind` | Frame-Ende (nur mit MARKER) |
| `0x0B`+Flags | `U_FrameState.ind` | Frame-Fehler (re/ce/te) |
| `0b aa ap c m 01` | `U_Configure.ind` | Aktueller Feature-Zustand |
| `10r1 p1p0 00`+n | `L_Data_Standard.ind` | Empf. Standard-Frame (danach n Bytes) |
| `00r1 p1p0 00`+n | `L_Data_Extended.ind` | Empf. Extended-Frame |
| `0x8B`/`0x0B` | `L_Data.con` | Sende-Quittung: `0x8B` positiv, `0x0B` negativ |

### A.4 Empfang + DATA_READY

Empfangene KNX-Frames werden **byte-transparent** übertragen: zuerst das
`L_Data_*.ind`-Control-Byte, dann die Frame-Bytes (bei `c`-Feature +2 CRC-Bytes; bei
`m`-Feature `0xCB`+`U_FrameState.ind` als Ende, sonst 2.6 ms Stille).

Der Emulator signalisiert **DATA_READY** über eine virtuelle IRQ-Leitung (PINT/`irq_inject`,
Level-gehalten). Die Referenzsoftware kann **interrupt-** oder **poll-basiert** lesen
(`U_State.req` bzw. Ready-Leitung). Nach vollständigem Frame quittiert der Host mit
`U_Ackn.req` (`a/b/n`).

### A.5 Minimal-Init-Sequenz (Empfehlung Referenzsoftware)

```
U_Reset.req (0x01)                     → warte U_Reset.ind (0x03)
U_SystemState.req (0x0D)               → prüfe Mode == Normal (11)
U_Configure.req (0x18|Features)        → optional CRC/MARKER/Poll
U_SetAddress.req (0xF1, AH, AL, 0x00)  → Auto-Ack aktiv; warte U_Configure.ind
--- betriebsbereit: senden via U_L_Data*, empfangen via DATA_READY + L_Data_*.ind ---
```

---

## B. Emulator-interne Schnittstelle

### B.1 Modul-API `src/ncn5130.h`

```cpp
namespace ncn5130 {

// Einmalige Initialisierung (aus peripherals::init bzw. beim Aktivieren).
void init();

// Aktivierung je LPC-SSP-Instanz (0=SSP0,1=SSP1). enable=false -> Loopback/Bridge.
void set_enabled(int lpc_ssp, bool enable);
bool enabled(int lpc_ssp);

// KERN: ein SPI-Byte-Austausch aus dem SSP-Intercept.
//   mosi = vom Gast geschriebenes Byte (Host->Device)
//   ret  = an den Gast zu liefernder Wert (Device->Host, MISO)
uint8_t spi_exchange(uint8_t mosi);

// Back-End (KNX-PHY am Sekundaerbus): vom PIO/CT-Pfad aufgerufen.
//   push_rx_frame: ein vom Bus empfangenes Frame in den RX-Puffer legen ->
//                  loest DATA_READY/IRQ aus (Level-gehalten).
void push_rx_frame(const uint8_t* data, uint16_t len, bool extended);
//   pop_tx_frame: naechstes zu sendendes Frame abholen (nullptr/0 = keins).
bool pop_tx_frame(const uint8_t** data, uint16_t* len);
//   tx_result: Ergebnis einer Bus-Sendung zurueckmelden -> erzeugt L_Data.con.
void tx_result(bool positive);

// Diagnose (CLI 'ncn status').
struct Debug {
    uint8_t  state, cfg;
    uint16_t phys_addr;
    uint32_t rx_frames, tx_frames, cmd_bytes;
    bool     data_ready;
};
Debug debug();

} // namespace ncn5130
```

### B.2 SSP-Anbindung (`peripherals.cpp`)

Im DR-Zweig von `ssp_write_byte` **vor** dem Bridge-/Loopback-Zweig:

```cpp
if (ncn5130::enabled(static_cast<int>(idx))) {
    s.dr_rx = ncn5130::spi_exchange(static_cast<uint8_t>(s.tx & 0xFFu));
} else if (ssp_is_bridged(idx)) {
    …  // reale SPI-HW (unveraendert)
} else {
    s.dr_rx = s.tx;  // Loopback
}
s.tx = 0;
s.ris |= 0x4u;                        // RX-FIFO not empty
if (s.imsc & 0x4u) irq_inject::pend(s.irq_num);
```

Der übrige SSP-Pfad (`SR`/`RIS`/IRQ) bleibt unverändert. `ssp_read_byte` liefert `s.dr_rx`
wie gehabt.

### B.3 DATA_READY / IRQ-Anbindung

`push_rx_frame` setzt `data_ready=true` und pendet – **level-gehalten** – den zugeordneten
LPC-IRQ über einen PINT-Kanal (`irq_inject::pend`). Das Halte-Flag wird gelöscht, wenn der
Gast den RX-Puffer geleert hat (analog `g_uart0_rx_signaled`, Lessons #31/#32). Pollt der
Gast nur, spiegelt ein GPIO-Read den `data_ready`-Zustand.

### B.4 Back-End-Kontrakt (PHY)

| Richtung | Emulator-Aufruf | PHY-Umsetzung |
|---|---|---|
| RX (Bus→Host) | PIO/CT liefert Frame → `push_rx_frame()` | CT-Capture bzw. PIO-RX auf Sekundär-Pin |
| TX (Host→Bus) | `pop_tx_frame()` → PIO/CT sendet | CT-Match bzw. PIO-TX auf Sekundär-Pin |
| TX-Ende | `tx_result(pos/neg)` → `L_Data.con` | nach Repetition/ACK-Fenster |
| Test | Loopback: `push_rx_frame(pop_tx_frame())` | ohne Transceiver |

### B.5 CLI (Vorschlag, analog `i2c`/`cdc`)

| Befehl | Wirkung |
|---|---|
| `ncn on <ssp-inst>` | Virtuellen NCN5130 an LPC-SSP 0/1 koppeln |
| `ncn off` | Deaktivieren (zurück zu Loopback/Bridge) |
| `ncn status` | Zustand, Features, Adresse, RX/TX-Zähler, DATA_READY |

Persistenz über den bestehenden Config-Mechanismus (`ncn_en`, `ncn_ssp`), **deferred**
gespeichert (kein Gast-Neustart im CLI-Pfad – siehe Option-B-Persistenz).

### B.6 Konfigurations-Keys (Vorschlag)

| Key | Werte | Bedeutung |
|---|---|---|
| `ncn_en` | 0/1 | Virtuellen NCN5130 aktivieren |
| `ncn_ssp` | 0/1 | Welcher LPC-SSP gekoppelt wird |
| `ncn_rx_pin` | GPIO | Sekundär-Bus RX (CT-Capture/PIO) |
| `ncn_tx_pin` | GPIO | Sekundär-Bus TX (CT-Match/PIO) |

---

## C. Abgrenzung / Nicht-Ziele

- **Kein KNX-L2-Stack im Emulator**: Adressierung, Länge, Routing macht der Gast (SBLib).
- **Analog-Frontend** (DC/DC, VBUS/VFILT, Quarz) wird **nicht** physikalisch modelliert –
  Statusbits melden „gut".
- **Interne Register** zunächst als RAM-Registerfile mit Read-back; reale Semantik nur
  soweit die Referenzsoftware sie anfasst.
- **Bit-genaue Analog-/Timing-Register** (Datenblatt S. 54) sind bei Bedarf nachzuziehen.

---

## D. Verifikationsquellen

- `docs/NCN5130-D.PDF` – ON-Semi Datenblatt (Primärquelle).
- `docs/ncn5130_text.txt` – daraus extrahierter, durchsuchbarer Text (Tabellen 10–13,
  SPI-Master/TREQ Fig. 25–27, Service-Beschreibungen S. 34 ff.).
- `docs/NCN5130_EMULATION_SPEC.md` – die zugehörige Emulations-Spezifikation.
