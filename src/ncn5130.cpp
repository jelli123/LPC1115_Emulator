#include "ncn5130.h"
#include "irq_inject.h"
#include "lpc_irqs.h"
#include "pio_glue.h"

#include "pico/time.h"          // time_us_64()
#include <cstring>

// Virtueller NCN5130 – siehe docs/NCN5130_EMULATION_SPEC.md.
//
// Byte-transparentes SPI-Kommandoprotokoll (ON-Semi NCN5130). Der KNX-L2-Stack
// liegt im Gast (SBLib); dieses Modell leistet nur: Kommando-Parser, Antwort-/
// Indication-FIFO, RX-Frame-Zustellung + DATA_READY, TX-Frame-Assembly und die
// Sende-Quittung (L_Data.con). Das Back-End (KNX-PHY) wird ueber push_rx_frame/
// pop_tx_frame/tx_result angebunden (CT-Capture/Match bzw. PIO).

namespace ncn5130 {

namespace {

// --- Digitale Zustaende (U_SystemStat.ind Mode-Bits) ----------------------
enum State : uint8_t { ST_POWERUP = 0, ST_SYNC = 1, ST_STOP = 2, ST_NORMAL = 3 };

// --- Feature-Bits (cfg) ----------------------------------------------------
constexpr uint8_t CFG_MARKER   = 0x01;   // frame end with MARKER
constexpr uint8_t CFG_CRC      = 0x02;   // CRC-CCITT
constexpr uint8_t CFG_AUTOPOLL = 0x04;   // auto-polling
constexpr uint8_t CFG_AUTOACK  = 0x08;   // auto-acknowledge (via U_SetAddress)
constexpr uint8_t CFG_BUSY     = 0x10;   // busy mode

// --- Services zum Host (Control-Bytes) ------------------------------------
constexpr uint8_t IND_RESET     = 0x03;  // U_Reset.ind
constexpr uint8_t IND_STATE     = 0x07;  // U_State.ind (fehlerfrei; +Flags)
constexpr uint8_t IND_SYSSTAT   = 0x4B;  // U_SystemStat.ind (+ 1 Statusbyte)
constexpr uint8_t IND_STOPMODE  = 0x2B;  // U_StopMode.ind
constexpr uint8_t IND_FRAMEEND  = 0xCB;  // U_FrameEnd.ind (nur mit MARKER)
constexpr uint8_t CON_POS       = 0x8B;  // L_Data.con positiv
constexpr uint8_t CON_NEG       = 0x0B;  // L_Data.con negativ
constexpr uint8_t IND_LDATA_STD = 0x90;  // L_Data_Standard.ind Basis (10r1..)
constexpr uint8_t IND_LDATA_EXT = 0x10;  // L_Data_Extended.ind Basis (00r1..)

// --- Kommando-Parser-Zustand ----------------------------------------------
enum Cmd : uint8_t {
    CMD_NONE = 0,
    CMD_SETADDR,      // 3 Folgebytes: AddrHi, AddrLo, X
    CMD_SETREP,       // 3 Folgebytes: RepCntrs, X, X
    CMD_POLLSTATE,    // 3 Folgebytes: PollAddrHi, PollAddrLo, PollState
    CMD_INTREGWR,     // 1 Folgebyte: Data
    CMD_LDATA_CTRL,   // 1 Folgebyte: Control-/Daten-/Check-Octet
};

constexpr uint16_t FRAME_MAX = 280;   // Standard/Extended KNX-Frame + Overhead
constexpr uint16_t RESP_MAX  = 512;   // Antwort-/Indication-Ring (Bytes)

struct Model {
    bool     enabled;
    int      ssp;              // gekoppelte LPC-SSP-Instanz (-1 = keine)

    uint8_t  state;            // ST_*
    bool     busmon;           // Bus-Monitor-Modus
    uint8_t  cfg;              // Feature-Bits
    uint16_t phys_addr;
    uint8_t  rep_cnt;
    uint8_t  data_offset;      // U_L_DataOffset.req (iii)
    uint8_t  ireg[8];          // interne Register (aaa)

    // Kommando-Parser
    uint8_t  cmd;              // Cmd (aktives Multi-Byte-Kommando)
    uint8_t  cmd_need;         // noch erwartete Folgebytes
    uint8_t  cmd_buf[4];       // gesammelte Folgebytes
    uint8_t  cmd_got;          // Anzahl gesammelter Folgebytes
    uint8_t  intreg_addr;      // aaa fuer INTREGWR

    // Sende-Frame (Host -> Bus): Assembly-Puffer
    uint8_t  tx_frame[FRAME_MAX];
    uint16_t tx_len;
    bool     tx_building;      // zwischen DataStart und DataEnd
    bool     tx_pending;       // fertiges Frame wartet auf Back-End
    uint64_t tx_done_us;       // Zeitpunkt, ab dem die Bus-Sendung "durch" ist
    bool     loopback;         // Selbsttest: gesendete Frames in RX zurueckspiegeln

    // Antwort-/Indication-FIFO (Device -> Host)
    uint8_t  resp[RESP_MAX];
    uint16_t resp_head, resp_tail;

    // RX-Frame-Zustellung (DATA_READY)
    bool     rx_ready;         // noch nicht vollstaendig vom Host abgeholt

    // Diagnose-Zaehler
    uint32_t rx_frames, tx_frames, cmd_bytes;
};

Model g;

// ===========================================================================
// Reale KNX-TP1-PHY (STKNX/Selfbus diskreter Buskoppler)
// ===========================================================================
// Bit-Timing exakt wie sblib (examples/sblib/.../bus_const.h): 9600 Bd,
// 104 us/Bit, "0"-Bit = 35 us aktiver Puls am Bit-Anfang, "1"-Bit = rezessiv.
// Zeichen = Startbit(0) + 8 Datenbits (LSB-first) + gerade Paritaet + Stopbit(1).
// TX ueber match_pulse-PIO (Idle-Low, HIGH-Puls) - identisch zur LPC-PWM-Match-
// Ausgabe. RX ueber timer_edge_ts-PIO (fallende Flanke = "0"-Bit; Idle-High,
// strikte Flanken-Alternation).
constexpr uint32_t KNX_BIT_US       = 104;                 // 1e6/9600
constexpr uint32_t KNX_PULSE_US     = 35;                  // BIT_PULSE_TIME
constexpr uint32_t KNX_CHAR_BITS    = 11;                  // start+8+parity+stop
constexpr uint32_t KNX_CHAR_US      = KNX_CHAR_BITS * KNX_BIT_US;     // 1144
constexpr uint32_t KNX_TX_STARTUP   = 20;                  // us Vorlauf 1. Puls
// Frame-Ende: Luecke > (2 Bit + STARTBIT_OFFSET_MAX) + Marge (sblib S.35).
constexpr uint32_t KNX_FRAME_GAP_US = 2 * KNX_BIT_US + 30 + 120;

// KNX-Kurz-Acknowledge-Zeichen (bus_const.h ShortAcknowledgeFrame).
constexpr uint8_t  KNX_ACK   = 0xCC;   // SB_BUS_ACK
constexpr uint8_t  KNX_NACK  = 0x0C;   // SB_BUS_NACK
constexpr uint8_t  KNX_BUSY  = 0xC0;   // SB_BUS_BUSY

// ACK-Timing (sblib): ACK startet 15 Bitzeiten nach dem Stopbit des Frames;
// Empfangsfenster fuer den eigenen Sende-ACK -5..+30 us (+ Marge).
constexpr uint32_t KNX_ACK_DELAY_US = 15 * KNX_BIT_US;               // 1560
constexpr uint32_t KNX_ACK_WAIT_MAX = 15 * KNX_BIT_US + 30 + 200;    // Fenster
constexpr uint32_t KNX_INNERFRAME_US = 2 * KNX_BIT_US + 30;          // MAX_INTER_CHAR

// TX-Zustand (unser Telegramm auf dem Bus).
enum TxState : uint8_t { TXS_IDLE = 0, TXS_SENDING, TXS_ACKWAIT };

struct Phy {
    bool     active;
    int      tx_pin, rx_pin;
    int      tx_h, rx_h;        // pio_glue-Handles (-1 = keins)
    float    tx_cpu, rx_cpu;    // Counts pro us (aus PIO-Rate)

    // TX-Bitstrom-Cursor (Quelle = Frame ODER einzelnes ACK-Zeichen)
    uint8_t  tx_state;         // TxState
    const uint8_t* tx_src;     // Quell-Oktette
    uint16_t tx_src_len;       // Anzahl Oktette
    bool     tx_feed_done;      // alle Pulse in die PIO-FIFO geschoben
    uint16_t tx_oct;           // aktuelles Oktett
    uint8_t  tx_bit;           // aktuelle Bitposition 0..10
    uint32_t tx_prev_start_us; // Startzeit (us) des zuletzt emittierten Pulses
    bool     tx_first;         // erster Puls (delay ab SM-Start)
    uint64_t tx_done_wall;     // time_us_64, ab dem der Bus-Versand fertig ist
    uint8_t  tx_retries_left;  // verbleibende Wiederholungen (NACK/kein ACK)
    uint64_t tx_ack_deadline;  // Frist fuer eingehenden ACK nach unserem Frame

    // ACK-Aussendung fuer ein empfangenes Frame (Host -> U_Ackn.req)
    bool     ack_pending;      // ACK-Zeichen wartet auf Aussendung
    uint8_t  ack_octet;        // KNX_ACK/NACK/BUSY
    uint64_t ack_at_wall;      // geplanter Sendezeitpunkt (>= now)
    uint8_t  ack_buf;          // Puffer fuer tx_src (1 Byte)

    // RX-Dekoder
    uint32_t rx_prev_counter;
    bool     rx_have_prev;
    bool     rx_expect_falling;// Alternation: Idle-high -> erste Flanke fallend
    uint64_t rx_bus_us;        // akkumulierte Bus-Zeit (us) aus PIO-Deltas
    bool     rx_in_char;
    uint64_t rx_char_start_bus;// rx_bus_us der Start-Flanke (Bit-Position)
    uint64_t rx_char_start_wall;// time_us_64 (Zeichen-Timeout)
    uint16_t rx_char_mask;     // Maske gesetzter "0"-Bits (Position 0..10)
    uint64_t rx_last_wall;     // time_us_64 letzter RX-Aktivitaet (Frame-Gap)
    uint8_t  rx_buf[FRAME_MAX];
    uint16_t rx_buf_len;
    bool     rx_frame_active;
    bool     rx_frame_err;     // Paritaets-/Stopbit-Fehler im laufenden Frame
    uint64_t rx_frame_end_wall;// Ende des letzten Empfangsframes (ACK-Timing)
    uint64_t rx_echo_until;    // bis dahin RX ignorieren (eigenes TX-Echo)

    // Diagnose
    uint32_t rx_ack_ok, rx_ack_err, collisions;
};
Phy p;

inline uint8_t even_parity_bit(uint8_t v) {
    return static_cast<uint8_t>(__builtin_popcount(v) & 1u);   // 1 -> Bit=1
}

// Liefert true, wenn Bit `pos` (0..10) des Zeichens fuer `octet` eine "0" ist
// (also einen Puls braucht). 0=Start, 1..8=Daten LSB-first, 9=Paritaet, 10=Stop.
bool char_bit_is_zero(uint8_t octet, uint8_t pos) {
    if (pos == 0)  return true;                 // Startbit = 0
    if (pos <= 8)  return ((octet >> (pos - 1)) & 1u) == 0u;
    if (pos == 9)  return even_parity_bit(octet) == 0u;
    return false;                               // Stopbit = 1 (kein Puls)
}

// Plant die Aussendung eines Kurz-Acknowledge fuer das zuletzt empfangene Frame
// (Host -> U_Ackn.req). bits: Bit0=addressed(a), Bit1=busy(b), Bit2=nack(n).
// KNX-ACK-Zeichen: ACK=0xCC, NACK=0x0C, BUSY=0xC0. Nur mit realer PHY.
void phy_queue_ack(uint8_t bits) {
    if (!p.active) return;
    bool a = bits & 0x01, b = bits & 0x02, n = bits & 0x04;
    if (!a && !b && !n) return;                 // nicht adressiert -> kein ACK
    p.ack_octet   = static_cast<uint8_t>((n ? 0x00u : 0xC0u) | (b ? 0x00u : 0x0Cu));
    p.ack_pending = true;
    uint64_t now  = time_us_64();
    uint64_t sched = p.rx_frame_end_wall + KNX_ACK_DELAY_US;
    p.ack_at_wall = (sched > now) ? sched : now;   // 15 Bitzeiten nach Frame-Ende
}

// --- Antwort-FIFO ----------------------------------------------------------
inline bool resp_empty() { return g.resp_head == g.resp_tail; }

void resp_push(uint8_t b) {
    uint16_t nxt = static_cast<uint16_t>((g.resp_head + 1u) % RESP_MAX);
    if (nxt == g.resp_tail) return;        // voll -> aeltestes verwerfen waere schlimmer
    g.resp[g.resp_head] = b;
    g.resp_head = nxt;
}

uint8_t resp_pop() {
    if (resp_empty()) return 0x00;         // idle-Byte
    uint8_t b = g.resp[g.resp_tail];
    g.resp_tail = static_cast<uint16_t>((g.resp_tail + 1u) % RESP_MAX);
    return b;
}

// DATA_READY-Level nachfuehren: solange Antwort-Bytes anstehen ODER ein
// RX-Frame nicht komplett abgeholt ist, signalisieren wir dem Gast einen IRQ.
// Level-gehalten (analog uart0-RX, Lessons #31/#32): nur bei Flanke penden.
bool g_irq_signaled = false;

void update_data_ready() {
    bool ready = !resp_empty() || g.rx_ready;
    if (ready) {
        if (!g_irq_signaled && g.ssp >= 0) {
            g_irq_signaled = true;
            irq_inject::pend(g.ssp == 0 ? lpc_irq::SSP0 : lpc_irq::SSP1);
        }
    } else {
        g_irq_signaled = false;
    }
}

// --- Zustandswechsel -------------------------------------------------------
void enter_reset() {
    g.state       = ST_POWERUP;
    g.busmon      = false;
    g.cfg         = 0;
    g.phys_addr   = 0;
    g.rep_cnt     = 3;
    g.data_offset = 0;
    g.cmd         = CMD_NONE;
    g.cmd_need    = 0;
    g.cmd_got     = 0;
    g.tx_len      = 0;
    g.tx_building = false;
    g.tx_pending  = false;
    g.rx_ready    = false;
    g.resp_head   = g.resp_tail = 0;
    // Analog-Vorbedingungen (VBUS/VFILT/XTAL) im Emulator sofort "gut" ->
    // direkt in Normal (die Referenzsoftware sieht denselben Ablauf wie an HW,
    // die U_SystemState-Abfrage liefert Mode=Normal).
    g.state = ST_NORMAL;
    resp_push(IND_RESET);                  // U_Reset.ind bei Erreichen Normal
    update_data_ready();
}

// U_SystemStat.ind-Statusbyte: obere Bits = Analog-Status ("gut"), Bits1..0 =
// Mode. Exaktes Bitlayout siehe Datenblatt S.37 (TODO: HW-Abgleich); fuer den
// Emulator sind die Analog-Bits fix gesetzt.
uint8_t sysstat_byte() {
    return static_cast<uint8_t>(0xFCu | (g.state & 0x03u));
}

// Ein empfangenes KNX-Frame als L_Data_*.ind + Bytes in den Antwort-FIFO legen.
void emit_rx_frame(const uint8_t* data, uint16_t len, bool extended) {
    // Control-Byte: r=1 (nicht wiederholt), Prio aus dem Frame nicht bekannt ->
    // 0. Basis 0x90 (Standard) bzw. 0x10 (Extended); der Gast wertet die Bytes
    // ohnehin selbst aus (L2 im Gast).
    resp_push(extended ? IND_LDATA_EXT : IND_LDATA_STD);
    for (uint16_t i = 0; i < len; ++i) resp_push(data[i]);
    if (g.cfg & CFG_MARKER) {
        resp_push(IND_FRAMEEND);           // U_FrameEnd.ind
        resp_push(IND_STATE);              // U_FrameState.ind (fehlerfrei-Basis)
    }
}

// --- Kommando-Ausfuehrung (nach vollstaendigem Empfang) --------------------
void exec_setaddr() {
    g.phys_addr = static_cast<uint16_t>((g.cmd_buf[0] << 8) | g.cmd_buf[1]);
    g.cfg |= CFG_AUTOACK;
    // U_Configure.ind: 0 b aa ap c m 0 1
    uint8_t ind = 0x01u
                | ((g.cfg & CFG_BUSY)     ? 0x40u : 0u)
                | ((g.cfg & CFG_AUTOACK)  ? 0x20u : 0u)
                | ((g.cfg & CFG_AUTOPOLL) ? 0x10u : 0u)
                | ((g.cfg & CFG_CRC)      ? 0x08u : 0u)
                | ((g.cfg & CFG_MARKER)   ? 0x04u : 0u);
    resp_push(ind);
}

void exec_configure(uint8_t ctrl) {
    // 0x18 | p c m  (p=Bit2 auto-poll, c=Bit1 CRC, m=Bit0 MARKER). Nur SETZEN;
    // 0-Bits ohne Wirkung. Alle 0 -> nur aktuellen Zustand zuruecklesen.
    if (ctrl & 0x04u) g.cfg |= CFG_AUTOPOLL;
    if (ctrl & 0x02u) g.cfg |= CFG_CRC;
    if (ctrl & 0x01u) g.cfg |= CFG_MARKER;
    uint8_t ind = 0x01u
                | ((g.cfg & CFG_BUSY)     ? 0x40u : 0u)
                | ((g.cfg & CFG_AUTOACK)  ? 0x20u : 0u)
                | ((g.cfg & CFG_AUTOPOLL) ? 0x10u : 0u)
                | ((g.cfg & CFG_CRC)      ? 0x08u : 0u)
                | ((g.cfg & CFG_MARKER)   ? 0x04u : 0u);
    resp_push(ind);
}

void tx_finalize() {
    g.tx_building = false;
    if (g.tx_len == 0) return;
    g.tx_pending = true;
    ++g.tx_frames;
    // Modelliertes KNX-TP1-Timing: ~1.15 ms je Oktett (9600 bit/s, 11 bit je
    // Oktett) + ~5 ms Prioritaets-/ACK-Fenster. Die L_Data.con folgt in poll()
    // (Core1), NICHT synchron im SPI-Exchange - real dauert die Bus-Sendung
    // Millisekunden, und der Gast wartet zwischenzeitlich auf die con.
    uint64_t dur = static_cast<uint64_t>(g.tx_len) * 1150u + 5000u;
    g.tx_done_us = time_us_64() + dur;
    // Die eigentliche Bus-Sendung + L_Data.con uebernimmt das interne Back-End
    // (poll()) bzw. ein externer PHY (pop_tx_frame -> tx_result).
}

// Ein einzelnes vom Host empfangenes Control-Byte dekodieren + ausfuehren.
void decode_control(uint8_t b) {
    // Multi-Byte-Kommandos setzen g.cmd/g.cmd_need; die Folgebytes landen in
    // collect_data().
    if (b == 0x01) { enter_reset(); return; }                 // U_Reset.req
    if (b == 0x02) { resp_push(IND_STATE); return; }          // U_State.req
    if (b == 0x03) { g.cfg |= CFG_BUSY;  return; }            // U_SetBusy.req
    if (b == 0x04) { g.cfg &= ~CFG_BUSY; return; }            // U_QuitBusy.req
    if (b == 0x05) { g.busmon = true;    return; }            // U_Busmon.req
    if (b >= 0x08 && b <= 0x0C) { g.data_offset = static_cast<uint8_t>(b - 0x08); return; } // U_L_DataOffset
    if (b == 0x0D) { resp_push(IND_SYSSTAT); resp_push(sysstat_byte()); return; } // U_SystemState.req
    if (b == 0x0E) { g.state = ST_STOP;   resp_push(IND_STOPMODE); return; }      // U_StopMode.req
    if (b == 0x0F) { g.state = ST_NORMAL; return; }           // U_ExitStopMode.req
    if (b >= 0x10 && b <= 0x17) {                             // U_Ackn.req (n/b/a)
        // Host quittiert ein empfangenes Frame -> Kurz-Acknowledge auf den Bus
        // (adressiert=ACK 0xCC, busy=0xC0, nack=0x0C). RX-Ready loeschen.
        phy_queue_ack(static_cast<uint8_t>(b & 0x07u));
        g.rx_ready = false;
        g.cfg &= ~CFG_BUSY;                                   // Ackn hebt Busy auf
        update_data_ready();
        return;
    }
    if (b >= 0x18 && b <= 0x1F) { exec_configure(b); return; }// U_Configure.req
    if (b >= 0x28 && b <= 0x2F) {                             // U_IntRegWr.req
        g.cmd = CMD_INTREGWR; g.cmd_need = 1; g.cmd_got = 0;
        g.intreg_addr = static_cast<uint8_t>(b & 0x07u);
        return;
    }
    if (b >= 0x38 && b <= 0x3F) {                             // U_IntRegRd.req
        resp_push(g.ireg[b & 0x07u]);
        return;
    }
    if (b >= 0x47 && b <= 0x7F) {                             // U_L_DataEnd.req (+FCS)
        g.cmd = CMD_LDATA_CTRL; g.cmd_need = 1; g.cmd_got = 0;
        g.cmd_buf[3] = 0xFF;   // Marker: DataEnd (vs. Start/Cont)
        return;
    }
    if (b == 0x80) {                                          // U_L_DataStart.req (+CTRL)
        g.tx_len = 0; g.tx_building = true;
        g.cmd = CMD_LDATA_CTRL; g.cmd_need = 1; g.cmd_got = 0;
        g.cmd_buf[3] = 0x00;   // Marker: Start/Cont-Byte anhaengen
        return;
    }
    if (b >= 0x81 && b <= 0xBF) {                             // U_L_DataCont.req (+Data)
        g.cmd = CMD_LDATA_CTRL; g.cmd_need = 1; g.cmd_got = 0;
        g.cmd_buf[3] = 0x00;   // anhaengen
        return;
    }
    if (b >= 0xE0 && b <= 0xEE) {                             // U_PollingState.req (3B)
        g.cmd = CMD_POLLSTATE; g.cmd_need = 3; g.cmd_got = 0;
        return;
    }
    if (b == 0xF1) { g.cmd = CMD_SETADDR; g.cmd_need = 3; g.cmd_got = 0; return; } // U_SetAddress
    if (b == 0xF2) { g.cmd = CMD_SETREP;  g.cmd_need = 3; g.cmd_got = 0; return; } // U_SetRepetition
    // Unbekannt -> ignorieren (kein Haenger; Dummy).
}

// Ein Folgebyte eines Multi-Byte-Kommandos verarbeiten.
void collect_data(uint8_t b) {
    if (g.cmd_got < sizeof g.cmd_buf) g.cmd_buf[g.cmd_got] = b;
    ++g.cmd_got;
    if (g.cmd_got < g.cmd_need) return;   // noch nicht vollstaendig

    switch (g.cmd) {
        case CMD_SETADDR:   exec_setaddr(); break;
        case CMD_SETREP:    g.rep_cnt = g.cmd_buf[0]; break;
        case CMD_POLLSTATE: /* Poll-Slot gesetzt; im Emulator ohne Wirkung */ break;
        case CMD_INTREGWR:  g.ireg[g.intreg_addr] = g.cmd_buf[0]; break;
        case CMD_LDATA_CTRL:
            if (g.cmd_buf[3] == 0xFF) {          // DataEnd: FCS anhaengen + senden
                if (g.tx_building && g.tx_len < FRAME_MAX) g.tx_frame[g.tx_len++] = g.cmd_buf[0];
                tx_finalize();
            } else {                             // Start/Cont: Octet anhaengen
                if (g.tx_building && g.tx_len < FRAME_MAX) g.tx_frame[g.tx_len++] = g.cmd_buf[0];
            }
            break;
        default: break;
    }
    g.cmd = CMD_NONE; g.cmd_need = 0; g.cmd_got = 0;
}

// --- PHY: Sende-Pfad (Frame/ACK -> KNX-Bitpulse ueber match_pulse-PIO) ------
void phy_tx_ack(bool ok);   // vorwaerts (RX-Pfad meldet Quittung)

// Startet die Aussendung einer Oktett-Sequenz (Frame ODER einzelnes ACK-Byte).
void phy_tx_begin(const uint8_t* src, uint16_t len, uint64_t now) {
    p.tx_src         = src;
    p.tx_src_len     = len;
    p.tx_feed_done   = false;
    p.tx_oct         = 0;
    p.tx_bit         = 0;
    p.tx_first       = true;
    p.tx_prev_start_us = 0;
    uint64_t dur = static_cast<uint64_t>(len) * KNX_CHAR_US
                 + KNX_TX_STARTUP + KNX_FRAME_GAP_US;
    p.tx_done_wall   = now + dur;
    p.rx_echo_until  = p.tx_done_wall + KNX_BIT_US;   // eigenes Echo ausblenden
}

// Schiebt so viele "0"-Bit-Pulse in die PIO-FIFO wie hineinpassen. Die PIO
// taktet die Flanken hardware-genau; der Feeder muss die FIFO nur gefuellt
// halten. Rueckgabe: true, wenn alle Pulse des aktuellen Frames eingereiht sind.
bool phy_tx_feed() {
    while (!p.tx_feed_done) {
        while (p.tx_oct < p.tx_src_len &&
               !char_bit_is_zero(p.tx_src[p.tx_oct], p.tx_bit)) {
            if (++p.tx_bit >= KNX_CHAR_BITS) { p.tx_bit = 0; ++p.tx_oct; }
        }
        if (p.tx_oct >= p.tx_src_len) { p.tx_feed_done = true; break; }

        uint32_t start_us = p.tx_oct * KNX_CHAR_US + p.tx_bit * KNX_BIT_US;
        uint32_t delay_us = p.tx_first
                          ? (start_us + KNX_TX_STARTUP)
                          : (start_us - (p.tx_prev_start_us + KNX_PULSE_US));
        uint32_t delay_c = static_cast<uint32_t>(delay_us * p.tx_cpu);
        uint32_t width_c = static_cast<uint32_t>(KNX_PULSE_US * p.tx_cpu);
        if (!pio_glue::tx_emit(p.tx_h, delay_c, width_c)) break;  // FIFO voll

        p.tx_prev_start_us = start_us;
        p.tx_first = false;
        if (++p.tx_bit >= KNX_CHAR_BITS) { p.tx_bit = 0; ++p.tx_oct; }
    }
    return p.tx_feed_done;
}

void phy_tx_poll(uint64_t now) {
    switch (p.tx_state) {
    case TXS_IDLE:
        // Prioritaet 1: faelliges Kurz-Acknowledge fuer ein empfangenes Frame.
        if (p.ack_pending && now >= p.ack_at_wall) {
            p.ack_pending = false;
            p.ack_buf     = p.ack_octet;
            phy_tx_begin(&p.ack_buf, 1, now);
            p.tx_state    = TXS_SENDING;
            phy_tx_feed();
            return;
        }
        // Prioritaet 2: neues Telegramm vom Host.
        if (g.tx_pending) {
            g.tx_pending      = false;
            p.tx_retries_left = g.rep_cnt;      // NCN wiederholt autonom
            phy_tx_begin(g.tx_frame, g.tx_len, now);
            p.tx_state        = TXS_SENDING;
            phy_tx_feed();
        }
        return;

    case TXS_SENDING:
        if (!phy_tx_feed()) return;             // FIFO noch nicht ganz gefuellt
        if (now < p.tx_done_wall) return;        // Bus-Aussendung laeuft noch
        if (p.tx_src == &p.ack_buf) {            // ACK-Zeichen: keine con
            p.tx_state = TXS_IDLE;
            p.rx_expect_falling = true;          // Alternation auf Idle-high resync
            return;
        }
        // Telegramm gesendet -> auf ACK des Zielgeraets warten.
        p.tx_state        = TXS_ACKWAIT;
        p.tx_ack_deadline = now + KNX_ACK_WAIT_MAX;
        p.rx_expect_falling = true;              // Bus ist high -> naechste Flanke fallend
        return;

    case TXS_ACKWAIT:
        if (now >= p.tx_ack_deadline) phy_tx_ack(false);   // kein ACK im Fenster
        return;
    }
}

// Vom RX-Pfad gemeldete Sende-Quittung (ok=0xCC empfangen, sonst NACK/BUSY/kein).
void phy_tx_ack(bool ok) {
    if (p.tx_state != TXS_ACKWAIT) return;
    if (ok) {
        p.tx_state = TXS_IDLE;
        tx_result(true);                         // L_Data.con positiv
        return;
    }
    if (p.tx_retries_left > 0) {
        --p.tx_retries_left;                     // erneut senden (dasselbe Frame)
        phy_tx_begin(g.tx_frame, g.tx_len, time_us_64());
        p.tx_state = TXS_SENDING;
        phy_tx_feed();
        return;
    }
    p.tx_state = TXS_IDLE;
    tx_result(false);                            // L_Data.con negativ
}

// --- PHY: Empfangs-Pfad (KNX-Bitpulse -> Frame) ----------------------------
// Ein abgeschlossenes Zeichen aus der Bitmaske verrechnen (Oktett + Paritaets-/
// Stopbit-Pruefung) und routen: im ACK-Wartefenster = unsere Sende-Quittung,
// sonst als Frame-Byte anhaengen.
void phy_finish_char() {
    uint8_t octet = 0;
    for (int i = 0; i < 8; ++i)
        if (!(p.rx_char_mask & (1u << (i + 1)))) octet |= (1u << i);  // Bit==1
    // Gerade Paritaet: Anzahl "1" (8 Daten + Paritaetsbit) muss gerade sein.
    uint8_t par1 = (p.rx_char_mask & (1u << 9)) ? 0u : 1u;   // kein Puls -> "1"
    bool parity_ok = ((static_cast<unsigned>(__builtin_popcount(octet)) + par1) & 1u) == 0u;
    bool stop_ok   = !(p.rx_char_mask & (1u << 10));         // Stopbit = "1"
    if (!parity_ok || !stop_ok) p.rx_frame_err = true;
    p.rx_in_char = false;

    // Im ACK-Wartefenster nach eigenem TX ist dieses Zeichen die Quittung.
    if (p.tx_state == TXS_ACKWAIT) {
        bool ok = (octet == KNX_ACK);
        if (ok) ++p.rx_ack_ok; else ++p.rx_ack_err;
        phy_tx_ack(ok);
        return;
    }
    if (p.rx_buf_len < FRAME_MAX) p.rx_buf[p.rx_buf_len++] = octet;
}

// Frame abschliessen und an den Host zustellen.
void phy_finish_frame() {
    uint64_t now = time_us_64();
    if (p.rx_buf_len == 1) {
        // Einzelnes Zeichen = Kurz-Acknowledge (L_Ackn), im Normalmodus nicht
        // als L_Data zustellen (nur im Bus-Monitor sichtbar -> hier verworfen).
        p.rx_buf_len = 0;
        p.rx_frame_active = false;
        p.rx_frame_err = false;
        p.rx_expect_falling = true;
        return;
    }
    if (p.rx_buf_len > 0) {
        bool extended = !(p.rx_buf[0] & 0x80u);   // FT-Bit: 1=Standard 0=Extended
        emit_rx_frame(p.rx_buf, p.rx_buf_len, extended);
        g.rx_ready = true;
        ++g.rx_frames;
        update_data_ready();
        p.rx_frame_end_wall = now;                // Basis fuer ACK-Timing
    }
    p.rx_buf_len = 0;
    p.rx_frame_active = false;
    p.rx_frame_err = false;
    p.rx_expect_falling = true;                   // Idle-high -> naechste Flanke fallend
}

void phy_rx_poll(uint64_t now) {
    // Alle vorliegenden Flanken-Timestamps abholen.
    uint32_t counter;
    while (pio_glue::ts_read(p.rx_h, counter)) {
        bool falling = p.rx_expect_falling;   // strikte Alternation ab Idle-high
        p.rx_expect_falling = !p.rx_expect_falling;
        if (p.rx_have_prev) {
            uint32_t dc = p.rx_prev_counter - counter;   // Abwaertszaehler
            p.rx_bus_us += static_cast<uint64_t>(dc / p.rx_cpu);
        }
        p.rx_prev_counter = counter;
        p.rx_have_prev = true;

        // Waehrend eigener Aussendung (Frame/ACK) das eigene Echo verwerfen,
        // aber die Alternation weiterfuehren (Flanken zaehlen konsistent).
        if (now < p.rx_echo_until || p.tx_state == TXS_SENDING) continue;
        if (!falling) continue;                           // nur "0"-Bit-Anfaenge

        if (!p.rx_in_char) {
            // Neues Zeichen: Startbit (Position 0).
            p.rx_in_char = true;
            p.rx_char_start_bus  = p.rx_bus_us;
            p.rx_char_start_wall = now;
            p.rx_char_mask = 0x0001u;             // Startbit = 0
            p.rx_frame_active = true;
        } else {
            uint32_t pos = static_cast<uint32_t>(
                (p.rx_bus_us - p.rx_char_start_bus + KNX_BIT_US / 2) / KNX_BIT_US);
            if (pos >= KNX_CHAR_BITS) {
                // Flanke gehoert zum naechsten Zeichen: aktuelles abschliessen.
                phy_finish_char();
                p.rx_in_char = true;
                p.rx_char_start_bus  = p.rx_bus_us;
                p.rx_char_start_wall = now;
                p.rx_char_mask = 0x0001u;
            } else if (pos <= 10) {
                p.rx_char_mask |= static_cast<uint16_t>(1u << pos);
            }
        }
        p.rx_last_wall = now;
    }

    // Zeichen-Timeout: nach ~1 Zeichenlaenge ohne weitere Flanke abschliessen
    // (ein Zeichen mit lauter "1"-Datenbits hat nur die Startbit-Flanke).
    if (p.rx_in_char && (now - p.rx_char_start_wall) >= KNX_CHAR_US) {
        phy_finish_char();
    }
    // Frame-Ende: Luecke seit letzter Flanke groesser als inter-char-Fenster.
    if (p.rx_frame_active && !p.rx_in_char &&
        (now - p.rx_last_wall) >= KNX_FRAME_GAP_US) {
        phy_finish_frame();
    }
}

void phy_poll_all() {
    uint64_t now = time_us_64();
    if (p.tx_h >= 0) phy_tx_poll(now);
    if (p.rx_h >= 0) phy_rx_poll(now);
    // ACK-Aussendung kann faellig werden, ohne dass RX-Flanken eintreffen.
    if (p.tx_h >= 0 && p.tx_state == TXS_IDLE && p.ack_pending && now >= p.ack_at_wall)
        phy_tx_poll(now);
}

} // namespace

// ===========================================================================
// Oeffentliche API
// ===========================================================================

void init() {
    std::memset(&g, 0, sizeof g);
    g.ssp = -1;
    g.enabled = false;
    enter_reset();
}

void set_enabled(int lpc_ssp, bool enable) {
    if (lpc_ssp < 0 || lpc_ssp > 1) return;
    if (enable) {
        g.enabled = true;
        g.ssp     = lpc_ssp;
        enter_reset();
    } else if (g.ssp == lpc_ssp) {
        g.enabled = false;
        g.ssp     = -1;
    }
}

bool enabled(int lpc_ssp)  { return g.enabled && g.ssp == lpc_ssp; }
bool any_enabled()         { return g.enabled; }

void set_loopback(bool enable) { g.loopback = enable; }
bool loopback()                { return g.loopback; }

// Reale KNX-PHY einrichten/abschalten (Core0). Beide Pins >=0 -> PHY aktiv:
// match_pulse-PIO fuer TX, timer_edge_ts-PIO fuer RX. Vorherige Handles werden
// freigegeben (Rekonfiguration bei 'cfg save'/CONFIG.INI-Reload).
void phy_init(int tx_pin, int rx_pin) {
    // Alte State-Machines freigeben.
    if (p.tx_h >= 0) pio_glue::tx_teardown(p.tx_h);
    if (p.rx_h >= 0) pio_glue::ts_teardown(p.rx_h);
    std::memset(&p, 0, sizeof p);
    p.tx_h = p.rx_h = -1;
    p.tx_pin = tx_pin;
    p.rx_pin = rx_pin;
    p.rx_expect_falling = true;                // Idle-high -> erste Flanke fallend

    if (tx_pin >= 0) {
        float rate = 0.f;
        p.tx_h = pio_glue::tx_setup(static_cast<uint8_t>(tx_pin), rate);
        p.tx_cpu = (rate > 0.f) ? rate / 1'000'000.0f : 1.0f;   // Counts pro us
    }
    if (rx_pin >= 0) {
        float rate = 0.f;
        p.rx_h = pio_glue::ts_setup(static_cast<uint8_t>(rx_pin), rate);
        p.rx_cpu = (rate > 0.f) ? rate / 1'000'000.0f : 1.0f;
    }
    p.active = (p.tx_h >= 0 && p.rx_h >= 0);
}

bool phy_active() { return p.active; }

// Internes KNX-Back-End: schliesst eine anstehende Bus-Sendung ab, sobald das
// modellierte Timing abgelaufen ist -> L_Data.con (positiv) und optional
// Loopback/Monitor-Ruecklauf. Laeuft auf Core1 (aus poll_timed_sources), damit
// irq_inject::pend greift und der Modellzugriff single-core bleibt.
void poll() {
    if (!g.enabled) return;

    // Reale PHY aktiv -> sie uebernimmt TX (Bitpulse) und RX (Frame-Empfang).
    if (p.active) { phy_poll_all(); return; }

    // Sonst internes Back-End (Software-Loopback/Monitor + Timing-con).
    if (!g.tx_pending) return;
    if (time_us_64() < g.tx_done_us) return;

    // Selbsttest/Monitor: eigenes Frame als L_Data_*.ind zurueckspiegeln.
    // FT-Bit (Frame Type) = Bit7 des Control-Octets: 1=Standard, 0=Extended.
    if (g.loopback || g.busmon) {
        bool extended = !(g.tx_len > 0 && (g.tx_frame[0] & 0x80u));
        emit_rx_frame(g.tx_frame, g.tx_len, extended);
        g.rx_ready = true;
        ++g.rx_frames;
    }
    g.tx_pending = false;
    tx_result(true);        // L_Data.con positiv (+ update_data_ready)
}

uint8_t spi_exchange(uint8_t mosi) {
    ++g.cmd_bytes;
    // 1) MISO-Byte bestimmen: naechstes Antwort-Byte (falls vorhanden), sonst 0.
    uint8_t miso = resp_pop();
    // 2) MOSI-Byte in den Kommando-Parser geben.
    if (g.cmd != CMD_NONE) collect_data(mosi);
    else                   decode_control(mosi);
    update_data_ready();
    return miso;
}

void push_rx_frame(const uint8_t* data, uint16_t len, bool extended) {
    if (!data || len == 0) return;
    emit_rx_frame(data, len, extended);
    g.rx_ready = true;
    ++g.rx_frames;
    update_data_ready();
}

bool pop_tx_frame(const uint8_t** data, uint16_t* len) {
    if (!g.tx_pending) return false;
    if (data) *data = g.tx_frame;
    if (len)  *len  = g.tx_len;
    g.tx_pending = false;
    return true;
}

void tx_result(bool positive) {
    resp_push(positive ? CON_POS : CON_NEG);
    update_data_ready();
}

bool data_ready() { return !resp_empty() || g.rx_ready; }

Debug debug() {
    Debug d{};
    d.enabled   = g.enabled;
    d.ssp       = g.ssp;
    d.state     = g.state;
    d.busmon    = g.busmon;
    d.cfg       = g.cfg;
    d.phys_addr = g.phys_addr;
    d.rx_frames = g.rx_frames;
    d.tx_frames = g.tx_frames;
    d.cmd_bytes = g.cmd_bytes;
    d.rx_ready  = data_ready();
    d.loopback  = g.loopback;
    d.tx_inflight = g.tx_pending;
    d.phy_active  = p.active;
    d.phy_tx_pin  = p.tx_h >= 0 ? p.tx_pin : -1;
    d.phy_rx_pin  = p.rx_h >= 0 ? p.rx_pin : -1;
    d.rx_ack_ok   = p.rx_ack_ok;
    d.rx_ack_err  = p.rx_ack_err;
    return d;
}

} // namespace ncn5130
