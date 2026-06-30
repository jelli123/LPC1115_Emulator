#include "xmodem.h"

#include <cstdio>

#include "pico/stdlib.h"

namespace xmodem {

namespace {

constexpr uint8_t SOH = 0x01; // 128-Byte-Block
constexpr uint8_t STX = 0x02; // 1024-Byte-Block (1K)
constexpr uint8_t EOT = 0x04; // Ende der Uebertragung
constexpr uint8_t ACK = 0x06;
constexpr uint8_t NAK = 0x15;
constexpr uint8_t CAN = 0x18; // Abbruch
constexpr uint8_t C   = 'C';  // CRC-Modus anfordern

constexpr int      MAX_SYNC_TRIES = 16;   // ~16 s auf Sender-Start warten
constexpr int      MAX_ERRORS     = 10;   // Fehlerbloecke bis Abbruch
constexpr uint32_t BYTE_TIMEOUT   = 1'000'000u; // 1 s pro Zeichen

// CRC-16-CCITT (poly 0x1021, init 0x0000), wie von XMODEM-CRC verwendet.
uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            else               crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void put_ctrl(uint8_t c) {
    putchar_raw(c);
    fflush(stdout);
}

// Nicht-blockierendes Lesen mit Timeout: pollt stdin und bedient zwischendurch
// den USB-Stack ueber den Pump-Callback, damit ACK/NAK tatsaechlich gesendet
// und neue Bytes empfangen werden.
int rx_byte(uint32_t timeout_us, Pump pump) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    do {
        if (pump) pump();
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) return c;
    } while (!time_reached(deadline));
    return PICO_ERROR_TIMEOUT;
}

// Eingangspuffer leeren (nach Fehlern), bis fuer ~100 ms Ruhe herrscht.
void purge_input(Pump pump) {
    while (rx_byte(100'000u, pump) != PICO_ERROR_TIMEOUT) { /* verwerfen */ }
}

void cancel() {
    put_ctrl(CAN);
    put_ctrl(CAN);
    put_ctrl(CAN);
}

} // namespace

Result receive(DataSink sink, void* ctx, Pump pump, uint32_t& bytes_received) {
    bytes_received = 0;
    static uint8_t block[1024];

    // --- Synchronisation: 'C' senden, bis der Sender den ersten Block schickt.
    int first = PICO_ERROR_TIMEOUT;
    for (int tries = 0; tries < MAX_SYNC_TRIES; ++tries) {
        put_ctrl(C);
        first = rx_byte(BYTE_TIMEOUT, pump);
        if (first == SOH || first == STX || first == EOT || first == CAN) break;
        first = PICO_ERROR_TIMEOUT;
    }
    if (first == PICO_ERROR_TIMEOUT) return Result::SyncFailed;
    if (first == CAN)               return Result::Canceled;
    if (first == EOT) { put_ctrl(ACK); return Result::Ok; }

    int     header   = first; // SOH/STX bereits gelesen
    uint8_t expected = 1;     // XMODEM beginnt bei Blocknummer 1
    int     errors   = 0;

    while (true) {
        if (header == PICO_ERROR_TIMEOUT) {
            // Auf naechsten Frame-Header warten.
            header = rx_byte(BYTE_TIMEOUT, pump);
            if (header == PICO_ERROR_TIMEOUT) {
                if (++errors > MAX_ERRORS) { cancel(); return Result::Canceled; }
                put_ctrl(NAK);
                continue;
            }
        }

        if (header == EOT) { put_ctrl(ACK); return Result::Ok; }
        if (header == CAN) { return Result::Canceled; }
        if (header != SOH && header != STX) {
            header = PICO_ERROR_TIMEOUT;
            continue; // unbekanntes Byte ignorieren
        }

        const std::size_t data_len = (header == STX) ? 1024u : 128u;

        // Blocknummer + Komplement lesen.
        int blk  = rx_byte(BYTE_TIMEOUT, pump);
        int nblk = rx_byte(BYTE_TIMEOUT, pump);
        bool frame_ok = (blk != PICO_ERROR_TIMEOUT && nblk != PICO_ERROR_TIMEOUT);

        // Nutzdaten + 2 CRC-Bytes lesen.
        for (std::size_t i = 0; frame_ok && i < data_len; ++i) {
            int b = rx_byte(BYTE_TIMEOUT, pump);
            if (b == PICO_ERROR_TIMEOUT) { frame_ok = false; break; }
            block[i] = static_cast<uint8_t>(b);
        }
        int crc_hi = frame_ok ? rx_byte(BYTE_TIMEOUT, pump) : PICO_ERROR_TIMEOUT;
        int crc_lo = frame_ok ? rx_byte(BYTE_TIMEOUT, pump) : PICO_ERROR_TIMEOUT;
        if (crc_hi == PICO_ERROR_TIMEOUT || crc_lo == PICO_ERROR_TIMEOUT) frame_ok = false;

        // Rahmen-/Pruefsummenvalidierung.
        if (frame_ok) {
            uint16_t want = static_cast<uint16_t>((crc_hi << 8) | crc_lo);
            uint16_t got  = crc16_ccitt(block, data_len);
            if (((blk + nblk) & 0xFF) != 0xFF || want != got) frame_ok = false;
        }

        if (!frame_ok) {
            if (++errors > MAX_ERRORS) { cancel(); return Result::Canceled; }
            purge_input(pump);
            put_ctrl(NAK);
            header = PICO_ERROR_TIMEOUT;
            continue;
        }

        uint8_t bnum = static_cast<uint8_t>(blk);
        if (bnum == expected) {
            if (!sink(block, data_len, ctx)) { cancel(); return Result::Canceled; }
            bytes_received += static_cast<uint32_t>(data_len);
            expected = static_cast<uint8_t>(expected + 1);
            errors = 0;
            put_ctrl(ACK);
        } else if (bnum == static_cast<uint8_t>(expected - 1)) {
            // Wiederholter Block (ACK ging verloren) -> erneut bestaetigen.
            put_ctrl(ACK);
        } else {
            // Unerwartete Blocknummer -> Transfer ist desynchronisiert.
            cancel();
            return Result::Canceled;
        }
        header = PICO_ERROR_TIMEOUT;
    }
}

} // namespace xmodem
