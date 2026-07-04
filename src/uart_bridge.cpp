#include "uart_bridge.h"

#include "config.h"
#include "tusb.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>
#include <atomic>

// ---------------------------------------------------------------------------
// PIO-Programme für 8N1 UART TX und RX.
//
// TX (side_set 1 opt, OUT-Pin = side-set-Pin = TX-GPIO):
//   .wrap_target
//   pull       side 1 [7]   ; Stop-Bit halten / idle high, auf Daten warten
//   set  x, 7 side 0 [7]   ; Start-Bit (low) senden, 8 Bit laden
//   bitloop:
//   out  pins, 1            ; 1 Bit aus OSR ausgeben
//   jmp  x-- bitloop  [6]  ; 8 Zyklen pro Bit
//   .wrap
//
// RX (kein side-set, IN-Pin = RX-GPIO):
//   .wrap_target
//   wait 0 pin 0            ; Auf Start-Bit warten (fallende Flanke)
//   set  x, 7         [10] ; Mitte des ersten Datenbits (12 Zyklen ab Flanke)
//   bitloop:
//   in   pins, 1            ; 1 Bit sampeln
//   jmp  x-- bitloop  [6]  ; 8 Zyklen pro Bit
//   push                    ; 8-Bit-Wort in RX-FIFO
//   .wrap
// ---------------------------------------------------------------------------

namespace uart_bridge {

namespace {

// --- TX-Programm (4 Instruktionen, side_set 1 opt) ---
constexpr uint16_t uart_tx_program_instructions[] = {
    0x9fa0, // 0: pull   block           side 1 [7]
    0xf727, // 1: set    x, 7            side 0 [7]
    0x6001, // 2: out    pins, 1
    0x0642, // 3: jmp    x--, 2                 [6]
};

const struct pio_program uart_tx_program = {
    .instructions = uart_tx_program_instructions,
    .length       = 4,
    .origin       = -1,
    .pio_version  = 0,
    .used_gpio_ranges = 0,
};

// --- RX-Programm (5 Instruktionen, kein side-set) ---
constexpr uint16_t uart_rx_program_instructions[] = {
    0x2020, // 0: wait   0 pin, 0
    0xea27, // 1: set    x, 7                   [10]
    0x4001, // 2: in     pins, 1
    0x0642, // 3: jmp    x--, 2                 [6]
    0x8020, // 4: push   block
};

const struct pio_program uart_rx_program = {
    .instructions = uart_rx_program_instructions,
    .length       = 5,
    .origin       = -1,
    .pio_version  = 0,
    .used_gpio_ranges = 0,
};

// --- Zustand ---
constexpr uint8_t CDC_ITF = 2;  // dritte CDC-Schnittstelle

int      g_tx_pin   = -1;
int      g_rx_pin   = -1;
bool     g_active   = false;
uint32_t g_baud     = 115200;

PIO      g_pio      = nullptr;
uint     g_sm_tx    = 0;
uint     g_sm_rx    = 0;
int      g_offset_tx = -1;
int      g_offset_rx = -1;

// Puffer für RX → CDC#2 TX (PIO liefert Wort-weise, CDC will Bulk)
uint8_t  g_rx_buf[64];
uint     g_rx_buf_len = 0;

// Diagnose-Zaehler (via 'uart status' sichtbar). Zeigen, an welcher Stelle der
// Datenfluss CDC#2 -> PIO-TX -> (Draht) -> PIO-RX -> CDC#2 bricht.
uint32_t g_cnt_cdc_rx = 0;   // Bytes vom PC (CDC#2) gelesen
uint32_t g_cnt_pio_tx = 0;   // Bytes in die PIO-TX-FIFO geschrieben
uint32_t g_cnt_pio_rx = 0;   // Bytes aus der PIO-RX-FIFO gelesen
uint32_t g_cnt_cdc_tx = 0;   // Bytes an den PC (CDC#2) geschrieben

// Clock-Divider berechnen: 8 PIO-Zyklen pro Bit
float calc_clkdiv(uint32_t baud) {
    if (baud == 0) baud = 115200;
    float clk = static_cast<float>(clock_get_hz(clk_sys));
    return clk / (8.0f * static_cast<float>(baud));
}

void configure_tx(PIO pio, uint sm, int offset, uint pin, uint32_t baud) {
    pio_sm_config c = pio_get_default_sm_config();

    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 2, true, false); // 1 bit side-set + opt flag

    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, true, false, 32); // shift right, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, calc_clkdiv(baud));

    pio_gpio_init(pio, pin);
    pio_sm_set_pins_with_mask(pio, sm, 1u << pin, 1u << pin); // idle high
    pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin, 1u << pin); // output

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void configure_rx(PIO pio, uint sm, int offset, uint pin, uint32_t baud) {
    pio_sm_config c = pio_get_default_sm_config();

    sm_config_set_wrap(&c, offset, offset + 4);

    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    // autopush AUS: das RX-Programm pusht selbst explizit (push block) nach 8
    // gesampelten Bits. Mit zusaetzlichem Autopush (threshold 8) wuerde JEDES
    // Byte ausserdem ein redundantes 0x00-Wort erzeugen (autopush leert die ISR,
    // das folgende push schiebt eine 0 nach) -> der Host saehe "Zeichen, NUL,
    // Zeichen, NUL, ...". Ein einziger expliziter push liefert genau ein Byte.
    sm_config_set_in_shift(&c, true, false, 8); // shift right, KEIN autopush
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, calc_clkdiv(baud));

    pio_gpio_init(pio, pin);
    gpio_pull_up(pin); // Idle-High bei offenem Pin
    pio_sm_set_pindirs_with_mask(pio, sm, 0, 1u << pin); // input

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void teardown() {
    if (!g_pio) return;
    if (g_active) {
        pio_sm_set_enabled(g_pio, g_sm_tx, false);
        pio_sm_set_enabled(g_pio, g_sm_rx, false);
        pio_sm_unclaim(g_pio, g_sm_tx);
        pio_sm_unclaim(g_pio, g_sm_rx);
        if (g_offset_tx >= 0) pio_remove_program(g_pio, &uart_tx_program, g_offset_tx);
        if (g_offset_rx >= 0) pio_remove_program(g_pio, &uart_rx_program, g_offset_rx);
    }
    g_offset_tx = -1;
    g_offset_rx = -1;
    g_pio = nullptr;
    g_active = false;
}

bool setup_pio() {
    // Versuche PIO1, dann PIO2 (PIO0 bleibt für pio_glue / edge-capture)
    PIO candidates[] = { pio1, pio2 };

    for (auto pio : candidates) {
        if (!pio_can_add_program(pio, &uart_tx_program)) continue;
        if (!pio_can_add_program(pio, &uart_rx_program)) continue;

        int sm_tx_claim = pio_claim_unused_sm(pio, false);
        if (sm_tx_claim < 0) continue;
        int sm_rx_claim = pio_claim_unused_sm(pio, false);
        if (sm_rx_claim < 0) {
            pio_sm_unclaim(pio, sm_tx_claim);
            continue;
        }

        g_pio = pio;
        g_sm_tx = static_cast<uint>(sm_tx_claim);
        g_sm_rx = static_cast<uint>(sm_rx_claim);
        g_offset_tx = pio_add_program(pio, &uart_tx_program);
        g_offset_rx = pio_add_program(pio, &uart_rx_program);
        return true;
    }
    return false;
}

void apply_baud(uint32_t baud) {
    if (!g_active || !g_pio) return;
    float div = calc_clkdiv(baud);
    pio_sm_set_clkdiv(g_pio, g_sm_tx, div);
    pio_sm_set_clkdiv(g_pio, g_sm_rx, div);
    // Restart damit neuer Divider sofort wirkt
    pio_sm_clkdiv_restart(g_pio, g_sm_tx);
    pio_sm_clkdiv_restart(g_pio, g_sm_rx);
}

} // namespace

void init() {
    // Nichts zu tun — start() aktiviert die Bridge.
}

bool start() {
    if (g_active) return true;
    if (g_tx_pin < 0 || g_rx_pin < 0) return false;
    if (g_tx_pin > 47 || g_rx_pin > 47) return false;

    // Konflikte mit dem Emulator-Pin-Mapping auflösen: Bridge-Pins aus
    // der LPC→RP2350-Tabelle entfernen, damit peripherals.cpp nicht
    // versucht, die gleichen GPIOs zu treiben.
    {
        const auto& pm = config::pin_map();
        for (std::size_t i = 0; i < config::LPC_PIN_COUNT; ++i) {
            if (pm.lpc_to_rp[i] == g_tx_pin || pm.lpc_to_rp[i] == g_rx_pin) {
                std::printf("[uart-bridge] entferne Konflikt: LPC P%u_%u -> GP%d\n",
                            static_cast<unsigned>(i / 12),
                            static_cast<unsigned>(i % 12),
                            pm.lpc_to_rp[i]);
                config::set_pin_map(static_cast<uint8_t>(i), -1);
            }
        }
    }

    if (!setup_pio()) {
        std::puts("[uart-bridge] PIO voll");
        return false;
    }

    configure_tx(g_pio, g_sm_tx, g_offset_tx, static_cast<uint>(g_tx_pin), g_baud);
    configure_rx(g_pio, g_sm_rx, g_offset_rx, static_cast<uint>(g_rx_pin), g_baud);

    g_active = true;
    std::printf("[uart-bridge] TX=GP%d RX=GP%d baud=%lu\n",
                g_tx_pin, g_rx_pin, static_cast<unsigned long>(g_baud));
    return true;
}

void stop() {
    if (!g_active) return;
    teardown();
    std::puts("[uart-bridge] stopped");
}

bool active() { return g_active; }

void set_tx_pin(int gpio) { g_tx_pin = gpio; }
void set_rx_pin(int gpio) { g_rx_pin = gpio; }
int  tx_pin()             { return g_tx_pin; }
int  rx_pin()             { return g_rx_pin; }
uint32_t baud_rate()      { return g_baud; }

void poll() {
    if (!g_active) return;

    // --- CDC#2 RX → PIO UART TX ---
    // KEIN tud_cdc_n_connected()-Gate: manche Terminalprogramme setzen DTR
    // nicht, wodurch connected() dauerhaft false meldet und die Bridge stumm
    // bliebe. tud_cdc_n_available() reicht — liegen Daten vor, verarbeiten wir sie.
    if (tud_cdc_n_available(CDC_ITF)) {
        uint8_t buf[64];
        uint32_t count = tud_cdc_n_read(CDC_ITF, buf, sizeof buf);
        for (uint32_t i = 0; i < count; ++i) {
            // Warten, falls TX-FIFO voll (sollte selten sein bei normalen Baudraten)
            while (pio_sm_is_tx_fifo_full(g_pio, g_sm_tx)) tight_loop_contents();
            pio_sm_put(g_pio, g_sm_tx, static_cast<uint32_t>(buf[i]));
            ++g_cnt_pio_tx;
        }
        g_cnt_cdc_rx += count;
    }

    // --- PIO UART RX → CDC#2 TX ---
    g_rx_buf_len = 0;
    while (!pio_sm_is_rx_fifo_empty(g_pio, g_sm_rx) &&
           g_rx_buf_len < sizeof(g_rx_buf)) {
        uint32_t raw = pio_sm_get(g_pio, g_sm_rx);
        g_rx_buf[g_rx_buf_len++] = static_cast<uint8_t>(raw >> 24);
    }
    if (g_rx_buf_len > 0) {
        g_cnt_pio_rx += g_rx_buf_len;
        // Unbedingt schreiben (kein connected-Gate): TinyUSB puffert bzw. verwirft
        // bei fehlendem Host selbst, der Datenfluss bleibt so aber sichtbar.
        tud_cdc_n_write(CDC_ITF, g_rx_buf, g_rx_buf_len);
        tud_cdc_n_write_flush(CDC_ITF);
        g_cnt_cdc_tx += g_rx_buf_len;
    }
}

void debug_counts(uint32_t& cdc_rx, uint32_t& pio_tx,
                  uint32_t& pio_rx, uint32_t& cdc_tx) {
    cdc_rx = g_cnt_cdc_rx;
    pio_tx = g_cnt_pio_tx;
    pio_rx = g_cnt_pio_rx;
    cdc_tx = g_cnt_cdc_tx;
}

} // namespace uart_bridge

// ---------------------------------------------------------------------------
// TinyUSB-Callback: Host setzt Line-Coding (Baudrate) auf CDC#2.
// ---------------------------------------------------------------------------
extern "C" void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding) {
    if (itf != 2) return;
    if (!p_line_coding || p_line_coding->bit_rate == 0) return;

    uart_bridge::g_baud = p_line_coding->bit_rate;
    if (uart_bridge::g_active) {
        uart_bridge::apply_baud(uart_bridge::g_baud);
    }
}
