#include "pio_glue.h"

#include <cstdio>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "timer_edge_ts.pio.h"  // von pioasm erzeugt (pico_generate_pio_header)
#include "match_pulse.pio.h"     // von pioasm erzeugt (pico_generate_pio_header)

// ---------------------------------------------------------------------------
// PIO-Programm: Edge-Capture mit 32-Bit-Cycle-Counter.
//
//     .program edge_capture
//     .wrap_target
//         mov   x, !null            ; X = 0xFFFFFFFF
//     loop:
//         jmp   pin   capture       ; springt, wenn Pin == 1
//         jmp   x--   loop
//     capture:
//         mov   isr, x              ; ISR = X
//         push  noblock             ; → RX-FIFO
//     .wrap
//
// Maschinen-Code (16-Bit):
//   0x80a0   ; mov   x, !null
//   0x00a3   ; jmp   pin, capture(=offset+3)
//   0x0041   ; jmp   x--, loop(=offset+1)
//   0xa0a3   ; mov   isr, x
//   0x8000   ; push  noblock
// wrap_target=0, wrap=4
// ---------------------------------------------------------------------------

namespace pio_glue {

namespace {

constexpr uint16_t edge_capture_program_instructions[] = {
    0x80a0,
    0x00a3,
    0x0041,
    0xa0a3,
    0x8000,
};

const struct pio_program edge_capture_program = {
    .instructions = edge_capture_program_instructions,
    .length       = 5,
    .origin       = -1,
    .pio_version  = 0,
    .used_gpio_ranges = 0,
};

constexpr unsigned MAX_HANDLES = 4;

struct Capture {
    bool      used;
    PIO       pio;
    uint      sm;
    int       offset;
    uint8_t   gpio;
    uint32_t  last;
};
Capture g_caps[MAX_HANDLES]{};

PIO     g_pio = pio0;
bool    g_program_loaded = false;
int     g_program_offset = -1;

bool ensure_program() {
    if (g_program_loaded) return true;
    if (!pio_can_add_program(g_pio, &edge_capture_program)) {
        g_pio = pio1;
        if (!pio_can_add_program(g_pio, &edge_capture_program)) return false;
    }
    g_program_offset = pio_add_program(g_pio, &edge_capture_program);
    g_program_loaded = true;
    return true;
}

void config_sm(PIO pio, uint sm, int offset, uint8_t gpio) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 4);
    sm_config_set_jmp_pin(&c, gpio);
    sm_config_set_in_pins(&c, gpio);
    sm_config_set_in_shift(&c, /*shift_right=*/false,
                               /*autopush=*/false, 32);
    sm_config_set_clkdiv(&c, 1.0f);

    pio_gpio_init(pio, gpio);
    pio_sm_set_consecutive_pindirs(pio, sm, gpio, 1, /*is_out=*/false);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

} // namespace

void init() {
    for (auto& c : g_caps) c = {};
    g_program_loaded = false;
    g_program_offset = -1;
    g_pio = pio0;
}
uint16_t setup_capture(uint8_t rp_gpio, bool /*rising_edge*/) {
    if (!ensure_program()) {
        std::printf("[PIO] kein Platz für edge_capture-Programm\n");
        return 0xFFFF;
    }
    for (uint16_t i = 0; i < MAX_HANDLES; ++i) {
        if (g_caps[i].used) continue;
        int sm = pio_claim_unused_sm(g_pio, false);
        if (sm < 0) return 0xFFFF;
        g_caps[i] = { true, g_pio, static_cast<uint>(sm),
                      g_program_offset, rp_gpio, 0 };
        config_sm(g_pio, static_cast<uint>(sm), g_program_offset, rp_gpio);
        return i;
    }
    return 0xFFFF;
}

bool capture_read(uint16_t handle, uint32_t& out) {
    if (handle >= MAX_HANDLES) return false;
    auto& c = g_caps[handle];
    if (!c.used) return false;
    if (pio_sm_is_rx_fifo_empty(c.pio, c.sm)) {
        out = c.last;
        return false;
    }
    uint32_t raw = pio_sm_get(c.pio, c.sm);
    c.last = 0xFFFF'FFFFu - raw;
    out = c.last;
    return true;
}

// ---------------------------------------------------------------------------
// Flankengenaues Timestamping (timer_edge_ts-Programm).
// ---------------------------------------------------------------------------
namespace {

struct Ts {
    bool    used;
    PIO     pio;
    uint    sm;
    uint8_t gpio;
};
Ts   g_ts[MAX_HANDLES]{};

PIO  g_ts_pio = pio0;
bool g_ts_loaded = false;
int  g_ts_offset = -1;

bool ts_ensure_program() {
    if (g_ts_loaded) return true;
    g_ts_pio = pio0;
    if (!pio_can_add_program(g_ts_pio, &timer_edge_ts_program)) {
        g_ts_pio = pio1;
        if (!pio_can_add_program(g_ts_pio, &timer_edge_ts_program)) return false;
    }
    g_ts_offset = pio_add_program(g_ts_pio, &timer_edge_ts_program);
    g_ts_loaded = true;
    return true;
}

} // namespace

int ts_setup(uint8_t rp_gpio, float& out_rate_hz) {
    if (!ts_ensure_program()) {
        std::printf("[PIO] kein Platz fuer timer_edge_ts-Programm\n");
        return -1;
    }
    int slot = -1;
    for (int i = 0; i < static_cast<int>(MAX_HANDLES); ++i)
        if (!g_ts[i].used) { slot = i; break; }
    if (slot < 0) return -1;

    int sm = pio_claim_unused_sm(g_ts_pio, false);
    if (sm < 0) return -1;

    // Zaehlrate ~1 MHz anpeilen: clkdiv = clk_sys / (2 * Zielrate).
    // (2 Takte je Zaehlschritt, siehe .pio). clkdiv ist 16.8-Fixpunkt.
    float sysclk = static_cast<float>(clock_get_hz(clk_sys));
    float clkdiv = sysclk / (2.0f * 1'000'000.0f);
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    out_rate_hz = sysclk / (2.0f * clkdiv);

    pio_sm_config c = timer_edge_ts_program_get_default_config(g_ts_offset);
    sm_config_set_jmp_pin(&c, rp_gpio);
    sm_config_set_in_pins(&c, rp_gpio);
    sm_config_set_in_shift(&c, /*shift_right=*/false, /*autopush=*/false, 32);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_gpio_init(g_ts_pio, rp_gpio);
    pio_sm_set_consecutive_pindirs(g_ts_pio, static_cast<uint>(sm),
                                   rp_gpio, 1, /*is_out=*/false);
    pio_sm_init(g_ts_pio, static_cast<uint>(sm), g_ts_offset, &c);
    pio_sm_set_enabled(g_ts_pio, static_cast<uint>(sm), true);

    g_ts[slot] = { true, g_ts_pio, static_cast<uint>(sm), rp_gpio };
    return slot;
}

bool ts_read(int handle, uint32_t& counter) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return false;
    auto& t = g_ts[handle];
    if (!t.used) return false;
    if (pio_sm_is_rx_fifo_empty(t.pio, t.sm)) return false;
    counter = pio_sm_get(t.pio, t.sm);
    return true;
}

void ts_teardown(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_ts[handle];
    if (!t.used) return;
    pio_sm_set_enabled(t.pio, t.sm, false);
    pio_sm_unclaim(t.pio, t.sm);
    t = {};
}

// ---------------------------------------------------------------------------
// TX-Puls-Erzeugung (match_pulse-Programm).
// ---------------------------------------------------------------------------
namespace {

struct Tx {
    bool    used;
    PIO     pio;
    uint    sm;
    uint8_t gpio;
};
Tx   g_tx[MAX_HANDLES]{};

PIO  g_tx_pio = pio0;
bool g_tx_loaded = false;
int  g_tx_offset = -1;

bool tx_ensure_program() {
    if (g_tx_loaded) return true;
    g_tx_pio = pio0;
    if (!pio_can_add_program(g_tx_pio, &match_pulse_program)) {
        g_tx_pio = pio1;
        if (!pio_can_add_program(g_tx_pio, &match_pulse_program)) return false;
    }
    g_tx_offset = pio_add_program(g_tx_pio, &match_pulse_program);
    g_tx_loaded = true;
    return true;
}

} // namespace

int tx_setup(uint8_t rp_gpio, float& out_rate_hz) {
    if (!tx_ensure_program()) {
        std::printf("[PIO] kein Platz fuer match_pulse-Programm\n");
        return -1;
    }
    int slot = -1;
    for (int i = 0; i < static_cast<int>(MAX_HANDLES); ++i)
        if (!g_tx[i].used) { slot = i; break; }
    if (slot < 0) return -1;

    int sm = pio_claim_unused_sm(g_tx_pio, false);
    if (sm < 0) return -1;

    // Zaehlrate ~1 MHz (1 Count = 1 PIO-Instruktion = 1 Tick der jmp-Schleife).
    float sysclk = static_cast<float>(clock_get_hz(clk_sys));
    float clkdiv = sysclk / 1'000'000.0f;
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    out_rate_hz = sysclk / clkdiv;

    match_pulse_program_init(g_tx_pio, static_cast<uint>(sm),
                             static_cast<uint>(g_tx_offset), rp_gpio, clkdiv);

    g_tx[slot] = { true, g_tx_pio, static_cast<uint>(sm), rp_gpio };
    return slot;
}

bool tx_emit(int handle, uint32_t delay_counts, uint32_t width_counts) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return false;
    auto& t = g_tx[handle];
    if (!t.used) return false;
    // Beide Worte muessen zusammen passen, sonst Puls verwerfen (kein Teilpuls).
    if (pio_sm_get_tx_fifo_level(t.pio, t.sm) > 2) return false;
    pio_sm_put(t.pio, t.sm, delay_counts);
    pio_sm_put(t.pio, t.sm, width_counts);
    return true;
}

void tx_teardown(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    pio_sm_set_enabled(t.pio, t.sm, false);
    pio_sm_unclaim(t.pio, t.sm);
    t = {};
}

} // namespace pio_glue
