#include "pio_glue.h"

#include <cstdio>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

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

} // namespace pio_glue
