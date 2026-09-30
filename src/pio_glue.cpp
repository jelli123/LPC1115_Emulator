#include "fast_time.h"
#include "pio_glue.h"

#include <cstdio>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "hardware/structs/pads_bank0.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "timer_edge_ts.pio.h"  // von pioasm erzeugt (pico_generate_pio_header)
#include "match_pulse.pio.h"     // von pioasm erzeugt (pico_generate_pio_header)

namespace pio_glue {

namespace {
constexpr unsigned MAX_HANDLES = 4;   // je Programmtyp
} // namespace

void init() {}

// ---------------------------------------------------------------------------
// Flankengenaues Timestamping (timer_edge_ts-Programm).
// ---------------------------------------------------------------------------
namespace {

struct Ts {
    bool     used;
    PIO      pio;
    uint     sm;
    uint8_t  gpio;
    uint64_t t0_us;      // time_us_64 beim Start der SM (X = 0xFFFFFFFF)
    float    rate;       // Zaehlschritte je Sekunde
    uint32_t last_e31;   // zuletzt gesehene verstrichene Schritte (31 Bit)
    uint64_t wrap;       // aufsummierte 2^31-Ueberlaeufe
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

int ts_setup(uint8_t rp_gpio, float& out_rate_hz, bool claim_pin) {
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
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);   // 8 Eintraege Flanken-Puffer
    sm_config_set_jmp_pin(&c, rp_gpio);
    sm_config_set_in_pins(&c, rp_gpio);
    sm_config_set_in_shift(&c, /*shift_right=*/false, /*autopush=*/false, 32);
    sm_config_set_clkdiv(&c, clkdiv);

    if (claim_pin) {
        pio_gpio_init(g_ts_pio, rp_gpio);
        pio_sm_set_consecutive_pindirs(g_ts_pio, static_cast<uint>(sm),
                                       rp_gpio, 1, /*is_out=*/false);
    } else {
        // Nur mitlesen: PIO sieht jeden GPIO-Eingang unabhaengig von dessen
        // Funktion. Pad-Eingang freischalten, Funktion/Richtung unveraendert.
        gpio_set_input_enabled(rp_gpio, true);
        hw_clear_bits(&pads_bank0_hw->io[rp_gpio], PADS_BANK0_GPIO0_ISO_BITS);
    }
    pio_sm_init(g_ts_pio, static_cast<uint>(sm), g_ts_offset, &c);
    const uint32_t irq = save_and_disable_interrupts();
    pio_sm_set_enabled(g_ts_pio, static_cast<uint>(sm), true);
    const uint64_t t0 = fast_time_us();
    restore_interrupts(irq);

    g_ts[slot] = { true, g_ts_pio, static_cast<uint>(sm), rp_gpio, t0, out_rate_hz, 0, 0 };
    return slot;
}

bool __not_in_flash_func(ts_read_edge)(int handle, uint64_t& t_us, bool& level) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return false;
    auto& t = g_ts[handle];
    if (!t.used) return false;
    if (pio_sm_is_rx_fifo_empty(t.pio, t.sm)) return false;
    const uint32_t raw = pio_sm_get(t.pio, t.sm);
    level = (raw & 1u) != 0;
    // X laeuft ab 0xFFFFFFFF abwaerts; FIFO-Wert = X[30:0] << 1 | Pegel.
    const uint32_t e31 = 0x7FFF'FFFFu - (raw >> 1);        // verstrichene Schritte (31 Bit)
    if (e31 < t.last_e31) t.wrap += (1ull << 31);         // 31-Bit-Ueberlauf
    t.last_e31 = e31;
    const uint64_t steps = t.wrap + e31;
    t_us = t.t0_us + ((t.rate > 999'999.0f && t.rate < 1'000'001.0f)
                          ? steps
                          : static_cast<uint64_t>(static_cast<double>(steps) * 1e6 / t.rate));
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

bool tx_debug(int handle, uint32_t& pc, uint32_t& fifo, uint32_t& pin_out, uint32_t& pin_oe) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES) || !g_tx[handle].used) return false;
    const auto& t = g_tx[handle];
    pc      = pio_sm_get_pc(t.pio, t.sm) - static_cast<uint32_t>(g_tx_offset);
    fifo    = pio_sm_get_tx_fifo_level(t.pio, t.sm);
    pin_out = (t.pio->dbg_padout >> t.gpio) & 1u;
    pin_oe  = (t.pio->dbg_padoe  >> t.gpio) & 1u;
    return true;
}

void __not_in_flash_func(tx_cancel)(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    // SM anhalten, FIFO leeren, Pin low, zurueck an den Programmanfang.
    pio_sm_set_enabled(t.pio, t.sm, false);
    pio_sm_clear_fifos(t.pio, t.sm);
    pio_sm_restart(t.pio, t.sm);
    pio_sm_exec(t.pio, t.sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(t.pio, t.sm, pio_encode_jmp(static_cast<uint>(g_tx_offset)));
    pio_sm_set_enabled(t.pio, t.sm, true);
}

void __not_in_flash_func(tx_rebuild_begin)(int handle, bool keep_high) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    pio_sm_set_enabled(t.pio, t.sm, false);
    pio_sm_clear_fifos(t.pio, t.sm);
    pio_sm_restart(t.pio, t.sm);
    if (!keep_high) pio_sm_exec(t.pio, t.sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(t.pio, t.sm, pio_encode_jmp(static_cast<uint>(g_tx_offset)));
}

void __not_in_flash_func(tx_park)(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    tx_cancel(handle);                                  // SM leer, Pin low
    pio_sm_set_enabled(t.pio, t.sm, false);
    gpio_put(t.gpio, false);
    gpio_set_dir(t.gpio, true);
    hw_write_masked(&io_bank0_hw->io[t.gpio].ctrl, GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB,
                    IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS);
}

void __not_in_flash_func(tx_unpark)(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    tx_rebuild_begin(handle, false);                    // Pin-Ausgabewert der SM = low
    hw_write_masked(&io_bank0_hw->io[t.gpio].ctrl,
                    static_cast<uint32_t>(pio_get_funcsel(t.pio)) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB,
                    IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS);
    pio_sm_set_enabled(t.pio, t.sm, true);
}

void __not_in_flash_func(tx_rebuild_end)(int handle) {
    if (handle < 0 || handle >= static_cast<int>(MAX_HANDLES)) return;
    auto& t = g_tx[handle];
    if (!t.used) return;
    pio_sm_set_enabled(t.pio, t.sm, true);
}

bool __not_in_flash_func(tx_emit)(int handle, uint32_t delay_counts, uint32_t width_counts) {
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

void usage(uint32_t& sm_used, uint32_t& sm_total,
           uint32_t& instr_used, uint32_t& instr_total) {
    sm_used = sm_total = instr_used = instr_total = 0;
    // Ein 1-Instruktions-Dummy (jmp 0). Passt es an einen Offset NICHT, ist der
    // Slot belegt -> so zaehlen wir belegte Instruktions-Slots exakt, nur mit
    // oeffentlicher API (kein Zugriff auf SDK-interne Belegungs-Bitmap).
    static const uint16_t probe_instr[] = { 0x0000 };  // jmp 0
    static const pio_program_t probe = {
        .instructions = probe_instr, .length = 1, .origin = -1,
        .pio_version = 0, .used_gpio_ranges = 0,
    };
    for (uint32_t i = 0; i < NUM_PIOS; ++i) {
        PIO pio = PIO_INSTANCE(i);
        for (uint sm = 0; sm < NUM_PIO_STATE_MACHINES; ++sm) {
            ++sm_total;
            if (pio_sm_is_claimed(pio, sm)) ++sm_used;
        }
        for (uint off = 0; off < 32u; ++off) {
            ++instr_total;
            if (!pio_can_add_program_at_offset(pio, &probe, off)) ++instr_used;
        }
    }
}

} // namespace pio_glue
