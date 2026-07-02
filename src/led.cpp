#include "led.h"
#include "emulator.h"
#include "peripherals.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace led {
namespace {

#ifndef PICO_DEFAULT_LED_PIN
#  define PICO_DEFAULT_LED_PIN 25
#endif

constexpr uint LED_GPIO = PICO_DEFAULT_LED_PIN;

void set(bool on) { gpio_put(LED_GPIO, on); }

// --- Diagnose-Sampling ----------------------------------------------------
// poll() laeuft ~1000x/s auf Core0. Wir zaehlen ueber die Zeit, in welchem
// State der Emulator ist und wie der Gast-Pin PIO0.7 (die gespiegelte Blink-
// LED) gesehen wird. Aus dem Verhaeltnis high:low laesst sich ablesen, ob der
// Pin sauber toggelt (≈ gleiche Zaehler) oder haengt (ein Zaehler ≈ 0), ohne zu
// raten. In 'stats' sichtbar.
struct LedDiag {
    uint32_t polls;
    uint32_t st_run, st_idle, st_halt, st_fault;
    uint32_t p07_high, p07_low, p07_noout;
    uint32_t gp17_high, gp17_low;   // physischer Pegel des gemappten Pins
};
LedDiag g_diag{};

} // namespace

void init() {
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, true);
    set(false);
}

void debug_counters(uint32_t& polls, uint32_t& run, uint32_t& idle,
                    uint32_t& halt, uint32_t& fault,
                    uint32_t& p07_high, uint32_t& p07_low, uint32_t& p07_noout) {
    polls    = g_diag.polls;
    run      = g_diag.st_run;
    idle     = g_diag.st_idle;
    halt     = g_diag.st_halt;
    fault    = g_diag.st_fault;
    p07_high = g_diag.p07_high;
    p07_low  = g_diag.p07_low;
    p07_noout= g_diag.p07_noout;
}

void debug_reset() { g_diag = {}; }

void poll() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    auto st = emulator::state();
    ++g_diag.polls;
    switch (st) {
        case emulator::State::Running:  ++g_diag.st_run;   break;
        case emulator::State::Idle:     ++g_diag.st_idle;  break;
        case emulator::State::Halted:   ++g_diag.st_halt;  break;
        case emulator::State::Faulted:  ++g_diag.st_fault; break;
        default: break;
    }
    {
        bool lvl = false;
        if (peripherals::guest_output_level(0, 7, lvl)) {
            if (lvl) ++g_diag.p07_high; else ++g_diag.p07_low;
        } else {
            ++g_diag.p07_noout;
        }
    }
    switch (st) {
        case emulator::State::Running:
            // Onboard-LED spiegelt die Gast-Blink-LED PIO0.7 (LPCxpresso-
            // Konvention), damit ein Blink-Programm sichtbar auf der einzigen
            // Bordschnittstelle blinkt. Treibt der Gast PIO0.7 nicht als
            // Ausgang, bleibt es beim reinen "laeuft"-Dauerlicht.
            {
                bool lvl = false;
                if (peripherals::guest_output_level(0, 7, lvl))
                    set(lvl);
                else
                    set(true);
            }
            break;

        case emulator::State::Faulted: {
            // 8 Hz Flackern (62 ms toggle)
            set(((now / 62u) & 1u) != 0);
            break;
        }

        case emulator::State::Halted: {
            // Doppelblitz alle 2 s: 0..80ms an, 80..160ms aus, 160..240ms an,
            // 240..2000ms aus.
            uint32_t t = now % 2000u;
            set((t < 80u) || (t >= 160u && t < 240u));
            break;
        }

        case emulator::State::Idle:
        default:
            // 1 Hz Heartbeat, 50 % duty
            set((now % 1000u) < 500u);
            break;
    }
}

} // namespace led
