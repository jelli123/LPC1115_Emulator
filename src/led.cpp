#include "led.h"
#include "emulator.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace led {
namespace {

#ifndef PICO_DEFAULT_LED_PIN
#  define PICO_DEFAULT_LED_PIN 25
#endif

constexpr uint LED_GPIO = PICO_DEFAULT_LED_PIN;

void set(bool on) { gpio_put(LED_GPIO, on); }

} // namespace

void init() {
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, true);
    set(false);
}

void poll() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    auto st = emulator::state();
    switch (st) {
        case emulator::State::Running:
            set(true);
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
