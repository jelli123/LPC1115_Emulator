#include "led.h"
#include "emulator.h"
#include "peripherals.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace led {
namespace {

// Status-LED = PICO_DEFAULT_LED_PIN des Board-Headers (Pico 2: GP25). Definiert
// das Board keine LED (z. B. selfbus_io8_rp2354: GP25 ist dort EIB_IN), bleibt
// die Status-LED aus und der Pin unangetastet.
#ifdef PICO_DEFAULT_LED_PIN
constexpr int LED_GPIO = PICO_DEFAULT_LED_PIN;
#else
constexpr int LED_GPIO = -1;
#endif

void set(bool on) {
    if constexpr (LED_GPIO >= 0) gpio_put(static_cast<uint>(LED_GPIO), on);
    else (void)on;
}

} // namespace

void init() {
    if constexpr (LED_GPIO >= 0) {
        gpio_init(static_cast<uint>(LED_GPIO));
        gpio_set_dir(static_cast<uint>(LED_GPIO), true);
        set(false);
    }
}

void poll() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    auto st = emulator::state();
    switch (st) {
        case emulator::State::Running:
            // Dauerlicht = "Gast laeuft". Die Onboard-LED spiegelt bewusst KEINEN
            // Gast-Pin mehr (frueher PIO0.7): das war irrefuehrend, sobald der
            // Gast diesen Pin anderweitig nutzt, und koppelte die Status-Anzeige
            // an eine willkuerliche Gast-Konvention. Wer die Gast-LED sehen will,
            // mappt den Pin per CONFIG.INI auf einen echten GPIO.
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
