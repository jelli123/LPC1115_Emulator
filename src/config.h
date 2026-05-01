#pragma once
//
// Konfiguration des Emulators. Persistent in `storage` abgelegt.
// Schlüssel sind als Konstanten exportiert, damit die CLI sie validieren kann.
//

#include <cstdint>
#include <cstddef>

namespace config {

// Schlüssel
inline constexpr const char* KEY_AUTOSTART       = "autostart";        // "0"/"1"
inline constexpr const char* KEY_TARGET_FREQ_HZ  = "freq_hz";          // dezimal, 1..150_000_000
inline constexpr const char* KEY_PIN_PREFIX      = "pin.";             // pin.<lpc>=<rp2350>

// Pin-Mapping LPC1115 -> RP2350 GPIO. -1 = nicht zugeordnet.
constexpr std::size_t LPC_PIN_COUNT = 64;
struct PinMap {
    int8_t lpc_to_rp[LPC_PIN_COUNT];
};

bool load();                         // ruft storage::config_load + bedient Defaults
bool save();

bool        autostart();
void        set_autostart(bool v);

uint32_t    target_frequency_hz();
void        set_target_frequency_hz(uint32_t hz);

const PinMap& pin_map();
bool         set_pin_map(uint8_t lpc_pin, int rp2350_gpio);

} // namespace config
