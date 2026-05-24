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
inline constexpr const char* KEY_UART_BRIDGE_EN  = "uart_bridge_en";   // "0"/"1"
inline constexpr const char* KEY_UART_BRIDGE_TX  = "uart_bridge_tx";   // GPIO-Nummer
inline constexpr const char* KEY_UART_BRIDGE_RX  = "uart_bridge_rx";   // GPIO-Nummer

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
void         apply_default_pinmap();

// UART-Bridge-Konfiguration
bool        uart_bridge_enabled();
void        set_uart_bridge_enabled(bool v);
int         uart_bridge_tx_pin();
void        set_uart_bridge_tx_pin(int gpio);
int         uart_bridge_rx_pin();
void        set_uart_bridge_rx_pin(int gpio);

} // namespace config
