#include "config.h"
#include "storage.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

namespace config {

namespace {

PinMap   g_pin_map{};
uint32_t g_target_freq = 48'000'000;
bool     g_autostart   = false;
bool     g_uart_bridge_en = false;
int      g_uart_bridge_tx = -1;
int      g_uart_bridge_rx = -1;

constexpr uint32_t MAX_FREQ_HZ = 150'000'000; // RP2350-Limit, defensiv
constexpr int      MAX_GPIO    = 47;          // RP2350-Pinanzahl konservativ

// Sicherer strtoul, lehnt Überlauf, Leerstring, Vorzeichen ab.
bool parse_uint32(const char* s, uint32_t& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') return false;
    if (v > 0xFFFF'FFFFul) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

// Default-Pinmap: Weist LPC1115-Pins sinnvolle RP2350-GPIOs zu.
// Layout orientiert sich an der physischen Anordnung am LQFP48-Gehäuse:
//   Port 0 (P0_0..P0_11) → GP2..GP13 (linke Seite des Pico 2)
//   Port 1 (P1_0..P1_7)  → GP14..GP21 (rechte Seite)
//   Port 1 UART: P1_8 → GP1 (UART0 RX), P1_9 → GP0 (UART0 TX)
//   P1_10 → GP22, P1_11 → GP26
//   Port 2: P2_0 → GP27, P2_1 → GP28 (ADC-fähig)
//   Port 3: nicht gemappt (zu wenige freie Pins)
// GP25 = Onboard-LED (reserviert für Status).
void apply_default_pinmap_impl() {
    for (auto& v : g_pin_map.lpc_to_rp) v = -1;

    // Port 0: P0_0..P0_11 → GP2..GP13
    for (int i = 0; i <= 11; ++i)
        g_pin_map.lpc_to_rp[0 * 12 + i] = static_cast<int8_t>(2 + i);

    // Port 1: P1_0..P1_7 → GP14..GP21
    for (int i = 0; i <= 7; ++i)
        g_pin_map.lpc_to_rp[1 * 12 + i] = static_cast<int8_t>(14 + i);

    // Port 1 UART (KNX-Bus): P1_8=RX → GP1, P1_9=TX → GP0
    g_pin_map.lpc_to_rp[1 * 12 + 8]  = 1;   // UART0 RX
    g_pin_map.lpc_to_rp[1 * 12 + 9]  = 0;   // UART0 TX
    g_pin_map.lpc_to_rp[1 * 12 + 10] = 22;
    g_pin_map.lpc_to_rp[1 * 12 + 11] = 26;

    // Port 2: P2_0 → GP27, P2_1 → GP28
    g_pin_map.lpc_to_rp[2 * 12 + 0] = 27;
    g_pin_map.lpc_to_rp[2 * 12 + 1] = 28;
}

void apply_defaults() {
    apply_default_pinmap_impl();
    g_target_freq = 48'000'000;
    g_autostart   = false;
    g_uart_bridge_en = false;
    g_uart_bridge_tx = -1;
    g_uart_bridge_rx = -1;
}

void load_pin_map_from_storage() {
    char buf[16];
    char key[24];
    for (std::size_t i = 0; i < LPC_PIN_COUNT; ++i) {
        std::snprintf(key, sizeof key, "%s%u", KEY_PIN_PREFIX,
                      static_cast<unsigned>(i));
        if (!storage::config_get(key, buf, sizeof buf)) continue;
        uint32_t v;
        if (!parse_uint32(buf, v)) continue;
        if (v > static_cast<uint32_t>(MAX_GPIO)) continue;
        g_pin_map.lpc_to_rp[i] = static_cast<int8_t>(v);
    }
}

} // namespace

bool load() {
    apply_defaults();
    storage::config_load(); // ok wenn leer
    char buf[32];
    if (storage::config_get(KEY_AUTOSTART, buf, sizeof buf)) {
        g_autostart = (buf[0] == '1');
    }
    if (storage::config_get(KEY_TARGET_FREQ_HZ, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v > 0 && v <= MAX_FREQ_HZ) g_target_freq = v;
    }
    if (storage::config_get(KEY_UART_BRIDGE_EN, buf, sizeof buf)) {
        g_uart_bridge_en = (buf[0] == '1');
    }
    if (storage::config_get(KEY_UART_BRIDGE_TX, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_uart_bridge_tx = static_cast<int>(v);
    }
    if (storage::config_get(KEY_UART_BRIDGE_RX, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_uart_bridge_rx = static_cast<int>(v);
    }
    load_pin_map_from_storage();
    return true;
}

bool save() {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_autostart ? 1 : 0));
    if (!storage::config_set(KEY_AUTOSTART, buf)) return false;
    std::snprintf(buf, sizeof buf, "%lu", static_cast<unsigned long>(g_target_freq));
    if (!storage::config_set(KEY_TARGET_FREQ_HZ, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_uart_bridge_en ? 1 : 0));
    if (!storage::config_set(KEY_UART_BRIDGE_EN, buf)) return false;
    if (g_uart_bridge_tx >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_uart_bridge_tx);
        if (!storage::config_set(KEY_UART_BRIDGE_TX, buf)) return false;
    }
    if (g_uart_bridge_rx >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_uart_bridge_rx);
        if (!storage::config_set(KEY_UART_BRIDGE_RX, buf)) return false;
    }
    char key[24];
    for (std::size_t i = 0; i < LPC_PIN_COUNT; ++i) {
        if (g_pin_map.lpc_to_rp[i] < 0) continue;
        std::snprintf(key, sizeof key, "%s%u", KEY_PIN_PREFIX,
                      static_cast<unsigned>(i));
        std::snprintf(buf, sizeof buf, "%d", g_pin_map.lpc_to_rp[i]);
        if (!storage::config_set(key, buf)) return false;
    }
    return storage::config_commit();
}

bool        autostart()                    { return g_autostart; }
void        set_autostart(bool v)          { g_autostart = v; }
uint32_t    target_frequency_hz()          { return g_target_freq; }

void set_target_frequency_hz(uint32_t hz) {
    if (hz == 0 || hz > MAX_FREQ_HZ) return;
    g_target_freq = hz;
}

const PinMap& pin_map() { return g_pin_map; }

bool set_pin_map(uint8_t lpc_pin, int rp2350_gpio) {
    if (lpc_pin >= LPC_PIN_COUNT) return false;
    if (rp2350_gpio < -1 || rp2350_gpio > MAX_GPIO) return false;
    g_pin_map.lpc_to_rp[lpc_pin] = static_cast<int8_t>(rp2350_gpio);
    return true;
}

void apply_default_pinmap() {
    apply_default_pinmap_impl();
}

bool uart_bridge_enabled()           { return g_uart_bridge_en; }
void set_uart_bridge_enabled(bool v) { g_uart_bridge_en = v; }
int  uart_bridge_tx_pin()            { return g_uart_bridge_tx; }
void set_uart_bridge_tx_pin(int gpio){ g_uart_bridge_tx = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
int  uart_bridge_rx_pin()            { return g_uart_bridge_rx; }
void set_uart_bridge_rx_pin(int gpio){ g_uart_bridge_rx = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }

} // namespace config
