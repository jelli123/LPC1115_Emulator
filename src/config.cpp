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

void apply_defaults() {
    for (auto& v : g_pin_map.lpc_to_rp) v = -1;
    g_target_freq = 48'000'000;
    g_autostart   = false;
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
    load_pin_map_from_storage();
    return true;
}

bool save() {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_autostart ? 1 : 0));
    if (!storage::config_set(KEY_AUTOSTART, buf)) return false;
    std::snprintf(buf, sizeof buf, "%lu", static_cast<unsigned long>(g_target_freq));
    if (!storage::config_set(KEY_TARGET_FREQ_HZ, buf)) return false;
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

} // namespace config
