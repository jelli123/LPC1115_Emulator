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
bool     g_i2c_bridge_en   = false;
int      g_i2c_bridge_inst = 0;
int      g_i2c_bridge_sda  = -1;
int      g_i2c_bridge_scl  = -1;
uint32_t g_i2c_bridge_hz   = 100'000;
bool     g_spi_bridge_en   = false;
int      g_spi_bridge_inst = 0;
int      g_spi_bridge_lpc  = 0;
int      g_spi_bridge_sck  = -1;
int      g_spi_bridge_mosi = -1;
int      g_spi_bridge_miso = -1;
uint32_t g_spi_bridge_hz   = 1'000'000;
bool     g_adc_bridge_en   = false;
bool     g_wfi_pin_wakeup  = false;
bool     g_primask_shadow  = false;

// Timer-Capture-/Match-Pin-Bindung (4 Timer, je 1 Capture + 4 Match).
// -1 = nicht gebunden.
int8_t   g_ct_cap[4]    = { -1, -1, -1, -1 };
int8_t   g_ct_mat[4][4] = {
    { -1, -1, -1, -1 }, { -1, -1, -1, -1 },
    { -1, -1, -1, -1 }, { -1, -1, -1, -1 },
};
bool     g_tcap_pio = false;

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
//   P1_10 → GP22
//   Port 2/3: standardmäßig NICHT gemappt.
// GP25 = Onboard-LED (reserviert für Status).
// GP26..GP29 bleiben absichtlich UNGEMAPPT: Das sind die einzigen ADC-
// fähigen RP2350A-Pins (ADC0..3). Die ADC-Bridge (config::adc_bridge_en)
// belegt sie exklusiv; ein paralleles GPIO-Mapping würde mit dem ADC
// kollidieren. Wer P1_10/11 bzw. Port 2/3 als GPIO braucht und kein ADC
// nutzt, kann sie per CONFIG.INI (pin.<port>_<pin>=<gpio>) frei zuordnen.
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
    // P1_11, P2_x: ungemappt (GP26..29 für ADC reserviert, siehe oben).
}

void apply_defaults() {
    apply_default_pinmap_impl();
    g_target_freq = 48'000'000;
    g_autostart   = false;
    g_uart_bridge_en = false;
    g_uart_bridge_tx = -1;
    g_uart_bridge_rx = -1;
    g_i2c_bridge_en   = false;
    g_i2c_bridge_inst = 0;
    g_i2c_bridge_sda  = -1;
    g_i2c_bridge_scl  = -1;
    g_i2c_bridge_hz   = 100'000;
    g_spi_bridge_en   = false;
    g_spi_bridge_inst = 0;
    g_spi_bridge_lpc  = 0;
    g_spi_bridge_sck  = -1;
    g_spi_bridge_mosi = -1;
    g_spi_bridge_miso = -1;
    g_spi_bridge_hz   = 1'000'000;
    g_adc_bridge_en   = false;
    g_wfi_pin_wakeup  = false;
    g_primask_shadow  = false;
    for (int t = 0; t < 4; ++t) {
        g_ct_cap[t] = -1;
        for (int m = 0; m < 4; ++m) g_ct_mat[t][m] = -1;
    }
    g_tcap_pio = false;
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
    if (storage::config_get(KEY_I2C_BRIDGE_EN, buf, sizeof buf)) {
        g_i2c_bridge_en = (buf[0] == '1');
    }
    if (storage::config_get(KEY_I2C_BRIDGE_INST, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= 1) g_i2c_bridge_inst = static_cast<int>(v);
    }
    if (storage::config_get(KEY_I2C_BRIDGE_SDA, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_i2c_bridge_sda = static_cast<int>(v);
    }
    if (storage::config_get(KEY_I2C_BRIDGE_SCL, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_i2c_bridge_scl = static_cast<int>(v);
    }
    if (storage::config_get(KEY_I2C_BRIDGE_HZ, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v > 0 && v <= 1'000'000) g_i2c_bridge_hz = v;
    }
    if (storage::config_get(KEY_SPI_BRIDGE_EN, buf, sizeof buf)) {
        g_spi_bridge_en = (buf[0] == '1');
    }
    if (storage::config_get(KEY_SPI_BRIDGE_INST, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= 1) g_spi_bridge_inst = static_cast<int>(v);
    }
    if (storage::config_get(KEY_SPI_BRIDGE_LPC, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= 1) g_spi_bridge_lpc = static_cast<int>(v);
    }
    if (storage::config_get(KEY_SPI_BRIDGE_SCK, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_spi_bridge_sck = static_cast<int>(v);
    }
    if (storage::config_get(KEY_SPI_BRIDGE_MOSI, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_spi_bridge_mosi = static_cast<int>(v);
    }
    if (storage::config_get(KEY_SPI_BRIDGE_MISO, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
            g_spi_bridge_miso = static_cast<int>(v);
    }
    if (storage::config_get(KEY_SPI_BRIDGE_HZ, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32(buf, v) && v > 0 && v <= 50'000'000) g_spi_bridge_hz = v;
    }
    if (storage::config_get(KEY_ADC_BRIDGE_EN, buf, sizeof buf)) {
        g_adc_bridge_en = (buf[0] == '1');
    }
    if (storage::config_get(KEY_WFI_PIN_WAKEUP, buf, sizeof buf)) {
        g_wfi_pin_wakeup = (buf[0] == '1');
    }
    if (storage::config_get(KEY_PRIMASK_SHADOW, buf, sizeof buf)) {
        g_primask_shadow = (buf[0] == '1');
    }
    {
        char key[24];
        for (int t = 0; t < 4; ++t) {
            std::snprintf(key, sizeof key, "%s%d", KEY_TCAP_PREFIX, t);
            if (storage::config_get(key, buf, sizeof buf)) {
                uint32_t v;
                if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
                    g_ct_cap[t] = static_cast<int8_t>(v);
            }
            for (int m = 0; m < 4; ++m) {
                std::snprintf(key, sizeof key, "%s%d.%d", KEY_TMAT_PREFIX, t, m);
                if (storage::config_get(key, buf, sizeof buf)) {
                    uint32_t v;
                    if (parse_uint32(buf, v) && v <= static_cast<uint32_t>(MAX_GPIO))
                        g_ct_mat[t][m] = static_cast<int8_t>(v);
                }
            }
        }
    }
    if (storage::config_get(KEY_TCAP_PIO, buf, sizeof buf)) {
        g_tcap_pio = (buf[0] == '1');
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
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_i2c_bridge_en ? 1 : 0));
    if (!storage::config_set(KEY_I2C_BRIDGE_EN, buf)) return false;
    std::snprintf(buf, sizeof buf, "%d", g_i2c_bridge_inst);
    if (!storage::config_set(KEY_I2C_BRIDGE_INST, buf)) return false;
    if (g_i2c_bridge_sda >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_i2c_bridge_sda);
        if (!storage::config_set(KEY_I2C_BRIDGE_SDA, buf)) return false;
    }
    if (g_i2c_bridge_scl >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_i2c_bridge_scl);
        if (!storage::config_set(KEY_I2C_BRIDGE_SCL, buf)) return false;
    }
    std::snprintf(buf, sizeof buf, "%lu", static_cast<unsigned long>(g_i2c_bridge_hz));
    if (!storage::config_set(KEY_I2C_BRIDGE_HZ, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_spi_bridge_en ? 1 : 0));
    if (!storage::config_set(KEY_SPI_BRIDGE_EN, buf)) return false;
    std::snprintf(buf, sizeof buf, "%d", g_spi_bridge_inst);
    if (!storage::config_set(KEY_SPI_BRIDGE_INST, buf)) return false;
    std::snprintf(buf, sizeof buf, "%d", g_spi_bridge_lpc);
    if (!storage::config_set(KEY_SPI_BRIDGE_LPC, buf)) return false;
    if (g_spi_bridge_sck >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_spi_bridge_sck);
        if (!storage::config_set(KEY_SPI_BRIDGE_SCK, buf)) return false;
    }
    if (g_spi_bridge_mosi >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_spi_bridge_mosi);
        if (!storage::config_set(KEY_SPI_BRIDGE_MOSI, buf)) return false;
    }
    if (g_spi_bridge_miso >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_spi_bridge_miso);
        if (!storage::config_set(KEY_SPI_BRIDGE_MISO, buf)) return false;
    }
    std::snprintf(buf, sizeof buf, "%lu", static_cast<unsigned long>(g_spi_bridge_hz));
    if (!storage::config_set(KEY_SPI_BRIDGE_HZ, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_adc_bridge_en ? 1 : 0));
    if (!storage::config_set(KEY_ADC_BRIDGE_EN, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_wfi_pin_wakeup ? 1 : 0));
    if (!storage::config_set(KEY_WFI_PIN_WAKEUP, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_primask_shadow ? 1 : 0));
    if (!storage::config_set(KEY_PRIMASK_SHADOW, buf)) return false;
    {
        char key[24];
        for (int t = 0; t < 4; ++t) {
            if (g_ct_cap[t] >= 0) {
                std::snprintf(key, sizeof key, "%s%d", KEY_TCAP_PREFIX, t);
                std::snprintf(buf, sizeof buf, "%d", g_ct_cap[t]);
                if (!storage::config_set(key, buf)) return false;
            }
            for (int m = 0; m < 4; ++m) {
                if (g_ct_mat[t][m] < 0) continue;
                std::snprintf(key, sizeof key, "%s%d.%d", KEY_TMAT_PREFIX, t, m);
                std::snprintf(buf, sizeof buf, "%d", g_ct_mat[t][m]);
                if (!storage::config_set(key, buf)) return false;
            }
        }
    }
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_tcap_pio ? 1 : 0));
    if (!storage::config_set(KEY_TCAP_PIO, buf)) return false;
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

bool i2c_bridge_enabled()            { return g_i2c_bridge_en; }
void set_i2c_bridge_enabled(bool v)  { g_i2c_bridge_en = v; }
int  i2c_bridge_instance()           { return g_i2c_bridge_inst; }
void set_i2c_bridge_instance(int inst){ g_i2c_bridge_inst = (inst == 1) ? 1 : 0; }
int  i2c_bridge_sda_pin()            { return g_i2c_bridge_sda; }
void set_i2c_bridge_sda_pin(int gpio){ g_i2c_bridge_sda = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
int  i2c_bridge_scl_pin()            { return g_i2c_bridge_scl; }
void set_i2c_bridge_scl_pin(int gpio){ g_i2c_bridge_scl = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
uint32_t i2c_bridge_hz()             { return g_i2c_bridge_hz; }
void set_i2c_bridge_hz(uint32_t hz)  { if (hz > 0 && hz <= 1'000'000) g_i2c_bridge_hz = hz; }

bool spi_bridge_enabled()            { return g_spi_bridge_en; }
void set_spi_bridge_enabled(bool v)  { g_spi_bridge_en = v; }
int  spi_bridge_instance()           { return g_spi_bridge_inst; }
void set_spi_bridge_instance(int inst){ g_spi_bridge_inst = (inst == 1) ? 1 : 0; }
int  spi_bridge_lpc()                { return g_spi_bridge_lpc; }
void set_spi_bridge_lpc(int idx)     { g_spi_bridge_lpc = (idx == 1) ? 1 : 0; }
int  spi_bridge_sck_pin()            { return g_spi_bridge_sck; }
void set_spi_bridge_sck_pin(int gpio){ g_spi_bridge_sck = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
int  spi_bridge_mosi_pin()           { return g_spi_bridge_mosi; }
void set_spi_bridge_mosi_pin(int gpio){ g_spi_bridge_mosi = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
int  spi_bridge_miso_pin()           { return g_spi_bridge_miso; }
void set_spi_bridge_miso_pin(int gpio){ g_spi_bridge_miso = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
uint32_t spi_bridge_hz()             { return g_spi_bridge_hz; }
void set_spi_bridge_hz(uint32_t hz)  { if (hz > 0 && hz <= 50'000'000) g_spi_bridge_hz = hz; }

bool adc_bridge_enabled()            { return g_adc_bridge_en; }
void set_adc_bridge_enabled(bool v)  { g_adc_bridge_en = v; }

int  ct_capture_pin(int t) {
    return (t >= 0 && t < 4) ? g_ct_cap[t] : -1;
}
void set_ct_capture_pin(int t, int gpio) {
    if (t < 0 || t >= 4) return;
    g_ct_cap[t] = (gpio >= 0 && gpio <= MAX_GPIO) ? static_cast<int8_t>(gpio) : -1;
}
int  ct_match_pin(int t, int m) {
    return (t >= 0 && t < 4 && m >= 0 && m < 4) ? g_ct_mat[t][m] : -1;
}
void set_ct_match_pin(int t, int m, int gpio) {
    if (t < 0 || t >= 4 || m < 0 || m >= 4) return;
    g_ct_mat[t][m] = (gpio >= 0 && gpio <= MAX_GPIO) ? static_cast<int8_t>(gpio) : -1;
}

bool tcap_pio()            { return g_tcap_pio; }
void set_tcap_pio(bool v)  { g_tcap_pio = v; }

bool wfi_pin_wakeup()                { return g_wfi_pin_wakeup; }
void set_wfi_pin_wakeup(bool v)      { g_wfi_pin_wakeup = v; }

bool primask_shadow()                { return g_primask_shadow; }
void set_primask_shadow(bool v)      { g_primask_shadow = v; }

} // namespace config
