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
bool     g_cli_enable    = true;
bool     g_gdb_enable    = true;
bool     g_serial_enable = true;
bool     g_uart_bridge_en = false;
int      g_uart_bridge_tx = -1;
int      g_uart_bridge_rx = -1;
bool     g_uart0_cdc      = false;
int      g_uart0_tx_gpio  = -1;
int      g_uart0_rx_gpio  = -1;
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
uint32_t g_app_start       = 0x3000;
uint32_t g_desc_addr       = 0;       // 0 = automatisch app_start - 0x100
bool     g_autodesc        = true;

// Timer-Capture-/Match-Pin-Bindung (4 Timer, je 1 Capture + 4 Match).
// -1 = nicht gebunden.
int8_t   g_ct_cap[4]    = { -1, -1, -1, -1 };
int8_t   g_ct_mat[4][4] = {
    { -1, -1, -1, -1 }, { -1, -1, -1, -1 },
    { -1, -1, -1, -1 }, { -1, -1, -1, -1 },
};
bool     g_tcap_pio = false;
bool     g_tmatch_pio = false;

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

// Wie parse_uint32, akzeptiert aber zusaetzlich Hex mit "0x"/"0X"-Praefix
// (Basis-Autodetektion). Fuer Adress-Konfiguration (app_start, desc_addr).
bool parse_uint32_auto(const char* s, uint32_t& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s, &end, 0);
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

    // GP0/GP1 werden BEWUSST NICHT als GPIO gemappt: sie sind die RP2350-uart0-
    // Pads. Wer den Gast-UART0 auf echte Pins legen will, nutzt uart0_tx/uart0_rx
    // (Config) -> das routet GP0/GP1 exklusiv auf GPIO_FUNC_UART. Eine doppelte
    // GPIO-Belegung (frueher P1_8→GP1, P1_9→GP0) liess das GPIO-Modell die
    // UART-Pads uebersteuern (GP0 als GPIO-low getrieben) -> uart0-RX las Dauer-
    // 0x00, serial.begin() haing in der RX-Drain-Schleife. Daher hier ungemappt.
    g_pin_map.lpc_to_rp[1 * 12 + 10] = 22;
    // P1_8/P1_9/P1_11, P2_x: ungemappt (GP26..29 für ADC reserviert, siehe oben).
}

void apply_defaults() {
    apply_default_pinmap_impl();
    g_target_freq = 48'000'000;
    g_autostart   = false;
    g_cli_enable    = true;
    g_gdb_enable    = true;
    g_serial_enable = true;
    g_uart_bridge_en = false;
    g_uart_bridge_tx = -1;
    g_uart_bridge_rx = -1;
    g_uart0_cdc      = false;
    g_uart0_tx_gpio  = -1;
    g_uart0_rx_gpio  = -1;
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
    g_app_start       = 0x3000;
    g_desc_addr       = 0;
    g_autodesc        = true;
    for (int t = 0; t < 4; ++t) {
        g_ct_cap[t] = -1;
        for (int m = 0; m < 4; ++m) g_ct_mat[t][m] = -1;
    }
    g_tcap_pio = false;
}

// Weist einem LPC-Pin einen RP2350-GPIO zu und erzwingt dabei Eindeutigkeit:
// ein physischer GPIO gehoert immer nur EINEM LPC-Pin. Ist der Ziel-GPIO schon
// einem anderen Pin (z. B. der Default-Belegung) zugeordnet, wird die alte
// Zuordnung geloest. verbose=true gibt das auf der CLI aus (Nutzer-Aktion),
// verbose=false bleibt still (Boot-Ladepfad).
void assign_pin_unique(uint8_t lpc_pin, int gpio, bool verbose) {
    if (gpio >= 0) {
        for (std::size_t i = 0; i < LPC_PIN_COUNT; ++i) {
            if (i != lpc_pin && g_pin_map.lpc_to_rp[i] == gpio) {
                if (verbose) {
                    std::printf("[CFG] GP%d: P%u_%u -> P%u_%u (Neuzuordnung, alte geloest)\n",
                                gpio,
                                static_cast<unsigned>(i / 12), static_cast<unsigned>(i % 12),
                                static_cast<unsigned>(lpc_pin / 12), static_cast<unsigned>(lpc_pin % 12));
                }
                g_pin_map.lpc_to_rp[i] = -1;
            }
        }
    }
    g_pin_map.lpc_to_rp[lpc_pin] = static_cast<int8_t>(gpio);
}

void load_pin_map_from_storage() {
    char buf[16];
    char key[24];
    for (std::size_t i = 0; i < LPC_PIN_COUNT; ++i) {
        // Schluesselformat konsistent mit CONFIG.INI + save(): pin.<port>_<pin>
        // (z. B. pin.1_8). Frueher wurde hier der reine Pin-INDEX genutzt
        // (pin.20), was NICHT zur INI-/build_config_ini-Nomenklatur passte und
        // 'cfg dump' unverstaendlich + inkonsistent zur INI machte.
        std::snprintf(key, sizeof key, "%s%u_%u", KEY_PIN_PREFIX,
                      static_cast<unsigned>(i / 12), static_cast<unsigned>(i % 12));
        if (!storage::config_get(key, buf, sizeof buf)) continue;
        uint32_t v;
        if (!parse_uint32(buf, v)) continue;
        if (v > static_cast<uint32_t>(MAX_GPIO)) continue;
        // Ueber die Eindeutigkeits-Zuweisung: eine persistierte Zuordnung
        // verdraengt einen kollidierenden Default (der von apply_defaults davor
        // gesetzt wurde), damit nach einem Reboot keine doppelte Belegung
        // wieder auftaucht. Still (kein CLI-Spam beim Boot).
        assign_pin_unique(static_cast<uint8_t>(i), static_cast<int>(v), /*verbose=*/false);
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
    // USB-CDC-Praesenz (Default on, wenn Key fehlt).
    if (storage::config_get(KEY_CLI_ENABLE, buf, sizeof buf))
        g_cli_enable = (buf[0] == '1');
    if (storage::config_get(KEY_GDB_ENABLE, buf, sizeof buf))
        g_gdb_enable = (buf[0] == '1');
    if (storage::config_get(KEY_SERIAL_ENABLE, buf, sizeof buf))
        g_serial_enable = (buf[0] == '1');
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
    if (storage::config_get(KEY_UART0_CDC, buf, sizeof buf)) {
        g_uart0_cdc = (buf[0] == '1');
    }
    if (storage::config_get(KEY_UART0_TX_GPIO, buf, sizeof buf)) {
        // -1 (kein Routing) explizit zulassen.
        int v = static_cast<int>(std::atol(buf));
        g_uart0_tx_gpio = (v >= -1 && v <= MAX_GPIO) ? v : -1;
    }
    if (storage::config_get(KEY_UART0_RX_GPIO, buf, sizeof buf)) {
        int v = static_cast<int>(std::atol(buf));
        g_uart0_rx_gpio = (v >= -1 && v <= MAX_GPIO) ? v : -1;
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
    if (storage::config_get(KEY_APP_START, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32_auto(buf, v) && v < 0x10000u) g_app_start = v;
    }
    if (storage::config_get(KEY_DESC_ADDR, buf, sizeof buf)) {
        uint32_t v;
        if (parse_uint32_auto(buf, v) && v < 0x10000u) g_desc_addr = v;
    }
    if (storage::config_get(KEY_AUTODESC, buf, sizeof buf)) {
        g_autodesc = (buf[0] == '1');
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
    if (storage::config_get(KEY_TMATCH_PIO, buf, sizeof buf)) {
        g_tmatch_pio = (buf[0] == '1');
    }
    load_pin_map_from_storage();
    return true;
}

bool save() {
    // Kompletter Neuaufbau des KV-Snapshots: erst leeren, dann NUR die aktuell
    // gueltigen Werte schreiben. Ohne das blieben veraltete Schluessel (z. B.
    // eine per assign_pin_unique geloeste Pinmap-Zuordnung oder ein entfernter
    // tmat-Eintrag) als Karteileichen im Flash -> 'cfg dump' zeigte Duplikate/
    // Werte, die weder dem Live-Zustand noch der CONFIG.INI entsprachen.
    storage::config_clear();
    char buf[24];
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_autostart ? 1 : 0));
    if (!storage::config_set(KEY_AUTOSTART, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_cli_enable ? 1 : 0));
    if (!storage::config_set(KEY_CLI_ENABLE, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_gdb_enable ? 1 : 0));
    if (!storage::config_set(KEY_GDB_ENABLE, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_serial_enable ? 1 : 0));
    if (!storage::config_set(KEY_SERIAL_ENABLE, buf)) return false;
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
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_uart0_cdc ? 1 : 0));
    if (!storage::config_set(KEY_UART0_CDC, buf)) return false;
    if (g_uart0_tx_gpio >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_uart0_tx_gpio);
        if (!storage::config_set(KEY_UART0_TX_GPIO, buf)) return false;
    }
    if (g_uart0_rx_gpio >= 0) {
        std::snprintf(buf, sizeof buf, "%d", g_uart0_rx_gpio);
        if (!storage::config_set(KEY_UART0_RX_GPIO, buf)) return false;
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
    std::snprintf(buf, sizeof buf, "0x%lx", static_cast<unsigned long>(g_app_start));
    if (!storage::config_set(KEY_APP_START, buf)) return false;
    std::snprintf(buf, sizeof buf, "0x%lx", static_cast<unsigned long>(g_desc_addr));
    if (!storage::config_set(KEY_DESC_ADDR, buf)) return false;
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_autodesc ? 1 : 0));
    if (!storage::config_set(KEY_AUTODESC, buf)) return false;
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
    std::snprintf(buf, sizeof buf, "%u", static_cast<unsigned>(g_tmatch_pio ? 1 : 0));
    if (!storage::config_set(KEY_TMATCH_PIO, buf)) return false;
    char key[24];
    for (std::size_t i = 0; i < LPC_PIN_COUNT; ++i) {
        if (g_pin_map.lpc_to_rp[i] < 0) continue;
        // Konsistent mit CONFIG.INI: pin.<port>_<pin> (z. B. pin.1_8=1).
        std::snprintf(key, sizeof key, "%s%u_%u", KEY_PIN_PREFIX,
                      static_cast<unsigned>(i / 12), static_cast<unsigned>(i % 12));
        std::snprintf(buf, sizeof buf, "%d", g_pin_map.lpc_to_rp[i]);
        if (!storage::config_set(key, buf)) return false;
    }
    return storage::config_commit();
}

// Setzt NUR die "listenartigen" Pin-Zuordnungen (Pinmap + Timer-Capture/Match)
// auf ihren Default zurueck. Wird von parse_config (usb_msc) VOR dem Einlesen
// einer CONFIG.INI aufgerufen, damit die INI fuer diese Familie AUTORITATIV ist:
// ein entfernter/auskommentierter Eintrag (z. B. tmat.2.1) verschwindet dann
// wirklich, statt aus dem alten Zustand "haengen zu bleiben". Skalare Keys
// (autostart/freq/Bridges) sind eindeutig und werden weiterhin gemerged.
void reset_pin_mappings() {
    apply_default_pinmap_impl();
    for (int t = 0; t < 4; ++t) {
        g_ct_cap[t] = -1;
        for (int m = 0; m < 4; ++m) g_ct_mat[t][m] = -1;
    }
}

bool        autostart()                    { return g_autostart; }
void        set_autostart(bool v)          { g_autostart = v; }
bool        cli_enabled()                  { return g_cli_enable; }
void        set_cli_enabled(bool v)        { g_cli_enable = v; }
bool        gdb_enabled()                  { return g_gdb_enable; }
void        set_gdb_enabled(bool v)        { g_gdb_enable = v; }
bool        serial_cdc_enabled()           { return g_serial_enable; }
void        set_serial_cdc_enabled(bool v) { g_serial_enable = v; }
uint32_t    target_frequency_hz()          { return g_target_freq; }

void set_target_frequency_hz(uint32_t hz) {
    if (hz == 0 || hz > MAX_FREQ_HZ) return;
    g_target_freq = hz;
}

const PinMap& pin_map() { return g_pin_map; }

bool set_pin_map(uint8_t lpc_pin, int rp2350_gpio) {
    if (lpc_pin >= LPC_PIN_COUNT) return false;
    if (rp2350_gpio < -1 || rp2350_gpio > MAX_GPIO) return false;
    // Eindeutigkeit erzwingen (siehe assign_pin_unique): ein physischer GPIO
    // kann nur EINEM LPC-Pin gehoeren. Eine kollidierende alte Zuordnung (z. B.
    // die Default-Belegung P1_3->GP17) wird geloest, damit die neue tatsaechlich
    // greift und `pinmap show` keine doppelte, mehrdeutige Belegung mehr zeigt.
    assign_pin_unique(lpc_pin, rp2350_gpio, /*verbose=*/true);
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
bool uart0_cdc_enabled()             { return g_uart0_cdc; }
void set_uart0_cdc_enabled(bool v)   { g_uart0_cdc = v; }
int  uart0_tx_gpio()                 { return g_uart0_tx_gpio; }
void set_uart0_tx_gpio(int gpio)     { g_uart0_tx_gpio = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }
int  uart0_rx_gpio()                 { return g_uart0_rx_gpio; }
void set_uart0_rx_gpio(int gpio)     { g_uart0_rx_gpio = (gpio >= -1 && gpio <= MAX_GPIO) ? gpio : -1; }

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
bool tmatch_pio()            { return g_tmatch_pio; }
void set_tmatch_pio(bool v)  { g_tmatch_pio = v; }

bool wfi_pin_wakeup()                { return g_wfi_pin_wakeup; }
void set_wfi_pin_wakeup(bool v)      { g_wfi_pin_wakeup = v; }

bool primask_shadow()                { return g_primask_shadow; }
void set_primask_shadow(bool v)      { g_primask_shadow = v; }

uint32_t app_start_addr()            { return g_app_start; }
void set_app_start_addr(uint32_t a)  { if (a < 0x10000u) g_app_start = a; }
uint32_t descriptor_addr() {
    if (g_desc_addr != 0) return g_desc_addr;
    // Automatik: eine Flash-Page (256 B) vor der Applikation.
    return (g_app_start >= 0x100u) ? (g_app_start - 0x100u) : 0u;
}
void set_descriptor_addr(uint32_t a) { if (a < 0x10000u) g_desc_addr = a; }
bool autodesc()                      { return g_autodesc; }
void set_autodesc(bool v)            { g_autodesc = v; }

} // namespace config
