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
// USB-Schnittstellen: einzelne CDC-Ports am USB komplett ein/ausblenden. Bei
// "off" verschwindet das zugehoerige CDC-Interface aus dem USB-Deskriptor (der
// Host sieht dann einen COM-Port weniger). MSC (Laufwerk) bleibt IMMER da, damit
// CONFIG.INI zur Wiederherstellung erreichbar ist. Default alle on.
inline constexpr const char* KEY_CLI_ENABLE      = "cli_enable";       // "0"/"1" CDC-CLI
inline constexpr const char* KEY_GDB_ENABLE      = "gdb_enable";       // "0"/"1" CDC-GDB
inline constexpr const char* KEY_SERIAL_ENABLE   = "serial_enable";    // "0"/"1" CDC-Serial(#2)
inline constexpr const char* KEY_PIN_PREFIX      = "pin.";             // pin.<lpc>=<rp2350>
inline constexpr const char* KEY_UART_BRIDGE_EN  = "uart_bridge_en";   // "0"/"1"
inline constexpr const char* KEY_UART_BRIDGE_TX  = "uart_bridge_tx";   // GPIO-Nummer
inline constexpr const char* KEY_UART_BRIDGE_RX  = "uart_bridge_rx";   // GPIO-Nummer
// LPC-UART0 virtuell direkt an USB CDC#2 koppeln (statt HW-uart0-Pins). "0"/"1".
// Wenn aktiv, gehen Gast-UART0-TX-Bytes ohne Draht/Pin an CDC#2 und CDC#2-
// Eingaben an den Gast-UART0-RX. Schliesst die PIO-UART-Bridge auf CDC#2 aus.
inline constexpr const char* KEY_UART0_CDC       = "uart0_cdc";        // "0"/"1"
// LPC-UART0 auf echte RP2350-UART-Pads routen (Hardwareentwurf). GPIO-Nummer,
// -1 = kein Routing. TX/RX muessen zum SELBEN RP-Peripheral (uart0 ODER uart1)
// gehoeren: uart0 TX GP0/12/16 RX GP1/13/17; uart1 TX GP4/8/20/24 RX GP5/9/21/25.
inline constexpr const char* KEY_UART0_TX_GPIO   = "uart0_tx";         // RP-GPIO oder -1
inline constexpr const char* KEY_UART0_RX_GPIO   = "uart0_rx";         // RP-GPIO oder -1
inline constexpr const char* KEY_I2C_BRIDGE_EN   = "i2c_bridge_en";    // "0"/"1"
inline constexpr const char* KEY_I2C_BRIDGE_INST = "i2c_bridge_inst";  // 0=i2c0, 1=i2c1
inline constexpr const char* KEY_I2C_BRIDGE_SDA  = "i2c_bridge_sda";   // GPIO-Nummer
inline constexpr const char* KEY_I2C_BRIDGE_SCL  = "i2c_bridge_scl";   // GPIO-Nummer
inline constexpr const char* KEY_I2C_BRIDGE_HZ   = "i2c_bridge_hz";    // Bus-Takt in Hz
inline constexpr const char* KEY_SPI_BRIDGE_EN   = "spi_bridge_en";    // "0"/"1"
inline constexpr const char* KEY_SPI_BRIDGE_INST = "spi_bridge_inst";  // 0=spi0, 1=spi1
inline constexpr const char* KEY_SPI_BRIDGE_LPC  = "spi_bridge_lpc";   // 0=SSP0, 1=SSP1
inline constexpr const char* KEY_SPI_BRIDGE_SCK  = "spi_bridge_sck";   // GPIO-Nummer
inline constexpr const char* KEY_SPI_BRIDGE_MOSI = "spi_bridge_mosi";  // GPIO-Nummer
inline constexpr const char* KEY_SPI_BRIDGE_MISO = "spi_bridge_miso";  // GPIO-Nummer
inline constexpr const char* KEY_SPI_BRIDGE_HZ   = "spi_bridge_hz";    // Bus-Takt in Hz
inline constexpr const char* KEY_ADC_BRIDGE_EN   = "adc_bridge_en";    // "0"/"1"
// Timer-Capture-/Match-Pin-Bridges (für KNX-Bus-Empfang/-Senden via CT16/CT32).
//   tcap.<t>=<gpio>        Capture-Eingang CAP0 von Timer t (0..3)
//   tmat.<t>.<m>=<gpio>    Match-Ausgang MATm von Timer t (m=0..3)
// t: 0=CT16B0, 1=CT16B1, 2=CT32B0, 3=CT32B1.
inline constexpr const char* KEY_TCAP_PREFIX     = "tcap.";
inline constexpr const char* KEY_TMAT_PREFIX     = "tmat.";
inline constexpr const char* KEY_TCAP_PIO        = "tcap_pio";        // "0"/"1" (opt-in)
inline constexpr const char* KEY_TMATCH_PIO      = "tmatch_pio";      // "0"/"1" (opt-in, Match/PWM)
inline constexpr const char* KEY_WFI_PIN_WAKEUP  = "wfi_pin_wakeup";   // "0"/"1" (opt-in)
inline constexpr const char* KEY_PRIMASK_SHADOW  = "primask_shadow";   // "0"/"1" (opt-in)
// Bootloader->Applikation: Erkennungs-/Descriptor-Adressen (hex oder dezimal).
inline constexpr const char* KEY_APP_START       = "app_start";        // Flash-Adresse der Applikation (Default 0x3000)
inline constexpr const char* KEY_DESC_ADDR       = "desc_addr";        // Adresse des Boot-Descriptors (Default app_start-0x100)
inline constexpr const char* KEY_AUTODESC        = "autodesc";         // "0"/"1": Descriptor automatisch erzeugen (Default 1)

// Pin-Mapping LPC1115 -> RP2350 GPIO. -1 = nicht zugeordnet.
constexpr std::size_t LPC_PIN_COUNT = 64;
struct PinMap {
    int8_t lpc_to_rp[LPC_PIN_COUNT];
};

bool load();                         // ruft storage::config_load + bedient Defaults
bool save();

bool        autostart();
void        set_autostart(bool v);

// USB-CDC-Praesenz (wirkt beim Boot: baut den USB-Deskriptor entsprechend).
bool        cli_enabled();
void        set_cli_enabled(bool v);
bool        gdb_enabled();
void        set_gdb_enabled(bool v);
bool        serial_cdc_enabled();
void        set_serial_cdc_enabled(bool v);

uint32_t    target_frequency_hz();
void        set_target_frequency_hz(uint32_t hz);

const PinMap& pin_map();
// verbose=true (CLI 'pinmap set'): meldet eine geloeste Kollision auf der Konsole.
// verbose=false (autoritatives CONFIG.INI-Batch-Parsen): still, da intra-Batch-
// GPIO-Kollisionen deterministisch (last-wins) aufgeloest werden und keine
// Nutzer-Aktion sind (sonst Spam beim Neueinlesen einer generierten INI).
bool         set_pin_map(uint8_t lpc_pin, int rp2350_gpio, bool verbose = true);
void         apply_default_pinmap();

// Setzt Pinmap + Timer-Capture/Match auf Default zurueck (fuer autoritatives
// CONFIG.INI-Parsen: entfernte Eintraege verschwinden statt haengenzubleiben).
void         reset_pin_mappings();

// UART-Bridge-Konfiguration
bool        uart_bridge_enabled();
void        set_uart_bridge_enabled(bool v);
int         uart_bridge_tx_pin();
void        set_uart_bridge_tx_pin(int gpio);
int         uart_bridge_rx_pin();
void        set_uart_bridge_rx_pin(int gpio);

// LPC-UART0 <-> USB CDC#2 virtuelle Kopplung (statt HW-uart0-Pins).
bool        uart0_cdc_enabled();
void        set_uart0_cdc_enabled(bool v);

// LPC-UART0 auf echte RP2350-UART-Pads (uart0/uart1). -1 = kein HW-Routing.
int         uart0_tx_gpio();
void        set_uart0_tx_gpio(int gpio);
int         uart0_rx_gpio();
void        set_uart0_rx_gpio(int gpio);

// I²C-Bridge-Konfiguration (LPC-I²C-Master → RP2350-Hardware-I²C)
bool        i2c_bridge_enabled();
void        set_i2c_bridge_enabled(bool v);
int         i2c_bridge_instance();         // 0=i2c0, 1=i2c1
void        set_i2c_bridge_instance(int inst);
int         i2c_bridge_sda_pin();
void        set_i2c_bridge_sda_pin(int gpio);
int         i2c_bridge_scl_pin();
void        set_i2c_bridge_scl_pin(int gpio);
uint32_t    i2c_bridge_hz();
void        set_i2c_bridge_hz(uint32_t hz);

// SPI-Bridge-Konfiguration (LPC-SSP-Master → RP2350-Hardware-SPI)
bool        spi_bridge_enabled();
void        set_spi_bridge_enabled(bool v);
int         spi_bridge_instance();         // 0=spi0, 1=spi1
void        set_spi_bridge_instance(int inst);
int         spi_bridge_lpc();              // 0=SSP0, 1=SSP1 (welcher LPC-SSP)
void        set_spi_bridge_lpc(int idx);
int         spi_bridge_sck_pin();
void        set_spi_bridge_sck_pin(int gpio);
int         spi_bridge_mosi_pin();
void        set_spi_bridge_mosi_pin(int gpio);
int         spi_bridge_miso_pin();
void        set_spi_bridge_miso_pin(int gpio);
uint32_t    spi_bridge_hz();
void        set_spi_bridge_hz(uint32_t hz);

// ADC-Bridge-Konfiguration (LPC-ADC-Kanäle → RP2350-Hardware-ADC).
// Mapping ist fix durch die RP2350-Hardware vorgegeben: LPC-Kanal 0..3 →
// RP2350-ADC-Eingang 0..3 (GPIO26..29). Kanäle 4..7 haben keinen RP2350-
// ADC-Pin und liefern weiterhin den Mittenwert. Nur Enable konfigurierbar.
bool        adc_bridge_enabled();
void        set_adc_bridge_enabled(bool v);

// Timer-Capture-/Match-Pin-Bridge (CT16B0/1, CT32B0/1 → echte RP2350-GPIOs).
// Wird primär für den Selfbus-KNX-Buszugriff benötigt: Capture-Eingang =
// Bus-Empfang (Flanken-Timestamps), Match-Ausgänge = Bus-Senden.
//   t = Timer-Index 0..3 (0=CT16B0, 1=CT16B1, 2=CT32B0, 3=CT32B1)
//   m = Match-Kanal 0..3
// Rückgabe < 0 = kein Pin gebunden.
int         ct_capture_pin(int t);
void        set_ct_capture_pin(int t, int gpio);
int         ct_match_pin(int t, int m);
void        set_ct_match_pin(int t, int m, int gpio);

// Opt-in: Timer-Capture per PIO statt Software-Polling. Eine PIO-State-
// Machine erfasst Flanken-Timestamps am Capture-Pin flankengenau und ohne
// Core-Last; die CPU rechnet die Zaehlerdifferenzen in TC-Ticks um. Default
// aus (Software-Capture als Fallback).
bool        tcap_pio();
void        set_tcap_pio(bool v);
// Opt-in: Match-/PWM-Ausgangspuls per PIO statt Software-Bit-Bang. Bei
// aktivem PWM-Match erzeugt eine PIO-State-Machine die Ausgangspulse
// (steigende Flanke an MRm, Pulsbreite bis Timer-Reset) hardware-getaktet,
// frei vom Poll-Jitter. Default aus (Software-PWM als generischer Fallback).
// Nutzbar fuer praezise PWM-/Trigger-/Bit-Timing-Ausgaben (z. B. KNX-Senden).
bool        tmatch_pio();
void        set_tmatch_pio(bool v);
// WFI-Pin-Wakeup (opt-in, Default aus): patcht WFI der Gast-Firmware auf
// einen SVC-Trap, sodass eine reine WFI-Warteschleife durch echte
// RP2350-Pin-Flanken (und zeitbasierte Modelle) geweckt werden kann.
bool        wfi_pin_wakeup();
void        set_wfi_pin_wakeup(bool v);

// PRIMASK-Schatten (opt-in, Default aus): patcht CPSID i / CPSIE i der
// Gast-Firmware auf SVC-Traps und fuehrt einen Schatten-PRIMASK nach. Da der
// Gast unprivilegiert laeuft, ignoriert die Hardware diese Instruktionen sonst
// (Kritische Sektionen via __disable_irq() blieben wirkungslos). Solange der
// Schatten gesetzt ist, haelt die IRQ-Injektion neue IRQs pending zurueck.
bool        primask_shadow();
void        set_primask_shadow(bool v);

// Bootloader->Applikation-Uebergang: Flash-Adresse, an der die Applikation
// (eigene Vektortabelle) erwartet wird, sowie die Adresse, an der der Selfbus-
// Boot-Descriptor liegt/erzeugt wird. Beide frei konfigurierbar, damit
// verschiedene Bootloader-Groessen/-Layouts unterstuetzt werden.
uint32_t    app_start_addr();
void        set_app_start_addr(uint32_t addr);
uint32_t    descriptor_addr();          // 0 = automatisch app_start - 0x100
void        set_descriptor_addr(uint32_t addr);
// Auto-Descriptor (Default an): erzeugt beim Laden einen gueltigen Boot-
// Descriptor, falls eine Applikation an app_start liegt und noch keiner da ist.
bool        autodesc();
void        set_autodesc(bool v);

} // namespace config
