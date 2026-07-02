#include "peripherals.h"
#include "config.h"
#include "vnvic.h"
#include "irq_inject.h"
#include "lpc_irqs.h"
#include "emulator.h"
#include "pio_glue.h"

#include <cstring>
#include <cstdio>

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/powman.h"
#include "hardware/structs/powman.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/scb.h"
#include "hardware/regs/powman.h"
#include "pico/stdlib.h"

// Pico-SDK 2.x definiert in addressmap.h Makros wie WDT_BASE, ADC_BASE,
// I2C_BASE, … für RP2350. Wir modellieren hier LPC1115-Adressen und
// brauchen die RP2350-Adressen nicht — Makros wegnehmen.
#undef  WDT_BASE
#undef  ADC_BASE
#undef  I2C_BASE
#undef  PMU_BASE
#undef  SSP0_BASE
#undef  SSP1_BASE
#undef  UART0_BASE
#undef  UART_BASE
#undef  IOCON_BASE
#undef  GPIO_BASE
#undef  SYSCON_BASE

// Forward-Declaration für die WDT-Bridge nach emulator::request_guest_reset().
extern "C" void peripherals_wdt_reset_guest();

// LPC1115 Peripheral-Adressen (Auswahl).
// SYSCON Block bei 0x40048000.
namespace {

// SYSCON
constexpr uint32_t SYSCON_BASE        = 0x4004'8000;
constexpr uint32_t SYSMEMREMAP        = SYSCON_BASE + 0x000;
constexpr uint32_t SYSPLLCTRL         = SYSCON_BASE + 0x008;
constexpr uint32_t SYSPLLSTAT         = SYSCON_BASE + 0x00C;
constexpr uint32_t SYSOSCCTRL         = SYSCON_BASE + 0x020;
constexpr uint32_t WDTOSCCTRL         = SYSCON_BASE + 0x024;
constexpr uint32_t IRCCTRL            = SYSCON_BASE + 0x028;
constexpr uint32_t SYSPLLCLKSEL       = SYSCON_BASE + 0x040;
constexpr uint32_t SYSPLLCLKUEN       = SYSCON_BASE + 0x044;
// BODCTRL (UM10398 Kap. 3.5.34): BODRSTLEV[1:0], BODINTVAL[3:2], BODRSTENA[4].
constexpr uint32_t BODCTRL            = SYSCON_BASE + 0x048;
constexpr uint32_t BODCTRL_BODRSTENA  = 1u << 4;
constexpr uint32_t MAINCLKSEL         = SYSCON_BASE + 0x070;
constexpr uint32_t MAINCLKUEN         = SYSCON_BASE + 0x074;
constexpr uint32_t SYSAHBCLKDIV       = SYSCON_BASE + 0x078;
constexpr uint32_t SYSAHBCLKCTRL      = SYSCON_BASE + 0x080;
constexpr uint32_t PDRUNCFG           = SYSCON_BASE + 0x238;

// IOCON Block bei 0x40044000 — Pinmux. Modellieren wir als RAM (kein Effekt).
constexpr uint32_t IOCON_BASE         = 0x4004'4000;
constexpr uint32_t IOCON_END          = 0x4004'4100;

// PINTSEL0..7 (Kanal→Pin-Auswahl der Pin-Interrupts) im SYSCON-Block.
// LPC11Exx-Map: 0x40048178..0x40048194 (8 Register).
constexpr uint32_t PINTSEL0           = SYSCON_BASE + 0x178;
constexpr uint32_t PINTSEL_END        = SYSCON_BASE + 0x198;

// GPIO0..GPIO3 @ 0x50000000 + N*0x10000.
// Pro Port:  0x0000-0x3FFC = maskierter DATA-Zugriff (Adress-Bits[11:2] = pin-mask)
//            0x8000 = DIR, 0x8004..0x801C = IRQ-Register (vereinfacht).
constexpr uint32_t GPIO_BASE          = 0x5000'0000;
constexpr uint32_t GPIO_PORT_STRIDE   = 0x0001'0000;
constexpr uint32_t GPIO_PORTS_END     = 0x5004'0000;
constexpr uint32_t GPIO_DATA_END      = 0x4000;   // 0x0000..0x3FFC = masked DATA
constexpr uint32_t GPIO_DIR_OFFSET    = 0x8000;

// SysTick im PPB
constexpr uint32_t SYSTICK_CTRL = 0xE000'E010;
constexpr uint32_t SYSTICK_LOAD = 0xE000'E014;
constexpr uint32_t SYSTICK_VAL  = 0xE000'E018;

// LPC1115 UART0 @ 0x40008000 (16550-kompatibel)
constexpr uint32_t UART0_BASE   = 0x4000'8000;
constexpr uint32_t UART0_END    = 0x4000'8100;
constexpr uint32_t UART0_RBR    = UART0_BASE + 0x000;   // R: rx data
constexpr uint32_t UART0_THR    = UART0_BASE + 0x000;   // W: tx data
constexpr uint32_t UART0_DLL    = UART0_BASE + 0x000;   // mit LCR.DLAB
constexpr uint32_t UART0_DLM    = UART0_BASE + 0x004;   // mit LCR.DLAB
constexpr uint32_t UART0_IER    = UART0_BASE + 0x004;
constexpr uint32_t UART0_IIR    = UART0_BASE + 0x008;
constexpr uint32_t UART0_FCR    = UART0_BASE + 0x008;
constexpr uint32_t UART0_LCR    = UART0_BASE + 0x00C;
constexpr uint32_t UART0_MCR    = UART0_BASE + 0x010;
constexpr uint32_t UART0_LSR    = UART0_BASE + 0x014;

// CT16B0/CT16B1 @ 0x4000C000 / 0x40010000
// CT32B0/CT32B1 @ 0x40014000 / 0x40018000
constexpr uint32_t CT16B0_BASE  = 0x4000'C000;
constexpr uint32_t CT16B1_BASE  = 0x4001'0000;
constexpr uint32_t CT32B0_BASE  = 0x4001'4000;
constexpr uint32_t CT32B1_BASE  = 0x4001'8000;
constexpr uint32_t CT_BLOCK_SIZE = 0x4000;

// LPC1115 Watchdog (WWDT) @ 0x40004000.
constexpr uint32_t WDT_BASE     = 0x4000'4000;
constexpr uint32_t WDT_MOD      = WDT_BASE + 0x000;
constexpr uint32_t WDT_TC       = WDT_BASE + 0x004;
constexpr uint32_t WDT_FEED     = WDT_BASE + 0x008;
constexpr uint32_t WDT_TV       = WDT_BASE + 0x00C;
constexpr uint32_t WDT_END      = WDT_BASE + 0x100;

// LPC1115 ADC @ 0x4001C000.
constexpr uint32_t ADC_BASE     = 0x4001'C000;
constexpr uint32_t ADC_CR       = ADC_BASE + 0x000;
constexpr uint32_t ADC_GDR      = ADC_BASE + 0x004;
constexpr uint32_t ADC_INTEN    = ADC_BASE + 0x00C;
constexpr uint32_t ADC_DR0      = ADC_BASE + 0x010;
constexpr uint32_t ADC_STAT     = ADC_BASE + 0x030;
constexpr uint32_t ADC_END      = ADC_BASE + 0x100;

// LPC1115 SSP0/SSP1 @ 0x40040000 / 0x40058000.
constexpr uint32_t SSP0_BASE    = 0x4004'0000;
constexpr uint32_t SSP1_BASE    = 0x4005'8000;
constexpr uint32_t SSP_BLOCK    = 0x4000;
constexpr uint32_t SSP_CR0      = 0x000;
constexpr uint32_t SSP_CR1      = 0x004;
constexpr uint32_t SSP_DR       = 0x008;
constexpr uint32_t SSP_SR       = 0x00C;
constexpr uint32_t SSP_CPSR     = 0x010;
constexpr uint32_t SSP_IMSC     = 0x014;
constexpr uint32_t SSP_RIS      = 0x018;
constexpr uint32_t SSP_MIS      = 0x01C;
constexpr uint32_t SSP_ICR      = 0x020;

// LPC1115 I²C0 @ 0x40000000.
constexpr uint32_t I2C_BASE     = 0x4000'0000;
constexpr uint32_t I2C_CONSET   = I2C_BASE + 0x000;
constexpr uint32_t I2C_STAT     = I2C_BASE + 0x004;
constexpr uint32_t I2C_DAT      = I2C_BASE + 0x008;
constexpr uint32_t I2C_ADR0     = I2C_BASE + 0x00C;
constexpr uint32_t I2C_SCLH     = I2C_BASE + 0x010;
constexpr uint32_t I2C_SCLL     = I2C_BASE + 0x014;
constexpr uint32_t I2C_CONCLR   = I2C_BASE + 0x018;
constexpr uint32_t I2C_END      = I2C_BASE + 0x100;

// PMU @ 0x40038000 (General Purpose Reg 0..3 + PCON).
constexpr uint32_t PMU_BASE     = 0x4003'8000;
constexpr uint32_t PMU_END      = PMU_BASE + 0x100;

struct GpioPort {
    uint32_t dir;
    uint32_t data;
};

GpioPort        g_gpio[4]{};

// LPC-Pin-Index in der Pinmap = port*12 + pin (LPC1115 hat max. 12 Pins/Port).
constexpr uint8_t lpc_pin_idx(uint8_t port, uint8_t pin) {
    return static_cast<uint8_t>(port * 12u + pin);
}

void apply_gpio_to_hw(uint8_t lpc_pin, bool out, bool level);

// Prüft, ob ein RP2350-GPIO exklusiv von einer Hardware-Bridge belegt ist
// (ADC, SPI, Timer-Capture/Match). Solche Pins darf das GPIO-Modell nicht
// als digitalen Aus-/Eingang übersteuern, sonst kollidiert es mit der Bridge.
bool bridge_owns_gpio(int g) {
    if (g < 0) return false;
    // ADC: feste RP2350A-ADC-Pins GP26..29.
    if (config::adc_bridge_enabled() && g >= 26 && g <= 29) return true;
    // SPI: SCK/MOSI/MISO.
    if (config::spi_bridge_enabled()) {
        if (g == config::spi_bridge_sck_pin() ||
            g == config::spi_bridge_mosi_pin() ||
            g == config::spi_bridge_miso_pin()) return true;
    }
    // Timer-Capture-/Match-Pins.
    for (int t = 0; t < 4; ++t) {
        if (g == config::ct_capture_pin(t)) return true;
        for (int m = 0; m < 4; ++m)
            if (g == config::ct_match_pin(t, m)) return true;
    }
    return false;
}

void gpio_apply_port(uint8_t port, uint32_t /*old_data*/, uint32_t new_data, uint32_t dir) {
    for (uint8_t pin = 0; pin < 12; ++pin) {
        bool out = (dir >> pin) & 1u;
        bool lvl = (new_data >> pin) & 1u;
        apply_gpio_to_hw(lpc_pin_idx(port, pin), out, lvl);
    }
}
uint32_t        g_systick_load = 0, g_systick_val = 0, g_systick_ctrl = 0;

// =========================================================================
// SysTick-Emulation (0xE000E010-0xE000E01F). Notwendig, weil SysTick auf dem
// Cortex-M33 privilegiert-only ist: der unprivilegierte Gast koennte es nativ
// weder konfigurieren noch korrekt lesen. Die MPU (mmu.cpp) laesst den Block
// 0xE000E000-0xE000E01F trappen; hier wird SysTick zeitbasiert nachgebildet und
// die SysTick-Exception (Slot 15) via irq_inject::pend_systick() injiziert.
//
// Zeitbasis: reale Wall-Clock (time_us_64) skaliert mit g_current_hz (der vom
// Gast programmierten LPC-Soll-Frequenz), analog ct_advance/wdt_advance. Damit
// tickt SysTick mit der von der Gast-Firmware erwarteten Rate (z. B. 1 ms bei
// RVR = f/1000), unabhaengig vom realen 150-MHz-RP2350-Takt.
extern uint32_t g_current_hz;   // weiter unten definiert (SYSCON-Takt-Schatten)
struct SysTick {
    uint32_t csr;      // [0]=ENABLE [1]=TICKINT [2]=CLKSOURCE [16]=COUNTFLAG
    uint32_t rvr;      // 24-bit Reload
    uint32_t cvr;      // 24-bit Current
    uint64_t last_us;  // Zeitpunkt des letzten advance
    double   frac;     // Rest-Tick-Bruchteil
    // Diagnose:
    uint32_t trap_reads;   // Anzahl getrappter SysTick-Reads
    uint32_t trap_writes;  // Anzahl getrappter SysTick-Writes
    uint64_t irq_ticks;    // Anzahl injizierter SysTick-Exceptions
};
SysTick g_systick{};

constexpr uint32_t SYST_CSR_ENABLE    = 1u << 0;
constexpr uint32_t SYST_CSR_TICKINT   = 1u << 1;
constexpr uint32_t SYST_CSR_COUNTFLAG = 1u << 16;

// Treibt den SysTick-Zaehler entsprechend der verstrichenen Zeit weiter und
// injiziert bei jedem Reload-Unterlauf (CVR laeuft durch 0) die SysTick-
// Exception, falls TICKINT gesetzt ist. Wird aus poll_timed_sources() und aus
// dem SysTick-MMIO-Trap aufgerufen.
void systick_advance() {
    uint64_t now = time_us_64();
    if (!(g_systick.csr & SYST_CSR_ENABLE) || g_systick.rvr == 0) {
        g_systick.last_us = now;
        return;
    }
    uint64_t dt = now - g_systick.last_us;
    g_systick.last_us = now;
    uint32_t hz = g_current_hz ? g_current_hz : 12'000'000u;
    // Verstrichene SysTick-Ticks (Prozessortakt) = dt[us] * hz / 1e6.
    double ticks_f = g_systick.frac +
        (static_cast<double>(dt) * static_cast<double>(hz)) / 1'000'000.0;
    uint64_t ticks = static_cast<uint64_t>(ticks_f);
    g_systick.frac = ticks_f - static_cast<double>(ticks);
    if (ticks == 0) return;

    uint32_t period = (g_systick.rvr & 0xFF'FFFFu) + 1u;   // RELOAD+1 Ticks/Zyklus
    uint32_t cur = g_systick.cvr & 0xFF'FFFFu;
    bool underflow = false;
    // cur zaehlt abwaerts; jeder Durchlauf durch 0 = ein SysTick-Ereignis.
    uint64_t rem = ticks;
    if (rem > cur) {
        underflow = true;
        rem -= (cur + 1u);                 // bis inkl. Nulldurchgang
        rem %= period;                     // volle Zyklen ueberspringen
        cur = period - 1u - static_cast<uint32_t>(rem);
    } else {
        cur -= static_cast<uint32_t>(rem);
    }
    g_systick.cvr = cur & 0xFF'FFFFu;
    if (underflow) {
        g_systick.csr |= SYST_CSR_COUNTFLAG;
        if (g_systick.csr & SYST_CSR_TICKINT) {
            ++g_systick.irq_ticks;
            irq_inject::pend_systick();
        }
    }
}

uint32_t systick_read32(uint32_t addr) {
    ++g_systick.trap_reads;
    switch (addr & 0xFFu) {
        case 0x10:                          // SYST_CSR
        {
            systick_advance();
            uint32_t v = g_systick.csr;
            g_systick.csr &= ~SYST_CSR_COUNTFLAG;   // COUNTFLAG liest sich clear
            return v;
        }
        case 0x14: return g_systick.rvr;    // SYST_RVR
        case 0x18: systick_advance(); return g_systick.cvr;   // SYST_CVR
        case 0x1C: return 0x4000'0000u;     // SYST_CALIB: NOREF=0, kein exakter 10ms
        default:   return 0;                // ICTR/ACTLR-Bereich (0x00..0x0C)
    }
}

void systick_write32(uint32_t addr, uint32_t value) {
    ++g_systick.trap_writes;
    switch (addr & 0xFFu) {
        case 0x10:                          // SYST_CSR
            systick_advance();
            g_systick.csr = value & 0x7u;   // ENABLE|TICKINT|CLKSOURCE
            g_systick.last_us = time_us_64();
            g_systick.frac = 0.0;
            break;
        case 0x14: g_systick.rvr = value & 0xFF'FFFFu; break;  // SYST_RVR
        case 0x18:                          // SYST_CVR: jeder Write -> 0, COUNTFLAG clear
            g_systick.cvr = 0;
            g_systick.csr &= ~SYST_CSR_COUNTFLAG;
            g_systick.frac = 0.0;
            break;
        default: break;                     // ICTR (RO) / ACTLR: ignorieren
    }
}

// Byteweiser SysTick-Write-Sammler: buendelt die 4 Bytes eines STR zu einem
// 32-bit-Wort und ruft systick_write32 beim letzten Byte des Registers. Nutzt
// den aktuellen Registerwert als Default fuer nicht (mit)geschriebene Bytes.
struct SysTickCollector { uint32_t aligned; uint8_t bytes[4]; uint8_t mask; };
SysTickCollector g_systick_collector{0, {0,0,0,0}, 0};

void systick_collect_byte(uint32_t addr, uint8_t val) {
    uint32_t aligned = addr & ~3u;
    if (g_systick_collector.mask != 0 &&
        g_systick_collector.aligned != aligned) {
        // Vorheriges Wort unvollstaendig -> mit aktuellem Wert flushen.
        uint32_t cur = systick_read32(g_systick_collector.aligned);
        for (int i = 0; i < 4; ++i)
            if (g_systick_collector.mask & (1u << i))
                cur = (cur & ~(0xFFu << (i * 8))) |
                      (static_cast<uint32_t>(g_systick_collector.bytes[i]) << (i * 8));
        systick_write32(g_systick_collector.aligned, cur);
        g_systick_collector = {0, {0,0,0,0}, 0};
    }
    g_systick_collector.aligned = aligned;
    uint32_t lane = addr & 3u;
    g_systick_collector.bytes[lane] = val;
    g_systick_collector.mask |= static_cast<uint8_t>(1u << lane);
    if (g_systick_collector.mask == 0xF) {
        uint32_t v = static_cast<uint32_t>(g_systick_collector.bytes[0])
                   | (static_cast<uint32_t>(g_systick_collector.bytes[1]) << 8)
                   | (static_cast<uint32_t>(g_systick_collector.bytes[2]) << 16)
                   | (static_cast<uint32_t>(g_systick_collector.bytes[3]) << 24);
        systick_write32(aligned, v);
        g_systick_collector = {0, {0,0,0,0}, 0};
    }
}

// SYSCON-Schattenregister
uint32_t        g_syspllctrl   = 0;       // MSEL[4:0], PSEL[6:5]
uint32_t        g_syspllclksel = 0;       // 0 = IRC, 1 = SYSOSC
uint32_t        g_mainclksel   = 0;       // 0 = IRC, 2 = SYSPLLOUT
uint32_t        g_sysahbclkdiv = 1;
uint32_t        g_pdruncfg     = 0xFFFF;
uint32_t        g_bodctrl      = 0;        // LPC BODCTRL-Schatten

// Bildet das LPC-Brownout-Detect auf die *echte* RP2350-POWMAN-BOD ab.
//
// WICHTIGE HARDWARE-EINSCHRÄNKUNG: Die LPC1115-BOD überwacht die 3,3-V-
// VDD-Versorgung (Schwellen ~1,7..2,9 V). Die RP2350-POWMAN-BOD überwacht
// dagegen die *Core-Rail* (~1,1 V, VSEL 0,473..1,204 V). Eine wörtliche
// Übersetzung der 3,3-V-Schwellen wäre physikalisch sinnlos und ein Anheben
// der Core-Schwelle würde im Feldgerät spurious Resets riskieren. Daher:
//   - VSEL bleibt auf RP2350-Default (sichere Core-Brownout-Schwelle).
//   - Setzt der Gast BODRSTENA, stellen wir nur sicher, dass die RP2350-BOD
//     überhaupt aktiv ist (EN). Damit löst echte Unterspannung am Gerät einen
//     Hardware-Reset aus — wie es die LPC-Firmware erwartet.
void bod_apply() {
    if (g_bodctrl & BODCTRL_BODRSTENA) {
        powman_set_bits(&powman_hw->bod, POWMAN_BOD_EN_BITS);
    }
}
uint8_t         g_iocon[256]{};
uint8_t         g_pintsel[8]{};            // Kanal n → LPC-Pin-Index (PINTSEL0..7)
uint32_t        g_current_hz   = 12'000'000; // Default IRC

peripherals::Stats g_stats{};
bool            g_in_post_hook = false;
bool            g_pll_reconfig_pending = false;

// IOCON-Registeroffset (Byte, relativ zu IOCON_BASE) je LPC-Pin. Reihenfolge =
// physisches LPC11xx-IOCON-Layout (UM10398 / CMSIS LPC_IOCON_TypeDef, exakt wie
// sblib platform.cpp ioconOffsets). 0xFFFF = kein IOCON-Register. Wird genutzt,
// um die MODE-Bits (Pull-up/-down) eines Pins auf den echten RP2350-Pad
// anzuwenden — sonst floaten LPC-Input-Pins (z. B. der Selfbus-PROG-Taster mit
// INPUT|PULL_UP) auf dem RP2350-Default (Pull-down) und werden nie erkannt.
constexpr uint16_t IOCON_NONE = 0xFFFFu;
constexpr uint16_t IOCON_OFF[4][12] = {
    // Port 0: P0_0..P0_11
    { 0x00C, 0x010, 0x01C, 0x02C, 0x030, 0x034,
      0x04C, 0x050, 0x060, 0x064, 0x068, 0x074 },
    // Port 1: P1_0..P1_11
    { 0x078, 0x07C, 0x080, 0x090, 0x094, 0x0A0,
      0x0A4, 0x0A8, 0x014, 0x038, 0x06C, 0x098 },
    // Port 2: P2_0..P2_11
    { 0x008, 0x028, 0x05C, 0x08C, 0x040, 0x044,
      0x000, 0x020, 0x024, 0x054, 0x058, 0x070 },
    // Port 3: P3_0..P3_5 (Rest nicht bestueckt)
    { 0x084, 0x088, 0x09C, 0x0AC, 0x03C, 0x048,
      IOCON_NONE, IOCON_NONE, IOCON_NONE, IOCON_NONE, IOCON_NONE, IOCON_NONE },
};

// Wendet die IOCON-MODE-Bits (Pull-up/-down) eines LPC-Pins auf den gemappten
// echten RP2350-Pad an. Fuer Output-Pins werden die Pulls deaktiviert (der Pin
// treibt aktiv). Bridge-Pins (ADC/SPI/Capture/Match) bleiben unberuehrt.
void apply_pull_to_hw(uint8_t port, uint8_t pin) {
    if (port >= 4u || pin >= 12u) return;
    uint16_t off = IOCON_OFF[port][pin];
    if (off == IOCON_NONE) return;
    const auto& pm = config::pin_map();
    uint8_t lpc = lpc_pin_idx(port, pin);
    if (lpc >= config::LPC_PIN_COUNT) return;
    int g = pm.lpc_to_rp[lpc];
    if (g < 0 || bridge_owns_gpio(g)) return;
    if ((g_gpio[port].dir >> pin) & 1u) {            // Output -> kein Pull
        gpio_disable_pulls(static_cast<uint>(g));
        return;
    }
    switch ((g_iocon[off] >> 3) & 0x3u) {            // IOCON MODE[1:0]
        case 0x1: gpio_pull_down(static_cast<uint>(g)); break;   // pull-down
        case 0x2: gpio_pull_up(static_cast<uint>(g));   break;   // pull-up
        default:  gpio_disable_pulls(static_cast<uint>(g)); break;// inactive/repeater
    }
}

// Reverse-Lookup: IOCON-Byteoffset (word-aligned) -> Pin, dann Pull anwenden.
void apply_iocon_pull(uint32_t byte_off) {
    for (uint8_t port = 0; port < 4u; ++port)
        for (uint8_t pin = 0; pin < 12u; ++pin)
            if (IOCON_OFF[port][pin] == byte_off) {
                apply_pull_to_hw(port, pin);
                return;
            }
}

void apply_gpio_to_hw(uint8_t lpc_pin, bool out, bool level) {
    auto pm = config::pin_map();
    if (lpc_pin >= config::LPC_PIN_COUNT) return;
    int g = pm.lpc_to_rp[lpc_pin];
    if (g < 0) return;
    if (bridge_owns_gpio(g)) return;   // ADC/SPI/Timer-Capture/Match besitzen den Pin
    gpio_init(static_cast<uint>(g));
    gpio_set_dir(static_cast<uint>(g), out);
    if (out) {
        gpio_disable_pulls(static_cast<uint>(g));    // Ausgang treibt aktiv
        gpio_put(static_cast<uint>(g), level);
    } else {
        // Eingang: Pull-Konfiguration aus dem IOCON-Schatten anwenden, damit ein
        // Taster gegen GND (PROG-Pin, INPUT|PULL_UP) sauber erkannt wird.
        apply_pull_to_hw(static_cast<uint8_t>(lpc_pin / 12u),
                         static_cast<uint8_t>(lpc_pin % 12u));
    }
}

// LPC1115-PLL-Modell (UM10398, Kap. 3.5.5):
//   F_CCO  = F_CLKIN * 2 * P * (M+1)
//   F_OUT  = F_CLKIN * (M+1)
//   gültig: 156 MHz <= F_CCO <= 320 MHz
uint32_t recompute_target_hz() {
    uint32_t f_in = 12'000'000; // IRC nominal 12 MHz
    if ((g_syspllclksel & 0x3u) == 1u) f_in = 12'000'000; // SYSOSC; 12 MHz Annahme
    uint32_t msel = (g_syspllctrl & 0x1Fu) + 1u;          // M + 1
    uint32_t f_pll = f_in * msel;
    uint32_t f_main = ((g_mainclksel & 0x3u) == 3u) ? f_pll
                    : ((g_mainclksel & 0x3u) == 0u) ? 12'000'000u
                    : f_in;
    uint32_t div = g_sysahbclkdiv ? g_sysahbclkdiv : 1u;
    return f_main / div;
}

// Übernimmt die vom Gast programmierte LPC-Soll-Frequenz als Zeitbasis der
// emulierten Peripherie (Timer-Tick-Skalierung in ct_advance, UART-Baud in
// uart0_ensure_hw, CLI-Anzeige). Der ECHTE RP2350-Systemtakt wird BEWUSST NICHT
// verändert.
//
// Grund: set_sys_clock_khz() von Core1 aus aufzurufen, während Core0 USB-CDC/
// CLI/Heartbeat bedient, legt Core0 lahm — die clk_sys-Transition (kurzer
// Wechsel auf clk_ref, pll_sys-Stop/Restart) plus Multicore-Hazard bringt
// USB-/Flash-Timing durcheinander, die CLI friert ein. Für die Korrektheit der
// Emulation ist der reale Takt ohnehin irrelevant: ct_advance skaliert REALE
// Wall-Clock-Zeit (time_us_64) mit g_current_hz, die UART-Baud wird aus
// g_current_hz berechnet und real über clk_peri gesetzt. Zusätzlich braucht der
// Trap-and-Emulate-Pfad die volle RP2350-Frequenz als Reserve, um die Gast-
// Peripherie in Echtzeit zu bedienen — ein Herunterregeln auf die LPC-Frequenz
// würde den Emulator selbst ausbremsen.
// Einzige Einbuße: native Busy-Loops des Gasts und die gast-eigene SysTick
// laufen mit der realen (stabilen) RP2350-Frequenz statt der LPC-Soll-Frequenz.
void retarget_rp2350_clock(uint32_t target_hz) {
    if (target_hz < 12'000'000u || target_hz > 150'000'000u) return;
    if (target_hz == g_current_hz)                            return;

    g_current_hz = target_hz;
    ++g_stats.pll_reconfigs;
    std::printf("[CLK] LPC-Soll-Takt %lu kHz uebernommen "
                "(emulierte Zeitbasis; RP2350-Takt unveraendert)\n",
                static_cast<unsigned long>((target_hz + 500u) / 1000u));
}

// SYSCON 32-Bit-Schreibzugriff (kommt in 4 Byte-Schritten an). Wir halten
// den Wert in einem RAM-Schatten, und merken uns für PLL-relevante Register
// dass eine Re-Konfiguration anliegt — die wird einmalig im Post-Hook nach
// dem letzten Byte ausgeführt, damit wir nicht 4× neu takten.
void syscon_write32(uint32_t addr, uint32_t value) {
    switch (addr) {
        case SYSMEMREMAP:
            // LPC SYSMEMREMAP: 0=BootROM, 1=User-RAM, 2=User-Flash. Ein
            // Schreiben in einen User-Mode markiert den Vektor-Remap. Der
            // Selfbus-Bootloader setzt 0x01 unmittelbar vor dem Sprung in die
            // Applikation (Vektortabelle wurde zuvor nach RAM kopiert). Auf dem
            // Cortex-M33 gibt es kein SYSMEMREMAP — wir bilden den Remap auf
            // VTOR ab und stellen IRQ-/Fault-Vektoren auf die Applikation um.
            // (Cortex-M0+-Reset-Default ist BootROM; nur User-Mode triggert.)
            if ((value & 0x3u) != 0u) {
                emulator::activate_bootloader_handover();
            }
            break;
        case SYSPLLCTRL:    g_syspllctrl   = value & 0x7Fu; g_pll_reconfig_pending = true; break;
        case SYSPLLCLKSEL:  g_syspllclksel = value & 0x3u;  g_pll_reconfig_pending = true; break;
        case MAINCLKSEL:    g_mainclksel   = value & 0x3u;  g_pll_reconfig_pending = true; break;
        case SYSAHBCLKDIV:  g_sysahbclkdiv = value & 0xFFu; g_pll_reconfig_pending = true; break;
        case PDRUNCFG:      g_pdruncfg     = value;                                           break;
        case BODCTRL:       g_bodctrl      = value & 0x1Fu; bod_apply();                       break;
        case SYSPLLCLKUEN:
        case MAINCLKUEN:
            if (value & 1u) g_pll_reconfig_pending = true;
            break;
        default:
            if (addr >= PINTSEL0 && addr < PINTSEL_END) {
                g_pintsel[(addr - PINTSEL0) >> 2] = static_cast<uint8_t>(value & 0x3Fu);
            }
            break;
    }
}

uint32_t syscon_read32(uint32_t addr) {
    switch (addr) {
        case SYSPLLCTRL:   return g_syspllctrl;
        case SYSPLLSTAT:   return 1; // PLL locked (immer)
        case SYSPLLCLKSEL: return g_syspllclksel;
        case SYSPLLCLKUEN: return 1;
        case MAINCLKSEL:   return g_mainclksel;
        case MAINCLKUEN:   return 1;
        case SYSAHBCLKDIV: return g_sysahbclkdiv;
        case PDRUNCFG:     return g_pdruncfg;
        case BODCTRL:      return g_bodctrl;
        default:
            if (addr >= PINTSEL0 && addr < PINTSEL_END)
                return g_pintsel[(addr - PINTSEL0) >> 2];
            return 0;
    }
}

// Sammler für Word-Zugriffe, die in 4 Byte-Calls anliegen. Wir bündeln nur
// SYSCON, weil dort Reihenfolge und Word-Atomarität wichtig ist.
struct WordCollector {
    uint32_t addr_aligned;
    uint8_t  bytes[4];
    uint8_t  written_mask;
};
WordCollector g_syscon_collector{0,{0,0,0,0},0};

bool collector_complete(WordCollector& c) {
    return c.written_mask == 0xF;
}

void syscon_collect_byte(uint32_t addr, uint8_t val) {
    uint32_t aligned = addr & ~3u;
    if (g_syscon_collector.written_mask != 0 &&
        g_syscon_collector.addr_aligned != aligned) {
        // Vorheriges Word unvollständig — flushen mit aktuellem Schatten als Default.
        uint32_t cur = syscon_read32(g_syscon_collector.addr_aligned);
        for (int i = 0; i < 4; ++i) {
            if (g_syscon_collector.written_mask & (1u << i)) {
                cur = (cur & ~(0xFFu << (i * 8))) |
                      (static_cast<uint32_t>(g_syscon_collector.bytes[i]) << (i * 8));
            }
        }
        syscon_write32(g_syscon_collector.addr_aligned, cur);
        g_syscon_collector = {0, {0,0,0,0}, 0};
    }
    g_syscon_collector.addr_aligned = aligned;
    uint32_t lane = addr & 3u;
    g_syscon_collector.bytes[lane] = val;
    g_syscon_collector.written_mask |= static_cast<uint8_t>(1u << lane);

    if (collector_complete(g_syscon_collector)) {
        uint32_t value = static_cast<uint32_t>(g_syscon_collector.bytes[0])       |
                         (static_cast<uint32_t>(g_syscon_collector.bytes[1]) << 8) |
                         (static_cast<uint32_t>(g_syscon_collector.bytes[2]) << 16) |
                         (static_cast<uint32_t>(g_syscon_collector.bytes[3]) << 24);
        syscon_write32(aligned, value);
        g_syscon_collector = {0, {0,0,0,0}, 0};
    }
}

// =========================================================================
// UART0-Modell: 16550-kompatibel; Backend = RP2350 uart0.
// =========================================================================
struct UartModel {
    uint8_t  ier;
    uint8_t  lcr;
    uint8_t  fcr;
    uint8_t  mcr;
    uint16_t divisor;
    uart_inst_t* hw;
    bool     init_done;
};
UartModel g_uart0{};

void uart0_ensure_hw(uint32_t f_cpu) {
    if (!g_uart0.hw) g_uart0.hw = uart0;
    if (g_uart0.divisor == 0) return;
    uint32_t baud = f_cpu / (16u * g_uart0.divisor);
    if (baud == 0) baud = 9600;
    if (!g_uart0.init_done) {
        uart_init(g_uart0.hw, baud);
        g_uart0.init_done = true;
    } else {
        uart_set_baudrate(g_uart0.hw, baud);
    }
}

uint8_t uart0_read_reg(uint32_t addr) {
    bool dlab = (g_uart0.lcr & 0x80u) != 0;
    switch (addr) {
        case UART0_RBR:
            if (dlab) return static_cast<uint8_t>(g_uart0.divisor & 0xFFu);
            if (g_uart0.hw && uart_is_readable(g_uart0.hw))
                return static_cast<uint8_t>(uart_getc(g_uart0.hw));
            return 0;
        case UART0_IER:
            if (dlab) return static_cast<uint8_t>((g_uart0.divisor >> 8) & 0xFFu);
            return g_uart0.ier;
        case UART0_IIR: return 0xC1;
        case UART0_LCR: return g_uart0.lcr;
        case UART0_MCR: return g_uart0.mcr;
        case UART0_LSR: {
            uint8_t s = 0x60;
            if (g_uart0.hw && uart_is_readable(g_uart0.hw)) s |= 0x01u;
            return s;
        }
        default: return 0;
    }
}

void uart0_write_reg(uint32_t addr, uint8_t val) {
    bool dlab = (g_uart0.lcr & 0x80u) != 0;
    switch (addr) {
        case UART0_THR:
            if (dlab) {
                g_uart0.divisor = static_cast<uint16_t>(
                    (g_uart0.divisor & 0xFF00u) | val);
                uart0_ensure_hw(g_current_hz);
            } else if (g_uart0.hw) {
                uart_putc_raw(g_uart0.hw, static_cast<char>(val));
                if (g_uart0.ier & 0x02u) irq_inject::pend(lpc_irq::UART0);
            }
            break;
        case UART0_IER:
            if (dlab) {
                g_uart0.divisor = static_cast<uint16_t>(
                    (g_uart0.divisor & 0x00FFu) |
                    (static_cast<uint16_t>(val) << 8));
                uart0_ensure_hw(g_current_hz);
            } else g_uart0.ier = val;
            break;
        case UART0_FCR: g_uart0.fcr = val; break;
        case UART0_LCR: g_uart0.lcr = val; uart0_ensure_hw(g_current_hz); break;
        case UART0_MCR: g_uart0.mcr = val; break;
        default: break;
    }
}

// =========================================================================
// CT16Bx / CT32Bx — Match-Timer mit Soft-Tick aus time_us_64().
// Modelliert generische LPC1115-Timer-Hardware: Match (MCR/MR/EMR), PWM
// (PWMC) und Capture (CCR/CR0). Die Capture-Eingaenge und Match-/PWM-Ausgaenge
// koennen optional an echte RP2350-GPIOs gebrueckt werden — nutzbar fuer jede
// Capture-/PWM-Anwendung (Frequenz-/Pulsbreitenmessung, Servo-/Trigger-PWM,
// Software-Bus-Treiber wie KNX-Selfbus: Capture = Empfang, Match = Senden).
// =========================================================================
struct CtModel {
    bool     enabled;
    uint32_t pre;
    uint32_t pc;
    uint32_t tc;
    uint32_t mr[4];
    uint32_t mcr;
    uint32_t ir;
    uint32_t ccr;          // Capture Control (CAP0RE/FE/I in Bits 0..2)
    uint32_t cr0;          // Capture-Wert 0 (read-only für Gast)
    uint32_t emr;          // External Match Register (EM0..3 + EMC0..3)
    uint32_t pwmc;         // PWM-Control: Bit m = MATm im PWM-Modus
    bool     is32;
    uint8_t  irq_num;
    uint64_t last_us;
    int8_t   cap_pin;      // RP2350-GPIO für CAP0-Eingang, -1 = keiner
    bool     cap_last;     // letzter gesampelter Pegel (Flankenerkennung)
    int8_t   mat_pin[4];   // RP2350-GPIO für MAT0..3-Ausgänge, -1 = keiner
    // PIO-Capture (opt-in, flankengenau): Handle der State-Machine, Zählrate
    // und Zustand zur Differenz-/Richtungsrekonstruktion.
    int      pio_handle;   // < 0 = Software-Capture (Fallback)
    float    pio_rate;     // PIO-Zählrate [Counts/s]
    uint32_t pio_prev;     // letzter Zählerstand
    bool     pio_have_prev;
    bool     pio_dir_rising; // nächste erwartete Flanke steigend? (Idle high → erst fallend)
    // PIO-Match (opt-in): Hardware-Puls-Erzeugung je PWM-Match-Kanal. Ein
    // Handle pro Kanal (m=0..3), -1 = Software-PWM (generischer Fallback).
    int      tx_handle[4]; // < 0 = Software-PWM für diesen Kanal
    float    tx_rate;      // PIO-Zählrate [Counts/s] (für alle Kanäle gleich)
};
CtModel g_ct[4];

// EMR-External-Match: EM0..3 = Bits 0..3, EMC0..3 = je 2 Bit ab Bit 4.
// EMC: 0=nichts, 1=Pin löschen(0), 2=Pin setzen(1), 3=Pin toggeln.
void ct_apply_external_match(CtModel& c, int m) {
    uint32_t emc = (c.emr >> (4u + 2u * static_cast<uint32_t>(m))) & 0x3u;
    if (emc == 0) return;
    uint32_t bit = 1u << m;
    bool level;
    switch (emc) {
        case 1:  c.emr &= ~bit; level = false; break;   // clear
        case 2:  c.emr |=  bit; level = true;  break;   // set
        default: c.emr ^=  bit; level = (c.emr & bit) != 0; break; // toggle
    }
    if (c.mat_pin[m] >= 0)
        gpio_put(static_cast<uint>(c.mat_pin[m]), level);
}

// PWM-Ausgang: Ist MATm im PWM-Modus (PWMC-Bit m), folgt der MATm-Pin dem
// PWM-Pegel: low solange TC < MRm, high sobald TC >= MRm; bei MRm==0 dauerhaft
// high (LPC-Spezialfall). Der Pegel wird zugleich im EMR-EM-Bit gespiegelt,
// damit getMatchChannelLevel() korrekt liest. Generischer Software-Pfad —
// greift fuer alle Kanaele, die NICHT von einer Match-PIO getrieben werden.
void ct_update_pwm(CtModel& c) {
    for (int m = 0; m < 4; ++m) {
        if (!(c.pwmc & (1u << m))) continue;
        // Match-PIO besitzt diesen Pin: kein Software-Bit-Bang (Doppeltreiben).
        if (c.tx_handle[m] >= 0) continue;
        bool level = (c.mr[m] == 0u) ? true : (c.tc >= c.mr[m]);
        uint32_t bit = 1u << m;
        bool cur = (c.emr & bit) != 0;
        if (level != cur) {
            if (level) c.emr |= bit; else c.emr &= ~bit;
            if (c.mat_pin[m] >= 0)
                gpio_put(static_cast<uint>(c.mat_pin[m]), level);
        }
    }
}

// Match-PIO-Puls (opt-in, je PWM-Kanal): Beim Schreiben des PWM-Match-Registers
// berechnet die CPU die Counts bis zur steigenden Flanke (MRm - TC) und die
// Pulsbreite bis zum Timer-Reset (Reset-MR - MRm) und stellt den Puls in die
// PIO-FIFO; die Flanken erzeugt danach die PIO hardware-getaktet.
//
// Sicherer Fallback: Eine PIO-State-Machine wird nur belegt, wenn ein gueltiges
// PWM-Periodenmodell existiert (ein Match-Kanal mit RESET als Periode und
// MRm < Periode). Fehlt das Modell, wird der Pin an den generischen
// Software-PWM-Pfad zurueckgegeben — andere Programme funktionieren so
// unveraendert, auch ohne Selfbus-typisches Puls-Schema.
void ct_emit_tx_pulse(CtModel& c, int m) {
    if (m < 0 || m > 3) return;
    if (!config::tmatch_pio()) return;
    if (!(c.pwmc & (1u << static_cast<uint32_t>(m)))) return;
    if (c.mat_pin[m] < 0) return;

    uint32_t mask   = c.is32 ? 0xFFFF'FFFFu : 0xFFFFu;
    uint32_t mr_pwm = c.mr[m] & mask;

    // Periode = erster Match-Kanal mit RESET-Bit (MCR Bit 1 je Kanal×3).
    uint32_t mr_reset = 0; bool have_reset = false;
    for (int r = 0; r < 4; ++r) {
        if ((c.mcr >> (r * 3)) & 0x2u) { mr_reset = c.mr[r] & mask; have_reset = true; break; }
    }

    // Kein Periodenmodell -> Pin an Software-PWM zurueckgeben (Handle freigeben).
    if (!have_reset) {
        if (c.tx_handle[m] >= 0) {
            pio_glue::tx_teardown(c.tx_handle[m]);
            c.tx_handle[m] = -1;
            gpio_init(static_cast<uint>(c.mat_pin[m]));
            gpio_set_dir(static_cast<uint>(c.mat_pin[m]), true);  // Software treibt Pegel
        }
        return;
    }

    // MRm >= Periode: in diesem Zyklus kein Puls (z. B. MR=0xffff = "aus").
    // Handle bleibt belegt; die PIO haelt den Pin idle-low. Kein Teardown,
    // damit kein Claim/Unclaim-Churn bei pulsweisem Senden entsteht.
    if (mr_pwm >= mr_reset) return;

    uint32_t delay_ticks = (mr_pwm - c.tc) & mask;     // bis zur steigenden Flanke
    if (delay_ticks > (mask >> 1)) return;             // Match bereits vorbei
    uint32_t width_ticks = (mr_reset - mr_pwm) & mask;
    if (width_ticks == 0) return;

    double f_tc = static_cast<double>(g_current_hz) / static_cast<double>(c.pre + 1u);
    if (f_tc <= 0.0) return;

    // Lazy-Allokation: SM erst beim ersten gueltigen Puls belegen.
    if (c.tx_handle[m] < 0) {
        float rate = 0.0f;
        int h = pio_glue::tx_setup(static_cast<uint8_t>(c.mat_pin[m]), rate);
        if (h >= 0) { c.tx_handle[m] = h; c.tx_rate = rate; }
    }
    if (c.tx_handle[m] < 0 || c.tx_rate <= 0.0f) return;  // Fallback: Software-PWM

    double cnt_per_tick = static_cast<double>(c.tx_rate) / f_tc;
    uint32_t delay_cnt = static_cast<uint32_t>(static_cast<double>(delay_ticks) * cnt_per_tick);
    uint32_t width_cnt = static_cast<uint32_t>(static_cast<double>(width_ticks) * cnt_per_tick);
    pio_glue::tx_emit(c.tx_handle[m], delay_cnt, width_cnt);
}

uint32_t ct_idx_for(uint32_t addr) {
    if (addr >= CT16B0_BASE && addr < CT16B0_BASE + CT_BLOCK_SIZE) return 0;
    if (addr >= CT16B1_BASE && addr < CT16B1_BASE + CT_BLOCK_SIZE) return 1;
    if (addr >= CT32B0_BASE && addr < CT32B0_BASE + CT_BLOCK_SIZE) return 2;
    if (addr >= CT32B1_BASE && addr < CT32B1_BASE + CT_BLOCK_SIZE) return 3;
    return 0xFFFFFFFFu;
}
uint32_t ct_base_for(uint32_t i) {
    static const uint32_t b[4] = { CT16B0_BASE, CT16B1_BASE, CT32B0_BASE, CT32B1_BASE };
    return b[i];
}

void ct_advance(CtModel& c) {
    if (!c.enabled) { c.last_us = time_us_64(); ct_update_pwm(c); return; }
    uint64_t now = time_us_64();
    uint64_t dt  = now - c.last_us;
    c.last_us = now;
    uint64_t ticks = (dt * static_cast<uint64_t>(g_current_hz))
                     / (1'000'000ull * static_cast<uint64_t>(c.pre + 1u));
    if (!ticks) { ct_update_pwm(c); return; }
    uint32_t mask = c.is32 ? 0xFFFF'FFFFu : 0xFFFFu;
    for (uint64_t i = 0; i < ticks; ++i) {
        c.tc = (c.tc + 1u) & mask;
        for (int m = 0; m < 4; ++m) {
            if (c.tc == c.mr[m]) {
                uint32_t mcr = (c.mcr >> (m * 3)) & 0x7u;
                if (mcr & 0x1) { c.ir |= (1u << m); irq_inject::pend(c.irq_num); }
                ct_apply_external_match(c, m);   // EMR: MATm-Pin treiben
                if (mcr & 0x2) c.tc = 0;
                if (mcr & 0x4) c.enabled = false;
            }
        }
    }
    ct_update_pwm(c);
}

// Flankenerkennung am Capture-Eingang: liest den echten RP2350-Pin, erkennt
// die per CCR scharfgeschalteten Flanken, schreibt den aktuellen TC nach CR0,
// setzt IR-Bit 4 und pendet den Timer-IRQ (falls CAP0I gesetzt). Wird mit der
// gleichen Kadenz wie sample_pin_interrupts() aufgerufen (jeder MMIO-Trap und
// die WFI-Warteschleife). Zeitliche Auflösung der Flanken = Sampling-Kadenz.
void ct_sample_capture_sw(CtModel& c) {
    bool rise_arm = (c.ccr & 0x1u) != 0;
    bool fall_arm = (c.ccr & 0x2u) != 0;
    bool level = gpio_get(static_cast<uint>(c.cap_pin));
    if (level == c.cap_last) return;
    bool rising  = level && !c.cap_last;
    bool falling = !level && c.cap_last;
    c.cap_last = level;
    if ((rising && rise_arm) || (falling && fall_arm)) {
        ct_advance(c);                 // TC auf "jetzt" bringen
        c.cr0 = c.tc;
        c.ir |= (1u << 4);             // CR0-Capture-Interrupt-Flag
        if (c.ccr & 0x4u) irq_inject::pend(c.irq_num);  // CAP0I
    }
}

// PIO-Capture (opt-in): drainiert die FIFO der Timestamp-State-Machine. Jeder
// Eintrag = eine flankengenaue (Hardware-)Zeitmarke. Die Flankenrichtung wird
// per strikter Alternation bestimmt (Bus-Idle = high → erste Flanke fallend).
// Die Zählerdifferenz wird in TC-Ticks umgerechnet und auf CR0 akkumuliert,
// sodass die vom Gast ausgewerteten CR0-Differenzen exakt stimmen — unabhängig
// davon, wann die CPU die FIFO ausliest (kein Jitter, keine verlorenen Flanken).
void ct_sample_capture_pio(CtModel& c) {
    bool rise_arm = (c.ccr & 0x1u) != 0;
    bool fall_arm = (c.ccr & 0x2u) != 0;
    uint32_t raw;
    while (pio_glue::ts_read(c.pio_handle, raw)) {
        bool rising = c.pio_dir_rising;
        c.pio_dir_rising = !c.pio_dir_rising;     // Flanken alternieren strikt

        if (!c.pio_have_prev) {
            c.pio_prev = raw;
            c.pio_have_prev = true;
            ct_advance(c);                        // CR0-Anker = aktueller TC
            c.cr0 = c.tc;
        } else {
            // Abwärtszähler: verstrichene Counts = prev - raw (mod 2^32).
            uint32_t delta = c.pio_prev - raw;
            c.pio_prev = raw;
            double f_tc = static_cast<double>(g_current_hz)
                          / static_cast<double>(c.pre + 1u);
            double tc_delta = (c.pio_rate > 0.0f)
                ? static_cast<double>(delta) * f_tc / static_cast<double>(c.pio_rate)
                : 0.0;
            uint32_t mask = c.is32 ? 0xFFFF'FFFFu : 0xFFFFu;
            c.cr0 = static_cast<uint32_t>(
                        (static_cast<uint64_t>(c.cr0) +
                         static_cast<uint64_t>(tc_delta)) & mask);
        }

        if ((rising && rise_arm) || (!rising && fall_arm)) {
            c.ir |= (1u << 4);
            if (c.ccr & 0x4u) irq_inject::pend(c.irq_num);  // CAP0I
        }
    }
}

void ct_sample_capture(CtModel& c) {
    if (c.cap_pin < 0) return;
    if (c.pio_handle >= 0) ct_sample_capture_pio(c);
    else                   ct_sample_capture_sw(c);
}

uint8_t ct_read_byte(uint32_t idx, uint32_t off) {
    CtModel& c = g_ct[idx];
    ct_advance(c);
    ct_sample_capture(c);
    auto byte_of = [&](uint32_t v) {
        return static_cast<uint8_t>((v >> ((off & 3u) * 8)) & 0xFFu);
    };
    switch (off & ~3u) {
        case 0x00: return byte_of(c.ir);
        case 0x04: return byte_of(c.enabled ? 1u : 0u);
        case 0x08: return byte_of(c.tc);
        case 0x0C: return byte_of(c.pre);
        case 0x10: return byte_of(c.pc);
        case 0x14: return byte_of(c.mcr);
        case 0x18: return byte_of(c.mr[0]);
        case 0x1C: return byte_of(c.mr[1]);
        case 0x20: return byte_of(c.mr[2]);
        case 0x24: return byte_of(c.mr[3]);
        case 0x28: return byte_of(c.ccr);
        case 0x2C: return byte_of(c.cr0);
        case 0x3C: return byte_of(c.emr);
        case 0x74: return byte_of(c.pwmc);
        default:   return 0;
    }
}

void ct_write_byte(uint32_t idx, uint32_t off, uint8_t val) {
    CtModel& c = g_ct[idx];
    ct_advance(c);
    auto patch = [&](uint32_t& v) {
        uint32_t lane = (off & 3u) * 8;
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    switch (off & ~3u) {
        case 0x00: { uint32_t m = 0; patch(m); c.ir &= ~m; break; }
        case 0x04: {
            uint32_t v = c.enabled ? 1u : 0u;
            patch(v);
            bool was = c.enabled;
            c.enabled = (v & 0x1u) != 0;
            if (v & 0x2u) { c.tc = 0; c.pc = 0; }
            if (!was && c.enabled) c.last_us = time_us_64();
            break;
        }
        case 0x08: patch(c.tc);    break;
        case 0x0C: patch(c.pre);   break;
        case 0x10: patch(c.pc);    break;
        case 0x14: patch(c.mcr);   break;
        case 0x18: patch(c.mr[0]); if ((off & 3u) == 3u) ct_emit_tx_pulse(c, 0); break;
        case 0x1C: patch(c.mr[1]); if ((off & 3u) == 3u) ct_emit_tx_pulse(c, 1); break;
        case 0x20: patch(c.mr[2]); if ((off & 3u) == 3u) ct_emit_tx_pulse(c, 2); break;
        case 0x24: patch(c.mr[3]); if ((off & 3u) == 3u) ct_emit_tx_pulse(c, 3); break;
        case 0x28: patch(c.ccr);   break;
        case 0x2C: /* CR0 read-only */ break;
        case 0x3C: {
            // EMR-Schreibzugriff: EMC-Steuerbits übernehmen; manuelle EM0..3-
            // Setzen treibt die MATm-Pins direkt (z. B. Initialpegel beim Senden).
            uint32_t old = c.emr;
            patch(c.emr);
            for (int m = 0; m < 4; ++m) {
                // Match-PIO besitzt seinen Pin; manuelles EM-Setzen nicht durchreichen.
                if (c.tx_handle[m] >= 0) continue;
                if (c.mat_pin[m] >= 0 && (((old ^ c.emr) >> m) & 1u))
                    gpio_put(static_cast<uint>(c.mat_pin[m]), ((c.emr >> m) & 1u) != 0);
            }
            break;
        }
        case 0x74: {
            patch(c.pwmc);
            // Kanal aus dem PWM-Modus genommen -> evtl. belegte Match-PIO
            // freigeben und Pin an Software-PWM/GPIO zurueckgeben.
            for (int m = 0; m < 4; ++m) {
                if (!(c.pwmc & (1u << static_cast<uint32_t>(m))) && c.tx_handle[m] >= 0) {
                    pio_glue::tx_teardown(c.tx_handle[m]);
                    c.tx_handle[m] = -1;
                    if (c.mat_pin[m] >= 0) {
                        gpio_init(static_cast<uint>(c.mat_pin[m]));
                        gpio_set_dir(static_cast<uint>(c.mat_pin[m]), true);
                    }
                }
            }
            break;
        }
        default: break;
    }
    ct_update_pwm(c);   // MRm-/PWMC-Schreibzugriff kann den PWM-Pegel ändern
}

// =========================================================================
// WDT (WWDT) — eigenes Watchdog. Bei Ablauf wird **nur der Guest** neu
// gestartet (emulator::request_guest_reset()), nicht der RP2350.
// =========================================================================
struct WdtModel {
    uint32_t mod;       // [0]=WDEN [1]=WDRESET [2]=WDTOF [3]=WDINT
    uint32_t tc;        // reload value (24-bit)
    uint32_t tv;        // current value
    uint8_t  feed_state; // 0 = idle, 1 = saw 0xAA
    uint64_t last_us;
    uint32_t wdt_clk_hz;// WDT-Clock; LPC default = WDOSC ~ 0.5 MHz, /4 vorscale
};
WdtModel g_wdt{};

void wdt_advance() {
    if ((g_wdt.mod & 0x1u) == 0) { g_wdt.last_us = time_us_64(); return; }
    uint64_t now = time_us_64();
    uint64_t dt  = now - g_wdt.last_us;
    g_wdt.last_us = now;
    uint32_t hz = g_wdt.wdt_clk_hz ? g_wdt.wdt_clk_hz : 500'000u;
    uint64_t ticks = (dt * static_cast<uint64_t>(hz)) / 1'000'000ull;
    if (!ticks) return;
    if (ticks >= g_wdt.tv) {
        g_wdt.tv = 0;
        g_wdt.mod |= 0x4u;       // WDTOF (timeout)
        if (g_wdt.mod & 0x2u) {  // WDRESET → Guest neustarten
            ::peripherals_wdt_reset_guest();
        } else {
            g_wdt.mod |= 0x8u;   // WDINT, IRQ
            irq_inject::pend(lpc_irq::WWDT);
        }
    } else {
        g_wdt.tv -= static_cast<uint32_t>(ticks);
    }
}

uint8_t wdt_read_byte(uint32_t addr) {
    wdt_advance();
    uint32_t lane = (addr & 3u) * 8u;
    switch (addr & ~3u) {
        case WDT_MOD: return static_cast<uint8_t>((g_wdt.mod >> lane) & 0xFFu);
        case WDT_TC:  return static_cast<uint8_t>((g_wdt.tc  >> lane) & 0xFFu);
        case WDT_TV:  return static_cast<uint8_t>((g_wdt.tv  >> lane) & 0xFFu);
        default:      return 0;
    }
}

void wdt_write_byte(uint32_t addr, uint8_t val) {
    wdt_advance();
    uint32_t lane  = (addr & 3u) * 8u;
    uint32_t base  = addr & ~3u;
    auto patch = [&](uint32_t& v, uint32_t mask) {
        v = (v & ~(0xFFu << lane)) |
            ((static_cast<uint32_t>(val) << lane) & mask);
    };
    switch (base) {
        case WDT_MOD: {
            uint32_t old = g_wdt.mod;
            patch(g_wdt.mod, 0xFFu);
            // WDTOF/WDINT sind w1c
            if ((val & 0x4u) == 0 && (lane == 0)) g_wdt.mod = (g_wdt.mod & ~0x4u) | (old & 0x4u);
            if ((val & 0x8u) == 0 && (lane == 0)) g_wdt.mod = (g_wdt.mod & ~0x8u) | (old & 0x8u);
            // Schreiben einer 1 auf TOF/INT clearen (LPC: w1c).
            if ((val & 0x4u) && (lane == 0)) g_wdt.mod &= ~0x4u;
            if ((val & 0x8u) && (lane == 0)) g_wdt.mod &= ~0x8u;
            if ((g_wdt.mod & 0x1u) && !(old & 0x1u)) {
                g_wdt.tv = g_wdt.tc ? g_wdt.tc : 0xFFu;
                g_wdt.last_us = time_us_64();
            }
            break;
        }
        case WDT_TC:  patch(g_wdt.tc, 0xFFFFFFu); break;
        case WDT_FEED: {
            // Magic-Sequenz 0xAA, 0x55 → reload TC nach TV.
            if (lane != 0) break;
            if (g_wdt.feed_state == 0 && val == 0xAAu) g_wdt.feed_state = 1;
            else if (g_wdt.feed_state == 1 && val == 0x55u) {
                g_wdt.tv = g_wdt.tc ? g_wdt.tc : 0xFFu;
                g_wdt.feed_state = 0;
                g_wdt.last_us = time_us_64();
            } else g_wdt.feed_state = 0;
            break;
        }
        default: break;
    }
}

// =========================================================================
// ADC — Bridge auf den echten RP2350-ADC (opt-in via config::adc_bridge_en).
// Mapping ist durch die RP2350-Hardware fix vorgegeben: LPC-Kanal 0..3 →
// RP2350-ADC-Eingang 0..3 (GPIO26..29). Kanäle 4..7 haben keinen RP2350-ADC-
// Pin und liefern weiterhin den Mittenwert. Ist die Bridge aus, ist das
// Ergebnis deterministisch (halber Skalenwert), damit Apps booten.
// =========================================================================
struct AdcModel {
    uint32_t cr;
    uint32_t inten;
    uint32_t dr[8];   // ADC-Channel-Result-Register
    uint32_t gdr;
    uint32_t stat;
};
AdcModel g_adc{};

bool     g_adc_ready        = false;   // Bridge initialisiert?
uint32_t g_adc_gpio_inited  = 0;       // Bitmaske: GPIO je Kanal schon init'd

void adc_bridge_init() {
    g_adc_ready       = false;
    g_adc_gpio_inited = 0;
    if (!config::adc_bridge_enabled()) return;
    adc_init();
    g_adc_ready = true;
}

// Liefert einen 10-Bit-Sample (0..0x3FF) für LPC-ADC-Kanal `ch`.
uint16_t adc_sample_channel(int ch) {
    if (g_adc_ready && ch >= 0 && ch <= 3) {
        if (!(g_adc_gpio_inited & (1u << ch))) {
            adc_gpio_init(static_cast<uint>(26 + ch));   // ADC0..3 = GPIO26..29
            g_adc_gpio_inited |= (1u << ch);
        }
        adc_select_input(static_cast<uint>(ch));
        return static_cast<uint16_t>(adc_read() >> 2);   // 12-Bit → 10-Bit
    }
    return 0x200;                                          // Mittenwert-Fallback
}

// =========================================================================
// Timer-Capture-/Match-Pin-Bridge (CT16/CT32 → echte RP2350-GPIOs).
// Bindet die in CONFIG.INI konfigurierten Capture-Eingänge und Match-/PWM-
// Ausgänge an reale Pins. Capture-Pins werden als Eingang mit Pull-up
// (z. B. Bus-Idle = high) initialisiert, Match-Pins als Ausgang. Generisch
// fuer Capture-/PWM-Anwendungen (Beispiel: KNX-Selfbus Empfang/Senden).
// =========================================================================
void ct_bridge_init() {
    bool use_pio = config::tcap_pio();
    for (int t = 0; t < 4; ++t) {
        // Evtl. zuvor belegte PIO-State-Machine wieder freigeben (Reinit).
        if (g_ct[t].pio_handle >= 0) {
            pio_glue::ts_teardown(g_ct[t].pio_handle);
            g_ct[t].pio_handle = -1;
        }
        for (int m = 0; m < 4; ++m) {
            if (g_ct[t].tx_handle[m] >= 0) {
                pio_glue::tx_teardown(g_ct[t].tx_handle[m]);
                g_ct[t].tx_handle[m] = -1;
            }
        }
        g_ct[t].pio_have_prev  = false;
        g_ct[t].pio_dir_rising = false;

        int cap = config::ct_capture_pin(t);
        g_ct[t].cap_pin = static_cast<int8_t>(cap);
        if (cap >= 0) {
            gpio_init(static_cast<uint>(cap));
            gpio_set_dir(static_cast<uint>(cap), false);    // Eingang
            gpio_pull_up(static_cast<uint>(cap));           // Idle-Pegel high
            g_ct[t].cap_last = gpio_get(static_cast<uint>(cap));
            if (use_pio) {
                float rate = 0.0f;
                int h = pio_glue::ts_setup(static_cast<uint8_t>(cap), rate);
                if (h >= 0) {
                    g_ct[t].pio_handle = h;
                    g_ct[t].pio_rate   = rate;
                } else {
                    std::printf("[CT] PIO-Capture fuer Timer %d nicht moeglich, "
                                "Fallback auf Software\n", t);
                }
            }
        }
        for (int m = 0; m < 4; ++m) {
            int mp = config::ct_match_pin(t, m);
            g_ct[t].mat_pin[m] = static_cast<int8_t>(mp);
            if (mp >= 0) {
                gpio_init(static_cast<uint>(mp));
                gpio_set_dir(static_cast<uint>(mp), true);  // Ausgang
                gpio_put(static_cast<uint>(mp), false);
            }
        }
    }
}

void adc_simulate() {
    // Wenn START != 0 oder BURST gesetzt → Conversion sofort fertig.
    bool start = ((g_adc.cr >> 24) & 0x7u) != 0;
    bool burst = ((g_adc.cr >> 16) & 0x1u) != 0;
    if (!start && !burst) return;
    uint32_t sel = g_adc.cr & 0xFFu;
    for (int ch = 0; ch < 8; ++ch) {
        if (sel & (1u << ch)) {
            uint16_t sample = adc_sample_channel(ch);     // 10-Bit
            g_adc.dr[ch] = (1u << 31) |                   // DONE
                           (static_cast<uint32_t>(sample) << 6);
            g_adc.dr[ch] &= ~(1u << 30);                  // OVERRUN clearen
            g_adc.gdr = (1u << 31) | (static_cast<uint32_t>(sample) << 6) |
                        (static_cast<uint32_t>(ch) << 24);
            g_adc.stat |= (1u << ch);
        }
    }
    if (g_adc.inten & 0x100u) irq_inject::pend(lpc_irq::ADC);
    if (!burst) g_adc.cr &= ~(0x7u << 24);
}

uint8_t adc_read_byte(uint32_t addr) {
    adc_simulate();
    uint32_t lane = (addr & 3u) * 8u;
    uint32_t base = addr & ~3u;
    if (base == ADC_CR)    return static_cast<uint8_t>((g_adc.cr    >> lane) & 0xFFu);
    if (base == ADC_GDR)   return static_cast<uint8_t>((g_adc.gdr   >> lane) & 0xFFu);
    if (base == ADC_INTEN) return static_cast<uint8_t>((g_adc.inten >> lane) & 0xFFu);
    if (base == ADC_STAT)  return static_cast<uint8_t>((g_adc.stat  >> lane) & 0xFFu);
    if (base >= ADC_DR0 && base < ADC_DR0 + 32) {
        uint32_t ch = (base - ADC_DR0) / 4u;
        uint32_t v = g_adc.dr[ch];
        g_adc.dr[ch] &= ~(1u << 31);   // DONE w-after-read clearen
        g_adc.stat &= ~(1u << ch);
        return static_cast<uint8_t>((v >> lane) & 0xFFu);
    }
    return 0;
}

void adc_write_byte(uint32_t addr, uint8_t val) {
    uint32_t lane = (addr & 3u) * 8u;
    uint32_t base = addr & ~3u;
    auto patch = [&](uint32_t& v) {
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    if (base == ADC_CR)    { patch(g_adc.cr);    adc_simulate(); }
    else if (base == ADC_INTEN) patch(g_adc.inten);
}

// =========================================================================
// SSP0/SSP1 — Bridge auf RP2350-Hardware-SPI (spi0/spi1), konfigurierbar.
//
// Der per config::spi_bridge_lpc gewählte LPC-SSP wird voll-duplex auf die
// echte RP2350-SPI abgebildet: Jeder Schreibzugriff auf das Datenregister
// (DR) löst eine SPI-Transaktion aus; das gleichzeitig empfangene Byte/Wort
// landet im RX-Schatten und wird beim DR-Lesen zurückgeliefert. CR0 steuert
// Datenbreite (DSS) sowie SPI-Modus (CPOL/CPHA). Der nicht gebrückte SSP
// (und der Fall „Bridge inaktiv") fällt auf Loopback (RX=TX) zurück, damit
// Apps ohne angeschlossene SPI-Hardware weiter booten.
//
// Bekannte Grenze: Der SCK-Takt wird über spi_bridge_hz konfiguriert, nicht
// aus den LPC-CPSR/SCR-Registern abgeleitet. Chip-Selects sind nicht Teil der
// SSP-Bridge; sie laufen wie gehabt über das GPIO-Modell (echte Pins).
// =========================================================================
struct SspModel {
    uint32_t cr0, cr1, cpsr, imsc, ris, dr_rx, tx;
    uint8_t  irq_num;
};
SspModel g_ssp[2]{};

spi_inst_t* g_spi_hw    = nullptr;   // nullptr → Bridge inaktiv (Loopback)
bool        g_spi_ready = false;
int         g_spi_lpc   = 0;         // welcher LPC-SSP gebrückt ist

void ssp_apply_format(uint32_t idx);

void spi_bridge_init() {
    g_spi_hw    = nullptr;
    g_spi_ready = false;
    g_spi_lpc   = config::spi_bridge_lpc();
    if (!config::spi_bridge_enabled()) return;
    int sck  = config::spi_bridge_sck_pin();
    int mosi = config::spi_bridge_mosi_pin();
    int miso = config::spi_bridge_miso_pin();
    if (sck < 0 || mosi < 0 || miso < 0) return;
    g_spi_hw = config::spi_bridge_instance() ? spi1 : spi0;
    spi_init(g_spi_hw, config::spi_bridge_hz());
    gpio_set_function(static_cast<uint>(sck),  GPIO_FUNC_SPI);
    gpio_set_function(static_cast<uint>(mosi), GPIO_FUNC_SPI);
    gpio_set_function(static_cast<uint>(miso), GPIO_FUNC_SPI);
    g_spi_ready = true;
    ssp_apply_format(static_cast<uint32_t>(g_spi_lpc));
}

// Übernimmt Datenbreite (DSS) und SPI-Modus (CPOL/CPHA) aus CR0 in die HW.
void ssp_apply_format(uint32_t idx) {
    if (!g_spi_ready || static_cast<int>(idx) != g_spi_lpc) return;
    uint bits = (g_ssp[idx].cr0 & 0xFu) + 1u;
    if (bits < 4u)  bits = 4u;
    if (bits > 16u) bits = 16u;
    spi_cpol_t cpol = (g_ssp[idx].cr0 & (1u << 6)) ? SPI_CPOL_1 : SPI_CPOL_0;
    spi_cpha_t cpha = (g_ssp[idx].cr0 & (1u << 7)) ? SPI_CPHA_1 : SPI_CPHA_0;
    spi_set_format(g_spi_hw, bits, cpol, cpha, SPI_MSB_FIRST);
}

bool ssp_is_bridged(uint32_t idx) {
    return g_spi_ready && static_cast<int>(idx) == g_spi_lpc;
}

uint32_t ssp_idx_for(uint32_t addr) {
    if (addr >= SSP0_BASE && addr < SSP0_BASE + SSP_BLOCK) return 0;
    if (addr >= SSP1_BASE && addr < SSP1_BASE + SSP_BLOCK) return 1;
    return 0xFFFFFFFFu;
}
uint32_t ssp_base_for(uint32_t i) { return i ? SSP1_BASE : SSP0_BASE; }

uint8_t ssp_read_byte(uint32_t idx, uint32_t off) {
    SspModel& s = g_ssp[idx];
    uint32_t lane = (off & 3u) * 8u;
    switch (off & ~3u) {
        case SSP_CR0:  return static_cast<uint8_t>((s.cr0  >> lane) & 0xFFu);
        case SSP_CR1:  return static_cast<uint8_t>((s.cr1  >> lane) & 0xFFu);
        case SSP_DR:   {
            uint32_t v = s.dr_rx;
            s.ris &= ~0x4u;     // RX nicht mehr voll
            return static_cast<uint8_t>((v >> lane) & 0xFFu);
        }
        case SSP_SR: {
            // TFE(0)+TNF(1) immer gesetzt (synchroner Transfer); RNE(2), wenn
            // ein RX-Wort bereitsteht. BSY(4) nie (Transfer schon fertig).
            uint8_t sr = 0x03u | ((s.ris & 0x4u) ? 0x04u : 0x00u);
            return static_cast<uint8_t>((sr >> lane) & 0xFFu);
        }
        case SSP_CPSR: return static_cast<uint8_t>((s.cpsr >> lane) & 0xFFu);
        case SSP_IMSC: return static_cast<uint8_t>((s.imsc >> lane) & 0xFFu);
        case SSP_RIS:  return static_cast<uint8_t>((s.ris  >> lane) & 0xFFu);
        case SSP_MIS:  return static_cast<uint8_t>(((s.ris & s.imsc) >> lane) & 0xFFu);
        default: return 0;
    }
}

void ssp_write_byte(uint32_t idx, uint32_t off, uint8_t val) {
    SspModel& s = g_ssp[idx];
    uint32_t lane = (off & 3u) * 8u;
    auto patch = [&](uint32_t& v) {
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    switch (off & ~3u) {
        case SSP_CR0:  patch(s.cr0);  ssp_apply_format(idx); break;
        case SSP_CR1:  patch(s.cr1);  break;
        case SSP_DR:   {
            patch(s.tx);
            // Frame-Größe aus DSS; das letzte Byte des Frames löst den
            // Transfer aus (8-Bit → Lane 0, >8-Bit → Lane 1).
            uint32_t bits = (s.cr0 & 0xFu) + 1u;
            bool complete = (bits <= 8u) ? ((off & 3u) == 0u)
                                         : ((off & 3u) == 1u);
            if (!complete) break;
            if (ssp_is_bridged(idx)) {
                if (bits <= 8u) {
                    uint8_t tx = static_cast<uint8_t>(s.tx & 0xFFu), rx = 0;
                    spi_write_read_blocking(g_spi_hw, &tx, &rx, 1);
                    s.dr_rx = rx;
                } else {
                    uint16_t tx = static_cast<uint16_t>(s.tx & 0xFFFFu), rx = 0;
                    spi_write16_read16_blocking(g_spi_hw, &tx, &rx, 1);
                    s.dr_rx = rx;
                }
            } else {
                s.dr_rx = s.tx;        // Loopback-Fallback
            }
            s.tx = 0;
            s.ris |= 0x4u;             // RX-FIFO not empty
            if (s.imsc & 0x4u) irq_inject::pend(s.irq_num);
            break;
        }
        case SSP_CPSR: patch(s.cpsr); break;
        case SSP_IMSC: patch(s.imsc); break;
        case SSP_ICR:  s.ris &= ~static_cast<uint32_t>(val) << lane; break;
        default: break;
    }
}

// =========================================================================
// I²C0 — echte Bridge auf RP2350-Hardware-I²C (i2c0/i2c1), konfigurierbar.
//
// Wir bilden die LPC-I²C-Master-Statemachine (CONSET STA/STO/SI/AA/I2EN +
// STAT-Codes) auf SDK-Transfers ab:
//   • Schreib-Transaktionen werden gepuffert und beim STOP bzw. beim
//     wiederholten START (nostop=true) auf den Bus geschrieben.
//   • Lese-Transaktionen lesen Byte-für-Byte lazy (nostop bis zum letzten,
//     gesteuert über das AA-Bit) — funktioniert für die üblichen
//     Auto-Increment-Slaves (Sensoren, EEPROMs, write-reg-then-read).
//
// Ist die Bridge nicht aktiviert/initialisiert, fällt das Modell auf das
// alte Stub-Verhalten zurück (STAT=0xF8 idle, NAK 0x20 auf START), damit
// Apps ohne angeschlossenen Slave weiter booten.
//
// Bekannte Grenze: Adress-ACK bei reinen Schreib-Transaktionen wird optimi-
// stisch (0x18/0x28) gemeldet; das echte NAK-Ergebnis steht erst beim Flush
// (STOP/RepStart) fest. Reine Bus-Scans über Schreibzugriffe erkennen daher
// fehlende Slaves erst verzögert; Lese-basierte Probes funktionieren korrekt.
// =========================================================================
struct I2cModel {
    uint32_t conset, conclr, stat, dat;
    // Bridge-Transaktionszustand
    uint8_t  addr7;       // 7-Bit-Slave-Adresse der aktuellen Transaktion
    bool     is_read;     // Richtung
    bool     started;     // START gesendet, Adresse noch nicht
    bool     addr_done;   // Adressbyte verarbeitet
    bool     any_started; // schon mindestens ein START in dieser Sitzung
    uint8_t  wbuf[64];    // Puffer für Schreibdaten
    uint8_t  wlen;
};
I2cModel g_i2c{};

// CONSET/CONCLR-Bits
constexpr uint32_t I2C_AA   = 0x04u;
constexpr uint32_t I2C_SI   = 0x08u;
constexpr uint32_t I2C_STO  = 0x10u;
constexpr uint32_t I2C_STA  = 0x20u;
constexpr uint32_t I2C_I2EN = 0x40u;

i2c_inst_t* g_i2c_hw    = nullptr;   // nullptr → Bridge inaktiv (Stub)
bool        g_i2c_ready = false;

void i2c_bridge_init() {
    g_i2c_hw    = nullptr;
    g_i2c_ready = false;
    if (!config::i2c_bridge_enabled()) return;
    int sda = config::i2c_bridge_sda_pin();
    int scl = config::i2c_bridge_scl_pin();
    if (sda < 0 || scl < 0) return;
    g_i2c_hw = config::i2c_bridge_instance() ? i2c1 : i2c0;
    i2c_init(g_i2c_hw, config::i2c_bridge_hz());
    gpio_set_function(static_cast<uint>(sda), GPIO_FUNC_I2C);
    gpio_set_function(static_cast<uint>(scl), GPIO_FUNC_I2C);
    gpio_pull_up(static_cast<uint>(sda));
    gpio_pull_up(static_cast<uint>(scl));
    g_i2c_ready = true;
}

// Puffer-Schreibtransaktion auf den Bus schieben. nostop=true hält den Bus
// für einen folgenden (Repeated-START-)Lesezugriff offen.
void i2c_flush_write(bool nostop) {
    if (!g_i2c_ready || g_i2c.wlen == 0) { g_i2c.wlen = 0; return; }
    int r = i2c_write_timeout_us(g_i2c_hw, g_i2c.addr7, g_i2c.wbuf,
                                 g_i2c.wlen, nostop, 5000u * (g_i2c.wlen + 1));
    if (r < 0) g_i2c.stat = 0x30u;  // Datentransfer NAK
    g_i2c.wlen = 0;
}

void i2c_reset_txn() {
    g_i2c.started   = false;
    g_i2c.addr_done = false;
    g_i2c.wlen      = 0;
    g_i2c.stat      = 0xF8u;
}

uint8_t i2c_read_byte(uint32_t addr) {
    uint32_t lane = (addr & 3u) * 8u;
    switch (addr & ~3u) {
        case I2C_CONSET: return static_cast<uint8_t>((g_i2c.conset >> lane) & 0xFFu);
        case I2C_STAT:   return static_cast<uint8_t>((g_i2c.stat   >> lane) & 0xFFu);
        case I2C_DAT: {
            // Lazy-Read: nächstes Byte vom realen Slave holen.
            if (g_i2c_ready && g_i2c.addr_done && g_i2c.is_read) {
                bool last = (g_i2c.conset & I2C_AA) == 0;   // AA=0 → letztes Byte
                uint8_t b = 0;
                int r = i2c_read_timeout_us(g_i2c_hw, g_i2c.addr7, &b, 1,
                                            /*nostop=*/!last, 5000u);
                if (r < 0) {
                    g_i2c.stat = 0x48u;        // (Adress-)NAK beim Empfang
                } else {
                    g_i2c.dat  = b;
                    g_i2c.stat = last ? 0x58u : 0x50u;
                }
            }
            return static_cast<uint8_t>((g_i2c.dat >> lane) & 0xFFu);
        }
        default: return 0;
    }
}

void i2c_write_byte(uint32_t addr, uint8_t val) {
    uint32_t lane = (addr & 3u) * 8u;
    auto patch = [&](uint32_t& v) {
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    uint32_t reg = addr & ~3u;

    if (reg == I2C_CONCLR) {
        g_i2c.conset &= ~(static_cast<uint32_t>(val) << lane);
        return;
    }
    if (reg == I2C_DAT) {
        if (g_i2c.started && !g_i2c.addr_done) {
            // Adressbyte (SLA + R/W)
            g_i2c.addr7     = static_cast<uint8_t>(val >> 1);
            g_i2c.is_read   = (val & 1u) != 0;
            g_i2c.addr_done = true;
            g_i2c.stat      = g_i2c.is_read ? 0x40u : 0x18u;  // SLA+R/W ACK (optimistisch)
        } else if (g_i2c.addr_done && !g_i2c.is_read) {
            // Schreibdaten puffern
            if (g_i2c.wlen < sizeof g_i2c.wbuf) g_i2c.wbuf[g_i2c.wlen++] = static_cast<uint8_t>(val);
            g_i2c.stat = 0x28u;  // Daten gesendet, ACK
        }
        patch(g_i2c.dat);
        if (g_i2c.conset & I2C_I2EN) irq_inject::pend(lpc_irq::I2C0);
        return;
    }
    if (reg != I2C_CONSET) { patch(g_i2c.dat); return; }

    // --- CONSET ---
    patch(g_i2c.conset);

    if (g_i2c.conset & I2C_STO) {
        // STOP: ausstehende Schreibtransaktion abschließen.
        if (g_i2c.addr_done && !g_i2c.is_read) i2c_flush_write(/*nostop=*/false);
        g_i2c.conset &= ~I2C_STO;   // STO ist selbstlöschend
        i2c_reset_txn();
        return;
    }
    if (g_i2c.conset & I2C_STA) {
        // (Wiederholter) START. Bei laufender Schreibtransaktion zuerst den
        // Bus mit nostop=true offenhalten, dann neu adressieren.
        if (g_i2c.addr_done && !g_i2c.is_read && g_i2c.wlen > 0)
            i2c_flush_write(/*nostop=*/true);
        bool repeated = g_i2c.any_started;
        g_i2c.started     = true;
        g_i2c.any_started = true;
        g_i2c.addr_done   = false;
        if (!g_i2c_ready) {
            // Stub: kein Slave → START gemeldet, danach NAK beim Adress-Send.
            g_i2c.stat = 0x20u;
        } else {
            g_i2c.stat = repeated ? 0x10u : 0x08u;
        }
        if (g_i2c.conset & I2C_I2EN) irq_inject::pend(lpc_irq::I2C0);
        return;
    }
}

// =========================================================================
// PMU — RAM-Schatten von 4 GP-Registern + PCON. Power-Down-Modi werden
// auf RP2350-PM (sleep_us mit dormant) abgebildet, siehe pm_handle().
// =========================================================================
uint32_t g_pmu[5]{};   // [0]=PCON, [1..4]=GPREG0..3

void pm_handle();

uint8_t pmu_read_byte(uint32_t addr) {
    uint32_t off  = (addr - PMU_BASE) >> 2;
    uint32_t lane = (addr & 3u) * 8u;
    if (off < 5) return static_cast<uint8_t>((g_pmu[off] >> lane) & 0xFFu);
    return 0;
}

void pmu_write_byte(uint32_t addr, uint8_t val) {
    uint32_t off  = (addr - PMU_BASE) >> 2;
    uint32_t lane = (addr & 3u) * 8u;
    if (off >= 5) return;
    g_pmu[off] = (g_pmu[off] & ~(0xFFu << lane)) |
                 (static_cast<uint32_t>(val) << lane);
    if (off == 0) pm_handle();
}

// PCON.PM[2:0]: 0=sleep, 1=deep-sleep, 2=power-down, 3=deep-power-down.
// RP2350 erlaubt: WFI (sleep), Sleep-Mode (clock-gated), Dormant (DPD-ähnlich).
void pm_handle() {
    uint32_t pm = g_pmu[0] & 0x7u;
    switch (pm) {
        case 0: /* nothing — Guest macht WFI selbst */ break;
        case 1: /* deep-sleep — schon WFI; Marker setzen */ g_pmu[0] |= 0x800u; break;
        case 2: /* power-down — wir stoppen die meisten RP-Clocks erst beim WFI im Guest */
                g_pmu[0] |= 0x800u; break;
        case 3: /* deep-power-down — vom Guest aus nicht im Emulator zulässig */
                g_pmu[0] |= 0x800u; break;
    }
}

// Wird vom CLI/Run-Loop nach einem Guest-WFI aufgerufen, falls PM-Modus
// das verlangt. RP2350: clocks_hw->sleep_en* maskieren und WFI im Host.
// Aufwachen via beliebigem IRQ.
extern "C" void peripherals_lowpower_idle() {
    if ((g_pmu[0] & 0x7u) >= 2) {
        // Tief: Clocks heruntersetzen, dann WFI auf Host-Seite.
        clocks_hw->sleep_en0 = 0;
        clocks_hw->sleep_en1 = 0;
        scb_hw->scr |= M33_SCR_SLEEPDEEP_BITS;
        __asm volatile ("wfi");
        scb_hw->scr &= ~M33_SCR_SLEEPDEEP_BITS;
        clocks_hw->sleep_en0 = ~0u;
        clocks_hw->sleep_en1 = ~0u;
    } else {
        __asm volatile ("wfi");
    }
}

// --- Zustand für generischen MMIO-Schatten, PINT und GINT (siehe unten) ---
struct ShadowEntry { uint32_t addr; uint8_t val; bool used; };
constexpr uint32_t SHADOW_SLOTS = 1024;
ShadowEntry g_mmio_shadow[SHADOW_SLOTS]{};

struct PintModel {
    uint8_t isel, ienr, ienf;            // Konfiguration (1 Bit/Kanal)
    uint8_t rise, fall, ist;             // Status
};
PintModel g_pint{};
uint16_t  g_pint_prev = 0;               // letzter Pegel je Kanal

struct GintModel {
    uint32_t ctrl, pol[2], ena[2];
    uint8_t  irq_num;
    bool     prev_match;
};
GintModel g_gint[2]{};

} // namespace

namespace peripherals {

void init() {
    reset();
    i2c_bridge_init();
    spi_bridge_init();
    adc_bridge_init();
    ct_bridge_init();
}

void i2c_bridge_reinit() { i2c_bridge_init(); }
void spi_bridge_reinit() { spi_bridge_init(); }
void adc_bridge_reinit() { adc_bridge_init(); }
void ct_bridge_reinit()  { ct_bridge_init(); }

void reset() {
    std::memset(g_gpio, 0, sizeof g_gpio);
    std::memset(g_iocon, 0, sizeof g_iocon);
    g_systick_load = g_systick_val = g_systick_ctrl = 0;
    g_systick = {};
    g_systick.last_us = time_us_64();
    g_systick_collector = {0, {0,0,0,0}, 0};
    g_syspllctrl   = 0;
    g_syspllclksel = 0;
    g_mainclksel   = 0;
    g_sysahbclkdiv = 1;
    g_pdruncfg     = 0xFFFF;
    g_bodctrl      = 0;
    g_current_hz   = 12'000'000;
    g_syscon_collector = {0, {0,0,0,0}, 0};
    g_pll_reconfig_pending = false;
    g_stats = {};
    g_uart0 = {};
    for (auto& c : g_ct) c = {};
    g_ct[0].is32 = false; g_ct[0].irq_num = lpc_irq::CT16B0;
    g_ct[1].is32 = false; g_ct[1].irq_num = lpc_irq::CT16B1;
    g_ct[2].is32 = true;  g_ct[2].irq_num = lpc_irq::CT32B0;
    g_ct[3].is32 = true;  g_ct[3].irq_num = lpc_irq::CT32B1;
    for (auto& c : g_ct) {
        c.cap_pin = -1;
        for (auto& mp : c.mat_pin) mp = -1;
        c.pio_handle    = -1;
        c.pio_have_prev = false;
        c.pio_dir_rising = false;   // Idle high → erste Flanke fallend
        for (auto& h : c.tx_handle) h = -1;
    }

    g_wdt = {};
    g_wdt.tc = 0xFF;
    g_wdt.tv = 0xFF;
    g_wdt.wdt_clk_hz = 500'000;     // ≈ WDOSC default
    g_adc = {};
    for (auto& s : g_ssp) s = {};
    g_ssp[0].irq_num = lpc_irq::SSP0;
    g_ssp[1].irq_num = lpc_irq::SSP1;
    g_i2c = {};
    g_i2c.stat = 0xF8u;
    i2c_reset_txn();
    g_i2c.any_started = false;
    std::memset(g_pmu, 0, sizeof g_pmu);
    std::memset(g_pintsel, 0, sizeof g_pintsel);
    std::memset(g_mmio_shadow, 0, sizeof g_mmio_shadow);
    g_pint = {};
    g_pint_prev = 0;
    for (auto& g : g_gint) g = {};
    g_gint[0].irq_num = lpc_irq::GINT0;
    g_gint[1].irq_num = lpc_irq::GINT1;
}

// Bridge zum Emulator: WDT-Reset wird drüben asynchron behandelt.
extern "C" void peripherals_wdt_reset_guest() {
    emulator::request_guest_reset();
}

void on_post_write_hook() {
    if (g_in_post_hook) return;
    g_in_post_hook = true;
    if (g_pll_reconfig_pending) {
        g_pll_reconfig_pending = false;
        uint32_t target = recompute_target_hz();
        retarget_rp2350_clock(target);
    }
    g_in_post_hook = false;
}

uint32_t current_cpu_hz() { return g_current_hz; }

// =========================================================================
// A) Generischer Schatten für NICHT modellierte MMIO-Adressen.
// Unbekannte Schreibzugriffe landen hier statt im Watchdog-Reset; Rücklesen
// bleibt konsistent. Damit kann KEIN Programm mehr an einer nicht
// modellierten Peripherie abstürzen — es verliert dort nur die Funktion.
// Direkt-gemappte Tabelle (Kollision = Overwrite), ausreichend für die
// üblichen Konfig-Register-Rücklesemuster. (Datendefinition siehe oben.)
// =========================================================================
inline uint32_t shadow_slot(uint32_t addr) {
    return (addr * 2654435761u) >> 22;   // 32→10 Bit
}
void shadow_store(uint32_t addr, uint8_t val) {
    ShadowEntry& e = g_mmio_shadow[shadow_slot(addr)];
    e.addr = addr; e.val = val; e.used = true;
}
bool shadow_load(uint32_t addr, uint8_t& out) {
    const ShadowEntry& e = g_mmio_shadow[shadow_slot(addr)];
    if (e.used && e.addr == addr) { out = e.val; return true; }
    return false;
}

// =========================================================================
// GPIO-Eingänge: echte RP2350-Pins lesen (für Input-konfigurierte LPC-Pins).
// Output-Pins kommen weiter aus dem Schatten g_gpio[].data.
// =========================================================================
uint32_t gpio_live_port_data(uint32_t port) {
    if (port >= 4) return 0;
    uint32_t dir  = g_gpio[port].dir;
    uint32_t data = g_gpio[port].data;
    const auto& pm = config::pin_map();
    for (uint8_t pin = 0; pin < 12; ++pin) {
        if ((dir >> pin) & 1u) continue;            // Output → Schatten
        uint8_t lpc = lpc_pin_idx(static_cast<uint8_t>(port), pin);
        if (lpc >= config::LPC_PIN_COUNT) continue;
        int g = pm.lpc_to_rp[lpc];
        if (g < 0) continue;
        if (bridge_owns_gpio(g)) continue;          // ADC/SPI/Capture/Match-Pin
        bool lvl = gpio_get(static_cast<uint>(g));
        data = (data & ~(1u << pin)) | (static_cast<uint32_t>(lvl) << pin);
    }
    return data;
}

// =========================================================================
// PINT (Pin-Interrupts, LPC11Exx-flexint) @ 0x4004C000, 8 Kanäle → IRQ 0..7.
// =========================================================================
constexpr uint32_t PINT_BASE  = 0x4004'C000;
constexpr uint32_t PINT_ISEL  = 0x000;   // 0=edge, 1=level
constexpr uint32_t PINT_IENR  = 0x004;   // enable rising / level
constexpr uint32_t PINT_SIENR = 0x008;   // set IENR (W)
constexpr uint32_t PINT_CIENR = 0x00C;   // clear IENR (W)
constexpr uint32_t PINT_IENF  = 0x010;   // enable falling / level-polarität
constexpr uint32_t PINT_SIENF = 0x014;   // set IENF (W)
constexpr uint32_t PINT_CIENF = 0x018;   // clear IENF (W)
constexpr uint32_t PINT_RISE  = 0x01C;   // rising-edge detect (W1C)
constexpr uint32_t PINT_FALL  = 0x020;   // falling-edge detect (W1C)
constexpr uint32_t PINT_IST   = 0x024;   // interrupt status (W1C)
constexpr uint32_t PINT_END   = PINT_BASE + 0x100;

// =========================================================================
// GINT0/GINT1 (Group-Interrupts) @ 0x4005C000 / 0x40060000 → IRQ 8/9.
// =========================================================================
constexpr uint32_t GINT0_BASE      = 0x4005'C000;
constexpr uint32_t GINT1_BASE      = 0x4006'0000;
constexpr uint32_t GINT_BLOCK      = 0x4000;
constexpr uint32_t GINT_CTRL       = 0x000;   // [0]=INT(W1C) [1]=COMB(0=OR,1=AND) [2]=TRIG
constexpr uint32_t GINT_PORT_POL0  = 0x020;
constexpr uint32_t GINT_PORT_POL1  = 0x024;
constexpr uint32_t GINT_PORT_ENA0  = 0x040;
constexpr uint32_t GINT_PORT_ENA1  = 0x044;

uint32_t gint_idx_for(uint32_t addr) {
    if (addr >= GINT0_BASE && addr < GINT0_BASE + GINT_BLOCK) return 0;
    if (addr >= GINT1_BASE && addr < GINT1_BASE + GINT_BLOCK) return 1;
    return 0xFFFFFFFFu;
}
uint32_t gint_base_for(uint32_t i) { return i ? GINT1_BASE : GINT0_BASE; }

// Wird bei jedem MMIO-Trap aufgerufen: liest echte Eingänge, erkennt Flanken
// und pendet PINT-/GINT-IRQs. Da der Gast nativ ohne Host-Loop läuft, ist der
// MMIO-Trap der einzige synchrone Injektionspunkt — eine reine WFI-Warteschleife
// ganz ohne MMIO-Zugriff lässt sich so nicht wecken (Architektur-Grenze).
void sample_pin_interrupts() {
    uint32_t live[4];
    for (uint32_t p = 0; p < 4; ++p) live[p] = gpio_live_port_data(p);

    // --- Timer-Capture (KNX-Bus-Empfang): Flanken am CAP0-Pin timestampen. ---
    for (auto& c : g_ct) ct_sample_capture(c);

    // --- PINT ---
    uint16_t cur = 0;
    for (uint8_t ch = 0; ch < 8; ++ch) {
        uint8_t  lpc  = g_pintsel[ch];
        uint32_t port = lpc / 12u, pin = lpc % 12u;
        if (port < 4 && ((live[port] >> pin) & 1u)) cur |= (1u << ch);
    }
    uint16_t changed = static_cast<uint16_t>(cur ^ g_pint_prev);
    uint16_t rose = static_cast<uint16_t>(changed &  cur);
    uint16_t fell = static_cast<uint16_t>(changed & ~cur);
    g_pint_prev = cur;

    for (uint8_t ch = 0; ch < 8; ++ch) {
        uint8_t m = static_cast<uint8_t>(1u << ch);
        bool fire = false;
        if ((g_pint.isel & m) == 0u) {                 // Edge-sensitiv
            if ((rose & m) && (g_pint.ienr & m)) { g_pint.rise |= m; fire = true; }
            if ((fell & m) && (g_pint.ienf & m)) { g_pint.fall |= m; fire = true; }
        } else {                                       // Level-sensitiv
            bool active_high = (g_pint.ienf & m);      // IENF wählt Pegel
            bool lvl = (cur >> ch) & 1u;
            if ((g_pint.ienr & m) && (lvl == active_high) && ((g_pint.ist & m) == 0u))
                fire = true;
        }
        if (fire) {
            g_pint.ist |= m;
            irq_inject::pend(static_cast<uint8_t>(lpc_irq::PIN_INT0 + ch));
        }
    }

    // --- GINT0/GINT1 ---
    for (uint32_t gi = 0; gi < 2; ++gi) {
        GintModel& g = g_gint[gi];
        bool comb_and = (g.ctrl & 0x2u);
        bool match = comb_and;          // AND: true-Start, OR: false-Start
        bool any = false;
        for (uint32_t p = 0; p < 2; ++p) {
            uint32_t ena = g.ena[p];
            for (uint8_t pin = 0; pin < 12; ++pin) {
                if (!((ena >> pin) & 1u)) continue;
                any = true;
                bool active = (((live[p] >> pin) & 1u) == ((g.pol[p] >> pin) & 1u));
                if (comb_and) match = match && active;
                else          match = match || active;
            }
        }
        if (!any) match = false;
        if (match && !g.prev_match) {
            g.ctrl |= 0x1u;
            irq_inject::pend(g.irq_num);
        }
        g.prev_match = match;
    }
}

// Treibt die zeitbasierten Modelle (CT16/CT32, WWDT) weiter und pendet
// fällige IRQs. ct_advance/wdt_advance/g_ct/g_wdt liegen im anonymen
// Namespace oben, sind in dieser TU aber sichtbar.
void poll_timed_sources() {
    for (auto& c : g_ct) { ct_advance(c); ct_sample_capture(c); }
    wdt_advance();
    systick_advance();
}

bool capture_armed() {
    // Nur Software-Capture braucht enges Polling. PIO-Capture puffert Flanken
    // in der FIFO und braucht kein tight-loop → erlaubt die 50-µs-Pause.
    for (auto& c : g_ct)
        if (c.cap_pin >= 0 && (c.ccr & 0x3u) && c.pio_handle < 0) return true;
    return false;
}

uint8_t pint_read_byte(uint32_t off) {
    uint32_t lane = (off & 3u) * 8u;
    switch (off & ~3u) {
        case PINT_ISEL: return static_cast<uint8_t>((g_pint.isel >> lane) & 0xFFu);
        case PINT_IENR: case PINT_SIENR: case PINT_CIENR:
            return static_cast<uint8_t>((g_pint.ienr >> lane) & 0xFFu);
        case PINT_IENF: case PINT_SIENF: case PINT_CIENF:
            return static_cast<uint8_t>((g_pint.ienf >> lane) & 0xFFu);
        case PINT_RISE: return static_cast<uint8_t>((g_pint.rise >> lane) & 0xFFu);
        case PINT_FALL: return static_cast<uint8_t>((g_pint.fall >> lane) & 0xFFu);
        case PINT_IST:  return static_cast<uint8_t>((g_pint.ist  >> lane) & 0xFFu);
        default: return 0;
    }
}

void pint_write_byte(uint32_t off, uint8_t val) {
    if ((off & 3u) != 0u) return;        // nur Byte0-Lane relevant (8 Kanäle)
    switch (off & ~3u) {
        case PINT_ISEL:  g_pint.isel = val; break;
        case PINT_IENR:  g_pint.ienr = val; break;
        case PINT_SIENR: g_pint.ienr |= val; break;
        case PINT_CIENR: g_pint.ienr &= static_cast<uint8_t>(~val); break;
        case PINT_IENF:  g_pint.ienf = val; break;
        case PINT_SIENF: g_pint.ienf |= val; break;
        case PINT_CIENF: g_pint.ienf &= static_cast<uint8_t>(~val); break;
        case PINT_RISE:  g_pint.rise &= static_cast<uint8_t>(~val); break;  // W1C
        case PINT_FALL:  g_pint.fall &= static_cast<uint8_t>(~val); break;  // W1C
        case PINT_IST:   g_pint.ist  &= static_cast<uint8_t>(~val); break;  // W1C
        default: break;
    }
}

uint8_t gint_read_byte(uint32_t idx, uint32_t off) {
    GintModel& g = g_gint[idx];
    uint32_t lane = (off & 3u) * 8u;
    switch (off & ~3u) {
        case GINT_CTRL:      return static_cast<uint8_t>((g.ctrl   >> lane) & 0xFFu);
        case GINT_PORT_POL0: return static_cast<uint8_t>((g.pol[0] >> lane) & 0xFFu);
        case GINT_PORT_POL1: return static_cast<uint8_t>((g.pol[1] >> lane) & 0xFFu);
        case GINT_PORT_ENA0: return static_cast<uint8_t>((g.ena[0] >> lane) & 0xFFu);
        case GINT_PORT_ENA1: return static_cast<uint8_t>((g.ena[1] >> lane) & 0xFFu);
        default: return 0;
    }
}

void gint_write_byte(uint32_t idx, uint32_t off, uint8_t val) {
    GintModel& g = g_gint[idx];
    uint32_t lane = (off & 3u) * 8u;
    auto patch = [&](uint32_t& v) {
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    switch (off & ~3u) {
        case GINT_CTRL:
            // Bit0 = INT, write-1-to-clear; Bits1..2 normal beschreibbar.
            if (lane == 0) {
                if (val & 0x1u) g.ctrl &= ~0x1u;
                g.ctrl = (g.ctrl & ~0x6u) | (val & 0x6u);
            }
            break;
        case GINT_PORT_POL0: patch(g.pol[0]); break;
        case GINT_PORT_POL1: patch(g.pol[1]); break;
        case GINT_PORT_ENA0: patch(g.ena[0]); break;
        case GINT_PORT_ENA1: patch(g.ena[1]); break;
        default: break;
    }
}

bool mmio_read8(uint32_t addr, uint8_t& out) {
    ++g_stats.mmio_reads;
    sample_pin_interrupts();

    if (addr >= GPIO_BASE && addr < GPIO_PORTS_END) {
        uint32_t port_off = addr - GPIO_BASE;
        uint32_t port     = port_off / GPIO_PORT_STRIDE;
        uint32_t local    = port_off % GPIO_PORT_STRIDE;
        if (port < 4) {
            if (local < GPIO_DATA_END) {
                // Maskierter DATA-Read: Bits[11:2] der Adresse = Pin-Maske.
                // Eingänge kommen live von echten RP2350-Pins.
                uint32_t mask = (local >> 2) & 0xFFFu;
                uint32_t data = gpio_live_port_data(port) & mask;
                out = static_cast<uint8_t>((data >> ((addr & 3u) * 8u)) & 0xFFu);
                return true;
            }
            if (local >= GPIO_DIR_OFFSET && local < GPIO_DIR_OFFSET + 4) {
                out = static_cast<uint8_t>((g_gpio[port].dir >> ((addr & 3u) * 8u)) & 0xFFu);
                return true;
            }
        }
        out = 0; return true;
    }
    if (addr >= IOCON_BASE && addr < IOCON_END) {
        out = g_iocon[addr - IOCON_BASE]; return true;
    }
    if (addr >= SYSCON_BASE && addr < SYSCON_BASE + 0x300) {
        uint32_t aligned = addr & ~3u;
        uint32_t v = syscon_read32(aligned);
        out = static_cast<uint8_t>((v >> ((addr & 3u) * 8)) & 0xFFu);
        return true;
    }
    // SysTick + ICTR/ACTLR-Block (0xE000E000-0xE000E01F): von der MPU getrappt,
    // da SysTick auf dem M33 privilegiert-only ist. Emuliert (systick_read32).
    if (addr >= 0xE000'E000u && addr <= 0xE000'E01Fu) {
        uint32_t v = systick_read32(addr & ~3u);
        out = static_cast<uint8_t>((v >> ((addr & 3u) * 8u)) & 0xFFu);
        return true;
    }
    if (addr >= UART0_BASE && addr < UART0_END) {
        if ((addr & 3u) == 0u) out = uart0_read_reg(addr & ~3u);
        else                   out = 0;
        return true;
    }
    {
        uint32_t idx = ct_idx_for(addr);
        if (idx < 4) {
            uint32_t off = addr - ct_base_for(idx);
            out = ct_read_byte(idx, off);
            return true;
        }
    }
    if (addr >= WDT_BASE && addr < WDT_END) {
        out = wdt_read_byte(addr); return true;
    }
    if (addr >= ADC_BASE && addr < ADC_END) {
        out = adc_read_byte(addr); return true;
    }
    {
        uint32_t idx = ssp_idx_for(addr);
        if (idx < 2) {
            out = ssp_read_byte(idx, addr - ssp_base_for(idx));
            return true;
        }
    }
    if (addr >= I2C_BASE && addr < I2C_END) {
        out = i2c_read_byte(addr); return true;
    }
    if (addr >= PMU_BASE && addr < PMU_END) {
        out = pmu_read_byte(addr); return true;
    }
    if (addr >= PINT_BASE && addr < PINT_END) {
        out = pint_read_byte(addr - PINT_BASE); return true;
    }
    {
        uint32_t gi = gint_idx_for(addr);
        if (gi < 2) { out = gint_read_byte(gi, addr - gint_base_for(gi)); return true; }
    }

    // NVIC-Region wird via vnvic getrappt (eigene MPU-Region) — sollte hier
    // nicht ankommen, ist aber als Fallback definiert.
    if (vnvic::is_nvic_addr(addr)) {
        out = vnvic::read8(addr); return true;
    }

    // Unbekannte Adressen: zuletzt geschriebenen Schatten-Wert liefern, sonst
    // 0 — damit Polling-Loops nicht hängen bleiben.
    if (shadow_load(addr, out)) return true;
    out = 0;
    return true;
}

bool mmio_write8(uint32_t addr, uint8_t val) {
    ++g_stats.mmio_writes;
    sample_pin_interrupts();

    if (addr >= GPIO_BASE && addr < GPIO_PORTS_END) {
        uint32_t port_off = addr - GPIO_BASE;
        uint32_t port     = port_off / GPIO_PORT_STRIDE;
        uint32_t local    = port_off % GPIO_PORT_STRIDE;
        if (port < 4) {
            if (local < GPIO_DATA_END) {
                // Maskierter DATA-Write: schreibt nur Pins, deren Mask-Bit gesetzt ist.
                uint32_t mask  = (local >> 2) & 0xFFFu;
                uint32_t shift = (addr & 3u) * 8u;
                uint32_t lane  = (static_cast<uint32_t>(val) << shift);
                uint32_t old   = g_gpio[port].data;
                g_gpio[port].data = (old & ~(mask << 0)) |
                                    ((lane & (mask << 0)) /* lower bits */) |
                                    (old & ~mask);
                // Vereinfachung: für maskierten Schreibzugriff nur die in `mask`
                // gesetzten Bits aus `val` übernehmen.
                g_gpio[port].data = (old & ~mask) | (lane & mask);
                ++g_stats.gpio_writes;
                gpio_apply_port(static_cast<uint8_t>(port), old,
                                g_gpio[port].data, g_gpio[port].dir);
                return true;
            }
            if (local >= GPIO_DIR_OFFSET && local < GPIO_DIR_OFFSET + 4) {
                uint32_t shift = (addr & 3u) * 8u;
                g_gpio[port].dir = (g_gpio[port].dir & ~(0xFFu << shift)) |
                                   (static_cast<uint32_t>(val) << shift);
                gpio_apply_port(static_cast<uint8_t>(port), g_gpio[port].data,
                                g_gpio[port].data, g_gpio[port].dir);
                return true;
            }
        }
        return true;  // andere GPIO-Subregister still akzeptieren
    }
    if (addr >= IOCON_BASE && addr < IOCON_END) {
        uint32_t off = addr - IOCON_BASE;
        g_iocon[off] = val;
        // MODE-Bits (Pull-up/-down) liegen in Byte 0 des 32-bit-Registers.
        // Nach jedem Byte-Write den Pull des zugehoerigen Pins nachziehen.
        apply_iocon_pull(off & ~3u);
        return true;
    }
    if (addr >= SYSCON_BASE && addr < SYSCON_BASE + 0x300) {
        syscon_collect_byte(addr, val);
        return true;
    }
    // SysTick + ICTR/ACTLR-Block (0xE000E000-0xE000E01F): byteweise sammeln und
    // beim vollstaendigen 32-bit-Wort emulieren (systick_write32). Der Gast
    // schreibt via STR (4x mmio_write8); die Register-Semantik (CSR/RVR/CVR)
    // ist nur wortweise sinnvoll.
    if (addr >= 0xE000'E000u && addr <= 0xE000'E01Fu) {
        systick_collect_byte(addr, val);
        return true;
    }
    if (addr >= UART0_BASE && addr < UART0_END) {
        if ((addr & 3u) == 0u) uart0_write_reg(addr & ~3u, val);
        return true;
    }
    {
        uint32_t idx = ct_idx_for(addr);
        if (idx < 4) {
            uint32_t off = addr - ct_base_for(idx);
            ct_write_byte(idx, off, val);
            return true;
        }
    }
    if (addr >= WDT_BASE && addr < WDT_END) {
        wdt_write_byte(addr, val); return true;
    }
    if (addr >= ADC_BASE && addr < ADC_END) {
        adc_write_byte(addr, val); return true;
    }
    {
        uint32_t idx = ssp_idx_for(addr);
        if (idx < 2) {
            ssp_write_byte(idx, addr - ssp_base_for(idx), val);
            return true;
        }
    }
    if (addr >= I2C_BASE && addr < I2C_END) {
        i2c_write_byte(addr, val); return true;
    }
    if (addr >= PMU_BASE && addr < PMU_END) {
        pmu_write_byte(addr, val); return true;
    }
    if (addr >= PINT_BASE && addr < PINT_END) {
        pint_write_byte(addr - PINT_BASE, val); return true;
    }
    {
        uint32_t gi = gint_idx_for(addr);
        if (gi < 2) { gint_write_byte(gi, addr - gint_base_for(gi), val); return true; }
    }

    if (vnvic::is_nvic_addr(addr)) {
        ++g_stats.nvic_writes;
        vnvic::write8(addr, val);
        return true;
    }

    // Nicht modellierte Peripherie: in den generischen Schatten schreiben statt
    // Watchdog-Reset. Kein Programm stürzt mehr an unbekannten Registern ab.
    shadow_store(addr, val);
    return true;
}

Stats stats() { return g_stats; }

void systick_debug(uint32_t& reads, uint32_t& writes,
                   uint32_t& csr, uint32_t& rvr, uint32_t& cvr,
                   uint64_t& ticks) {
    reads  = g_systick.trap_reads;
    writes = g_systick.trap_writes;
    csr    = g_systick.csr;
    rvr    = g_systick.rvr;
    cvr    = g_systick.cvr;
    ticks  = g_systick.irq_ticks;
}

bool guest_output_level(uint8_t port, uint8_t pin, bool& level) {
    if (port >= 4u || pin >= 12u) return false;
    if (((g_gpio[port].dir >> pin) & 1u) == 0u) return false;   // nicht als Ausgang
    level = ((g_gpio[port].data >> pin) & 1u) != 0u;
    return true;
}

} // namespace peripherals
