#include "peripherals.h"
#include "config.h"
#include "vnvic.h"
#include "irq_inject.h"
#include "lpc_irqs.h"
#include "emulator.h"

#include <cstring>
#include <cstdio>

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/scb.h"
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
constexpr uint32_t SYSPLLCTRL         = SYSCON_BASE + 0x008;
constexpr uint32_t SYSPLLSTAT         = SYSCON_BASE + 0x00C;
constexpr uint32_t SYSOSCCTRL         = SYSCON_BASE + 0x020;
constexpr uint32_t WDTOSCCTRL         = SYSCON_BASE + 0x024;
constexpr uint32_t IRCCTRL            = SYSCON_BASE + 0x028;
constexpr uint32_t SYSPLLCLKSEL       = SYSCON_BASE + 0x040;
constexpr uint32_t SYSPLLCLKUEN       = SYSCON_BASE + 0x044;
constexpr uint32_t MAINCLKSEL         = SYSCON_BASE + 0x070;
constexpr uint32_t MAINCLKUEN         = SYSCON_BASE + 0x074;
constexpr uint32_t SYSAHBCLKDIV       = SYSCON_BASE + 0x078;
constexpr uint32_t SYSAHBCLKCTRL      = SYSCON_BASE + 0x080;
constexpr uint32_t PDRUNCFG           = SYSCON_BASE + 0x238;

// IOCON Block bei 0x40044000 — Pinmux. Modellieren wir als RAM (kein Effekt).
constexpr uint32_t IOCON_BASE         = 0x4004'4000;
constexpr uint32_t IOCON_END          = 0x4004'4100;

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

void gpio_apply_port(uint8_t port, uint32_t /*old_data*/, uint32_t new_data, uint32_t dir) {
    for (uint8_t pin = 0; pin < 12; ++pin) {
        bool out = (dir >> pin) & 1u;
        bool lvl = (new_data >> pin) & 1u;
        apply_gpio_to_hw(lpc_pin_idx(port, pin), out, lvl);
    }
}
uint32_t        g_systick_load = 0, g_systick_val = 0, g_systick_ctrl = 0;

// SYSCON-Schattenregister
uint32_t        g_syspllctrl   = 0;       // MSEL[4:0], PSEL[6:5]
uint32_t        g_syspllclksel = 0;       // 0 = IRC, 1 = SYSOSC
uint32_t        g_mainclksel   = 0;       // 0 = IRC, 2 = SYSPLLOUT
uint32_t        g_sysahbclkdiv = 1;
uint32_t        g_pdruncfg     = 0xFFFF;
uint8_t         g_iocon[256]{};
uint32_t        g_current_hz   = 12'000'000; // Default IRC

peripherals::Stats g_stats{};
bool            g_in_post_hook = false;
bool            g_pll_reconfig_pending = false;
void apply_gpio_to_hw(uint8_t lpc_pin, bool out, bool level) {
    auto pm = config::pin_map();
    if (lpc_pin >= config::LPC_PIN_COUNT) return;
    int g = pm.lpc_to_rp[lpc_pin];
    if (g < 0) return;
    gpio_init(static_cast<uint>(g));
    gpio_set_dir(static_cast<uint>(g), out);
    if (out) gpio_put(static_cast<uint>(g), level);
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

// Setzt RP2350 auf eine möglichst nahe Frequenz an die LPC-Soll-Frequenz.
void retarget_rp2350_clock(uint32_t target_hz) {
    if (target_hz < 12'000'000u || target_hz > 150'000'000u) return;
    if (target_hz == g_current_hz)                            return;

    // RP2350 set_sys_clock_khz akzeptiert "geeignete" Frequenzen; bei nicht
    // einstellbaren Werten gibt false zurück. Wir runden auf 1 MHz.
    uint32_t khz = (target_hz + 500u) / 1000u;
    if (set_sys_clock_khz(khz, false)) {
        g_current_hz = target_hz;
        ++g_stats.pll_reconfigs;
        std::printf("[CLK] retarget RP2350 -> %lu kHz\n",
                    static_cast<unsigned long>(khz));
    } else {
        std::printf("[CLK] set_sys_clock_khz(%lu) abgelehnt; bleibe bei %lu Hz\n",
                    static_cast<unsigned long>(khz),
                    static_cast<unsigned long>(g_current_hz));
    }
}

// SYSCON 32-Bit-Schreibzugriff (kommt in 4 Byte-Schritten an). Wir halten
// den Wert in einem RAM-Schatten, und merken uns für PLL-relevante Register
// dass eine Re-Konfiguration anliegt — die wird einmalig im Post-Hook nach
// dem letzten Byte ausgeführt, damit wir nicht 4× neu takten.
void syscon_write32(uint32_t addr, uint32_t value) {
    switch (addr) {
        case SYSPLLCTRL:    g_syspllctrl   = value & 0x7Fu; g_pll_reconfig_pending = true; break;
        case SYSPLLCLKSEL:  g_syspllclksel = value & 0x3u;  g_pll_reconfig_pending = true; break;
        case MAINCLKSEL:    g_mainclksel   = value & 0x3u;  g_pll_reconfig_pending = true; break;
        case SYSAHBCLKDIV:  g_sysahbclkdiv = value & 0xFFu; g_pll_reconfig_pending = true; break;
        case PDRUNCFG:      g_pdruncfg     = value;                                           break;
        case SYSPLLCLKUEN:
        case MAINCLKUEN:
            if (value & 1u) g_pll_reconfig_pending = true;
            break;
        default: break;
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
        default: return 0;
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
// =========================================================================
struct CtModel {
    bool     enabled;
    uint32_t pre;
    uint32_t pc;
    uint32_t tc;
    uint32_t mr[4];
    uint32_t mcr;
    uint32_t ir;
    bool     is32;
    uint8_t  irq_num;
    uint64_t last_us;
};
CtModel g_ct[4];

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
    if (!c.enabled) { c.last_us = time_us_64(); return; }
    uint64_t now = time_us_64();
    uint64_t dt  = now - c.last_us;
    c.last_us = now;
    uint64_t ticks = (dt * static_cast<uint64_t>(g_current_hz))
                     / (1'000'000ull * static_cast<uint64_t>(c.pre + 1u));
    if (!ticks) return;
    uint32_t mask = c.is32 ? 0xFFFF'FFFFu : 0xFFFFu;
    for (uint64_t i = 0; i < ticks; ++i) {
        c.tc = (c.tc + 1u) & mask;
        for (int m = 0; m < 4; ++m) {
            if (c.tc == c.mr[m]) {
                uint32_t mcr = (c.mcr >> (m * 3)) & 0x7u;
                if (mcr & 0x1) { c.ir |= (1u << m); irq_inject::pend(c.irq_num); }
                if (mcr & 0x2) c.tc = 0;
                if (mcr & 0x4) c.enabled = false;
            }
        }
    }
}

uint8_t ct_read_byte(uint32_t idx, uint32_t off) {
    CtModel& c = g_ct[idx];
    ct_advance(c);
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
        case 0x18: patch(c.mr[0]); break;
        case 0x1C: patch(c.mr[1]); break;
        case 0x20: patch(c.mr[2]); break;
        case 0x24: patch(c.mr[3]); break;
        default: break;
    }
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
// ADC — minimaler Modell. Wandlung gilt sofort als fertig, Ergebnis = 0.
// Reicht für Selfbus-Apps, die ADC nicht zwingend benötigen.
// =========================================================================
struct AdcModel {
    uint32_t cr;
    uint32_t inten;
    uint32_t dr[8];   // ADC-Channel-Result-Register
    uint32_t gdr;
    uint32_t stat;
};
AdcModel g_adc{};

void adc_simulate() {
    // Wenn START != 0 oder BURST gesetzt → Conversion sofort fertig.
    bool start = ((g_adc.cr >> 24) & 0x7u) != 0;
    bool burst = ((g_adc.cr >> 16) & 0x1u) != 0;
    if (!start && !burst) return;
    uint32_t sel = g_adc.cr & 0xFFu;
    for (int ch = 0; ch < 8; ++ch) {
        if (sel & (1u << ch)) {
            // 10-bit "Sample" = halber Skalenwert; deterministisch.
            uint16_t sample = 0x200;
            g_adc.dr[ch] = (1u << 31) | (1u << 30) |     // DONE, OVERRUN=0 (also 1u<<30 ist OVERRUN; here clear)
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
// SSP0/SSP1 — minimale Modelle. RX = TX (Loopback) wenn keine HW gebunden,
// das reicht für Selfbus-Apps, die SSP zur SPI-Datenausgabe benutzen, weil
// der Empfangs-Pfad der App egal ist.
// =========================================================================
struct SspModel {
    uint32_t cr0, cr1, cpsr, imsc, ris, dr_rx;
    uint8_t  irq_num;
};
SspModel g_ssp[2]{};

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
        case SSP_SR:   return static_cast<uint8_t>((0x03u) & 0xFFu);  // TFE+TNF
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
        case SSP_CR0:  patch(s.cr0);  break;
        case SSP_CR1:  patch(s.cr1);  break;
        case SSP_DR:   {
            // Loopback: was gesendet wird, kommt zurück.
            patch(s.dr_rx);
            s.ris |= 0x4u;     // RX-FIFO not empty
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
// I²C0 — sehr minimaler Modell. Liefert STAT=0xF8 (idle) und NAK auf jeden
// Adress-Send. Reicht damit die meisten sblib-Apps booten, auch wenn kein
// I²C-Slave angeschlossen ist.
// =========================================================================
struct I2cModel {
    uint32_t conset, conclr, stat, dat;
};
I2cModel g_i2c{};

uint8_t i2c_read_byte(uint32_t addr) {
    uint32_t lane = (addr & 3u) * 8u;
    switch (addr & ~3u) {
        case I2C_CONSET: return static_cast<uint8_t>((g_i2c.conset >> lane) & 0xFFu);
        case I2C_STAT:   return static_cast<uint8_t>((g_i2c.stat   >> lane) & 0xFFu);
        case I2C_DAT:    return static_cast<uint8_t>((g_i2c.dat    >> lane) & 0xFFu);
        default: return 0;
    }
}

void i2c_write_byte(uint32_t addr, uint8_t val) {
    uint32_t lane = (addr & 3u) * 8u;
    auto patch = [&](uint32_t& v) {
        v = (v & ~(0xFFu << lane)) | (static_cast<uint32_t>(val) << lane);
    };
    switch (addr & ~3u) {
        case I2C_CONSET: patch(g_i2c.conset); break;
        case I2C_CONCLR: g_i2c.conset &= ~(static_cast<uint32_t>(val) << lane); break;
        case I2C_DAT:    patch(g_i2c.dat);    break;
        default: break;
    }
    // STA gesetzt → "START gesendet, kein Slave da" → NAK-Status 0x20.
    if (g_i2c.conset & 0x20u) {
        g_i2c.stat = 0x20u;
        if (g_i2c.conset & 0x40u) irq_inject::pend(lpc_irq::I2C0);
    } else {
        g_i2c.stat = 0xF8u;  // idle
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

} // namespace

namespace peripherals {

void init() { reset(); }

void reset() {
    std::memset(g_gpio, 0, sizeof g_gpio);
    std::memset(g_iocon, 0, sizeof g_iocon);
    g_systick_load = g_systick_val = g_systick_ctrl = 0;
    g_syspllctrl   = 0;
    g_syspllclksel = 0;
    g_mainclksel   = 0;
    g_sysahbclkdiv = 1;
    g_pdruncfg     = 0xFFFF;
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
    std::memset(g_pmu, 0, sizeof g_pmu);
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

bool mmio_read8(uint32_t addr, uint8_t& out) {
    ++g_stats.mmio_reads;

    if (addr >= GPIO_BASE && addr < GPIO_PORTS_END) {
        uint32_t port_off = addr - GPIO_BASE;
        uint32_t port     = port_off / GPIO_PORT_STRIDE;
        uint32_t local    = port_off % GPIO_PORT_STRIDE;
        if (port < 4) {
            if (local < GPIO_DATA_END) {
                // Maskierter DATA-Read: Bits[11:2] der Adresse = Pin-Maske.
                uint32_t mask = (local >> 2) & 0xFFFu;
                uint32_t data = g_gpio[port].data & mask;
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
    if (addr >= SYSTICK_CTRL && addr < SYSTICK_CTRL + 4) {
        out = static_cast<uint8_t>((g_systick_ctrl >> ((addr - SYSTICK_CTRL)*8)) & 0xFFu);
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

    // NVIC-Region wird via vnvic getrappt (eigene MPU-Region) — sollte hier
    // nicht ankommen, ist aber als Fallback definiert.
    if (vnvic::is_nvic_addr(addr)) {
        out = vnvic::read8(addr); return true;
    }

    // Unbekannte Adressen: 0 zurückliefern, damit Polling-Loops nicht
    // hängen bleiben. Schreiben bleibt strikt.
    out = 0;
    return true;
}

bool mmio_write8(uint32_t addr, uint8_t val) {
    ++g_stats.mmio_writes;

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
        g_iocon[addr - IOCON_BASE] = val; return true;
    }
    if (addr >= SYSCON_BASE && addr < SYSCON_BASE + 0x300) {
        syscon_collect_byte(addr, val);
        return true;
    }
    if (addr == SYSTICK_CTRL) { g_systick_ctrl = val; return true; }
    if (addr == SYSTICK_LOAD) { g_systick_load = val; return true; }
    if (addr == SYSTICK_VAL ) { g_systick_val  = 0;   return true; }
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

    if (vnvic::is_nvic_addr(addr)) {
        ++g_stats.nvic_writes;
        vnvic::write8(addr, val);
        return true;
    }

    return false;
}

Stats stats() { return g_stats; }

} // namespace peripherals
