#include "swd_target.h"
#include "target_halt.h"
#include "emulator.h"

#include <atomic>
#include <cstdio>
#include <cstring>

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "RP2350.h"

// =============================================================================
// PIO-PHY: Sample SWDIO auf jeder steigenden SWCLK-Flanke. CPU drainiert
// das RX-FIFO bitweise und treibt SWDIO bei fallender Flanke aus.
//
// PIO-Programm (5 Instruktionen):
//
//   .program swd_rx
//   .wrap_target
//       wait 1 pin 1     ; SWCLK rising edge (in_pins[1])
//       in   pins, 1     ; sample SWDIO (in_pins[0]) → ISR (autopush at 1)
//       wait 0 pin 1     ; SWCLK falling edge
//   .wrap
//
// Mit autopush=1 erscheint jedes Bit als 32-bit-Wort (LSB) in der RX FIFO.
// Wir nutzen 8-bit autopush für effizientes Byte-Empfangen.
// =============================================================================

namespace swd_target {
namespace {

// PIO-Maschinencode (von Hand assembliert — pioasm-Output):
//   0: 2021 wait 1 pin, 1
//   1: 4001 in pins, 1
//   2: 2001 wait 0 pin, 1
constexpr uint16_t swd_rx_instructions[] = {
    0x2021,   // wait 1 pin 1
    0x4001,   // in pins, 1
    0x2001,   // wait 0 pin 1
};

const struct pio_program swd_rx_program = {
    .instructions = swd_rx_instructions,
    .length       = 3,
    .origin       = -1,
};

// =============================================================================
// PIO-Programm (TX): Voll-PIO Treiber für SWDIO.
//   .program swd_tx
//   .wrap_target
//       wait 0 pin 1     ; SWCLK falling edge → Bit setzen
//       out  pins, 1     ; OSR-LSB → SWDIO
//       wait 1 pin 1     ; SWCLK rising edge — Host sampled, halten
//   .wrap
// =============================================================================
constexpr uint16_t swd_tx_instructions[] = {
    0x2001,   // wait 0 pin 1
    0x6001,   // out pins, 1
    0x2021,   // wait 1 pin 1
};

const struct pio_program swd_tx_program = {
    .instructions = swd_tx_instructions,
    .length       = 3,
    .origin       = -1,
};

// =============================================================================
// SWD-Protokoll-Konstanten
// =============================================================================
constexpr uint8_t  ACK_OK    = 0x1;   // 001
constexpr uint8_t  ACK_WAIT  = 0x2;   // 010
constexpr uint8_t  ACK_FAULT = 0x4;   // 100

// Header-Bits (LSB zuerst gesendet):
//   start(1) APnDP RnW A2 A3 parity stop(0) park(1)
constexpr uint8_t  HEADER_START_MASK = 0x01;
constexpr uint8_t  HEADER_PARK_MASK  = 0x80;

// =============================================================================
// DP/AP-Registerstand
// =============================================================================
struct {
    uint32_t ctrl_stat;     // ORUNDETECT, STICKYORUN, TRNMODE, STICKYCMP, STICKYERR, READOK, WDATAERR, MASKLANE, TRNCNT, CDBGRSTREQ/ACK, CDBGPWRUPREQ/ACK, CSYSPWRUPREQ/ACK
    uint32_t select;        // APBANKSEL[7:4], DPBANKSEL[3:0], APSEL[31:24]
    uint32_t rdbuff;
    uint32_t target_id;
} g_dp{};

struct {
    uint32_t csw;
    uint32_t tar;
    uint32_t drw_pending;
    bool     drw_dirty;
} g_ahb{};

std::atomic<bool> g_active{false};
PinAssignment    g_pins{-1, -1};

PIO     g_pio = pio0;
int     g_sm  = -1;       // RX-SM
int     g_off = -1;
int     g_sm_tx = -1;     // TX-SM (Voll-PIO-TX)
int     g_off_tx = -1;

Stats   g_stats{};

// SWD-Synchronstatus
enum class LineState { Idle, AfterReset };
LineState g_line = LineState::Idle;

// =============================================================================
// CoreSight-ROM-Table für LPC1115 (Cortex-M0)
//
// Layout: 32-bit-Einträge, jeder zeigt relativ auf eine Komponente. Bit 0 =
// PRESENT. Komponenten-IDs entsprechen denen, die OpenOCD/J-Link erwarten,
// um den Chip als Cortex-M0 zu erkennen.
// =============================================================================
struct RomEntry {
    uint32_t addr;
    uint32_t value;
};

// SCS @ 0xE000_E000 (System Control Space): rel = -0x100000 = 0xFFF0_0000
// DWT @ 0xE000_1000: rel = -0xF000 = 0xFFFF_F000
// FPB @ 0xE000_2000: rel = -0xE000 = 0xFFFF_F000 (...)
//
// Für die ROM-Table genügt es, die wichtigsten 4 Einträge + Terminator zu
// liefern, die OpenOCD prüft (CIDR/PIDR der Komponenten).
constexpr uint32_t ROM_END = ROM_TABLE_ADDR + 0x1000;

uint32_t rom_word(uint32_t addr) {
    uint32_t off = addr - ROM_TABLE_ADDR;
    switch (off) {
        // ROM-Table-Entries (jeweils 4 byte):
        case 0x000: return 0xFFF0'F003u;  // SCS    (rel −0x000F'1000, present)
        case 0x004: return 0xFFF0'2003u;  // DWT
        case 0x008: return 0xFFF0'3003u;  // FPB
        case 0x00C: return 0x0000'0000u;  // Terminator
        // Component-ID / Peripheral-ID der ROM-Table selbst:
        case 0xFD0: return 0x0000'000Du;  // PIDR4 (4KB block)
        case 0xFD4: return 0x0000'0000u;  // PIDR5
        case 0xFD8: return 0x0000'0000u;  // PIDR6
        case 0xFDC: return 0x0000'0000u;  // PIDR7
        case 0xFE0: return 0x0000'0011u;  // PIDR0 (Part = 0x411 LPC1115)
        case 0xFE4: return 0x0000'00B4u;  // PIDR1
        case 0xFE8: return 0x0000'000Bu;  // PIDR2 (designer = ARM)
        case 0xFEC: return 0x0000'0000u;  // PIDR3
        case 0xFF0: return 0x0000'000Du;  // CIDR0
        case 0xFF4: return 0x0000'0010u;  // CIDR1 (class = ROM-Table)
        case 0xFF8: return 0x0000'0005u;  // CIDR2
        case 0xFFC: return 0x0000'00B1u;  // CIDR3
        default:    return 0;
    }
}

// =============================================================================
// Zugriff auf den emulierten Adressraum
// =============================================================================
uint32_t mem_read32(uint32_t lpc_addr) {
    if (lpc_addr >= ROM_TABLE_ADDR && lpc_addr < ROM_END) {
        return rom_word(lpc_addr);
    }
    // DHCSR/DCRSR/DCRDR/DEMCR werden separat behandelt — siehe ahb_*().
    uint32_t v = 0;
    target_halt::read_memory(lpc_addr, &v, 4);
    ++g_stats.mem_reads;
    return v;
}

void mem_write32(uint32_t lpc_addr, uint32_t v) {
    if (lpc_addr >= ROM_TABLE_ADDR && lpc_addr < ROM_END) return;
    target_halt::write_memory(lpc_addr, &v, 4);
    ++g_stats.mem_writes;
}

// =============================================================================
// Cortex-M-Debug-Schnittstelle (DCB)
// =============================================================================
constexpr uint32_t DHCSR = 0xE000'EDF0;   // Debug Halting Control & Status
constexpr uint32_t DCRSR = 0xE000'EDF4;   // Debug Core Register Selector
constexpr uint32_t DCRDR = 0xE000'EDF8;   // Debug Core Register Data
constexpr uint32_t DEMCR = 0xE000'EDFC;   // Debug Exception & Monitor Control
constexpr uint32_t FP_CTRL  = 0xE000'2000;
constexpr uint32_t FP_COMP0 = 0xE000'2008;

uint32_t g_dcrsr = 0, g_dcrdr = 0, g_demcr = 0;
uint32_t g_fp_ctrl = 0;
uint32_t g_fp_comp[8]{};

uint32_t dcb_read(uint32_t addr) {
    switch (addr) {
        case DHCSR: {
            // S_REGRDY=Bit16, S_HALT=17, S_SLEEP=18, S_LOCKUP=19,
            // S_RETIRE_ST=24, C_DEBUGEN=0, C_HALT=1, C_STEP=2.
            uint32_t v = 0x0103'0000u;        // DBGKEY upper read = 0xA05F (irrelevant)
            if (target_halt::is_halted())     v |= (1u << 17) | (1u << 0);
            return v;
        }
        case DCRSR:  return g_dcrsr;
        case DCRDR:  return g_dcrdr;
        case DEMCR:  return g_demcr;
        case FP_CTRL: return g_fp_ctrl | (8u << 4);   // 8 comparators
        default:
            if (addr >= FP_COMP0 && addr < FP_COMP0 + 32) {
                return g_fp_comp[(addr - FP_COMP0) / 4];
            }
            return mem_read32(addr);
    }
}

void dcb_write(uint32_t addr, uint32_t v) {
    switch (addr) {
        case DHCSR: {
            // DBGKEY-Match prüfen: oberen 16 Bit müssen 0xA05F sein.
            if ((v >> 16) != 0xA05Fu) return;
            bool c_debugen = v & 0x1u;
            bool c_halt    = v & 0x2u;
            bool c_step    = v & 0x4u;
            if (c_debugen && c_halt) target_halt::request_halt();
            else if (c_debugen && c_step) target_halt::request_step();
            else if (c_debugen) target_halt::request_resume();
            return;
        }
        case DCRSR: {
            g_dcrsr = v;
            unsigned reg = v & 0x7Fu;
            bool write = (v >> 16) & 1u;
            if (target_halt::is_halted()) {
                if (write) target_halt::write_register(reg, g_dcrdr);
                else       target_halt::read_register (reg, g_dcrdr);
            }
            return;
        }
        case DCRDR: g_dcrdr = v; return;
        case DEMCR: g_demcr = v; return;
        case FP_CTRL: g_fp_ctrl = v; return;
        default:
            if (addr >= FP_COMP0 && addr < FP_COMP0 + 32) {
                unsigned i = (addr - FP_COMP0) / 4;
                g_fp_comp[i] = v;
                bool enable = v & 1u;
                uint32_t bp_addr = v & ~3u;
                if (enable) target_halt::set_breakpoint  (bp_addr);
                else        target_halt::clear_breakpoint(bp_addr);
                return;
            }
            mem_write32(addr, v);
    }
}

// =============================================================================
// AHB-AP
// =============================================================================
constexpr uint32_t CSW_SIZE_MASK   = 0x07;
constexpr uint32_t CSW_ADDRINC_OFF = 0;
constexpr uint32_t CSW_ADDRINC_SGL = 1u << 4;
constexpr uint32_t CSW_ADDRINC_PCK = 2u << 4;

uint32_t ahb_size_bytes() {
    switch (g_ahb.csw & CSW_SIZE_MASK) {
        case 0: return 1;
        case 1: return 2;
        case 2: return 4;
        default: return 4;
    }
}

uint32_t ahb_read_drw() {
    uint32_t addr = g_ahb.tar;
    uint32_t v = 0;
    uint32_t sz = ahb_size_bytes();
    if (addr >= 0xE000'E000u && addr < 0xE001'0000u && sz == 4) {
        v = dcb_read(addr);
    } else if (sz == 4) {
        v = mem_read32(addr);
    } else {
        target_halt::read_memory(target_halt::map_guest_address(addr), &v, sz);
    }
    if ((g_ahb.csw & 0x30u) == CSW_ADDRINC_SGL ||
        (g_ahb.csw & 0x30u) == CSW_ADDRINC_PCK) {
        g_ahb.tar += sz;
    }
    return v;
}

void ahb_write_drw(uint32_t v) {
    uint32_t addr = g_ahb.tar;
    uint32_t sz = ahb_size_bytes();
    if (addr >= 0xE000'E000u && addr < 0xE001'0000u && sz == 4) {
        dcb_write(addr, v);
    } else if (sz == 4) {
        mem_write32(addr, v);
    } else {
        target_halt::write_memory(target_halt::map_guest_address(addr), &v, sz);
    }
    if ((g_ahb.csw & 0x30u) == CSW_ADDRINC_SGL ||
        (g_ahb.csw & 0x30u) == CSW_ADDRINC_PCK) {
        g_ahb.tar += sz;
    }
}

uint32_t ap_read(uint8_t addr4, uint8_t bank) {
    if (bank == 0xF) {
        // IDR
        if (addr4 == 0xC) return LPC1115_AHBAP_IDR;
        return 0;
    }
    switch (addr4) {
        case 0x0: return g_ahb.csw;
        case 0x4: return g_ahb.tar;
        case 0xC: return ahb_read_drw();
        default:  return 0;
    }
}

void ap_write(uint8_t addr4, uint8_t bank, uint32_t v) {
    if (bank != 0) return;
    switch (addr4) {
        case 0x0: g_ahb.csw = (g_ahb.csw & 0xFFFF'FF80u) | (v & 0x7Fu) | (v & 0x30u);
                  g_ahb.csw = v; break;
        case 0x4: g_ahb.tar = v; break;
        case 0xC: ahb_write_drw(v); break;
        default: break;
    }
}

// =============================================================================
// DP
// =============================================================================
uint32_t dp_read(uint8_t addr4) {
    switch (addr4) {
        case 0x0: g_stats.last_dp_idcode = LPC1115_DPIDR; return LPC1115_DPIDR;
        case 0x4: return g_dp.ctrl_stat
                    | (1u << 31)   // CSYSPWRUPACK
                    | (1u << 29)   // CDBGPWRUPACK
                    | (1u << 27);  // (CDBGRSTACK – wir signalisieren immer ack)
        case 0x8: return 0;        // RESEND – nicht implementiert
        case 0xC: return g_dp.rdbuff;
        default:  return 0;
    }
}

void dp_write(uint8_t addr4, uint32_t v) {
    switch (addr4) {
        case 0x0:                  // ABORT
            if (v & 0x1u) g_dp.ctrl_stat &= ~0xB2u;  // STKERRCLR & co.
            return;
        case 0x4: g_dp.ctrl_stat = v; return;
        case 0x8: g_dp.select    = v; return;
        case 0xC: return;          // RDBUFF read-only
        default: break;
    }
}

// =============================================================================
// PIO-Konfiguration & Bit-Layer-Helfer
// =============================================================================
inline bool swclk_high() { return gpio_get(static_cast<uint>(g_pins.swclk)); }

void wait_swclk_falling() {
    while (!swclk_high()) {}
    while ( swclk_high()) {}
}

void wait_swclk_rising() {
    while ( swclk_high()) {}
    while (!swclk_high()) {}
}

void drive_swdio(bool level) {
    gpio_set_dir(static_cast<uint>(g_pins.swdio), GPIO_OUT);
    gpio_put(static_cast<uint>(g_pins.swdio), level);
}

void release_swdio() {
    gpio_set_dir(static_cast<uint>(g_pins.swdio), GPIO_IN);
}

// PIO-TX-Helper: Pin-Direction auf Output umlegen, dann eine Anzahl Bits
// aus einem 32-bit-Word in die OSR/TX-FIFO der TX-SM schieben. Die TX-SM
// taktet diese Bits LSB-first auf SWDIO synchron mit SWCLK aus.
void pio_tx_drive_enable(bool drive) {
    uint32_t mask = 1u << static_cast<uint>(g_pins.swdio);
    pio_sm_set_pindirs_with_mask(g_pio, static_cast<uint>(g_sm_tx),
                                 drive ? mask : 0u, mask);
}

void pio_tx_send_bits(uint32_t v, unsigned n) {
    // OSR-Threshold ist auf 32 mit autopull konfiguriert. Wir füllen die
    // TX-FIFO mit dem Wert; die SM schiebt n Bits raus.
    pio_tx_drive_enable(true);
    // SM-Bits-Counter neu setzen über exec(set x, n-1) wäre ideal — wir
    // umgehen das, indem wir das Wort genau passend formen und nur n
    // Bits in die FIFO laden, wobei der OSR-Refill-Threshold 32 ist und
    // wir dann die FIFO leer ziehen. Variante: n ≤ 32 → ein Wort mit
    // pull-after-empty + threshold n.
    //
    // Implementierungstrick: wir setzen vor jeder Sendung dynamisch den
    // pull-threshold auf n und nutzen autopull. Das geht über die
    // shiftctrl-Register direkt:
    pio_sm_set_consecutive_pindirs(g_pio, static_cast<uint>(g_sm_tx),
                                   static_cast<uint>(g_pins.swdio), 1, true);
    pio_sm_clear_fifos(g_pio, static_cast<uint>(g_sm_tx));
    // Threshold konfigurieren (PULL_THRESH bits 25:30 in SHIFTCTRL).
    uint32_t sc = g_pio->sm[g_sm_tx].shiftctrl;
    uint32_t thr = (n & 0x1Fu);  // 0 = 32 Bits
    sc &= ~PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS;
    sc |=  (thr << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB) &
            PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS;
    g_pio->sm[g_sm_tx].shiftctrl = sc;
    // Daten in TX-FIFO.
    pio_sm_put_blocking(g_pio, static_cast<uint>(g_sm_tx), v);
    // Warten, bis OSR + TX-FIFO leer (Bits draußen). Pico-SDK ohne
    // dedizierten Helper → manuelle Flag-Polls.
    while (!pio_sm_is_tx_fifo_empty(g_pio, static_cast<uint>(g_sm_tx))) {}
    // Eine ganze Bit-Periode warten, damit das letzte Bit den
    // wait-1-Pin-1-Punkt passiert.
    wait_swclk_rising();
}

uint32_t pio_rx_byte(unsigned bits) {
    // Bits werden vom PIO als 1-bit-Words gepushed (autopush=1). Wir sammeln
    // sie hier zu einem Byte/Wort, LSB zuerst.
    uint32_t v = 0;
    for (unsigned i = 0; i < bits; ++i) {
        // pio_sm_get_blocking liefert einen 32-bit-Wert mit dem Bit in MSB
        // (in_pins shift_left=true), wir maskieren.
        uint32_t w = pio_sm_get_blocking(g_pio, static_cast<uint>(g_sm));
        v |= ((w >> 31) & 1u) << i;
    }
    return v;
}

bool calc_parity(uint32_t v) {
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4;
    v ^= v >> 2;  v ^= v >> 1;
    return v & 1u;
}

void send_bits(uint32_t v, unsigned n) {
    if (n == 0) return;
    if (n <= 32 && g_sm_tx >= 0) {
        pio_tx_send_bits(v, n);
        return;
    }
    // Fallback: CPU-getrieben (für n > 32 oder wenn TX-SM nicht verfügbar).
    for (unsigned i = 0; i < n; ++i) {
        wait_swclk_falling();
        drive_swdio((v >> i) & 1u);
    }
}

void send_ack(uint8_t ack) {
    // Turnaround-Bit: SWDIO loslassen, SWCLK-Flanke abwarten.
    if (g_sm_tx >= 0) pio_tx_drive_enable(false);
    release_swdio();
    wait_swclk_rising();   // Trn
    // 3 ACK-Bits ausgeben.
    if (g_sm_tx >= 0) {
        pio_tx_send_bits(ack & 0x7u, 3);
        return;
    }
    drive_swdio(false);
    for (unsigned i = 0; i < 3; ++i) {
        wait_swclk_falling();
        drive_swdio((ack >> i) & 1u);
    }
}

// =============================================================================
// Paket-Verarbeitung
// =============================================================================
void process_packet(uint8_t header) {
    if ((header & HEADER_START_MASK) == 0u || (header & HEADER_PARK_MASK) == 0u) {
        ++g_stats.packets_fault;
        send_ack(ACK_FAULT);
        return;
    }
    bool apndp = (header >> 1) & 1u;
    bool rnw   = (header >> 2) & 1u;
    uint8_t a  = static_cast<uint8_t>((header >> 3) & 0x3u) << 2;     // A2:A3 → addr[3:2]
    bool par   = (header >> 5) & 1u;
    bool calc  = ((apndp ^ rnw) ^ ((a >> 2) & 1u) ^ ((a >> 3) & 1u)) & 1u;
    if (par != calc) {
        ++g_stats.packets_fault;
        send_ack(ACK_FAULT);
        return;
    }

    if (rnw) {
        uint32_t v;
        if (apndp) {
            uint8_t bank = (g_dp.select >> 4) & 0xFu;
            v = ap_read(a, bank);
            g_dp.rdbuff = v;
        } else {
            v = dp_read(a);
        }
        send_ack(ACK_OK);
        // 1 trn target→host (target hält SWDIO bereits)
        send_bits(v, 32);
        // Parity
        wait_swclk_falling();
        drive_swdio(calc_parity(v));
        // Trn target→host abgeschlossen: SWDIO freigeben.
        wait_swclk_rising();
        release_swdio();
    } else {
        send_ack(ACK_OK);
        release_swdio();
        // 1 trn target→host nach ACK schon erledigt; nun host→target.
        // Empfange 32 Datenbits + 1 Parity.
        uint32_t v   = pio_rx_byte(32);
        uint32_t pbit = pio_rx_byte(1);
        if (calc_parity(v) != (pbit & 1u)) {
            g_dp.ctrl_stat |= (1u << 7);   // WDATAERR
            ++g_stats.packets_fault;
            return;
        }
        if (apndp) {
            uint8_t bank = (g_dp.select >> 4) & 0xFu;
            ap_write(a, bank, v);
        } else {
            dp_write(a, v);
        }
    }
    ++g_stats.packets_ok;
}

void detect_line_reset_and_select() {
    // Nach Line-Reset (50+ × 1) folgt optional JTAG-to-SWD = 0xE79E. Wir
    // verlassen uns hier auf den OpenOCD/J-Link-Default: nach Reset
    // schickt der Host als Erstes einen IDCODE-Read (Header 0xA5).
    g_line = LineState::AfterReset;
    g_dp.ctrl_stat = 0;
    g_dp.select    = 0;
    g_ahb.csw = g_ahb.tar = g_ahb.drw_pending = 0;
}

} // namespace

// =============================================================================
// Public API
// =============================================================================
void init() {
    g_active.store(false);
    g_pins = {-1, -1};
    std::memset(&g_stats, 0, sizeof g_stats);
    g_dp = {};
    g_ahb = {};
    g_dcrsr = g_dcrdr = g_demcr = 0;
    g_fp_ctrl = 0;
    std::memset(g_fp_comp, 0, sizeof g_fp_comp);
}

bool start(PinAssignment p) {
    if (p.swclk < 0 || p.swdio < 0) return false;
    g_pins = p;

    // PIO-Programm laden.
    g_pio = pio0;
    if (!pio_can_add_program(g_pio, &swd_rx_program)) {
        g_pio = pio1;
        if (!pio_can_add_program(g_pio, &swd_rx_program)) {
            std::printf("[SWD] kein Platz im PIO\n");
            return false;
        }
    }
    g_off = pio_add_program(g_pio, &swd_rx_program);
    int sm = pio_claim_unused_sm(g_pio, false);
    if (sm < 0) { std::printf("[SWD] keine SM frei\n"); return false; }
    g_sm = sm;

    // Pin-Setup. SWCLK = pure input. SWDIO open-drain-fähig.
    gpio_init(static_cast<uint>(p.swclk));
    gpio_init(static_cast<uint>(p.swdio));
    gpio_set_dir(static_cast<uint>(p.swclk), GPIO_IN);
    gpio_set_dir(static_cast<uint>(p.swdio), GPIO_IN);
    gpio_pull_up(static_cast<uint>(p.swdio));
    gpio_pull_up(static_cast<uint>(p.swclk));

    // Beide Pins müssen für die SM zugänglich sein.
    pio_gpio_init(g_pio, static_cast<uint>(p.swdio));
    pio_gpio_init(g_pio, static_cast<uint>(p.swclk));

    // Sicherstellen, dass swdio < swclk in Pin-Reihenfolge — wir verlangen
    // adjacent pins für `wait pin 1`. Falls nicht: einfach SWCLK direkt
    // hinter SWDIO mappen über Konfiguration; hier verlangen wir es.
    if (p.swclk != p.swdio + 1) {
        std::printf("[SWD] Bedingung verletzt: SWCLK muss SWDIO+1 sein "
                    "(SWDIO=%d, SWCLK=%d). Bitte CLI-Argumente entsprechend.\n",
                    p.swdio, p.swclk);
        return false;
    }

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, static_cast<uint>(g_off),
                            static_cast<uint>(g_off + 2));
    sm_config_set_in_pins (&c, static_cast<uint>(p.swdio));
    sm_config_set_in_shift(&c, /*shift_right=*/false,
                                /*autopush=*/true, /*push_thresh=*/1);
    // Wir wollen volle 150 MHz — keine Division.
    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(g_pio, static_cast<uint>(sm), static_cast<uint>(g_off), &c);
    pio_sm_set_enabled(g_pio, static_cast<uint>(sm), true);

    // ---------------- TX-SM ----------------
    if (pio_can_add_program(g_pio, &swd_tx_program)) {
        g_off_tx = pio_add_program(g_pio, &swd_tx_program);
        int sm_tx = pio_claim_unused_sm(g_pio, false);
        if (sm_tx >= 0) {
            g_sm_tx = sm_tx;
            pio_sm_config tc = pio_get_default_sm_config();
            sm_config_set_wrap(&tc, static_cast<uint>(g_off_tx),
                                    static_cast<uint>(g_off_tx + 2));
            sm_config_set_in_pins (&tc, static_cast<uint>(p.swdio));
            sm_config_set_out_pins(&tc, static_cast<uint>(p.swdio), 1);
            sm_config_set_set_pins(&tc, static_cast<uint>(p.swdio), 1);
            sm_config_set_out_shift(&tc, /*shift_right=*/true,
                                          /*autopull=*/true,
                                          /*pull_thresh=*/32);
            sm_config_set_clkdiv(&tc, 1.0f);
            pio_sm_init(g_pio, static_cast<uint>(sm_tx),
                        static_cast<uint>(g_off_tx), &tc);
            // Pin als Input starten — TX wird per pio_tx_drive_enable(true)
            // dynamisch in OUT umgeschaltet.
            pio_sm_set_pindirs_with_mask(g_pio, static_cast<uint>(sm_tx),
                                          0u, 1u << static_cast<uint>(p.swdio));
            pio_sm_set_enabled(g_pio, static_cast<uint>(sm_tx), true);
            std::printf("[SWD] TX-SM aktiv (Voll-PIO TX)\n");
        }
    }

    g_active.store(true);
    detect_line_reset_and_select();
    std::printf("[SWD] target listening on SWCLK=GP%d SWDIO=GP%d (DPIDR=0x%08lX)\n",
                p.swclk, p.swdio,
                static_cast<unsigned long>(LPC1115_DPIDR));
    return true;
}

void stop() {
    if (g_sm >= 0) {
        pio_sm_set_enabled(g_pio, static_cast<uint>(g_sm), false);
        pio_sm_unclaim   (g_pio, static_cast<uint>(g_sm));
        if (g_off >= 0) pio_remove_program(g_pio, &swd_rx_program,
                                           static_cast<uint>(g_off));
    }
    if (g_sm_tx >= 0) {
        pio_sm_set_enabled(g_pio, static_cast<uint>(g_sm_tx), false);
        pio_sm_unclaim   (g_pio, static_cast<uint>(g_sm_tx));
        if (g_off_tx >= 0) pio_remove_program(g_pio, &swd_tx_program,
                                              static_cast<uint>(g_off_tx));
    }
    g_sm = g_off = g_sm_tx = g_off_tx = -1;
    g_active.store(false);
    std::printf("[SWD] stopped\n");
}

bool active() { return g_active.load(); }

void poll() {
    if (!g_active.load()) return;

    // Wartet, bis ein vollständiger 8-bit-Header eingelaufen ist.
    if (pio_sm_is_rx_fifo_empty(g_pio, static_cast<uint>(g_sm))) return;

    uint8_t header = static_cast<uint8_t>(pio_rx_byte(8));

    // Line-Reset-Erkennung: 50 Bit lang nur 1 ⇒ 0xFF im PIO mehrfach.
    static unsigned ones = 0;
    if (header == 0xFFu) {
        ones += 8;
        if (ones >= 50) detect_line_reset_and_select();
        return;
    } else {
        ones = 0;
    }

    // JTAG-to-SWD-Sequenz 0xE79E (LSB-first transmitted = 0x79E7 in unserer
    // Order) — wir akzeptieren beide Varianten und ignorieren danach.
    if (header == 0x79u || header == 0xE7u) return;

    process_packet(header);
}

Stats stats() { return g_stats; }

} // namespace swd_target
