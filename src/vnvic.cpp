#include "vnvic.h"

#include <atomic>
#include <cstring>

namespace vnvic {
namespace {

// Schatten-Register
std::atomic<uint32_t> g_iser{0};
std::atomic<uint32_t> g_ispr{0};
std::atomic<uint32_t> g_iabr{0};
uint8_t               g_ipr[32]{};

uint32_t read32(uint32_t aligned) {
    switch (aligned) {
        case 0xE000'E100: return g_iser.load();          // ISER
        case 0xE000'E180: return g_iser.load();          // ICER (read = ISER)
        case 0xE000'E200: return g_ispr.load();          // ISPR
        case 0xE000'E280: return g_ispr.load();          // ICPR (read = ISPR)
        case 0xE000'E300: return g_iabr.load();          // IABR
        default:
            if (aligned >= 0xE000'E400 && aligned < 0xE000'E420) {
                uint32_t base = (aligned - 0xE000'E400);
                uint32_t v = 0;
                for (int i = 0; i < 4; ++i)
                    v |= static_cast<uint32_t>(g_ipr[base + i]) << (i * 8);
                return v;
            }
            return 0;
    }
}

void write32(uint32_t aligned, uint32_t v) {
    switch (aligned) {
        case 0xE000'E100: g_iser.fetch_or(v);  break;     // ISER: write 1 to set
        case 0xE000'E180: g_iser.fetch_and(~v); break;    // ICER: write 1 to clear
        case 0xE000'E200: g_ispr.fetch_or(v);  break;     // ISPR
        case 0xE000'E280: g_ispr.fetch_and(~v); break;    // ICPR
        default:
            if (aligned >= 0xE000'E400 && aligned < 0xE000'E420) {
                uint32_t base = (aligned - 0xE000'E400);
                for (int i = 0; i < 4; ++i)
                    g_ipr[base + i] = static_cast<uint8_t>((v >> (i*8)) & 0xFFu);
            }
            break;
    }
}

// Byte-Sammler analog zu peripherals::syscon_collect_byte. NVIC-Schreib-
// vorgänge sind in der Regel ohnehin word-aligned, aber wir bleiben safe.
struct WC { uint32_t aligned; uint8_t b[4]; uint8_t mask; };
WC g_wc{0,{0,0,0,0},0};

void collect(uint32_t addr, uint8_t v) {
    uint32_t a = addr & ~3u;
    if (g_wc.mask && g_wc.aligned != a) {
        uint32_t cur = read32(g_wc.aligned);
        for (int i = 0; i < 4; ++i)
            if (g_wc.mask & (1u << i))
                cur = (cur & ~(0xFFu << (i*8))) |
                      (static_cast<uint32_t>(g_wc.b[i]) << (i*8));
        write32(g_wc.aligned, cur);
        g_wc = {0,{0,0,0,0},0};
    }
    g_wc.aligned = a;
    g_wc.b[addr & 3u] = v;
    g_wc.mask |= static_cast<uint8_t>(1u << (addr & 3u));
    if (g_wc.mask == 0xF) {
        uint32_t value = static_cast<uint32_t>(g_wc.b[0])       |
                         (static_cast<uint32_t>(g_wc.b[1]) << 8) |
                         (static_cast<uint32_t>(g_wc.b[2]) << 16) |
                         (static_cast<uint32_t>(g_wc.b[3]) << 24);
        write32(a, value);
        g_wc = {0,{0,0,0,0},0};
    }
}

} // namespace

bool is_nvic_addr(uint32_t addr) { return addr >= NVIC_BASE && addr < NVIC_END; }

uint8_t read8(uint32_t addr) {
    uint32_t v = read32(addr & ~3u);
    return static_cast<uint8_t>((v >> ((addr & 3u) * 8)) & 0xFFu);
}

void write8(uint32_t addr, uint8_t val) { collect(addr, val); }

void pend_irq(uint8_t lpc_irq) {
    if (lpc_irq >= 32) return;
    g_ispr.fetch_or(1u << lpc_irq);
}

bool irq_pending() {
    return (g_iser.load() & g_ispr.load()) != 0u;
}

uint8_t next_pending_irq() {
    uint32_t mask = g_iser.load() & g_ispr.load();
    if (!mask) return 0xFF;
    return static_cast<uint8_t>(__builtin_ctz(mask));
}

void clear_pending(uint8_t lpc_irq) {
    if (lpc_irq >= 32) return;
    g_ispr.fetch_and(~(1u << lpc_irq));
}

Snapshot snapshot() {
    Snapshot s{};
    s.iser = g_iser.load();
    s.ispr = g_ispr.load();
    for (int i = 0; i < 8; ++i) {
        uint32_t v = 0;
        for (int j = 0; j < 4; ++j) v |= static_cast<uint32_t>(g_ipr[i*4 + j]) << (j*8);
        s.prio[i] = v;
    }
    return s;
}

void reset() {
    g_iser.store(0); g_ispr.store(0); g_iabr.store(0);
    std::memset(g_ipr, 0, sizeof g_ipr);
    g_wc = {0,{0,0,0,0},0};
}

} // namespace vnvic
