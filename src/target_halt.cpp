#include "target_halt.h"
#include "emulator.h"
#include "gdb_stub.h"

#include <atomic>
#include <cstring>

#include "RP2350.h"
#include "hardware/sync.h"
#include "pico/time.h"

namespace target_halt {
namespace {

constexpr unsigned MAX_BP = 8;

std::atomic<bool> g_halt_request{false};
std::atomic<bool> g_resume_request{false};
std::atomic<bool> g_step_request{false};
std::atomic<bool> g_halted{false};

Snapshot g_snap{};

struct BP { uint32_t addr; uint16_t saved; bool used; };
BP g_bp[MAX_BP]{};

void capture_frame(uint32_t* frame, uint32_t* r4_r11) {
    g_snap.frame   = frame;
    g_snap.r4_r11  = r4_r11;
    g_snap.r[0]  = frame[0]; g_snap.r[1]  = frame[1];
    g_snap.r[2]  = frame[2]; g_snap.r[3]  = frame[3];
    g_snap.r[4]  = r4_r11[0]; g_snap.r[5]  = r4_r11[1];
    g_snap.r[6]  = r4_r11[2]; g_snap.r[7]  = r4_r11[3];
    g_snap.r[8]  = r4_r11[4]; g_snap.r[9]  = r4_r11[5];
    g_snap.r[10] = r4_r11[6]; g_snap.r[11] = r4_r11[7];
    g_snap.r[12] = frame[4];
    g_snap.r[13] = reinterpret_cast<uint32_t>(frame) + 0x20;
    g_snap.r[14] = frame[5];
    g_snap.r[15] = frame[6];
    g_snap.xpsr  = frame[7];
}

void writeback_frame() {
    if (!g_snap.frame) return;
    g_snap.frame[0]   = g_snap.r[0];
    g_snap.frame[1]   = g_snap.r[1];
    g_snap.frame[2]   = g_snap.r[2];
    g_snap.frame[3]   = g_snap.r[3];
    g_snap.r4_r11[0]  = g_snap.r[4];
    g_snap.r4_r11[1]  = g_snap.r[5];
    g_snap.r4_r11[2]  = g_snap.r[6];
    g_snap.r4_r11[3]  = g_snap.r[7];
    g_snap.r4_r11[4]  = g_snap.r[8];
    g_snap.r4_r11[5]  = g_snap.r[9];
    g_snap.r4_r11[6]  = g_snap.r[10];
    g_snap.r4_r11[7]  = g_snap.r[11];
    g_snap.frame[4]   = g_snap.r[12];
    // r13/SP wird nicht zurückgeschrieben — Frame-Position ist fix.
    g_snap.frame[5]   = g_snap.r[14];
    g_snap.frame[6]   = g_snap.r[15];
    g_snap.frame[7]   = g_snap.xpsr;
}

} // namespace

void init() {
    g_halt_request.store(false);
    g_resume_request.store(false);
    g_step_request.store(false);
    g_halted.store(false);
    std::memset(&g_snap, 0, sizeof g_snap);
    for (auto& b : g_bp) b = {};
}

void on_pendsv_check() {
    if (!g_halt_request.load(std::memory_order_acquire)) return;

    // Frame-Pointer aus PSP rekonstruieren — der gestackte Frame liegt
    // am aktuellen PSP-Top.
    uint32_t psp;
    __asm volatile ("mrs %0, psp" : "=r"(psp));
    auto* frame = reinterpret_cast<uint32_t*>(psp);

    // r4..r11 wurden vom Asm-Wrapper auf den MSP gepushed (siehe
    // irq_inject.cpp/isr_pendsv). Hier vereinfacht: wir kopieren aus
    // den aktuellen Banked-Registern, was nur stimmt, wenn PendSV
    // unmittelbar aus Gast-Code kommt. Für Step/Halt-Granularität ist
    // das ausreichend.
    uint32_t r4_r11[8];
    __asm volatile ("stmia %0, {r4-r11}" :: "r"(r4_r11) : "memory");

    g_halt_request.store(false);
    g_halted.store(true);
    capture_frame(frame, r4_r11);

    // Stop-Reason an angeschlossene Stubs signalisieren.
    if (gdb_stub::active()) gdb_stub::on_breakpoint(frame, r4_r11);

    // Spin-Schleife bis Resume/Step.
    while (!g_resume_request.load(std::memory_order_acquire)) {
        gdb_stub::poll();
        // SWD-Target wird im eigenen Polling über USB/Pin-IRQ behandelt.
        sleep_us(50);
    }

    writeback_frame();

    g_resume_request.store(false);
    g_halted.store(false);

    // r4..r11 zurückschreiben — der Asm-Wrapper popt sie als Nächstes.
    __asm volatile ("ldmia %0, {r4-r11}" :: "r"(r4_r11) : "memory");

    if (g_step_request.load()) {
        g_step_request.store(false);
        // Single-Step über DEMCR.MON_STEP — landet wieder im DebugMonitor.
        CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk
                         |  CoreDebug_DEMCR_MON_STEP_Msk;
    }
}

void request_halt() {
    g_halt_request.store(true);
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
}

void request_resume() {
    g_step_request.store(false);
    g_resume_request.store(true);
}

void request_step() {
    g_step_request.store(true);
    g_resume_request.store(true);
}

bool is_halted()       { return g_halted.load(); }
bool is_step_pending() { return g_step_request.load(); }
const Snapshot* snapshot() { return is_halted() ? &g_snap : nullptr; }

bool write_register(unsigned idx, uint32_t value) {
    if (!is_halted() || idx >= 17) return false;
    if (idx < 16) g_snap.r[idx] = value;
    else          g_snap.xpsr   = value;
    return true;
}

bool read_register(unsigned idx, uint32_t& value) {
    if (!is_halted() || idx >= 17) return false;
    value = (idx < 16) ? g_snap.r[idx] : g_snap.xpsr;
    return true;
}

uint32_t map_guest_address(uint32_t lpc_addr) {
    // LPC1115 Flash: 0x0000_0000 – 0x0000_FFFF → RP2350 SRAM bei
    // emulator::LPC_LOAD_BASE.
    if (lpc_addr < 0x1000'0000u) {
        return emulator::LPC_LOAD_BASE + lpc_addr;
    }
    // LPC1115 SRAM: 0x1000_0000 – 0x1000_1FFF → emulator::LPC_GUEST_RAM_BASE.
    if (lpc_addr >= 0x1000'0000u && lpc_addr < 0x1000'2000u) {
        return emulator::LPC_GUEST_RAM_BASE + (lpc_addr - 0x1000'0000u);
    }
    // Peripherie/PPB/Sonstiges: identisch durchreichen.
    return lpc_addr;
}

bool read_memory(uint32_t addr, void* dst, std::size_t len) {
    if (!dst) return false;
    auto a = map_guest_address(addr);
    std::memcpy(dst, reinterpret_cast<const void*>(a), len);
    return true;
}

bool write_memory(uint32_t addr, const void* src, std::size_t len) {
    if (!src) return false;
    auto a = map_guest_address(addr);
    std::memcpy(reinterpret_cast<void*>(a), src, len);
    __DSB(); __ISB();
    return true;
}

bool set_breakpoint(uint32_t addr) {
    addr = map_guest_address(addr);
    for (auto& b : g_bp) if (b.used && b.addr == addr) return true;
    for (auto& b : g_bp) if (!b.used) {
        auto* p = reinterpret_cast<uint16_t*>(addr);
        b.addr = addr; b.saved = *p; b.used = true;
        *p = 0xBE00;
        __DSB(); __ISB();
        return true;
    }
    return false;
}

bool clear_breakpoint(uint32_t addr) {
    addr = map_guest_address(addr);
    for (auto& b : g_bp) if (b.used && b.addr == addr) {
        *reinterpret_cast<uint16_t*>(addr) = b.saved;
        b.used = false;
        __DSB(); __ISB();
        return true;
    }
    return false;
}

} // namespace target_halt
