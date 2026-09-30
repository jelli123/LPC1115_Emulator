#include "target_halt.h"
#include "emulator.h"

#include <atomic>
#include <cstring>

#include "RP2350.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "pico.h"

namespace target_halt {
namespace {

constexpr unsigned MAX_BP = 8;

std::atomic<bool> g_halt_request{false};
std::atomic<bool> g_resume_request{false};
std::atomic<bool> g_step_request{false};
std::atomic<bool> g_halted{false};
std::atomic<bool> g_step_active{false};   // MON_STEP gesetzt, DebugMonitor erwartet
std::atomic<uint32_t> g_halt_count{0};

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

void on_guest_reset() {
    // Run-Control-Flags zuruecksetzen — ein evtl. stehengebliebener Halt-Request
    // (z.B. nach erzwungenem Core-Reset, wenn der Gast nie kooperativ haltete)
    // wuerde sonst den naechsten Gast beim ersten PendSV/Trap sofort anhalten.
    g_halt_request.store(false);
    g_resume_request.store(false);
    g_step_request.store(false);
    g_halted.store(false);
    g_step_active.store(false);
    // Breakpoints gehoeren zum alten Image (wird beim Start neu kopiert) ->
    // Tabelle verwerfen, sonst wuerde ein spaeteres Loeschen alte Befehle
    // in das neue Image zurueckschreiben.
    for (auto& b : g_bp) b = {};
}

// Gemeinsamer Halt-Pfad (Core1, Handler-Mode): Snapshot nehmen, warten bis
// Resume/Step, Registeraenderungen zurueckschreiben. Kein USB-/printf-Zugriff
// hier - der GDB-Stub bedient die CDC ausschliesslich von Core0 aus.
void enter_halt(uint32_t* frame, uint32_t* r4_r11) {
    g_halt_request.store(false);
    capture_frame(frame, r4_r11);
    g_halt_count.fetch_add(1, std::memory_order_relaxed);
    g_halted.store(true, std::memory_order_release);

    while (!g_resume_request.load(std::memory_order_acquire)) busy_wait_us(50);

    writeback_frame();
    g_resume_request.store(false);
    g_halted.store(false, std::memory_order_release);

    if (g_step_request.exchange(false)) {
        // Einzelschritt: nach genau einer Instruktion DebugMonitor.
        CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk | CoreDebug_DEMCR_MON_STEP_Msk;
        g_step_active.store(true);
    }
}

void __not_in_flash_func(on_pendsv_check)(uint32_t* r4_r11) {
    if (!g_halt_request.load(std::memory_order_acquire)) return;
    uint32_t psp;
    __asm volatile ("mrs %0, psp" : "=r"(psp));
    enter_halt(reinterpret_cast<uint32_t*>(psp), r4_r11);
}

void __not_in_flash_func(core1_service)() {
    if (g_halt_request.load(std::memory_order_relaxed) && !g_halted.load())
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void on_debug_event(uint32_t* frame, uint32_t* r4_r11) {
    const uint32_t dfsr = SCB->DFSR;
    SCB->DFSR = dfsr;                                   // w1c
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_STEP_Msk;
    const bool was_step = g_step_active.exchange(false);
    const bool bkpt = (dfsr & SCB_DFSR_BKPT_Msk) != 0;
    if (!bkpt && !was_step) return;                     // unerwartet -> weiterlaufen
    enter_halt(frame, r4_r11);
}

uint32_t halt_count() { return g_halt_count.load(std::memory_order_relaxed); }

void request_halt() {
    // Wirkt ueber core1_service() (SysTick-Shim/MMIO-Trap auf Core1) bzw.
    // direkt, falls von Core1 aufgerufen.
    g_halt_request.store(true);
    if (get_core_num() == 1u) SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
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

bool __not_in_flash_func(is_halted)()       { return g_halted.load(); }
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
    // LPC1115 Flash: 0x0000_0000 – 0x0000_FFFF → RP2350 SRAM.
    if (lpc_addr < 0x1000'0000u) {
        return emulator::load_base() + lpc_addr;
    }
    // LPC1115 SRAM: 0x1000_0000 – 0x1000_1FFF → Guest-RAM.
    if (lpc_addr >= 0x1000'0000u && lpc_addr < 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
        return emulator::guest_ram_base() + (lpc_addr - 0x1000'0000u);
    }
    // Peripherie/PPB/Sonstiges: identisch durchreichen.
    return lpc_addr;
}

// Nur Flash-Image und Gast-RAM sind fuer Debugger zugaenglich. Andere Adressen
// (LPC-Peripherie, RP2350-Speicher) werden abgewiesen: ein roher Zugriff auf
// Core0 koennte echte RP2350-Register treffen oder einen BusFault ausloesen.
static bool guest_range(uint32_t host, std::size_t len) {
    const uint32_t img = emulator::load_base(), ram = emulator::guest_ram_base();
    auto in = [&](uint32_t base, uint32_t size) {
        return host >= base && len <= size && host - base <= size - len;
    };
    return in(img, emulator::LPC_LOAD_MAX_SIZE) || in(ram, emulator::LPC_GUEST_RAM_SIZE);
}

bool read_memory(uint32_t addr, void* dst, std::size_t len) {
    if (!dst) return false;
    auto a = map_guest_address(addr);
    if (!guest_range(a, len)) return false;
    std::memcpy(dst, reinterpret_cast<const void*>(a), len);
    return true;
}

bool write_memory(uint32_t addr, const void* src, std::size_t len) {
    if (!src) return false;
    auto a = map_guest_address(addr);
    if (!guest_range(a, len)) return false;
    std::memcpy(reinterpret_cast<void*>(a), src, len);
    __DSB(); __ISB();
    return true;
}

bool set_breakpoint(uint32_t addr) {
    addr = map_guest_address(addr & ~1u);
    const uint32_t img = emulator::load_base();                // nur im Code-Image
    if (addr < img || addr + 2u > img + emulator::LPC_LOAD_MAX_SIZE) return false;
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
    addr = map_guest_address(addr & ~1u);
    for (auto& b : g_bp) if (b.used && b.addr == addr) {
        *reinterpret_cast<uint16_t*>(addr) = b.saved;
        b.used = false;
        __DSB(); __ISB();
        return true;
    }
    return false;
}

} // namespace target_halt
