#include "emulator.h"
#include "mmu.h"
#include "fault.h"
#include "peripherals.h"
#include "storage.h"
#include "config.h"
#include "hex_patcher.h"

#include <atomic>
#include <cstdio>
#include <cstring>

#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "RP2350.h"

namespace {

std::atomic<emulator::State> g_state{emulator::State::Idle};
std::atomic<bool>            g_request_stop{false};
std::atomic<uint32_t>        g_pc{0};

alignas(32) uint8_t g_guest_ram[emulator::LPC_GUEST_RAM_SIZE]
    __attribute__((section(".guest_ram"))) = {};

// Externe Symbole der Fault-Handler — die linken weak im SDK; wir adressieren
// sie hier, um die Firmware-Vector-Table darauf zu patchen.
extern "C" void isr_busfault();
extern "C" void isr_memmanage();
extern "C" void isr_hardfault();
extern "C" void isr_usagefault();

// Naked Trampolin: setzt MPU, wechselt auf PSP/unprivileged Thread Mode,
// und springt mit BX in den Reset-Handler. Kehrt nie zurück (Gast läuft
// in Endlos-Loop wie eine echte MCU).
__attribute__((naked, noreturn))
void enter_guest(uint32_t initial_sp, uint32_t reset_handler) {
    __asm volatile (
        "msr   psp, r0             \n"   // PSP = initial_sp
        "mov   r2, #3              \n"   // CONTROL.SPSEL=1, nPRIV=1
        "msr   control, r2         \n"
        "isb                       \n"
        // r1 = reset handler address (mit Thumb-Bit)
        "bx    r1                  \n"
    );
}

// Patch-Helper: relocate Vector-Table-Eintrag, falls er auf den LPC-Flash-
// Bereich (< 0x10000000) zeigt.
uint32_t relocate_vector(uint32_t v) {
    uint32_t plain = v & ~1u;
    bool thumb = v & 1u;
    if (plain < 0x1000'0000u) {
        // LPC-Flash-Range: nach RP2350 SRAM relocaten
        plain += emulator::LPC_LOAD_BASE;
    }
    return plain | (thumb ? 1u : 0u);
}

void core1_main() {
    multicore_lockout_victim_init();
    while (true) {
        while (g_state.load(std::memory_order_acquire) != emulator::State::Running) {
            __asm volatile ("wfe");
        }
        g_request_stop.store(false, std::memory_order_release);

        // Firmware in RP2350-SRAM kopieren. Quelle ist storage::firmware_data()
        // — ein Pointer in den memory-mapped XIP-Bereich.
        const uint8_t* fw = storage::firmware_data();
        std::size_t   sz = storage::firmware_size();
        if (!fw || sz == 0 || sz > emulator::LPC_LOAD_MAX_SIZE) {
            std::printf("[EMU] keine valide Firmware (sz=%u)\n",
                        static_cast<unsigned>(sz));
            g_state.store(emulator::State::Halted);
            continue;
        }

        auto* dst = reinterpret_cast<uint32_t*>(emulator::LPC_LOAD_BASE);
        std::memcpy(dst, fw, sz);
        if (sz < emulator::LPC_LOAD_MAX_SIZE) {
            std::memset(reinterpret_cast<uint8_t*>(dst) + sz, 0xFF,
                        emulator::LPC_LOAD_MAX_SIZE - sz);
        }

        // Vector-Table relocaten + Fault-Handler einsetzen.
        constexpr uint32_t VEC_COUNT = 48; // 16 system + 32 IRQs (LPC1115)
        if (sz >= VEC_COUNT * 4) {
            for (uint32_t i = 1; i < VEC_COUNT; ++i) {
                dst[i] = relocate_vector(dst[i]);
            }
            // Initial-SP: falls in LPC-RAM-Range, auf Gast-RAM-Top umsetzen.
            uint32_t isp = dst[0];
            if (isp >= 0x1000'0000u && isp < 0x1000'0000u + 0x10000u) {
                dst[0] = emulator::LPC_GUEST_RAM_BASE
                       + emulator::LPC_GUEST_RAM_SIZE;
            }
            // System-Faults: unsere Handler eintragen.
            dst[3]  = reinterpret_cast<uint32_t>(&isr_hardfault);
            dst[4]  = reinterpret_cast<uint32_t>(&isr_memmanage);
            dst[5]  = reinterpret_cast<uint32_t>(&isr_busfault);
            dst[6]  = reinterpret_cast<uint32_t>(&isr_usagefault);
        }

        // RAM-Adressen aus Literal-Pools relocaten (LPC RAM 0x10000000+8KB
        // → RP2350 0x20060000+8KB).
        auto pr = hex_patcher::relocate_ram_refs(
            reinterpret_cast<uint8_t*>(dst), sz,
            0x1000'0000u, 0x2000u,
            emulator::LPC_GUEST_RAM_BASE);
        std::printf("[EMU] hex-patch: %u/%u Wörter relociert\n",
                    static_cast<unsigned>(pr.patched_words),
                    static_cast<unsigned>(pr.scanned_words));

        // VTOR setzen (muss 128-Byte-aligned sein → 0x40 Bits frei).
        SCB->VTOR = emulator::LPC_LOAD_BASE;
        __DSB(); __ISB();

        // MPU für den Gast scharf schalten.
        mpu_setup::enable_for_guest();

        uint32_t initial_sp = dst[0];
        uint32_t reset_h    = dst[1];
        std::printf("[EMU] starting: SP=0x%08lx PC=0x%08lx (load=0x%08lx, %u B)\n",
                    static_cast<unsigned long>(initial_sp),
                    static_cast<unsigned long>(reset_h),
                    static_cast<unsigned long>(emulator::LPC_LOAD_BASE),
                    static_cast<unsigned>(sz));

        // Sprung in den Gast — kommt nicht zurück.
        g_pc.store(reset_h);
        enter_guest(initial_sp, reset_h);
        // unreachable
    }
}

} // namespace

namespace emulator {

void boot_core1() {
    multicore_launch_core1(core1_main);
}

void load_and_start() {
    if (g_state.load() == State::Running) {
        std::printf("[EMU] bereits gestartet\n");
        return;
    }
    if (storage::firmware_size() == 0) {
        std::printf("[EMU] keine Firmware geflashed\n");
        return;
    }
    g_state.store(State::Running, std::memory_order_release);
    __asm volatile ("sev");
}

void stop() {
    // Ein laufender, in unprivileged Mode bxender Gast lässt sich nicht
    // ohne Weiteres anhalten. Wir schießen Core 1 ab und reinitialisieren.
    multicore_reset_core1();
    g_state.store(State::Idle);
    mpu_setup::disable();
    boot_core1();
}

void request_guest_reset() {
    // Vom WDT-Modell aufgerufen: nur Guest neustarten, RP2350 läuft weiter.
    // multicore_reset_core1 ist von beiden Cores aus sicher.
    multicore_reset_core1();
    g_state.store(State::Idle);
    mpu_setup::disable();
    boot_core1();
    g_state.store(State::Running, std::memory_order_release);
    __asm volatile ("sev");
}

State    state()    { return g_state.load(); }
uint32_t pc()       { return g_pc.load(); }
uint64_t mem_traps(){ return faultsys::stats().mem_traps; }

} // namespace emulator
