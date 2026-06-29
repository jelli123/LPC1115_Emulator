#include "emulator.h"
#include "mmu.h"
#include "fault.h"
#include "peripherals.h"
#include "storage.h"
#include "config.h"
#include "hex_patcher.h"
#include "vnvic.h"

#include <atomic>
#include <cstdio>
#include <cstring>

#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "RP2350.h"   // CMSIS, via cmsis_core lib

namespace {

std::atomic<emulator::State> g_state{emulator::State::Idle};
std::atomic<bool>            g_request_stop{false};
std::atomic<uint32_t>        g_pc{0};

// --- Guest-Speicherbereiche als echte Arrays ---
// Der Linker platziert sie automatisch ohne Kollision mit anderem BSS.
// VTOR verlangt 256-Byte-Alignment (128-Byte reicht ab ARMv7-M, aber
// 256 ist auf der sicheren Seite für 48 Vektoren × 4 = 192 → nächste 2^n = 256).
alignas(256) uint8_t g_firmware_image[emulator::LPC_LOAD_MAX_SIZE]
    __attribute__((used)) = {};

alignas(32) uint8_t g_guest_ram[emulator::LPC_GUEST_RAM_SIZE]
    __attribute__((used)) = {};

// Externe Symbole der Fault-Handler — die linken weak im SDK; wir adressieren
// sie hier, um die Firmware-Vector-Table darauf zu patchen.
extern "C" void isr_busfault();
extern "C" void isr_memmanage();
extern "C" void isr_hardfault();
extern "C" void isr_usagefault();
extern "C" void isr_pendsv();   // IRQ-Injektor (src/irq_inject.cpp)

// --- WFI-Pin-Wakeup + PRIMASK-Schatten (beide opt-in) ----------------------
//
// Beide Features lenken Gast-Instruktionen auf SVC-Traps um, die hier ueber
// die SVC-Immediate unterschieden werden (SVCall = Vektor-Slot 11):
//   SVC #0  = gepatchtes WFI  (config::wfi_pin_wakeup) -> Warteschleife
//   SVC #1  = CPSID i         (config::primask_shadow) -> Schatten-PRIMASK=1
//   SVC #2  = CPSIE i         (config::primask_shadow) -> Schatten-PRIMASK=0
//
// WFI-Warteschleife: Statt schlafen zu legen, pollt der Host die echten
// RP2350-Eingaenge (Pin-Flanken) und die zeitbasierten Modelle (CT/WWDT).
// Sobald ein vom Gast aktivierter IRQ pending wird, kehrt der Handler zurueck
// und setzt PendSV — die bestehende IRQ-Injektion liefert den LPC-Handler aus.
//
// PRIMASK-Schatten: Der Gast laeuft unprivilegiert, daher ignoriert die M33-
// Hardware CPSID/CPSIE. Wir fuehren den Maskierungszustand im vNVIC nach;
// solange gesetzt, haelt pendsv_inject_c() neue IRQs pending zurueck. Beim
// Loeschen (CPSIE) wird ein evtl. wartender IRQ sofort per PendSV nachgereicht.
//
// HINWEIS: Die WFI-Schleife pollt aktiv (busy-wait), nicht stromsparend.
// WFI muss mit aktivierten Interrupts (PRIMASK=0) ausgefuehrt werden; sonst
// eskaliert der SVC zum HardFault (Sonderbehandlung in src/fault.cpp).
extern "C" void svc_dispatch_c(uint32_t* frame) {
    // Gestapelter Exception-Frame: frame[6] = Return-PC (hinter dem SVC).
    // Die SVC-Instruktion liegt 2 Byte davor; ihr Low-Byte ist die Immediate.
    uint32_t pc  = frame[6] & ~1u;
    uint8_t  imm = *reinterpret_cast<const uint8_t*>(pc - 2u);
    switch (imm) {
        case 1:                                  // CPSID i -> Sektion betreten
            vnvic::set_primask(true);
            return;
        case 2:                                  // CPSIE i -> Sektion verlassen
            vnvic::set_primask(false);
            if (vnvic::irq_pending())
                SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            return;
        case 0:                                  // gepatchtes WFI
        default:
            for (;;) {
                if (g_request_stop.load(std::memory_order_acquire)) return;
                peripherals::sample_pin_interrupts();
                peripherals::poll_timed_sources();
                if (vnvic::irq_pending()) {
                    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
                    return;
                }
                // Bei aktivem Timer-Capture (KNX-Empfang) eng pollen, damit
                // Busflanken (~104 µs/Bit) zeitlich aufgelöst werden; sonst
                // 50 µs Pause zur RP2350-Entlastung.
                if (!peripherals::capture_armed())
                    busy_wait_us(50);
            }
    }
}

__attribute__((naked))
void isr_svc() {
    __asm volatile (
        "tst   lr, #4              \n"   // EXC_RETURN bit2: 0=MSP, 1=PSP
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4, lr}            \n"   // 8-Byte-Alignment + EXC_RETURN
        "bl    svc_dispatch_c      \n"
        "pop   {r4, pc}            \n"   // EXC_RETURN -> zurueck zum Gast
    );
}

// Naked Trampolin: setzt MPU, wechselt auf PSP/unprivileged Thread Mode,
// und springt mit BX in den Reset-Handler. Kehrt nie zurück (Gast läuft
// in Endlos-Loop wie eine echte MCU).
__attribute__((naked, noreturn))
void enter_guest(uint32_t /*initial_sp*/, uint32_t /*reset_handler*/) {
    __asm volatile (
        "msr   psp, r0             \n"   // PSP = initial_sp
        "mov   r2, #3              \n"   // CONTROL.SPSEL=1, nPRIV=1
        "msr   control, r2         \n"
        "isb                       \n"
        // r1 = reset handler address (mit Thumb-Bit)
        "bx    r1                  \n"
    );
}

// Patch-Helper: relocate Vector-Table-Eintrag.
// Flash (< 0x10000000) → g_firmware_image base + offset
// RAM (0x10000000..+LPC_GUEST_RAM_SIZE) → g_guest_ram base + offset
uint32_t relocate_vector(uint32_t v) {
    uint32_t plain = v & ~1u;
    bool thumb = v & 1u;
    uint32_t load_base = reinterpret_cast<uint32_t>(g_firmware_image);
    uint32_t ram_base  = reinterpret_cast<uint32_t>(g_guest_ram);
    if (plain < 0x1000'0000u) {
        // LPC-Flash-Range: nach RP2350 SRAM relocaten
        plain += load_base;
    } else if (plain >= 0x1000'0000u &&
               plain < 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
        // LPC-RAM-Range: nach RP2350 Guest-RAM relocaten
        plain = ram_base + (plain - 0x1000'0000u);
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

        auto* dst = reinterpret_cast<uint32_t*>(g_firmware_image);
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
                dst[0] = reinterpret_cast<uint32_t>(g_guest_ram)
                       + emulator::LPC_GUEST_RAM_SIZE;
            }
            // System-Faults: unsere Handler eintragen.
            dst[3]  = reinterpret_cast<uint32_t>(&isr_hardfault);
            dst[4]  = reinterpret_cast<uint32_t>(&isr_memmanage);
            dst[5]  = reinterpret_cast<uint32_t>(&isr_busfault);
            dst[6]  = reinterpret_cast<uint32_t>(&isr_usagefault);

            // PendSV (Slot 14) gehört dem Host: Über PendSV werden emulierte
            // LPC-IRQs in den Gast injiziert (irq_inject.cpp). Die Firmware
            // selbst nutzt PendSV typischerweise nicht (LPC-Startups legen dort
            // nur einen `B .`-Default-Stub ab) — würde der Stub stehenbleiben,
            // liefe der Gast beim ersten injizierten IRQ in eine Endlosschleife.
            // SysTick (Slot 15) bleibt beim Gast, da es eine echte M33-
            // Peripherie ist und der Gast seinen eigenen Tick-Handler braucht.
            dst[14] = reinterpret_cast<uint32_t>(&isr_pendsv);

            // Opt-in-Patches, die Gast-Instruktionen auf SVC-Traps umlenken
            // (gemeinsamer Dispatcher isr_svc, Vektor-Slot 11):
            //   WFI-Pin-Wakeup : WFI      -> SVC #0
            //   PRIMASK-Schatten: CPSID i -> SVC #1, CPSIE i -> SVC #2
            bool need_svc = false;
            if (config::wfi_pin_wakeup()) {
                uint32_t n = hex_patcher::patch_wfi_to_svc(
                    reinterpret_cast<uint8_t*>(dst), sz, VEC_COUNT * 4);
                need_svc = true;
                std::printf("[EMU] WFI-Pin-Wakeup aktiv: %u WFI gepatcht\n",
                            static_cast<unsigned>(n));
            }
            if (config::primask_shadow()) {
                uint32_t n = hex_patcher::patch_cps_to_svc(
                    reinterpret_cast<uint8_t*>(dst), sz, VEC_COUNT * 4);
                need_svc = true;
                std::printf("[EMU] PRIMASK-Schatten aktiv: %u CPSID/CPSIE gepatcht\n",
                            static_cast<unsigned>(n));
            }
            if (need_svc) {
                dst[11] = reinterpret_cast<uint32_t>(&isr_svc);
            }
        }

        // RAM-Adressen aus Literal-Pools relocaten (LPC RAM 0x10000000+8KB
        // → RP2350 g_guest_ram).
        uint32_t ram_base = reinterpret_cast<uint32_t>(g_guest_ram);
        auto pr = hex_patcher::relocate_ram_refs(
            reinterpret_cast<uint8_t*>(dst), sz,
            0x1000'0000u, emulator::LPC_GUEST_RAM_SIZE,
            ram_base);
        std::printf("[EMU] hex-patch: %u/%u Woerter relociert\n",
                    static_cast<unsigned>(pr.patched_words),
                    static_cast<unsigned>(pr.scanned_words));

        // VTOR setzen (muss 256-Byte-aligned sein, Array ist alignas(256)).
        uint32_t load_base = reinterpret_cast<uint32_t>(g_firmware_image);
        SCB->VTOR = load_base;
        __DSB(); __ISB();

        // MPU für den Gast scharf schalten.
        mpu_setup::enable_for_guest();

        uint32_t initial_sp = dst[0];
        uint32_t reset_h    = dst[1];
        std::printf("[EMU] starting: SP=0x%08lx PC=0x%08lx (load=0x%08lx, %u B)\n",
                    static_cast<unsigned long>(initial_sp),
                    static_cast<unsigned long>(reset_h),
                    static_cast<unsigned long>(load_base),
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

uint32_t load_base() {
    return reinterpret_cast<uint32_t>(g_firmware_image);
}

uint32_t guest_ram_base() {
    return reinterpret_cast<uint32_t>(g_guest_ram);
}

} // namespace emulator
