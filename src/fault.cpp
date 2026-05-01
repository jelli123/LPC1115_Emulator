#include "fault.h"
#include "opcodes.h"
#include "peripherals.h"
#include "gdb_stub.h"
#include "iap.h"

#include <cstdio>
#include <cstdint>
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "RP2350.h"

namespace faultsys {

namespace {
Stats g_stats{};
}

Stats stats() { return g_stats; }

void init() { g_stats = {}; }

} // namespace faultsys

// --- Trap-Handler (in C, von Asm-Wrapper aus aufgerufen) ------------------
//
// frame zeigt auf den exception-stacked Frame des Gastes
// (r0,r1,r2,r3,r12,lr,pc,xpsr). r4_r11_lr zeigt auf den vom Asm-Wrapper
// gepushten Block {r4,r5,r6,r7,r8,r9,r10,r11,lr_excret}. SP des Gastes
// während der gefehlerten Instruktion = (uint32_t)frame + 0x20 (8 Dwords).
//
// Zugriffe auf den unprivilegierten Gast-Stack durch privilegierten Code
// sind erlaubt (PRIVDEFENA=1, MPU schränkt nur unprivileged ein).

extern "C" void handle_memfault_c(trap_decoder::StackedFrame* frame,
                                  uint32_t* r4_r11_lr) {
    using namespace trap_decoder;

    // IAP-ROM-Trap: Guest hat über Funktionspointer 0x1FFF1FF1 in den
    // BootROM gesprungen. Adresse existiert auf RP2350 nicht → Prefetch-
    // Fault. Wir bedienen den IAP-Aufruf und springen über LR zurück.
    if ((frame->pc & ~1u) == iap::ROM_ENTRY_TARGET) {
        auto* params  = reinterpret_cast<uint32_t*>(frame->r0);
        auto* results = reinterpret_cast<uint32_t*>(frame->r1);
        iap::dispatch(params, results);
        frame->pc = frame->lr & ~1u;
        SCB->CFSR = SCB->CFSR;
        ++faultsys::g_stats.mem_traps;
        return;
    }

    uint32_t* r4_r11 = r4_r11_lr;       // 8 Werte
    uint32_t  guest_sp = reinterpret_cast<uint32_t>(frame) + 0x20;

    Access acc{};
    if (!decode_mem_access(frame, r4_r11, guest_sp, acc)) {
        ++faultsys::g_stats.real_faults;
        faultsys::g_stats.last_fault_pc   = frame->pc;
        faultsys::g_stats.last_fault_addr = SCB->BFAR;
        std::printf("[FAULT] non-decodable @PC=0x%08lx CFSR=0x%08lx BFAR=0x%08lx\n",
                    static_cast<unsigned long>(frame->pc),
                    static_cast<unsigned long>(SCB->CFSR),
                    static_cast<unsigned long>(SCB->BFAR));
        SCB->CFSR = SCB->CFSR;          // Sticky-Bits clearen
        // Endlosschleife mit Watchdog-Reset.
        watchdog_enable(50, true);
        for (;;) tight_loop_contents();
    }

    // Emulieren
    bool ok = true;
    if (acc.is_load) {
        uint32_t value = 0;
        switch (acc.size) {
            case AccessSize::B: {
                uint8_t v = 0;
                ok = peripherals::mmio_read8(acc.address, v);
                value = acc.is_signed
                    ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)))
                    : v;
                break;
            }
            case AccessSize::H: {
                uint8_t lo = 0, hi = 0;
                ok = peripherals::mmio_read8(acc.address, lo) &&
                     peripherals::mmio_read8(acc.address + 1, hi);
                uint16_t h = static_cast<uint16_t>(lo | (hi << 8));
                value = acc.is_signed
                    ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(h)))
                    : h;
                break;
            }
            case AccessSize::W: {
                uint8_t b[4]{};
                for (int i = 0; i < 4 && ok; ++i)
                    ok = peripherals::mmio_read8(acc.address + i, b[i]);
                value = static_cast<uint32_t>(b[0])       |
                        (static_cast<uint32_t>(b[1]) << 8) |
                        (static_cast<uint32_t>(b[2]) << 16) |
                        (static_cast<uint32_t>(b[3]) << 24);
                break;
            }
        }
        uint32_t* dst = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
        if (dst) *dst = value;
    } else {
        uint32_t  src_val = 0;
        if (acc.rt < 16) {
            uint32_t* p = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
            if (p) src_val = *p;
        }
        switch (acc.size) {
            case AccessSize::B:
                ok = peripherals::mmio_write8(acc.address,
                                              static_cast<uint8_t>(src_val & 0xFFu));
                break;
            case AccessSize::H:
                ok = peripherals::mmio_write8(acc.address,
                                              static_cast<uint8_t>(src_val & 0xFFu)) &&
                     peripherals::mmio_write8(acc.address + 1,
                                              static_cast<uint8_t>((src_val >> 8) & 0xFFu));
                break;
            case AccessSize::W:
                for (int i = 0; i < 4 && ok; ++i)
                    ok = peripherals::mmio_write8(acc.address + i,
                                                  static_cast<uint8_t>((src_val >> (i*8)) & 0xFFu));
                break;
        }
    }

    if (!ok) {
        ++faultsys::g_stats.real_faults;
        faultsys::g_stats.last_fault_pc   = frame->pc;
        faultsys::g_stats.last_fault_addr = acc.address;
        std::printf("[FAULT] mmio rejected @PC=0x%08lx addr=0x%08lx\n",
                    static_cast<unsigned long>(frame->pc),
                    static_cast<unsigned long>(acc.address));
        SCB->CFSR = SCB->CFSR;
        watchdog_enable(50, true);
        for (;;) tight_loop_contents();
    }

    ++faultsys::g_stats.mem_traps;
    // Post-Hook (z. B. PLL-Re-Konfiguration nach abgeschlossenem 32-bit-Wort).
    if (!acc.is_load) peripherals::on_post_write_hook();
    // PC vorrücken
    frame->pc += acc.instr_size;
    // Sticky-Bits clearen (CFSR write-1-to-clear)
    SCB->CFSR = SCB->CFSR;
}

// --- Naked Exception-Handler ---------------------------------------------
// Der Pico-SDK-Linker erwartet die ISR-Symbole `isr_*`. Wir registrieren
// MemManage und BusFault. UsageFault (z. B. UDF, illegal opcode) und
// HardFault behandeln wir als echten Fault → Watchdog-Reset.

extern "C" __attribute__((naked)) void isr_busfault() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"   // 8-Byte-Alignment
        "add   r1, sp, #4          \n"   // r1 = &{r4..r11, lr_excret}
        "bl    handle_memfault_c   \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

extern "C" __attribute__((naked)) void isr_memmanage() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    handle_memfault_c   \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

extern "C" void isr_hardfault() {
    watchdog_enable(50, true);
    while (true) { tight_loop_contents(); }
}

// Bei aktivem GDB-Stub fängt UsageFault BKPT-Instruktionen (Software-
// Breakpoints) und delegiert an den Stub. Andernfalls Watchdog-Reset.
extern "C" void usagefault_c(uint32_t* exc_frame, uint32_t* r4_r11) {
    if (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk) {
        uint16_t instr = *reinterpret_cast<uint16_t*>(exc_frame[6]);
        if ((instr & 0xFF00) == 0xBE00) {            // BKPT
            if (gdb_stub::active()) {
                gdb_stub::on_breakpoint(exc_frame, r4_r11);
                SCB->CFSR = SCB->CFSR;
                return;
            }
        }
    }
    SCB->CFSR = SCB->CFSR;
    watchdog_enable(50, true);
    while (true) { tight_loop_contents(); }
}

extern "C" __attribute__((naked)) void isr_usagefault() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    usagefault_c        \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

// DebugMonitor: M33 löst diese Exception nach genau einer ausgeführten
// Instruktion aus, sobald DEMCR.MON_EN=1 und MON_STEP=1 sind und
// DHCSR.C_DEBUGEN=0 (kein DAP angehängt). Wir nutzen das für sauberes
// GDB-Single-Step.
extern "C" void debugmon_c(uint32_t* exc_frame, uint32_t* r4_r11) {
    // MON_STEP wieder löschen, sonst tritt die Exception erneut auf.
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_STEP_Msk;
    if (gdb_stub::active()) {
        gdb_stub::on_breakpoint(exc_frame, r4_r11);
    }
}

extern "C" __attribute__((naked)) void isr_debugmon() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    debugmon_c          \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

void setup_fault_handlers() {
    faultsys::init();
}
