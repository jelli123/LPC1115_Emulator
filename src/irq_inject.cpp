#include "irq_inject.h"
#include "vnvic.h"
#include "lpc_irqs.h"
#include "emulator.h"
#include "target_halt.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "RP2350.h"
#include "hardware/sync.h"

// ---------------------------------------------------------------------------
// IRQ-Injektion via PendSV
// ---------------------------------------------------------------------------
//
// Annahme: Der Gast läuft in Thread Mode (CONTROL.SPSEL=1, nPRIV=1) und ist
// nicht selbst gerade unterbrochen. Wir nutzen PendSV als „Soft-IRQ" und
// schieben einen synthetischen Exception-Frame oben auf den PSP. Die
// Reihenfolge im Frame entspricht ARMv7-M B1.5.6: r0,r1,r2,r3,r12,lr,
// returnPC, xPSR.
//
// Beim EXC_RETURN nach unserem PendSV (LR=0xFFFFFFFD = thread mode, PSP)
// pop't der Core den synthetisierten Frame und springt in den LPC-IRQ-
// Handler. Wenn der Handler mit `BX LR` zurückkehrt (LR ist 0xFFFFFFFD
// im neuen Frame), pop't der Core den darunter liegenden, originalen
// Frame des Gast-Codes — dieser läuft nahtlos weiter.

namespace irq_inject {

namespace {

constexpr uint32_t XPSR_THUMB = 1u << 24;

uint32_t* read_guest_psp() {
    uint32_t psp;
    __asm volatile ("mrs %0, psp" : "=r"(psp));
    return reinterpret_cast<uint32_t*>(psp);
}

void write_guest_psp(uint32_t* p) {
    __asm volatile ("msr psp, %0" :: "r"(p));
    __ISB();
}

// Stack-Bytes, die zwischen Top-of-frame und SP-Alignment-Padding nötig
// sind — laut ARMv8-M-ARM B3.18 wird beim Stacking automatisch auf 8
// ausgerichtet, wir machen es im synthetischen Frame ebenfalls.
struct StackFrame {
    uint32_t r0, r1, r2, r3;
    uint32_t r12, lr, return_pc, xpsr;
};
static_assert(sizeof(StackFrame) == 32, "frame size");

// Liest aus der Gast-Vector-Tabelle (an LPC_LOAD_BASE) den IRQ-Handler.
uint32_t lookup_handler(uint8_t lpc_irq) {
    auto* vt = reinterpret_cast<uint32_t*>(emulator::LPC_LOAD_BASE);
    uint32_t v = vt[16 + lpc_irq];
    return v;                  // mit Thumb-Bit
}

// Atomar: nimm das nächste pending+enabled IRQ und claime es.
int8_t take_next_irq() {
    uint32_t save = save_and_disable_interrupts();
    uint8_t n = vnvic::next_pending_irq();
    if (n != 0xFF) {
        vnvic::clear_pending(n);
    }
    restore_interrupts(save);
    return (n == 0xFF) ? -1 : static_cast<int8_t>(n);
}

} // namespace

void init() {
    // PendSV-Priorität niedrigste (= 0xFF), damit wir keinen aktiven
    // Host-IRQ präemptieren.
    NVIC_SetPriority(PendSV_IRQn, 0xFFu);
}

void pend(uint8_t lpc_irq) {
    if (lpc_irq >= lpc_irq::COUNT) return;
    vnvic::pend_irq(lpc_irq);
    // PendSV anstoßen — atomar, in jedem Kontext zulässig.
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
}

void poll() {
    // Wenn eine Pending-Anforderung bereits gestellt ist und der Gast
    // gerade nicht erreichbar war (z. B. weil ein anderer Handler aktiv
    // war), genügt ein erneutes Setzen — PendSV wird erst getaktet, wenn
    // nichts mehr Höheres läuft.
    if (vnvic::irq_pending()) {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

extern "C" void pendsv_inject_c() {
    // Halt-Anforderungen haben Vorrang (Debugger-Pfad).
    target_halt::on_pendsv_check();

    // Solange Pending+Enabled vorliegt, einen Frame synthetisieren.
    // Wir injizieren maximal einen IRQ pro PendSV-Lauf, um Tail-Chaining
    // dem Hardware-Mechanismus zu überlassen.
    int8_t n = take_next_irq();
    if (n < 0) return;

    uint32_t handler = lookup_handler(static_cast<uint8_t>(n));
    if (handler == 0u || (handler & ~1u) == 0u) {
        // Kein gültiger Handler — fallback: zurück, ohne Injektion.
        return;
    }

    auto* psp = read_guest_psp();

    // Stack-Top muss 8-byte aligned sein, bevor wir den Frame ablegen.
    auto* base = psp;
    if ((reinterpret_cast<uintptr_t>(base) & 4u) != 0u) {
        --base;                         // 4 Byte Padding
        // Padding-Bit im xPSR setzen wir gleich.
    }

    auto* frame = reinterpret_cast<StackFrame*>(base) - 1;

    frame->r0  = 0; frame->r1 = 0; frame->r2 = 0; frame->r3 = 0;
    frame->r12 = 0;
    frame->lr  = 0xFFFF'FFFDu;          // EXC_RETURN: thread, PSP, no FP
    frame->return_pc = handler & ~1u;   // PC ohne Thumb-Bit
    frame->xpsr = XPSR_THUMB
                | (((reinterpret_cast<uintptr_t>(base) & 4u)
                    != (reinterpret_cast<uintptr_t>(psp) & 4u))
                   ? (1u << 9) : 0u);

    write_guest_psp(reinterpret_cast<uint32_t*>(frame));
}

} // namespace irq_inject

// ---------------------------------------------------------------------------
// Naked PendSV-Handler
// ---------------------------------------------------------------------------
extern "C" __attribute__((naked)) void isr_pendsv() {
    __asm volatile (
        "push  {lr}                \n"
        "sub   sp, #4              \n"   // 8-Byte-Alignment
        "bl    pendsv_inject_c     \n"
        "add   sp, #4              \n"
        "pop   {pc}                \n"
    );
}
