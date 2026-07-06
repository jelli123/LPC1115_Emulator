#include "irq_inject.h"
#include "vnvic.h"
#include "lpc_irqs.h"
#include "emulator.h"
#include "target_halt.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <atomic>

#include "RP2350.h"
#include "hardware/sync.h"
#include "pico/multicore.h"   // get_core_num()

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
// Handler. Dieser laeuft damit im THREAD-Mode (nicht Handler-Mode!) mit
// LR=0xFFFFFFFD. Kehrt er regulaer zurueck (POP {..,pc} / BX LR mit dem
// 0xFFFFFFFD), ist das im Thread-Mode KEINE gueltige Exception-Rueckkehr: der
// Core faultet beim Anspringen von 0xFFFFFFFC. Dieser Fault wird im Fault-
// Handler (fault.cpp try_injected_irq_return) abgefangen, der den darunter
// liegenden Original-Frame freilegt und den Gast nahtlos fortsetzt. Der frueher
// hier angenommene "der Core pop't den Original-Frame automatisch" gilt NUR im
// Handler-Mode und trifft auf diese Thread-Mode-Injektion NICHT zu.

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

// Liest aus der Gast-Vector-Tabelle den IRQ-Handler.
uint32_t lookup_handler(uint8_t lpc_irq) {
    auto* vt = reinterpret_cast<uint32_t*>(emulator::vtable_base());
    uint32_t v = vt[16 + lpc_irq];
    return v;                  // mit Thumb-Bit
}

// SysTick-Exception (Slot 15) pending-Flag. Wird vom Host-SysTick-Modell
// (peripherals.cpp) via pend_systick() gesetzt und in pendsv_inject_c mit
// Vorrang vor den NVIC-IRQs konsumiert.
std::atomic<bool> g_systick_pending{false};

// Verschachtelungstiefe der injizierten Handler (nur auf Core1 veraendert:
// inject_frame ++ / note_injected_return --). Injizierte Frames laufen im
// THREAD-Mode und haben KEINEN HW-"active"-Schutz wie echte Exceptions -> ohne
// dieses Gate koennte ein sich selbst neu pendender IRQ (UART0-RX im RBR-Read)
// seinen eigenen, noch laufenden Handler rekursiv preempten (ein Frame pro Byte)
// -> PSP-Stack-Overflow -> Gast-RAM-Korruption -> UNDEFINSTR. g_inject_depth_max
// nur fuer die 'stats'-Diagnose (Core0 liest lesend).
int g_inject_depth = 0;
std::atomic<uint32_t> g_inject_depth_max{0};

// Synthetisiert einen Exception-Frame fuer 'handler' oben auf den Gast-PSP,
// sodass beim PendSV-EXC_RETURN der Gast-Handler im Thread-Mode anlaeuft. Der
// Ruecksprung (LR=0xFFFFFFFD) wird vom Fault-Handler (try_injected_irq_return)
// abgefangen und der Original-Frame freigelegt. Gemeinsam fuer IRQs + SysTick.
void inject_frame(uint32_t handler) {
    auto* psp = read_guest_psp();
    auto* base = psp;
    if ((reinterpret_cast<uintptr_t>(base) & 4u) != 0u) {
        --base;                         // 4 Byte Padding fuer 8-Byte-Alignment
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
    ++g_inject_depth;
    uint32_t d = static_cast<uint32_t>(g_inject_depth);
    if (d > g_inject_depth_max.load(std::memory_order_relaxed))
        g_inject_depth_max.store(d, std::memory_order_relaxed);
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

void pend_systick() {
    g_systick_pending.store(true, std::memory_order_release);
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
    // PendSV gehoert ausschliesslich dem Gast-Core (Core1). Der emulatoreigene
    // isr_pendsv ueberschreibt das schwache SDK-PendSV-Symbol und ist daher
    // AUCH in Core0s Vektortabelle installiert. Wuerde PendSV jemals auf Core0
    // gepended (z.B. target_halt::request_halt() aus der CLI, die SCB->ICSR=
    // PENDSVSET auf Core0 schreibt), liefe Core0 sonst in den Halt-Pfad und
    // bliebe in der Resume-Spinschleife haengen (CLI-Freeze) bzw. wuerde einen
    // Exception-Frame auf Core0s (falschem) PSP synthetisieren. Auf Core0 daher
    // sofort und folgenlos zurueckkehren — Injektion/Halt sind nur auf Core1 gueltig.
    if (get_core_num() != 1u) return;

    // Halt-Anforderungen haben Vorrang (Debugger-Pfad).
    target_halt::on_pendsv_check();

    // PRIMASK-Schatten (opt-in): Befindet sich der Gast in einer kritischen
    // Sektion (__disable_irq()), wird der IRQ nicht ausgeliefert, bleibt aber
    // im vNVIC pending und feuert beim Verlassen (CPSIE -> erneutes PendSV).
    if (vnvic::primask()) return;

    // SysTick-Exception hat Vorrang (hoehere Exception-Prioritaet als IRQs) und
    // wird ueber denselben Frame-Mechanismus injiziert. Handler = Gast-vtable[15].
    if (g_systick_pending.exchange(false, std::memory_order_acq_rel)) {
        auto* vt = reinterpret_cast<uint32_t*>(emulator::vtable_base());
        uint32_t h = vt[15];
        if (h != 0u && (h & ~1u) != 0u) {
            inject_frame(h);
            return;   // ein Frame pro PendSV-Lauf; IRQs folgen beim naechsten
        }
    }

    // Solange Pending+Enabled vorliegt, einen Frame synthetisieren.
    // RE-ENTRANCY-GATE: Injizierte Handler laufen im THREAD-Mode und haben KEINEN
    // HW-"active"-Schutz wie echte Exceptions. Laeuft bereits ein injizierter
    // Handler (g_inject_depth>0), darf KEIN weiterer Frame injiziert werden —
    // sonst preemptet z. B. der UART0-RX-IRQ (der sich im RBR-Read selbst neu
    // pendet) seinen eigenen, noch laufenden Handler: pro empfangenem Byte ein
    // neuer Frame auf dem PSP -> unbeschraenkte Rekursion -> PSP-Stack-Overflow
    // -> Gast-RAM-Korruption -> UNDEFINSTR (bei FT12/knxd-Dauerstrom reproduziert).
    // Der IRQ bleibt im vNVIC pending und wird beim Ruecksprung des laufenden
    // Handlers (note_injected_return) nachgeliefert = Tail-Chaining wie in HW.
    if (g_inject_depth > 0) return;

    int8_t n = take_next_irq();
    if (n < 0) return;

    uint32_t handler = lookup_handler(static_cast<uint8_t>(n));
    if (handler == 0u || (handler & ~1u) == 0u) {
        // Kein gültiger Handler — fallback: zurück, ohne Injektion.
        return;
    }

    inject_frame(handler);
    // KEIN sofortiges Tail-Chain-Re-Pend hier: weitere pending IRQs werden erst
    // beim Ruecksprung DIESES Handlers (note_injected_return) geliefert. Ein
    // sofortiges PENDSVSET wuerde jetzt ohnehin am Tiefen-Gate abprallen.
}

void note_injected_return() {
    if (g_inject_depth > 0) --g_inject_depth;
    // Beim Verlassen des aeussersten injizierten Handlers einen evtl. noch
    // pendenden IRQ nachliefern (Tail-Chaining). PENDSVSET ist atomar/kontextfrei.
    if (g_inject_depth == 0 && vnvic::irq_pending())
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

uint32_t inject_depth_max() {
    return g_inject_depth_max.load(std::memory_order_relaxed);
}

uint32_t inject_depth_live() {
    int d = g_inject_depth;
    return d < 0 ? 0u : static_cast<uint32_t>(d);
}

void reset_inject_depth() {
    g_inject_depth = 0;
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
