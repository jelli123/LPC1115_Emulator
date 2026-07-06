#pragma once
//
// Automatische IRQ-Injektion für den emulierten LPC1115-Gast.
//
// Der Schatten-NVIC (vnvic) hält pending- und enable-Bits. Sobald ein
// emuliertes Peripheriemodell sein Event signalisiert (z. B. UART0
// RX-Buffer non-empty, CT16 Match), ruft es vnvic::pend_irq(N). Wenn
// das entsprechende ISER-Bit gesetzt ist, signalisieren wir an den Host
// PendSV via SCB->ICSR.PENDSVSET. Der PendSV-Handler im Host läuft
// privileged → er schreibt einen *frischen* Exception-Frame auf den
// PSP-Stack des Gastes, sodass beim EXC_RETURN der zugehörige LPC-IRQ-
// Handler aus dem Gast-Vector ausgeführt wird. Der Gast ist sich der
// Manipulation nicht bewusst — für ihn sieht es wie ein echter
// NVIC-Trigger aus.
//

#include <cstdint>

namespace irq_inject {

void init();

// Wird von Peripherie-Modellen aufgerufen.
void pend(uint8_t lpc_irq);

// SysTick-Exception (Vektor-Slot 15) in den Gast injizieren. SysTick ist auf
// dem Cortex-M33 privilegiert-only und wird daher vom Host emuliert
// (peripherals.cpp systick_advance); bei einem Reload-Unterlauf mit gesetztem
// TICKINT ruft das Modell diese Funktion, die den Gast-SysTick-Handler
// (vtable[15]) ueber denselben PendSV-Frame-Mechanismus wie IRQs ausfuehrt.
void pend_systick();

// Im Host-Loop pollen, falls direkter ICSR-Schreibvorgang aus IRQ-fremdem
// Code unsicher wäre (PendSV-Set ist atomar, daher leer).
void poll();

// Vom PendSV-Asm-Wrapper gerufen: führt eine Iteration der Injektion durch.
extern "C" void pendsv_inject_c();

// Vom Fault-Handler (try_injected_irq_return) beim Ruecksprung eines injizierten
// Handlers aufgerufen: dekrementiert die Injektions-Verschachtelungstiefe und
// liefert (Tail-Chaining) einen noch pendenden IRQ nach. Verhindert zusammen mit
// dem Tiefen-Gate in pendsv_inject_c die rekursive Selbst-Preemption eines im
// Thread-Mode laufenden Handlers (z. B. UART0-RX, der sich im RBR-Read neu pendet).
void note_injected_return();

// Groesste je erreichte Injektions-Verschachtelungstiefe (Diagnose via 'stats').
// >1 = ein injizierter Handler wurde (frueher) rekursiv preemptet.
uint32_t inject_depth_max();

// AKTUELLE (live) Injektions-Verschachtelungstiefe. 1 = Gast steckt gerade in
// GENAU einem injizierten Handler; >1 = Verschachtelung; bleibt es dauerhaft 1,
// haengt der Gast in einer ISR (z. B. Timer-IRQ-Sturm).
uint32_t inject_depth_live();

// Injektions-Verschachtelungstiefe auf 0 zuruecksetzen. MUSS bei jedem Gast-
// (Neu)start gerufen werden: nach einem fatalen Fault innerhalb eines injizierten
// Handlers bleibt g_inject_depth>0 stehen (der Handler kehrt nie zurueck); ohne
// Reset wuerde das Tiefen-Gate nach dem Neustart JEDE IRQ-Injektion blockieren.
void reset_inject_depth();

} // namespace irq_inject
