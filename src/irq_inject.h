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

// Im Host-Loop pollen, falls direkter ICSR-Schreibvorgang aus IRQ-fremdem
// Code unsicher wäre (PendSV-Set ist atomar, daher leer).
void poll();

// Vom PendSV-Asm-Wrapper gerufen: führt eine Iteration der Injektion durch.
extern "C" void pendsv_inject_c();

} // namespace irq_inject
