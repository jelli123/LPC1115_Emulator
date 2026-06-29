#pragma once
//
// Virtueller NVIC für die LPC1115. Die NVIC-Region des Cortex-M33
// (0xE000_E100 – 0xE000_E4FF) wird per MPU-Split aus dem PPB ausgenommen
// und im Trap-Handler über diese API virtualisiert. Damit kann der Gast
// IRQs anhand seiner LPC-IRQ-Nummern (0..31) konfigurieren, ohne den
// Host-NVIC zu beeinflussen.
//

#include <cstdint>

namespace vnvic {

constexpr uint32_t NVIC_BASE = 0xE000'E100;
constexpr uint32_t NVIC_END  = 0xE000'E500;

bool is_nvic_addr(uint32_t addr);

uint8_t  read8 (uint32_t addr);
void     write8(uint32_t addr, uint8_t val);

// Vom Host genutzt: signalisiert dem Gast einen Pending-IRQ. Wenn der Gast
// diesen IRQ aktiviert hat, wird beim nächsten Trap-Eintritt der Gast-LR
// auf den IRQ-Vektor umgebogen (vereinfachtes Modell — kein vollständiges
// Tail-Chaining, ausreichend für SBLib-Polling).
void     pend_irq(uint8_t lpc_irq);

// Ist IRQ aktiv (vom Gast enabled UND pending)?
bool     irq_pending();
uint8_t  next_pending_irq();
void     clear_pending(uint8_t lpc_irq);

// PRIMASK-Schatten (opt-in via config::primask_shadow). Der Gast laeuft
// unprivilegiert, daher ignoriert die M33-Hardware CPSID/CPSIE. Werden diese
// per Patch auf SVC-Traps umgelenkt, fuehrt der Host hier einen Schatten-
// PRIMASK nach. Ist er gesetzt, unterdrueckt die IRQ-Injektion das Ausliefern
// (der IRQ bleibt im vNVIC pending und feuert beim Verlassen der Sektion).
void     set_primask(bool masked);
bool     primask();

struct Snapshot {
    uint32_t iser;     // 1 Bit / IRQ, gesetzt = enabled
    uint32_t ispr;     // pending
    uint32_t prio[8];  // IPR0..IPR7 (32 IRQs * 8 Bit)
};
Snapshot snapshot();
void     reset();

} // namespace vnvic
