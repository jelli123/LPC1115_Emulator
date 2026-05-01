#include "mmu.h"

#include <cstdint>

#include "hardware/structs/mpu.h"
#include "RP2350.h"

// ARMv8-M MPU auf RP2350 Cortex-M33.
//
// Strategie:
//   * Gast läuft unprivileged (CONTROL.nPRIV=1).
//   * Host (Handler, CLI, alle Pico-SDK-Aufrufe) läuft privileged.
//   * MPU.CTRL.PRIVDEFENA=1 → privileged code sieht den vollen Default-
//     Memory-Map (kann also RP2350-Hardware bei 0x40000000 lesen/schreiben,
//     selbst wenn dort keine MPU-Region steht).
//   * Für den Gast (unprivileged) sind nur explizit konfigurierte Regionen
//     erlaubt. LPC-Peripherie 0x40000000-0x4FFFFFFF ist *nicht* konfiguriert
//     → jeder Gast-Zugriff dorthin schlägt mit MemManageFault fehl, der
//     Handler decodet die Instruktion und forwardet sie nach
//     peripherals::mmio_*.
//
// Region-Layout für den Gast:
//   0: Gast-RAM/Code-Region  0x20000000-0x20081FFF (520 KB SRAM, RWX)
//   1: PPB                   0xE0000000-0xE00FFFFF (RW, Device, XN)
//
// PPB liegt vorne im Default-Memory-Map ohnehin, aber wir whitelisten ihn,
// damit der Gast SCB/SysTick/NVIC unprivileged sehen darf.

namespace mpu_setup {

namespace {

constexpr uint32_t AP_RW_ANY   = 1u << 1;
constexpr uint32_t AP_RO_ANY   = 3u << 1;
constexpr uint32_t XN          = 1u << 0;
constexpr uint32_t SH_NS       = 0u << 3;
constexpr uint32_t SH_INNER    = 3u << 3;

constexpr uint32_t ATTR_NORMAL = 0u;
constexpr uint32_t ATTR_DEVICE = 1u;

inline uint32_t make_rbar(uint32_t base, uint32_t sh, uint32_t ap, bool xn) {
    return (base & 0xFFFF'FFE0u) | sh | ap | (xn ? XN : 0u);
}
inline uint32_t make_rlar(uint32_t limit_inclusive, uint32_t attr_idx, bool enable) {
    return (limit_inclusive & 0xFFFF'FFE0u) | (attr_idx << 1) | (enable ? 1u : 0u);
}

void set_region(uint32_t idx, uint32_t rbar, uint32_t rlar) {
    MPU->RNR  = idx;
    MPU->RBAR = rbar;
    MPU->RLAR = rlar;
}

} // namespace

void enable_for_guest() {
    MPU->CTRL = 0;
    __DSB(); __ISB();

    // MAIR: Idx0 Normal-WB, Idx1 Device-nGnRE
    MPU->MAIR0 = (0xFFu << 0) | (0x04u << 8);
    MPU->MAIR1 = 0;

    uint32_t r = 0;

    // Gast-SRAM (RP2350 SRAM 520 KB).
    set_region(r++,
        make_rbar(0x2000'0000u, SH_INNER, AP_RW_ANY, /*xn=*/false),
        make_rlar(0x2008'1FFFu, ATTR_NORMAL, true));

    // PPB Teil A: 0xE0000000 - 0xE000_E0FF (DCB, ITM, DWT, FPB, SCB-Anfang,
    // SysTick). RW any, Device, XN.
    set_region(r++,
        make_rbar(LPC_PRIVPERI_BASE, SH_NS, AP_RW_ANY, /*xn=*/true),
        make_rlar(0xE000'E0FFu, ATTR_DEVICE, true));

    // NVIC-Region 0xE000_E100 - 0xE000_E4FF wird *nicht* whitelistet.
    // Jeder Gast-Zugriff darauf trapt → vnvic::read8/write8.

    // PPB Teil B: 0xE000_E500 - 0xE00F_FFFF (Rest des PPB inkl. SCB-Tail,
    // CPUID, ICTR, ACTLR, CFSR usw.).
    set_region(r++,
        make_rbar(0xE000'E500u, SH_NS, AP_RW_ANY, /*xn=*/true),
        make_rlar(LPC_PRIVPERI_END - 1, ATTR_DEVICE, true));

    // Restliche Regionen disabled lassen.
    for (; r < 8; ++r) {
        MPU->RNR = r;
        MPU->RBAR = 0;
        MPU->RLAR = 0;
    }

    __DSB();
    __ISB();
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk
              | MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();

    // MemManage/BusFault/UsageFault aktivieren.
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
               |  SCB_SHCSR_BUSFAULTENA_Msk
               |  SCB_SHCSR_USGFAULTENA_Msk;
}

void disable() {
    MPU->CTRL = 0;
    __DSB(); __ISB();
}

} // namespace mpu_setup
