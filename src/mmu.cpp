#include "mmu.h"
#include "emulator.h"   // LPC_LOAD_MAX_SIZE, LPC_GUEST_RAM_SIZE

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
// Region-Layout für den Gast (eng gefasst — NUR Gast-eigene Bereiche):
//   0: Code-Image     firmware_base  .. +64 KB  (RWX, Normal)
//   1: Gast-RAM       guest_ram_base .. +8 KB   (RWX, Normal)
//   2: enter_guest    trampoline     .. +32 B   (RX,  Normal)
//   3: PPB Teil A     0xE0000000-0xE000E0FF      (RW,  Device, XN)
//   4: PPB Teil B1    0xE000E500-0xE000ECFF      (RW,  Device, XN)
//   5: SCB-Ctrl-Block 0xE000ED00-0xE000ED1F      (RO,  Device, XN) -> AIRCR-Trap
//   6: PPB Teil B2    0xE000ED20-0xE00FFFFF      (RW,  Device, XN)
//
// WICHTIG: Frueher gab Region 0 das GESAMTE 520-KB-SRAM unprivileged RWX frei.
// Ein wilder Gast-Pointer, Heap- oder Stack-Ueberlauf konnte damit TinyUSB-
// Puffer, Core-Stacks und Emulator-Daten still ueberschreiben -> USB-Stack-
// Korruption, CLI-Verlust, Host-Instabilitaet. Jetzt sind nur die drei Gast-
// Bereiche freigegeben; jeder Zugriff darueber hinaus trappt sauber.
// PPB wird whitelistet, damit der Gast SCB/SysTick unprivileged sieht (NVIC
// 0xE000E100-0xE000E4FF bleibt ausgespart -> vnvic-Trap).

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

void enable_for_guest(uint32_t firmware_base, uint32_t guest_ram_base,
                      uint32_t trampoline_base) {
    MPU->CTRL = 0;
    __DSB(); __ISB();

    // MAIR: Idx0 Normal-WB, Idx1 Device-nGnRE
    MPU->MAIR0 = (0xFFu << 0) | (0x04u << 8);
    MPU->MAIR1 = 0;

    uint32_t r = 0;

    // Gast-Code-Image (64 KB, RWX): Vektortabelle (VTOR), Gast-Code und die als
    // Flash gemappten Daten. firmware_base ist alignas(256) -> 32-Byte-tauglich.
    set_region(r++,
        make_rbar(firmware_base, SH_INNER, AP_RW_ANY, /*xn=*/false),
        make_rlar(firmware_base + emulator::LPC_LOAD_MAX_SIZE - 1u,
                  ATTR_NORMAL, true));

    // Gast-RAM (8 KB, RWX — manche Firmware fuehrt Code aus dem RAM aus, etwa
    // ins RAM kopierte IAP-Routinen). guest_ram_base ist alignas(32).
    set_region(r++,
        make_rbar(guest_ram_base, SH_INNER, AP_RW_ANY, /*xn=*/false),
        make_rlar(guest_ram_base + emulator::LPC_GUEST_RAM_SIZE - 1u,
                  ATTR_NORMAL, true));

    // enter_guest-Trampolin (32 B, RX). Wird nach `msr control` (nPRIV=1) noch
    // unprivileged ausgefuehrt (isb; bx r1) und MUSS daher fuer den Gast
    // ausfuehrbar sein. aligned(32) in emulator.cpp garantiert, dass diese
    // 32-Byte-Region die ganze Funktion abdeckt. Nur RX (kein W) -> der Gast
    // kann das Trampolin nicht modifizieren.
    set_region(r++,
        make_rbar(trampoline_base, SH_INNER, AP_RO_ANY, /*xn=*/false),
        make_rlar(trampoline_base + 31u, ATTR_NORMAL, true));

    // PPB Teil A: 0xE0000000 - 0xE000_E0FF (DCB, ITM, DWT, FPB, SCB-Anfang,
    // SysTick). RW any, Device, XN.
    set_region(r++,
        make_rbar(LPC_PRIVPERI_BASE, SH_NS, AP_RW_ANY, /*xn=*/true),
        make_rlar(0xE000'E0FFu, ATTR_DEVICE, true));

    // NVIC-Region 0xE000_E100 - 0xE000_E4FF wird *nicht* whitelistet.
    // Jeder Gast-Zugriff darauf trapt → vnvic::read8/write8.

    // PPB Teil B1: 0xE000_E500 - 0xE000_ECFF (SCB-Tail, CPUID, ICTR, ACTLR,
    // SysTick-Kalibrierung usw.). RW any, Device, XN.
    set_region(r++,
        make_rbar(0xE000'E500u, SH_NS, AP_RW_ANY, /*xn=*/true),
        make_rlar(0xE000'ECFFu, ATTR_DEVICE, true));

    // SCB-Kontroll-Block 0xE000_ED00 - 0xE000_ED1F: READ-ONLY fuer den Gast.
    // Enthaelt CPUID/ICSR/VTOR/AIRCR/SCR/CCR/SHPR1/SHPR2. Schreibzugriffe
    // trappen (MemManage) -> fault.cpp emuliert sie. KRITISCH: AIRCR (0xED0C)
    // mit SYSRESETREQ (NVIC_SystemReset) wuerde nativ das ECHTE RP2350-Silizium
    // rebooten; durch das Trapping faengt der Fault-Handler es ab und startet
    // nur den Gast neu. Reads (CPUID/ICSR-Status/...) bleiben nativ erlaubt.
    set_region(r++,
        make_rbar(0xE000'ED00u, SH_NS, AP_RO_ANY, /*xn=*/true),
        make_rlar(0xE000'ED1Fu, ATTR_DEVICE, true));

    // PPB Teil B2: 0xE000_ED20 - 0xE00F_FFFF (SHPR3, SHCSR, CFSR, HFSR, MMFAR,
    // BFAR, MPU-Register, DWT, ROM-Table ...). RW any, Device, XN.
    set_region(r++,
        make_rbar(0xE000'ED20u, SH_NS, AP_RW_ANY, /*xn=*/true),
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
