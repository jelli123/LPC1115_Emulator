#pragma once
//
// ADIv5 SWD-Target auf zwei externen RP2350-GPIOs.
//
// Bewusste Auslassungen (im README dokumentiert):
//   - Multi-Drop-SWD (Targetsel-Selektion mehrerer Targets an einem Bus)
//   - JTAG-Mode (nur SWD)
//   - Dormant State / Activation Code Sequence (selten genutzt)
//   - Banked APs außer AHB-AP[0]
//
// Implementiert:
//   - SWD line reset, JTAG-to-SWD (0xE79E), 8-bit Header mit Parity,
//     ACK (OK/WAIT/FAULT), 32-bit Daten + Parity, Turnaround
//   - DP-Register: DPIDR(0x00), CTRL/STAT(0x04), SELECT(0x08), RDBUFF(0x0C),
//     ABORT(0x00 W)
//   - AHB-AP[0]: CSW(0x00), TAR(0x04), DRW(0x0C), IDR(0xFC banked)
//   - Auto-Increment im CSW (Single, Packed)
//   - DHCSR/DCRSR/DCRDR/DEMCR über target_halt
//   - FPB FP_CTRL/FP_COMPx → BKPT-Insertion via target_halt::set_breakpoint
//   - CoreSight-ROM-Table @ 0xE00FF000 mit LPC1115-Komponenten
//

#include <cstdint>

namespace swd_target {

struct PinAssignment {
    int8_t  swclk;
    int8_t  swdio;
};

constexpr uint32_t LPC1115_DPIDR     = 0x0BB1'1477u;
constexpr uint32_t LPC1115_AHBAP_IDR = 0x0477'0031u;
constexpr uint32_t ROM_TABLE_ADDR    = 0xE00F'F000u;

void init();
bool start(PinAssignment p);
void stop();
bool active();

// Im Host-Loop pollen — drainiert PIO-RX-FIFO und treibt die Statemachine.
void poll();

struct Stats {
    uint64_t packets_ok;
    uint64_t packets_fault;
    uint64_t mem_reads;
    uint64_t mem_writes;
    uint32_t last_dp_idcode;
};
Stats stats();

} // namespace swd_target
