#pragma once
//
// Fault-Handler-Setup für den Native-Execution-Modus.
//
// Bei einem MemManage- oder BusFault aus dem Gast-Kontext (unprivileged
// Thread-Mode) decoded der Handler die Speicher-Zugriffs-Instruktion und
// forwarded sie an peripherals::mmio_*. Anschließend wird PC um die
// Instruktionslänge inkrementiert und in den Gast zurückgekehrt.
//

#include <cstdint>

namespace faultsys {

// Statistik
struct Stats {
    uint64_t mem_traps;          // erfolgreich emulierte MMIO-Zugriffe
    uint64_t real_faults;        // nicht decodierbare / unzulässige Faults
    uint32_t last_fault_pc;
    uint32_t last_fault_addr;
};

void init();
Stats stats();

} // namespace faultsys

void setup_fault_handlers();
