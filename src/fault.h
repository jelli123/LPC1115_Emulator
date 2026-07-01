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

// --- Fault-Report (fuer USB-MSC FAULT.TXT) --------------------------------
// Bei einem fatalen Gast-Fault sammelt der Fault-Handler (Core1) die komplette
// Diagnose zusaetzlich zum seriellen printf in einem RAM-Puffer. Core0 (USB-MSC)
// legt den Text als FAULT.TXT auf das Laufwerk, damit ein Fehler auch ohne
// CLI/Serial analysierbar ist. Der Puffer wird beim Beginn jeder Fault-Sequenz
// zurueckgesetzt und am Ende (enter_fatal_halt) freigegeben.
bool        report_ready();     // true: neuer, noch nicht abgeholter Report da
const char* report_data();      // NUL-terminierter Report-Text
uint32_t    report_length();    // Laenge ohne NUL
void        clear_report();      // vom Konsumenten (Core0) nach Abholung

} // namespace faultsys

void setup_fault_handlers();
