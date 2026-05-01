#pragma once
//
// Halt-Bridge für externe Debugger. Der Gast läuft auf Core 1; sowohl der
// GDB-Stub als auch das SWD-Target wollen ihn anhalten/fortsetzen können
// und Register/Memory lesen/schreiben.
//
// Mechanismus (kooperativ, ohne externes Halt-Hardware):
//   - target_request_halt() setzt die Halt-Flag und triggert PendSV im
//     Gast. PendSV-Handler erkennt das Flag und bleibt in einer Spin-
//     Schleife stehen, die periodisch poll() ruft.
//   - Während der Spin-Schleife können Debugger-Schichten Register und
//     Speicher des Gastes manipulieren — der Gast-Frame liegt auf dem PSP
//     und kann durch privilegierten Code (Host) modifiziert werden.
//   - target_request_resume()/_step() löst die Spin-Schleife.
//

#include <cstdint>

namespace target_halt {

void init();

// Wird vom PendSV-Handler aufgerufen.
void on_pendsv_check();

// Asynchroner Halt-Request — wirkt nicht sofort, sondern beim nächsten
// PendSV/Trap des Gastes.
void request_halt();
void request_resume();
void request_step();

bool is_halted();
bool is_step_pending();

// Während halt: Snapshot des Gast-Frames für Debugger.
struct Snapshot {
    uint32_t r[16];      // r0..r15 (sp = r13, lr = r14, pc = r15)
    uint32_t xpsr;
    uint32_t* frame;     // raw pointer, im halted Zustand stabil
    uint32_t* r4_r11;
};

// Liefert Snapshot. Nur gültig solange is_halted() true ist.
const Snapshot* snapshot();

// Reg-/Mem-Modifikation. Vom GDB-Stub und SWD-Target gemeinsam genutzt.
bool write_register(unsigned idx, uint32_t value);
bool read_register (unsigned idx, uint32_t& value);
bool read_memory   (uint32_t addr, void* dst, std::size_t len);
bool write_memory  (uint32_t addr, const void* src, std::size_t len);

// Adress-Mapping LPC → RP2350 (für Debugger, die mit LPC-Adressen arbeiten).
uint32_t map_guest_address(uint32_t lpc_addr);

// Setzt einen Hardware-style Breakpoint (FPB) durch BKPT-Insertion in
// Gast-RAM. Bis zu 8 simultane Breakpoints.
bool set_breakpoint(uint32_t addr);
bool clear_breakpoint(uint32_t addr);

} // namespace target_halt
