#include "fault.h"
#include "opcodes.h"
#include "peripherals.h"
#include "gdb_stub.h"
#include "iap.h"
#include "emulator.h"
#include "config.h"
#include "vnvic.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "RP2350.h"

namespace faultsys {

namespace {
Stats g_stats{};
}

Stats stats() { return g_stats; }

void init() { g_stats = {}; }

} // namespace faultsys

namespace {

// Bildet eine Gast-Adresse (LPC-Sicht 0x0000.. / 0x1000.. ODER bereits auf
// das RP2350-SRAM relozierte Adresse) auf einen lesbaren Host-Pointer ab und
// liest 4 Bytes. Liefert false, wenn die Adresse in keinem gemappten Bereich
// liegt (dann kein Zugriff -> handler-safe, auch fuer Muelladressen).
bool guest_read32_safe(uint32_t addr, uint32_t& out) {
    const uint32_t lb = emulator::load_base();
    const uint32_t rb = emulator::guest_ram_base();
    uint32_t host;
    if (addr < emulator::LPC_LOAD_MAX_SIZE) {
        host = lb + addr;                                   // LPC-Flash [0,64K)
    } else if (addr >= 0x1000'0000u &&
               addr <  0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
        host = rb + (addr - 0x1000'0000u);                  // LPC-SRAM
    } else if (addr >= lb && addr < lb + emulator::LPC_LOAD_MAX_SIZE) {
        host = addr;                                        // reloziertes Flash
    } else if (addr >= rb && addr < rb + emulator::LPC_GUEST_RAM_SIZE) {
        host = addr;                                        // reloziertes SRAM
    } else {
        return false;
    }
    std::memcpy(&out, reinterpret_cast<const void*>(host), 4);
    return true;
}

// Dekodiert die wichtigsten CFSR/HFSR-Bits (ARMv8-M) in Klartext.
void print_fault_cause() {
    const uint32_t c = SCB->CFSR;
    const uint32_t h = SCB->HFSR;
    // MemManage (MMFSR, Bits 0-7)
    if (c & (1u<<0))  std::printf("[FAULT]  MMFSR.IACCVIOL  (Code-Fetch verboten)\n");
    if (c & (1u<<1))  std::printf("[FAULT]  MMFSR.DACCVIOL  (Daten-Zugriff verboten)\n");
    if (c & (1u<<3))  std::printf("[FAULT]  MMFSR.MUNSTKERR (Unstacking-Fehler)\n");
    if (c & (1u<<4))  std::printf("[FAULT]  MMFSR.MSTKERR   (Stacking-Fehler)\n");
    if (c & (1u<<5))  std::printf("[FAULT]  MMFSR.MLSPERR   (Lazy-FP-Stacking)\n");
    // BusFault (BFSR, Bits 8-15)
    if (c & (1u<<8))  std::printf("[FAULT]  BFSR.IBUSERR    (Instruktions-Bus)\n");
    if (c & (1u<<9))  std::printf("[FAULT]  BFSR.PRECISERR  (praeziser Daten-Bus)\n");
    if (c & (1u<<10)) std::printf("[FAULT]  BFSR.IMPRECISERR(impraeziser Daten-Bus)\n");
    if (c & (1u<<11)) std::printf("[FAULT]  BFSR.UNSTKERR\n");
    if (c & (1u<<12)) std::printf("[FAULT]  BFSR.STKERR\n");
    // UsageFault (UFSR, Bits 16-25)
    if (c & (1u<<16)) std::printf("[FAULT]  UFSR.UNDEFINSTR (illegale Instruktion)\n");
    if (c & (1u<<17)) std::printf("[FAULT]  UFSR.INVSTATE   (ungueltiger Thumb/EPSR-State)\n");
    if (c & (1u<<18)) std::printf("[FAULT]  UFSR.INVPC      (ungueltiger EXC_RETURN/PC)\n");
    if (c & (1u<<19)) std::printf("[FAULT]  UFSR.NOCP       (Coprozessor/FPU nicht da)\n");
    if (c & (1u<<20)) std::printf("[FAULT]  UFSR.STKOF      (Stack-Overflow)\n");
    if (c & (1u<<24)) std::printf("[FAULT]  UFSR.UNALIGNED  (unaligned Zugriff)\n");
    if (c & (1u<<25)) std::printf("[FAULT]  UFSR.DIVBYZERO  (Division durch 0)\n");
    if (h & (1u<<30)) std::printf("[FAULT]  HFSR.FORCED     (eskalierter Fault)\n");
    if (h & (1u<<1))  std::printf("[FAULT]  HFSR.VECTTBL    (Vektortabellen-Lesefehler)\n");
}

// Gemeinsame Diagnose-Ausgabe fuer die fatalen Exception-Handler. f = Standard-
// exception-stacked Frame (r0,r1,r2,r3,r12,lr,pc,xpsr). r4_r11 optional (vom
// Asm-Wrapper gepushter Block) — nullptr, wenn nicht verfuegbar.
void print_exc_diag(const char* tag, const uint32_t* f, const uint32_t* r4_r11) {
    const uint32_t lb = emulator::load_base();
    auto lpc = [&](uint32_t a) -> long {            // Host->LPC-Offset, sonst -1
        return (a >= lb && a < lb + emulator::LPC_LOAD_MAX_SIZE)
                   ? static_cast<long>(a - lb) : -1L;
    };
    std::printf("[FAULT] %s @PC=0x%08lx (LPC 0x%lx) CFSR=0x%08lx HFSR=0x%08lx\n",
                tag, (unsigned long)f[6], lpc(f[6] & ~1u),
                (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR);
    print_fault_cause();
    std::printf("[FAULT]  r0=%08lx r1=%08lx r2=%08lx r3=%08lx r12=%08lx\n",
                (unsigned long)f[0], (unsigned long)f[1], (unsigned long)f[2],
                (unsigned long)f[3], (unsigned long)f[4]);
    std::printf("[FAULT]  LR=%08lx (LPC 0x%lx)  xPSR=%08lx  SP=%08lx\n",
                (unsigned long)f[5], lpc(f[5] & ~1u), (unsigned long)f[7],
                (unsigned long)(reinterpret_cast<uint32_t>(f) + 0x20u));
    if (r4_r11)
        std::printf("[FAULT]  r4=%08lx r5=%08lx r6=%08lx r7=%08lx\n",
                    (unsigned long)r4_r11[0], (unsigned long)r4_r11[1],
                    (unsigned long)r4_r11[2], (unsigned long)r4_r11[3]);
    // Gefehlerte Instruktion(en) am PC, wenn die Adresse gemappt ist.
    uint32_t iw = 0;
    if (guest_read32_safe(f[6] & ~1u, iw))
        std::printf("[FAULT]  instr@PC = 0x%04lx 0x%04lx\n",
                    (unsigned long)(iw & 0xFFFFu), (unsigned long)(iw >> 16));
    // Fault-Adressen, falls gueltig (MMARVALID=Bit7, BFARVALID=Bit15 in CFSR).
    if (SCB->CFSR & (1u<<7))
        std::printf("[FAULT]  MMFAR=0x%08lx\n", (unsigned long)SCB->MMFAR);
    if (SCB->CFSR & (1u<<15))
        std::printf("[FAULT]  BFAR=0x%08lx\n", (unsigned long)SCB->BFAR);
}

// Nicht-emulierbarer Gast-Fault: Gast ANHALTEN statt das ganze Silizium per
// Watchdog zu rebooten. Frueher loeste jeder fatale Fault watchdog_enable(50)
// + Spin aus -> voller RP2350-Reboot, der die soeben ausgegebene [FAULT]-
// Diagnose UND die USB-CDC-Sitzung vernichtete ("RP2350 resettet"). Jetzt:
// State=Faulted setzen (LED flackert, 'stats' zeigt Faulted), Core0/CLI laeuft
// weiter, die Diagnose bleibt lesbar. Recovery ueber CLI ('reset'/'run') oder
// erneutes Flashen. Laeuft auf Core1 und kehrt nie zurueck (Core1 parkt).
[[noreturn]] void enter_fatal_halt() {
    emulator::notify_guest_faulted();
    std::printf("[FAULT] Gast angehalten (State=Faulted) — 'reset' oder neue "
                "Firmware zum Fortfahren.\n");
    for (;;) __asm volatile ("wfe");
}

} // namespace

// --- Trap-Handler (in C, von Asm-Wrapper aus aufgerufen) ------------------
//
// frame zeigt auf den exception-stacked Frame des Gastes
// (r0,r1,r2,r3,r12,lr,pc,xpsr). r4_r11_lr zeigt auf den vom Asm-Wrapper
// gepushten Block {r4,r5,r6,r7,r8,r9,r10,r11,lr_excret}. SP des Gastes
// während der gefehlerten Instruktion = (uint32_t)frame + 0x20 (8 Dwords).
//
// Zugriffe auf den unprivilegierten Gast-Stack durch privilegierten Code
// sind erlaubt (PRIVDEFENA=1, MPU schränkt nur unprivileged ein).

extern "C" void handle_memfault_c(trap_decoder::StackedFrame* frame,
                                  uint32_t* r4_r11_lr) {
    using namespace trap_decoder;

    // IAP-ROM-Trap: Guest hat über Funktionspointer 0x1FFF1FF1 in den
    // BootROM gesprungen. Adresse existiert auf RP2350 nicht → Prefetch-
    // Fault. Wir bedienen den IAP-Aufruf und springen über LR zurück.
    if ((frame->pc & ~1u) == iap::ROM_ENTRY_TARGET) {
        auto* params  = reinterpret_cast<uint32_t*>(frame->r0);
        auto* results = reinterpret_cast<uint32_t*>(frame->r1);
        iap::dispatch(params, results);
        frame->pc = frame->lr & ~1u;
        SCB->CFSR = SCB->CFSR;
        ++faultsys::g_stats.mem_traps;
        return;
    }

    // IACCVIOL: Guest versuchte Code an einer nicht-relocierten LPC-Adresse
    // auszufuehren (Flash-Funktionspointer in Literal-Pool / .data).
    // Wir leiten den PC auf die tatsaechliche Position im RP2350-SRAM um.
    uint32_t cfsr = SCB->CFSR;
    if (cfsr & 0x01u) {  // MMFSR.IACCVIOL
        uint32_t fpc = frame->pc;
        // LPC-Flash [0, 64 KB) -> load_base + offset. Der gesamte LPC-Flash ist
        // ausfuehrbar und liegt vollstaendig (0xFF-gepaddet) im g_firmware_image;
        // IAP-/OTA-Schreibvorgaenge landen ebenfalls dort. Daher jede Flash-
        // Adresse bedienen, nicht nur die urspruenglich geladene Code-Laenge —
        // sonst faultet Code einer ueber den Bus nachgeladenen (ggf. groesseren)
        // Applikation jenseits von firmware_size() in einen Watchdog-Reset.
        if (fpc < emulator::LPC_LOAD_MAX_SIZE) {
            frame->pc = emulator::load_base() + fpc;
            SCB->CFSR = cfsr;
            ++faultsys::g_stats.mem_traps;
            return;
        }
        // LPC SRAM [0x10000000, 0x10000000 + RAM_SIZE) → guest_ram + offset
        if (fpc >= 0x1000'0000u &&
            fpc <  0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
            frame->pc = emulator::guest_ram_base() + (fpc - 0x1000'0000u);
            SCB->CFSR = cfsr;
            ++faultsys::g_stats.mem_traps;
            return;
        }
    }

    uint32_t* r4_r11 = r4_r11_lr;       // 8 Werte
    uint32_t  guest_sp = reinterpret_cast<uint32_t>(frame) + 0x20;

    Access acc{};
    if (!decode_mem_access(frame, r4_r11, guest_sp, acc)) {
        ++faultsys::g_stats.real_faults;
        faultsys::g_stats.last_fault_pc   = frame->pc;
        faultsys::g_stats.last_fault_addr = SCB->BFAR;
        std::printf("[FAULT] non-decodable @PC=0x%08lx CFSR=0x%08lx BFAR=0x%08lx\n",
                    static_cast<unsigned long>(frame->pc),
                    static_cast<unsigned long>(SCB->CFSR),
                    static_cast<unsigned long>(SCB->BFAR));

        // --- Erweiterte Diagnose ------------------------------------------
        // Register-Frame des Gastes (Cortex-M Standard-Stacking) + Gast-SP.
        const uint32_t lb = emulator::load_base();
        auto lpc = [&](uint32_t a) -> long {            // Host->LPC-Offset, sonst -1
            return (a >= lb && a < lb + emulator::LPC_LOAD_MAX_SIZE)
                       ? static_cast<long>(a - lb) : -1L;
        };
        std::printf("[FAULT]  r0=%08lx r1=%08lx r2=%08lx r3=%08lx r12=%08lx\n",
                    (unsigned long)frame->r0, (unsigned long)frame->r1,
                    (unsigned long)frame->r2, (unsigned long)frame->r3,
                    (unsigned long)frame->r12);
        std::printf("[FAULT]  LR=%08lx (LPC 0x%lx)  xPSR=%08lx  SP=%08lx\n",
                    (unsigned long)frame->lr, lpc(frame->lr & ~1u),
                    (unsigned long)frame->xpsr, (unsigned long)guest_sp);
        std::printf("[FAULT]  r4=%08lx r5=%08lx r6=%08lx r7=%08lx\n",
                    (unsigned long)r4_r11[0], (unsigned long)r4_r11[1],
                    (unsigned long)r4_r11[2], (unsigned long)r4_r11[3]);

        // Bei IACCVIOL (Code-Fetch an nicht gemappter Adresse): wilder Sprung,
        // typ. ueber korrupten C++-Vtable-/Funktionspointer. LR = Aufrufer,
        // r0 = this-Pointer bei virtuellen Aufrufen. Vtable-Kette aufdroeseln.
        if (SCB->CFSR & 0x01u) {
            uint32_t vt = 0, m0 = 0, m2 = 0, m11 = 0;
            std::printf("[FAULT]  Code-Fetch (IACCVIOL): wilder Sprung nach 0x%08lx\n",
                        (unsigned long)frame->pc);
            if (guest_read32_safe(frame->r0, vt)) {
                std::printf("[FAULT]  this=r0=0x%08lx -> vtable=0x%08lx\n",
                            (unsigned long)frame->r0, (unsigned long)vt);
                bool b0  = guest_read32_safe(vt + 0,  m0);
                bool b2  = guest_read32_safe(vt + 8,  m2);
                bool b11 = guest_read32_safe(vt + 44, m11);
                std::printf("[FAULT]  vtable[0]=%s vtable[2]=%s vtable[11]=%s\n",
                            b0  ? "" : "?", b2 ? "" : "?", b11 ? "" : "?");
                std::printf("[FAULT]   = 0x%08lx / 0x%08lx / 0x%08lx\n",
                            (unsigned long)m0, (unsigned long)m2,
                            (unsigned long)m11);
            } else {
                std::printf("[FAULT]  this=r0=0x%08lx (nicht gemappt)\n",
                            (unsigned long)frame->r0);
            }
        }
        SCB->CFSR = SCB->CFSR;          // Sticky-Bits clearen
        // Gast anhalten (kein Silizium-Reboot mehr) — Diagnose bleibt sichtbar.
        enter_fatal_halt();
    }

    // Emulieren
    bool ok = true;

    // SCB-Kontroll-Block (0xE000ED00-0xE000ED1F): liegt in einer Read-Only-MPU-
    // Region, damit Gast-SCHREIBzugriffe hier trappen (Reads passieren dank RO
    // nativ). Wichtigster Fall: AIRCR.SYSRESETREQ via NVIC_SystemReset() (z. B.
    // sblib BcuBase::softSystemReset). Auf dem M33 wuerde das nativ das ECHTE
    // RP2350-Silizium rebooten (USB/CLI weg) — wir fangen es ab und starten nur
    // den Gast neu. ICSR/SCR/CCR/SHPR werden auf den realen SCB durchgereicht
    // (der Gast laeuft ja nativ auf dem M33); VTOR wird ignoriert (Host
    // verwaltet die Vektorbasis ueber den Handover).
    if (acc.address >= 0xE000'ED00u && acc.address <= 0xE000'ED1Fu) {
        const uint32_t reg = acc.address & ~3u;
        if (acc.is_load) {
            uint32_t v = *reinterpret_cast<volatile uint32_t*>(reg);
            uint32_t* dst = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
            if (dst) *dst = v;
            frame->pc += acc.instr_size;
            SCB->CFSR = SCB->CFSR;
            ++faultsys::g_stats.mem_traps;
            return;
        }
        uint32_t src_val = 0;
        if (acc.rt < 16) {
            uint32_t* p = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
            if (p) src_val = *p;
        }
        if (reg == 0xE000'ED0Cu) {                    // AIRCR
            // VECTKEY (0x05FA) korrekt UND SYSRESETREQ (Bit 2) -> nur Gast-Reset.
            if (((src_val >> 16) == 0x05FAu) && (src_val & (1u << 2))) {
                ++faultsys::g_stats.mem_traps;
                emulator::request_guest_reset();     // Core1 parkt bis Core0-Reset
                return;                              // (falls je Core0: regulaer)
            }
            // sonstige AIRCR-Schreibzugriffe (VECTCLRACTIVE etc.) ignorieren
        } else if (reg != 0xE000'ED08u) {            // != VTOR -> auf realen SCB
            volatile uint32_t* p = reinterpret_cast<volatile uint32_t*>(reg);
            if (acc.size == AccessSize::W) {
                *p = src_val;
            } else {                                  // partieller Zugriff: RMW
                uint32_t shift = (acc.address & 3u) * 8u;
                uint32_t mask  = (acc.size == AccessSize::B) ? 0xFFu : 0xFFFFu;
                *p = (*p & ~(mask << shift)) | ((src_val & mask) << shift);
            }
        }   // VTOR (0xE000ED08): ignorieren
        frame->pc += acc.instr_size;
        SCB->CFSR = SCB->CFSR;
        ++faultsys::g_stats.mem_traps;
        return;
    }

    // LPC-Flash-Bereich (gesamte 64 KB): Daten aus dem geladenen Firmware-
    // Image liefern. Der Guest liest z. B. .data-Initialisierungswerte ueber
    // einen nicht-relocierten Flash-Pointer (Scatter-Load-Tabelle) ODER greift
    // direkt auf hohe Flash-Sektoren zu, die nicht zum Code gehoeren — etwa
    // ein flash-basiertes EEPROM. sblib (BCU2/MASK0701) legt sein userEeprom
    // z. B. im letzten Sektor (0xF000-0xFFFF) ab und liest es per memcpy direkt
    // aus dem Flash. Das g_firmware_image ist stets volle LPC_LOAD_MAX_SIZE
    // gross und mit 0xFF gepaddet (== geloeschtes Flash); IAP-Schreibvorgaenge
    // landen ebenfalls dort. Daher den gesamten Flashbereich bedienen, nicht
    // nur die geladene Code-Laenge — sonst faulten EEPROM-Reads jenseits des
    // Images und loesen einen Watchdog-Reset aus.
    if (acc.address < emulator::LPC_LOAD_MAX_SIZE) {
        if (acc.is_load) {
            auto* img = reinterpret_cast<const uint8_t*>(emulator::load_base());
            uint32_t value = 0;
            switch (acc.size) {
                case AccessSize::B: {
                    uint8_t v = img[acc.address];
                    value = acc.is_signed
                        ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)))
                        : v;
                    break;
                }
                case AccessSize::H: {
                    uint16_t h;
                    std::memcpy(&h, img + acc.address, 2);
                    value = acc.is_signed
                        ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(h)))
                        : h;
                    break;
                }
                case AccessSize::W: {
                    std::memcpy(&value, img + acc.address, 4);
                    break;
                }
            }
            uint32_t* dst = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
            if (dst) *dst = value;
        }
        // Writes to Flash: silently ignore (Flash is read-only).
        frame->pc += acc.instr_size;
        SCB->CFSR = SCB->CFSR;
        ++faultsys::g_stats.mem_traps;
        return;
    }

    // LPC-SRAM-Bereich [0x10000000, +RAM_SIZE): Wird ein RAM-Pointer von der
    // Lade-Heuristik (hex_patcher) nicht reloziert — typisch bei zur Laufzeit
    // *berechneten* Adressen (base + offset), die als reiner Basiswert nicht
    // exakt im Scan-Bereich liegen — dann wuerde der Zugriff sonst bei
    // peripherals::mmio_* abgelehnt und einen Watchdog-Reset ausloesen.
    // Statt zu faulten lenken wir den Zugriff hier deterministisch auf den
    // Gast-RAM um. So wird jeder von der Heuristik verpasste RAM-Pointer
    // *korrekt* bedient (nur etwas langsamer, da getrappt) statt fatal.
    {
        const uint32_t asz = (acc.size == AccessSize::B) ? 1u
                           : (acc.size == AccessSize::H) ? 2u : 4u;
        // Strikte Host-Bounds-Pruefung (inkl. Straddle am Bereichsende):
        // verhindert OOB-Zugriffe auf den g_guest_ram-Puffer.
        if (acc.address >= 0x1000'0000u &&
            acc.address + asz <= 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
            uint32_t off = acc.address - 0x1000'0000u;
            auto* ram = reinterpret_cast<uint8_t*>(emulator::guest_ram_base());
            if (acc.is_load) {
                uint32_t value = 0;
                switch (acc.size) {
                    case AccessSize::B: {
                        uint8_t v = ram[off];
                        value = acc.is_signed
                            ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)))
                            : v;
                        break;
                    }
                    case AccessSize::H: {
                        uint16_t h;
                        std::memcpy(&h, ram + off, 2);
                        value = acc.is_signed
                            ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(h)))
                            : h;
                        break;
                    }
                    case AccessSize::W:
                        std::memcpy(&value, ram + off, 4);
                        break;
                }
                uint32_t* dst = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
                if (dst) *dst = value;
            } else {
                uint32_t src_val = 0;
                if (acc.rt < 16) {
                    uint32_t* p = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
                    if (p) src_val = *p;
                }
                switch (acc.size) {
                    case AccessSize::B:
                        ram[off] = static_cast<uint8_t>(src_val & 0xFFu);
                        break;
                    case AccessSize::H: {
                        uint16_t h = static_cast<uint16_t>(src_val & 0xFFFFu);
                        std::memcpy(ram + off, &h, 2);
                        break;
                    }
                    case AccessSize::W:
                        std::memcpy(ram + off, &src_val, 4);
                        break;
                }
            }
            frame->pc += acc.instr_size;
            SCB->CFSR = SCB->CFSR;
            ++faultsys::g_stats.mem_traps;
            return;
        }
    }

    if (acc.is_load) {
        uint32_t value = 0;
        switch (acc.size) {
            case AccessSize::B: {
                uint8_t v = 0;
                ok = peripherals::mmio_read8(acc.address, v);
                value = acc.is_signed
                    ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)))
                    : v;
                break;
            }
            case AccessSize::H: {
                uint8_t lo = 0, hi = 0;
                ok = peripherals::mmio_read8(acc.address, lo) &&
                     peripherals::mmio_read8(acc.address + 1, hi);
                uint16_t h = static_cast<uint16_t>(lo | (hi << 8));
                value = acc.is_signed
                    ? static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(h)))
                    : h;
                break;
            }
            case AccessSize::W: {
                uint8_t b[4]{};
                for (int i = 0; i < 4 && ok; ++i)
                    ok = peripherals::mmio_read8(acc.address + i, b[i]);
                value = static_cast<uint32_t>(b[0])       |
                        (static_cast<uint32_t>(b[1]) << 8) |
                        (static_cast<uint32_t>(b[2]) << 16) |
                        (static_cast<uint32_t>(b[3]) << 24);
                break;
            }
        }
        uint32_t* dst = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
        if (dst) *dst = value;
    } else {
        uint32_t  src_val = 0;
        if (acc.rt < 16) {
            uint32_t* p = reg_ptr(frame, r4_r11, &guest_sp, acc.rt);
            if (p) src_val = *p;
        }
        switch (acc.size) {
            case AccessSize::B:
                ok = peripherals::mmio_write8(acc.address,
                                              static_cast<uint8_t>(src_val & 0xFFu));
                break;
            case AccessSize::H:
                ok = peripherals::mmio_write8(acc.address,
                                              static_cast<uint8_t>(src_val & 0xFFu)) &&
                     peripherals::mmio_write8(acc.address + 1,
                                              static_cast<uint8_t>((src_val >> 8) & 0xFFu));
                break;
            case AccessSize::W:
                for (int i = 0; i < 4 && ok; ++i)
                    ok = peripherals::mmio_write8(acc.address + i,
                                                  static_cast<uint8_t>((src_val >> (i*8)) & 0xFFu));
                break;
        }
    }

    if (!ok) {
        ++faultsys::g_stats.real_faults;
        faultsys::g_stats.last_fault_pc   = frame->pc;
        faultsys::g_stats.last_fault_addr = acc.address;
        std::printf("[FAULT] mmio rejected @PC=0x%08lx addr=0x%08lx\n",
                    static_cast<unsigned long>(frame->pc),
                    static_cast<unsigned long>(acc.address));
        SCB->CFSR = SCB->CFSR;
        enter_fatal_halt();
    }

    ++faultsys::g_stats.mem_traps;
    // Post-Hook (z. B. PLL-Re-Konfiguration nach abgeschlossenem 32-bit-Wort).
    if (!acc.is_load) peripherals::on_post_write_hook();

    // Bootloader->App-Handover: SP-Korrektur anwenden, falls der soeben
    // emulierte SYSMEMREMAP-Store den Uebergang ausgeloest hat. Der Bootloader
    // fuehrt als naechste Instruktion `mov SP, StackTop` aus; wir ersetzen den
    // rohen LPC-RAM-StackTop im gestackten r4..r11-Block durch die relocierte
    // Gast-RAM-Adresse, damit der SP auf gueltiges RP2350-SRAM zeigt.
    {
        uint32_t sp_raw = 0, sp_reloc = 0;
        if (emulator::take_handover_sp_fixup(sp_raw, sp_reloc)) {
            for (int i = 0; i < 8; ++i) {
                if (r4_r11[i] == sp_raw) r4_r11[i] = sp_reloc;
            }
        }
    }

    // PC vorrücken
    frame->pc += acc.instr_size;
    // Sticky-Bits clearen (CFSR write-1-to-clear)
    SCB->CFSR = SCB->CFSR;
}

// --- Naked Exception-Handler ---------------------------------------------
// Der Pico-SDK-Linker erwartet die ISR-Symbole `isr_*`. Wir registrieren
// MemManage und BusFault. UsageFault (z. B. UDF, illegal opcode) und
// HardFault behandeln wir als echten Fault → Watchdog-Reset.

extern "C" __attribute__((naked)) void isr_busfault() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"   // 8-Byte-Alignment
        "add   r1, sp, #4          \n"   // r1 = &{r4..r11, lr_excret}
        "bl    handle_memfault_c   \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

extern "C" __attribute__((naked)) void isr_memmanage() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    handle_memfault_c   \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

// HardFault. Sonderfall WFI-Pin-Wakeup (opt-in): Führt der Gast `WFI` mit
// gesperrten Interrupts aus (`PRIMASK=1`, das gängige
// "__disable_irq(); __WFI();"-Idiom), so kann der aus dem Patch entstandene
// `SVC #0` die SVCall-Exception nicht aktivieren (Priorität maskiert) und
// eskaliert zu HardFault (HFSR.FORCED=1). Wir erkennen das an der
// gefehlerten Instruktion (`0xDF00`) und behandeln es wie WFI: pollen echte
// Pin-Flanken/Timer, setzen bei fälligem IRQ PendSV (bleibt unter PRIMASK
// pending und feuert, sobald der Gast die Interrupts wieder freigibt) und
// kehren hinter das WFI zurück. PRIMASK des Gastes bleibt unangetastet.
extern "C" void hardfault_c(uint32_t* exc_frame, uint32_t* r4_r11) {
    if (config::wfi_pin_wakeup() && (SCB->HFSR & SCB_HFSR_FORCED_Msk)) {
        uint32_t pc = exc_frame[6];
        const uint16_t* ip = reinterpret_cast<const uint16_t*>(pc & ~1u);
        if (*ip == 0xDF00) {                 // SVC #0 (gepatchtes WFI)
            for (;;) {
                peripherals::sample_pin_interrupts();
                peripherals::poll_timed_sources();
                if (vnvic::irq_pending()) {
                    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
                    break;
                }
                busy_wait_us(50);
            }
            exc_frame[6] = (pc & ~1u) + 2u;  // hinter das WFI/SVC zurück
            SCB->HFSR = SCB->HFSR;           // FORCED w1c
            ++faultsys::g_stats.mem_traps;
            return;
        }
    }
    print_exc_diag("HardFault", exc_frame, r4_r11);
    enter_fatal_halt();
}

extern "C" __attribute__((naked)) void isr_hardfault() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"   // 8-Byte-Alignment
        "add   r1, sp, #4          \n"   // r1 = &{r4..r11, lr_excret}
        "bl    hardfault_c         \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

// Bei aktivem GDB-Stub fängt UsageFault BKPT-Instruktionen (Software-
// Breakpoints) und delegiert an den Stub. Andernfalls Watchdog-Reset.
extern "C" void usagefault_c(uint32_t* exc_frame, uint32_t* r4_r11) {
    if (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk) {
        uint16_t instr = *reinterpret_cast<uint16_t*>(exc_frame[6]);
        if ((instr & 0xFF00) == 0xBE00) {            // BKPT
            if (gdb_stub::active()) {
                gdb_stub::on_breakpoint(exc_frame, r4_r11);
                SCB->CFSR = SCB->CFSR;
                return;
            }
        }
    }
    print_exc_diag("UsageFault", exc_frame, r4_r11);
    enter_fatal_halt();
}

extern "C" __attribute__((naked)) void isr_usagefault() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    usagefault_c        \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

// DebugMonitor: M33 löst diese Exception nach genau einer ausgeführten
// Instruktion aus, sobald DEMCR.MON_EN=1 und MON_STEP=1 sind und
// DHCSR.C_DEBUGEN=0 (kein DAP angehängt). Wir nutzen das für sauberes
// GDB-Single-Step.
extern "C" void debugmon_c(uint32_t* exc_frame, uint32_t* r4_r11) {
    // MON_STEP wieder löschen, sonst tritt die Exception erneut auf.
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_STEP_Msk;
    if (gdb_stub::active()) {
        gdb_stub::on_breakpoint(exc_frame, r4_r11);
    }
}

extern "C" __attribute__((naked)) void isr_debugmon() {
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4-r11, lr}        \n"
        "sub   sp, #4              \n"
        "add   r1, sp, #4          \n"
        "bl    debugmon_c          \n"
        "add   sp, #4              \n"
        "pop   {r4-r11, lr}        \n"
        "bx    lr                  \n"
    );
}

void setup_fault_handlers() {
    faultsys::init();
}
