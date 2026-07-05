#include "emulator.h"
#include "mmu.h"
#include "fault.h"
#include "peripherals.h"
#include "storage.h"
#include "config.h"
#include "hex_patcher.h"
#include "vnvic.h"
#include "target_halt.h"   // on_guest_reset() bei Core-Reset

#include <atomic>
#include <cstdio>
#include <cstring>

#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "hardware/irq.h"   // irq_set_enabled, SIO_IRQ_FIFO
#include "RP2350.h"   // CMSIS, via cmsis_core lib

namespace {

std::atomic<emulator::State> g_state{emulator::State::Idle};
std::atomic<bool>            g_request_stop{false};
std::atomic<uint32_t>        g_pc{0};

// Zaehlt jeden Gast-Start (Firmware-Kopie + Launch in core1_main). Diagnose:
// steigt der Wert waehrend des Betriebs, wird der Gast wiederholt neu gestartet
// (z. B. durch MSC-Re-Processing) -> erklaert LED-Glitches/Blink-Aussetzer.
std::atomic<uint32_t>        g_start_count{0};

// Adresse des relozierten Gast-SysTick-Handlers (VTOR[15], mit Thumb-Bit). Der
// reale Core1-SysTick feuert stattdessen isr_systick_shim, der die zeitbasierten
// LPC-Modelle treibt und dann diesen Gast-Handler aufruft. 0 = keiner.
std::atomic<uint32_t>        g_guest_systick_handler{0};

// PC-Sampler (Diagnose): der reale Core1-SysTick unterbricht den Gast periodisch;
// im Handler-Mode liegt der unterbrochene Gast-PC im gestackten PSP-Frame
// (Offset 6). Wir merken die letzten Samples als LPC-Offset (PC - load_base).
// Zeigt, WO ein "State=Running, mmio-traps eingefroren"-Gast in einer reinen
// CPU-Schleife (ohne MMIO/WFI) dreht -> im .map/.lst der Firmware nachschlagbar.
constexpr uint32_t PC_SAMPLES = 8;
std::atomic<uint32_t>        g_pc_samples[PC_SAMPLES]{};
std::atomic<uint32_t>        g_pc_sample_pos{0};

// SysTick-Shim-Heartbeat (Diagnose). enter wird am SHIM-ANFANG, exit am SHIM-ENDE
// erhoeht. enter>exit (Differenz waechst) => Shim haengt INNERHALB (Runaway-Loop
// in poll_timed_sources/ct_advance). enter==exit aber eingefroren => Shim feuert
// nicht mehr (realer SysTick gestoppt/maskiert). last_us = Zeit des letzten Eintritts.
std::atomic<uint32_t>        g_shim_enter{0};
std::atomic<uint32_t>        g_shim_exit{0};
std::atomic<uint64_t>        g_shim_last_us{0};

// Vom Gast (Core1) angeforderter Soft-Reset (NVIC_SystemReset/WDT). Wird vom
// Core0-Loop konsumiert, der den eigentlichen Core1-Reset ausfuehrt — ein
// multicore_reset_core1() VON Core1 aus wuerde sich selbst abschiessen und nie
// relaunchen.
std::atomic<bool>            g_guest_reset_req{false};

// Basis der aktiven Guest-Vektortabelle (0 = noch nicht gesetzt -> load_base).
// Wird beim Guest-Start auf load_base gesetzt und bei einem Bootloader-
// Handover auf die Applikations-Vektortabelle umgestellt.
std::atomic<uint32_t>        g_vtable_base{0};

// Vom Bootloader->App-Handover vorgemerkte SP-Korrektur. Der Bootloader laedt
// den (noch rohen) Applikations-StackTop in ein Register und fuehrt direkt
// nach dem ausloesenden SYSMEMREMAP-Store `mov SP, StackTop` aus. Liegt der
// Wert in der LPC-RAM-Range, ist er auf RP2350 nicht als Stack nutzbar; der
// Fault-Handler ersetzt ihn dann im gestackten Registerblock durch reloc.
std::atomic<bool>            g_sp_fixup_pending{false};
std::atomic<uint32_t>        g_sp_fixup_raw{0};
std::atomic<uint32_t>        g_sp_fixup_reloc{0};

// --- Guest-Speicherbereiche als echte Arrays ---
// Der Linker platziert sie automatisch ohne Kollision mit anderem BSS.
// VTOR verlangt 256-Byte-Alignment (128-Byte reicht ab ARMv7-M, aber
// 256 ist auf der sicheren Seite für 48 Vektoren × 4 = 192 → nächste 2^n = 256).
alignas(256) uint8_t g_firmware_image[emulator::LPC_LOAD_MAX_SIZE]
    __attribute__((used)) = {};

alignas(32) uint8_t g_guest_ram[emulator::LPC_GUEST_RAM_SIZE]
    __attribute__((used)) = {};

// Externe Symbole der Fault-Handler — die linken weak im SDK; wir adressieren
// sie hier, um die Firmware-Vector-Table darauf zu patchen.
extern "C" void isr_busfault();
extern "C" void isr_memmanage();
extern "C" void isr_hardfault();
extern "C" void isr_usagefault();
extern "C" void isr_pendsv();   // IRQ-Injektor (src/irq_inject.cpp)

// --- WFI-Pin-Wakeup + PRIMASK-Schatten (beide opt-in) ----------------------
//
// Beide Features lenken Gast-Instruktionen auf SVC-Traps um, die hier ueber
// die SVC-Immediate unterschieden werden (SVCall = Vektor-Slot 11):
//   SVC #0  = gepatchtes WFI  (config::wfi_pin_wakeup) -> Warteschleife
//   SVC #1  = CPSID i         (config::primask_shadow) -> Schatten-PRIMASK=1
//   SVC #2  = CPSIE i         (config::primask_shadow) -> Schatten-PRIMASK=0
//
// WFI-Warteschleife: Statt schlafen zu legen, pollt der Host die echten
// RP2350-Eingaenge (Pin-Flanken) und die zeitbasierten Modelle (CT/WWDT).
// Sobald ein vom Gast aktivierter IRQ pending wird, kehrt der Handler zurueck
// und setzt PendSV — die bestehende IRQ-Injektion liefert den LPC-Handler aus.
//
// PRIMASK-Schatten: Der Gast laeuft unprivilegiert, daher ignoriert die M33-
// Hardware CPSID/CPSIE. Wir fuehren den Maskierungszustand im vNVIC nach;
// solange gesetzt, haelt pendsv_inject_c() neue IRQs pending zurueck. Beim
// Loeschen (CPSIE) wird ein evtl. wartender IRQ sofort per PendSV nachgereicht.
//
// HINWEIS: Die WFI-Schleife pollt aktiv (busy-wait), nicht stromsparend.
// WFI muss mit aktivierten Interrupts (PRIMASK=0) ausgefuehrt werden; sonst
// eskaliert der SVC zum HardFault (Sonderbehandlung in src/fault.cpp).
extern "C" void svc_dispatch_c(uint32_t* frame) {
    // Gestapelter Exception-Frame: frame[6] = Return-PC (hinter dem SVC).
    // Die SVC-Instruktion liegt 2 Byte davor; ihr Low-Byte ist die Immediate.
    uint32_t pc  = frame[6] & ~1u;
    uint8_t  imm = *reinterpret_cast<const uint8_t*>(pc - 2u);
    switch (imm) {
        case 1:                                  // CPSID i -> Sektion betreten
            vnvic::set_primask(true);
            return;
        case 2:                                  // CPSIE i -> Sektion verlassen
            vnvic::set_primask(false);
            if (vnvic::irq_pending())
                SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            return;
        case 0:                                  // gepatchtes WFI
        default:
            for (;;) {
                if (g_request_stop.load(std::memory_order_acquire)) return;
                peripherals::sample_pin_interrupts();
                peripherals::poll_timed_sources();
                if (vnvic::irq_pending()) {
                    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
                    return;
                }
                // Bei aktivem Timer-Capture (KNX-Empfang) eng pollen, damit
                // Busflanken (~104 µs/Bit) zeitlich aufgelöst werden; sonst
                // 50 µs Pause zur RP2350-Entlastung.
                if (!peripherals::capture_armed())
                    busy_wait_us(50);
            }
    }
}

__attribute__((naked))
void isr_svc() {
    __asm volatile (
        "tst   lr, #4              \n"   // EXC_RETURN bit2: 0=MSP, 1=PSP
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "push  {r4, lr}            \n"   // 8-Byte-Alignment + EXC_RETURN
        "bl    svc_dispatch_c      \n"
        "pop   {r4, pc}            \n"   // EXC_RETURN -> zurueck zum Gast
    );
}

// SysTick-Host-Shim (Core1, laeuft im Handler-Mode als SysTick-Exception).
// Der reale Core1-SysTick (systick_hw_sync in peripherals.cpp) zeigt ueber
// VTOR[15] hierher statt direkt auf den Gast-Handler. Zweck: die zeitbasierten
// LPC-Modelle (CT16/CT32-Timer, WWDT) periodisch (jede SysTick-Periode, i.d.R.
// 1 ms) voranzutreiben. Sonst wuerden deren Match-Interrupts NUR bei einem
// Gast-MMIO-Zugriff erkannt (ct_advance laeuft dort lazy) — ein interrupt-
// getriebenes Programm, das in __WFI() idlet (z. B. example-int-blink: Timer-
// Match toggelt die LED im IRQ), bekaeme so nach dem ersten nie einen weiteren
// Timer-IRQ. poll_timed_sources() erkennt faellige Matches und pendet den
// zugehoerigen LPC-IRQ im vNVIC (+ PendSV). Danach wird der eigentliche Gast-
// SysTick-Handler (z. B. sblib systemTime++) als normale Subroutine gerufen —
// ein CMSIS-Handler ist eine gewoehnliche C-Funktion, das ist zulaessig. Beim
// Exception-Return dieses Shims wird ein evtl. gependeter LPC-IRQ per PendSV
// (tail-chained) in den Gast injiziert.
extern "C" void isr_systick_shim() {
    g_shim_enter.fetch_add(1, std::memory_order_relaxed);
    g_shim_last_us.store(time_us_64(), std::memory_order_relaxed);
    // PC-Sampler: der unterbrochene Gast lief in Thread-Mode auf PSP; die HW hat
    // dort {r0,r1,r2,r3,r12,lr,pc,xpsr} gestackt -> psp[6] = Gast-PC. Als
    // LPC-Offset (PC - load_base) merken. Nur gueltige Thread-Frames (Gast lief,
    // nicht der Shim selbst verschachtelt). Erlaubt Post-Mortem eines CPU-Loops.
    {
        uint32_t psp; __asm volatile ("mrs %0, psp" : "=r"(psp));
        auto* f = reinterpret_cast<uint32_t*>(psp);
        if (f) {
            uint32_t pc = f[6];
            uint32_t lb = emulator::load_base();
            uint32_t off = (pc >= lb && pc < lb + emulator::LPC_LOAD_MAX_SIZE)
                               ? (pc - lb) : pc;   // im Image: LPC-Offset, sonst roh
            uint32_t pos = g_pc_sample_pos.fetch_add(1, std::memory_order_relaxed);
            g_pc_samples[pos % PC_SAMPLES].store(off, std::memory_order_relaxed);
        }
    }

    // Host-Zeitbasis-Tick (reale Core1-SysTick-Exception, Handler-Mode). Feuert
    // ADAPTIV (bis in den Sub-ms-Bereich) am naechsten faelligen Timer-Match,
    // sonst spaetestens jede 1 ms. Treibt die zeitbasierten LPC-Modelle und
    // weckt den Gast aus __WFI().
    peripherals::poll_timed_sources();      // CT16/CT32/WWDT + SysTick-Schatten
    peripherals::sample_pin_interrupts();

    // Gast-SysTick_Handler nur so oft aufrufen, wie echte Gast-Perioden
    // verstrichen sind. So bleibt systemTime korrekt, auch wenn der reale
    // SysTick fuer einen Sub-ms-Timer schneller tickt als die Gast-Periode
    // (dann sind 0 Perioden faellig -> kein systemTime++).
    uint32_t due = peripherals::systick_take_guest_ticks();
    if (due) {
        uint32_t h = g_guest_systick_handler.load(std::memory_order_relaxed);
        if ((h & ~1u) != 0u) {
            auto fn = reinterpret_cast<void(*)()>(h);
            if (due > 8u) due = 8u;         // Nachhol-Deckel (nach langem Stall)
            for (uint32_t i = 0; i < due; ++i) fn();
        }
    }

    // Faellige LPC-IRQs (Timer-Match etc.) in den Gast injizieren (PendSV tail-
    // chained beim Exception-Return dieses Shims).
    if (vnvic::irq_pending())
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

    // Naechsten Host-Tick am kommenden Deadline neu programmieren (adaptiv).
    peripherals::systick_hw_rearm();
    g_shim_exit.fetch_add(1, std::memory_order_relaxed);
}

// Naked Trampolin: wechselt auf PSP/unprivileged Thread Mode und springt mit
// BX in den Reset-Handler. Kehrt nie zurück (Gast läuft in Endlos-Loop wie
// eine echte MCU).
//
// WICHTIG: Diese Funktion MUSS im SRAM liegen (__not_in_flash_func), nicht im
// XIP-Flash, UND auf 32 Byte aligned sein (aligned(32)). Nach `msr control`
// (nPRIV=1) + `isb` ist der Code unprivileged. Die MPU (mmu.cpp) gibt fuer den
// unprivilegierten Gast nur drei enge SRAM-Bereiche frei: das Code-Image, den
// Gast-RAM und genau dieses Trampolin (eigene 32-Byte-RX-Region bei
// &enter_guest — daher das 32-Byte-Alignment, damit eine einzelne MPU-Region
// die Funktion sauber abdeckt). Das XIP-Flash (0x10000000+) ist NICHT abgedeckt.
// Laege das Trampolin im Flash, wuerde bereits der naechste Instruktions-Fetch
// (`bx r1`) unprivileged aus dem Flash erfolgen und von der MPU als IACCVIOL
// (MemManageFault) abgewiesen — der Gast startet nie. In der dedizierten SRAM-
// RX-Region ist der Fetch erlaubt; `bx r1` springt dann in den Gast-Code im
// Code-Image (load_base, eigene RWX-Region).
__attribute__((naked, noreturn, aligned(32)))
void __not_in_flash_func(enter_guest)(uint32_t /*initial_sp*/,
                                      uint32_t /*reset_handler*/) {
    __asm volatile (
        "msr   psp, r0             \n"   // PSP = initial_sp
        "mov   r2, #3              \n"   // CONTROL.SPSEL=1, nPRIV=1
        "msr   control, r2         \n"
        "isb                       \n"
        // r1 = reset handler address (mit Thumb-Bit)
        "bx    r1                  \n"
    );
}

// Patch-Helper: relocate Vector-Table-Eintrag.
// Flash (< 0x10000000) → g_firmware_image base + offset
// RAM (0x10000000..+LPC_GUEST_RAM_SIZE) → g_guest_ram base + offset
uint32_t relocate_vector(uint32_t v) {
    uint32_t plain = v & ~1u;
    bool thumb = v & 1u;
    uint32_t load_base = reinterpret_cast<uint32_t>(g_firmware_image);
    uint32_t ram_base  = reinterpret_cast<uint32_t>(g_guest_ram);
    if (plain < 0x1000'0000u) {
        // LPC-Flash-Range: nach RP2350 SRAM relocaten
        plain += load_base;
    } else if (plain >= 0x1000'0000u &&
               plain < 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
        // LPC-RAM-Range: nach RP2350 Guest-RAM relocaten
        plain = ram_base + (plain - 0x1000'0000u);
    }
    return plain | (thumb ? 1u : 0u);
}

// --- Zweistufiger Boot (Bootloader -> Applikation) -------------------------
//
// Layout im g_firmware_image (LPC-Adressraum, 0xFF-gepaddet):
//   [0 .. app_start)         Bootloader (wird relociert/gepatcht)
//   [desc_addr .. app_start) Boot-Descriptor (16 B, Selfbus-Format)
//   [app_start .. ende]      Applikation (bleibt unangetastet/pristine)
//
// Der Selfbus-Bootloader prueft per checkApplication() eine CRC32 ueber den
// App-Bereich und vergleicht sie mit descriptor->crc. Damit das fuer eine
// gemeinsam (kombiniert) geladene Applikation funktioniert, muss der
// App-Bereich byte-genau dem entsprechen, worueber die CRC gebildet wurde.
// Deshalb wird die Applikation NICHT relociert (Literal-Pool-/RAM-Referenzen
// werden zur Laufzeit per Trap umgeleitet) und der Descriptor ueber die
// pristinen Bytes berechnet. Identisch zum bereits funktionierenden
// OTA-Pfad (Bus-Update).

// Standard-CRC-32 (poly 0xEDB88320, reflektiert), bit-identisch zur crc32()
// des Selfbus-Bootloaders (src/crc.cpp). Seed 0xFFFFFFFF, finales Komplement.
uint32_t crc32_selfbus(const uint8_t* data, std::size_t len) {
    uint32_t c = 0xFFFF'FFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        c ^= data[i];
        for (int k = 0; k < 8; ++k) {
            uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(c & 1u));
            c = (c >> 1) ^ (0xEDB8'8320u & mask);
        }
    }
    return ~c;
}

// Layout exakt wie AppDescriptionBlock des Bootloaders (16 Bytes Nutzdaten).
struct AppDescriptor {
    uint32_t startAddress;       // LPC-Flash-Adresse der Applikation
    uint32_t endAddress;         // LPC-Flash-Adresse des letzten App-Bytes
    uint32_t crc;                // crc32 ueber [startAddress .. endAddress]
    uint32_t appVersionAddress;  // Adresse des "!AVP!@:"-Strings (oder start)
};

// Pruefe, ob an app_off eine plausible Applikations-Vektortabelle liegt:
// Wort[0] = Initial-SP im LPC-RAM, Wort[1] = Reset-Vektor (Thumb) im Flash.
bool has_valid_app_vectors(const uint8_t* img, uint32_t app_off, std::size_t sz) {
    if (static_cast<std::size_t>(app_off) + 8 > sz) return false;
    uint32_t sp, rv;
    std::memcpy(&sp, img + app_off,     4);
    std::memcpy(&rv, img + app_off + 4, 4);
    bool sp_ok = (sp >= 0x1000'0000u &&
                  sp <= 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE);
    uint32_t rvp = rv & ~1u;
    bool rv_ok = ((rv & 1u) != 0) && (rvp >= app_off) &&
                 (rvp < emulator::LPC_LOAD_MAX_SIZE);
    return sp_ok && rv_ok;
}

// Erzeugt bei Bedarf einen gueltigen Boot-Descriptor im Image. Ein bereits
// vorhandener, gueltiger Descriptor (z. B. via OTA) bleibt unangetastet.
void ensure_boot_descriptor(uint8_t* img, std::size_t sz,
                            uint32_t app_off, uint32_t desc_off) {
    if (desc_off + sizeof(AppDescriptor) > app_off) {
        std::printf("[EMU] Auto-Descriptor: desc_addr 0x%lx ueberlappt app 0x%lx\n",
                    static_cast<unsigned long>(desc_off),
                    static_cast<unsigned long>(app_off));
        return;
    }
    // Letztes Nicht-0xFF-Byte im App-Bereich bestimmen.
    uint32_t upper = (sz < emulator::LPC_LOAD_MAX_SIZE)
                         ? static_cast<uint32_t>(sz)
                         : emulator::LPC_LOAD_MAX_SIZE;
    uint32_t end_off = 0;
    bool     any     = false;
    for (uint32_t o = app_off; o < upper; ++o) {
        if (img[o] != 0xFFu) { end_off = o; any = true; }
    }
    if (!any) {
        std::printf("[EMU] Auto-Descriptor: keine App-Daten ab 0x%04lx\n",
                    static_cast<unsigned long>(app_off));
        return;
    }

    uint32_t crc = crc32_selfbus(img + app_off, end_off - app_off + 1);

    // Bereits gueltiger Descriptor? -> nicht ueberschreiben.
    AppDescriptor existing;
    std::memcpy(&existing, img + desc_off, sizeof existing);
    if (existing.startAddress == app_off &&
        existing.endAddress  >  app_off &&
        existing.endAddress  <  emulator::LPC_LOAD_MAX_SIZE) {
        uint32_t ecrc = crc32_selfbus(img + app_off,
                                      existing.endAddress - app_off + 1);
        if (ecrc == existing.crc) {
            std::printf("[EMU] Boot-Descriptor bereits gueltig "
                        "(start=0x%04lx end=0x%04lx)\n",
                        static_cast<unsigned long>(existing.startAddress),
                        static_cast<unsigned long>(existing.endAddress));
            return;
        }
    }

    // Schutz: nur schreiben, wenn die Descriptor-Region leer (geloescht, 0xFF)
    // ist ODER bereits wie ein Descriptor fuer diese app_start aussieht (dann
    // ggf. veraltet -> neu erzeugen). Enthaelt sie sonstige echte Daten (z. B.
    // ein eigenstaendiges App-Image bei einer Fehlerkennung), bleibt sie
    // unangetastet.
    bool blank = true;
    for (uint32_t o = desc_off; o < desc_off + sizeof(AppDescriptor); ++o) {
        if (img[o] != 0xFFu) { blank = false; break; }
    }
    bool looks_like_descriptor =
        (existing.startAddress == app_off) &&
        (existing.endAddress   >  app_off) &&
        (existing.endAddress   <  emulator::LPC_LOAD_MAX_SIZE);
    if (!blank && !looks_like_descriptor) {
        std::printf("[EMU] Auto-Descriptor: Region @0x%04lx enthaelt Fremddaten "
                    "-> uebersprungen\n", static_cast<unsigned long>(desc_off));
        return;
    }

    // Optionalen Versions-String "!AVP!@:" im App-Bereich suchen.
    static const char kMagic[7] = {'!', 'A', 'V', 'P', '!', '@', ':'};
    uint32_t ver_off = app_off;
    for (uint32_t o = app_off; o + sizeof kMagic <= upper; ++o) {
        if (std::memcmp(img + o, kMagic, sizeof kMagic) == 0) { ver_off = o; break; }
    }

    AppDescriptor desc;
    desc.startAddress      = app_off;
    desc.endAddress        = end_off;
    desc.crc               = crc;
    desc.appVersionAddress = ver_off;
    std::memcpy(img + desc_off, &desc, sizeof desc);

    std::printf("[EMU] Boot-Descriptor erzeugt @0x%04lx: start=0x%04lx "
                "end=0x%04lx crc=0x%08lx ver=0x%04lx\n",
                static_cast<unsigned long>(desc_off),
                static_cast<unsigned long>(app_off),
                static_cast<unsigned long>(end_off),
                static_cast<unsigned long>(crc),
                static_cast<unsigned long>(ver_off));
}


void core1_main() {
    multicore_lockout_victim_init();
    while (true) {
        while (g_state.load(std::memory_order_acquire) != emulator::State::Running) {
            __asm volatile ("wfe");
        }
        g_request_stop.store(false, std::memory_order_release);
        g_start_count.fetch_add(1, std::memory_order_relaxed);   // Diagnose: Gast-Starts

        // Firmware in RP2350-SRAM kopieren. Quelle ist storage::firmware_data()
        // — ein Pointer in den memory-mapped XIP-Bereich.
        const uint8_t* fw = storage::firmware_data();
        std::size_t   sz = storage::firmware_size();
        if (!fw || sz == 0 || sz > emulator::LPC_LOAD_MAX_SIZE) {
            std::printf("[EMU] keine valide Firmware (sz=%u)\n",
                        static_cast<unsigned>(sz));
            g_state.store(emulator::State::Halted);
            continue;
        }

        auto* dst = reinterpret_cast<uint32_t*>(g_firmware_image);
        std::memcpy(dst, fw, sz);
        if (sz < emulator::LPC_LOAD_MAX_SIZE) {
            std::memset(reinterpret_cast<uint8_t*>(dst) + sz, 0xFF,
                        emulator::LPC_LOAD_MAX_SIZE - sz);
        }

        // Zweistufige Konstellation erkennen: liegt an config::app_start eine
        // gueltige Applikations-Vektortabelle, so ist Slot 0 ein Bootloader und
        // die Applikation bleibt unangetastet (pristine), damit der vom
        // Bootloader geprueften CRC32 die Bytes exakt entsprechen.
        const uint32_t app_off = config::app_start_addr();
        const bool two_stage = has_valid_app_vectors(
            reinterpret_cast<const uint8_t*>(dst), app_off, sz);
        // Relokation/Patches in der zweistufigen Variante auf den Bootloader-
        // Bereich [0, app_off) begrenzen; sonst das gesamte Image.
        const std::size_t reloc_len =
            two_stage ? static_cast<std::size_t>(app_off) : sz;

        // Vector-Table relocaten + Fault-Handler einsetzen.
        constexpr uint32_t VEC_COUNT = 48; // 16 system + 32 IRQs (LPC1115)
        if (sz >= VEC_COUNT * 4) {
            for (uint32_t i = 1; i < VEC_COUNT; ++i) {
                dst[i] = relocate_vector(dst[i]);
            }
            // Initial-SP: falls in LPC-RAM-Range, auf Gast-RAM-Top umsetzen.
            uint32_t isp = dst[0];
            if (isp >= 0x1000'0000u && isp < 0x1000'0000u + 0x10000u) {
                dst[0] = reinterpret_cast<uint32_t>(g_guest_ram)
                       + emulator::LPC_GUEST_RAM_SIZE;
            }
            // System-Faults: unsere Handler eintragen.
            dst[3]  = reinterpret_cast<uint32_t>(&isr_hardfault);
            dst[4]  = reinterpret_cast<uint32_t>(&isr_memmanage);
            dst[5]  = reinterpret_cast<uint32_t>(&isr_busfault);
            dst[6]  = reinterpret_cast<uint32_t>(&isr_usagefault);

            // PendSV (Slot 14) gehört dem Host: Über PendSV werden emulierte
            // LPC-IRQs in den Gast injiziert (irq_inject.cpp). Die Firmware
            // selbst nutzt PendSV typischerweise nicht (LPC-Startups legen dort
            // nur einen `B .`-Default-Stub ab) — würde der Stub stehenbleiben,
            // liefe der Gast beim ersten injizierten IRQ in eine Endlosschleife.
            // SysTick (Slot 15) bleibt beim Gast, da es eine echte M33-
            // Peripherie ist und der Gast seinen eigenen Tick-Handler braucht.
            dst[14] = reinterpret_cast<uint32_t>(&isr_pendsv);

            // SysTick (Slot 15): Host-Shim davorschalten. Der relozierte Gast-
            // Handler wird gemerkt; der reale Core1-SysTick feuert isr_systick_shim,
            // der die LPC-Timer treibt und dann den Gast-Handler aufruft. Ohne das
            // liefe ein WFI-idlender Interrupt-Blink nur einmal (Timer stand still).
            g_guest_systick_handler.store(dst[15], std::memory_order_relaxed);
            dst[15] = reinterpret_cast<uint32_t>(&isr_systick_shim);

            // Opt-in-Patches, die Gast-Instruktionen auf SVC-Traps umlenken
            // (gemeinsamer Dispatcher isr_svc, Vektor-Slot 11):
            //   WFI-Pin-Wakeup : WFI      -> SVC #0
            //   PRIMASK-Schatten: CPSID i -> SVC #1, CPSIE i -> SVC #2
            bool need_svc = false;
            if (config::wfi_pin_wakeup()) {
                uint32_t n = hex_patcher::patch_wfi_to_svc(
                    reinterpret_cast<uint8_t*>(dst), reloc_len, VEC_COUNT * 4);
                need_svc = true;
                std::printf("[EMU] WFI-Pin-Wakeup aktiv: %u WFI gepatcht\n",
                            static_cast<unsigned>(n));
            }
            if (config::primask_shadow()) {
                uint32_t n = hex_patcher::patch_cps_to_svc(
                    reinterpret_cast<uint8_t*>(dst), reloc_len, VEC_COUNT * 4);
                need_svc = true;
                std::printf("[EMU] PRIMASK-Schatten aktiv: %u CPSID/CPSIE gepatcht\n",
                            static_cast<unsigned>(n));
            }
            if (need_svc) {
                dst[11] = reinterpret_cast<uint32_t>(&isr_svc);
            }
        }

        // RAM-Adressen aus Literal-Pools relocaten (LPC RAM 0x10000000+8KB
        // → RP2350 g_guest_ram). In der zweistufigen Variante nur den
        // Bootloader-Bereich; der App-Bereich bleibt pristine (Trap-Umleitung
        // zur Laufzeit) und wird im Handover relociert.
        uint32_t ram_base = reinterpret_cast<uint32_t>(g_guest_ram);
        auto pr = hex_patcher::relocate_ram_refs(
            reinterpret_cast<uint8_t*>(dst), reloc_len,
            0x1000'0000u, emulator::LPC_GUEST_RAM_SIZE,
            ram_base);
        std::printf("[EMU] hex-patch: %u/%u Woerter relociert%s\n",
                    static_cast<unsigned>(pr.patched_words),
                    static_cast<unsigned>(pr.scanned_words),
                    two_stage ? " (nur Bootloader)" : "");

        // Zweistufig: bei Bedarf einen gueltigen Boot-Descriptor erzeugen, damit
        // der Bootloader die kombiniert geladene Applikation akzeptiert.
        if (two_stage && config::autodesc()) {
            ensure_boot_descriptor(reinterpret_cast<uint8_t*>(dst), sz, app_off,
                                   config::descriptor_addr());
        } else if (two_stage) {
            std::printf("[EMU] zweistufig erkannt (app@0x%04lx), Auto-Descriptor "
                        "deaktiviert\n", static_cast<unsigned long>(app_off));
        }

        // VTOR setzen (muss 256-Byte-aligned sein, Array ist alignas(256)).
        uint32_t load_base = reinterpret_cast<uint32_t>(g_firmware_image);
        SCB->VTOR = load_base;
        __DSB(); __ISB();

        // Aktive Vektortabelle = Anfang des Images (Bootloader bzw. direkt
        // geflashte Applikation). Ein evtl. spaeterer Handover stellt das um.
        g_vtable_base.store(load_base, std::memory_order_release);
        g_sp_fixup_pending.store(false, std::memory_order_release);

        // MPU für den Gast scharf schalten. Es werden NUR die Gast-eigenen
        // SRAM-Bereiche freigegeben (Code-Image, Gast-RAM, enter_guest-
        // Trampolin) — der restliche RP2350-SRAM (TinyUSB-Puffer, Core-Stacks,
        // Emulator-Daten) bleibt fuer den unprivilegierten Gast gesperrt. So
        // wird ein wilder Gast-Pointer/Heap-/Stack-Ueberlauf zu einem sauberen
        // Trap statt zu stiller Korruption des Host-USB-Stacks.
        mpu_setup::enable_for_guest(
            load_base,
            reinterpret_cast<uint32_t>(g_guest_ram),
            reinterpret_cast<uint32_t>(&enter_guest));

        uint32_t initial_sp = dst[0];
        uint32_t reset_h    = dst[1];
        std::printf("[EMU] starting: SP=0x%08lx PC=0x%08lx (load=0x%08lx, %u B)\n",
                    static_cast<unsigned long>(initial_sp),
                    static_cast<unsigned long>(reset_h),
                    static_cast<unsigned long>(load_base),
                    static_cast<unsigned>(sz));

        // Sprung in den Gast — kommt nicht zurück.
        g_pc.store(reset_h);
        enter_guest(initial_sp, reset_h);
        // unreachable
    }
}

} // namespace

namespace emulator {

void boot_core1() {
    multicore_launch_core1(core1_main);
}

// Core1 hart resetten und core1_main neu starten. NUR von Core0 aufrufen.
// target == Running: Gast danach sofort wieder anlaufen lassen (Soft-Reset).
// target == Idle:    Gast bleibt gestoppt (CLI 'stop'/Flash-Pause).
//
// Core0 ist seit main() Multicore-Lockout-Victim, d.h. sein SIO-FIFO-IRQ ist
// aktiv. multicore_reset_core1() und boot_core1() (multicore_launch_core1)
// kommunizieren ueber genau diesen FIFO; wuerde Core0s Lockout-Handler
// dazwischenfunken, draint er die Handshake-Bytes weg -> Hang. Daher den
// FIFO-IRQ um Reset+Relaunch herum stilllegen.
void core1_reset_and_relaunch(State target) {
    const bool fifo_irq = irq_is_enabled(SIO_IRQ_FIFO);
    if (fifo_irq) irq_set_enabled(SIO_IRQ_FIFO, false);
    multicore_reset_core1();
    g_state.store(State::Idle);          // core1_main soll zunaechst warten
    mpu_setup::disable();
    target_halt::on_guest_reset();       // stehengebliebene Halt-Flags loeschen
    boot_core1();
    if (fifo_irq) irq_set_enabled(SIO_IRQ_FIFO, true);
    if (target == State::Running) {
        g_state.store(State::Running, std::memory_order_release);
        __asm volatile ("sev");
    }
}

void load_and_start() {
    if (g_state.load() == State::Running) {
        std::printf("[EMU] bereits gestartet\n");
        return;
    }
    if (storage::firmware_size() == 0) {
        std::printf("[EMU] keine Firmware geflashed\n");
        return;
    }
    g_state.store(State::Running, std::memory_order_release);
    __asm volatile ("sev");
}

void stop() {
    // Ein laufender, in unprivileged Mode bxender Gast lässt sich nicht
    // ohne Weiteres anhalten. Wir schießen Core 1 ab und reinitialisieren.
    core1_reset_and_relaunch(State::Idle);
}

void request_guest_reset() {
    // Vom Gast angeforderter Soft-Reset (WDT-Ablauf ODER NVIC_SystemReset, das
    // der Fault-Handler abfaengt). NUR der Gast wird neu gestartet, der RP2350
    // laeuft weiter.
    //
    // KRITISCH: Diese Funktion wird typischerweise AUF Core1 aufgerufen
    // (WDT-Advance im WFI-Loop / Fault-Handler). multicore_reset_core1() von
    // Core1 aus wuerde Core1 sofort abschalten — die Relaunch-Schritte danach
    // liefen nie, der Gast bliebe tot. Daher von Core1 nur eine Anforderung an
    // Core0 stellen und parken; Core0 fuehrt den Reset im Loop aus.
    if (get_core_num() != 0u) {
        g_guest_reset_req.store(true, std::memory_order_release);
        __DSB();
        (void) save_and_disable_interrupts();   // kein Fehl-Vektoring mehr
        for (;;) __asm volatile ("wfe");         // Core0 resettet diesen Core
    }
    core1_reset_and_relaunch(State::Running);
}

// Von Core0 im Loop konsumiert: fuehrt einen vom Gast angeforderten Reset aus.
bool guest_reset_pending() {
    return g_guest_reset_req.exchange(false, std::memory_order_acq_rel);
}

// Vom Fault-Handler (Core1) aufgerufen, wenn ein Gast-Fault nicht emulierbar
// ist: Gast anhalten statt das ganze Silizium per Watchdog zu rebooten. Core0
// (CLI/USB) laeuft weiter, die [FAULT]-Diagnose bleibt sichtbar.
void notify_guest_faulted() {
    g_state.store(State::Faulted, std::memory_order_release);
}

bool pause_for_flash() {
    // Nur sinnvoll von Core0 aus. Der IAP-Pfad laeuft auf Core1 im Fault-
    // Handler und sperrt Core0 selbst per Lockout aus (FlashGuard) -> hier
    // No-op, damit Core1 sich nicht selbst zu resetten versucht.
    if (get_core_num() != 0u) return false;
    const State st = g_state.load(std::memory_order_acquire);
    // Running ODER Faulted: in beiden Faellen haengt Core1 am Gast (laufend
    // bzw. im Fault-Handler gehalten) und kann den Lockout-Handshake nicht
    // bedienen -> vor dem Flash-Zugriff sicher per Core-Reset in die
    // SDK-Spin-Schleife zuruecksetzen.
    if (st != State::Running && st != State::Faulted) return false;
    // stop() resettet Core1 und relaunched core1_main -> Gast haelt, Core1
    // steht in der Spin-Schleife (SDK-VTOR). Flash-Writes sind nun sicher.
    stop();
    return true;
}

void resume_for_flash(bool was_running) {
    if (was_running) load_and_start();   // Firmware (ggf. neu) laden + starten
}

State    state()    { return g_state.load(); }
uint32_t pc()       { return g_pc.load(); }
uint64_t mem_traps(){ return faultsys::stats().mem_traps; }
uint32_t start_count(){ return g_start_count.load(std::memory_order_relaxed); }

uint32_t pc_samples(uint32_t* out, uint32_t max) {
    if (!out || max == 0) return 0;
    uint32_t pos = g_pc_sample_pos.load(std::memory_order_relaxed);
    uint32_t n = (pos < PC_SAMPLES) ? pos : PC_SAMPLES;
    if (n > max) n = max;
    // Neueste zuerst: pos zeigt auf den naechsten Schreibindex.
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t idx = (pos - 1u - i) % PC_SAMPLES;
        out[i] = g_pc_samples[idx].load(std::memory_order_relaxed);
    }
    return n;
}

void shim_debug(uint32_t& enter, uint32_t& exit, uint32_t& age_ms) {
    enter = g_shim_enter.load(std::memory_order_relaxed);
    exit  = g_shim_exit.load(std::memory_order_relaxed);
    uint64_t last = g_shim_last_us.load(std::memory_order_relaxed);
    uint64_t now  = time_us_64();
    age_ms = (last && now > last) ? static_cast<uint32_t>((now - last) / 1000u) : 0u;
}

uint32_t load_base() {
    return reinterpret_cast<uint32_t>(g_firmware_image);
}

uint32_t guest_ram_base() {
    return reinterpret_cast<uint32_t>(g_guest_ram);
}

uint32_t vtable_base() {
    uint32_t v = g_vtable_base.load(std::memory_order_acquire);
    return v ? v : reinterpret_cast<uint32_t>(g_firmware_image);
}

bool take_handover_sp_fixup(uint32_t& raw, uint32_t& reloc) {
    if (!g_sp_fixup_pending.exchange(false, std::memory_order_acquire))
        return false;
    raw   = g_sp_fixup_raw.load(std::memory_order_relaxed);
    reloc = g_sp_fixup_reloc.load(std::memory_order_relaxed);
    return true;
}

void activate_bootloader_handover() {
    // Aufgerufen aus dem SYSCON-Modell, wenn der Gast SYSMEMREMAP in einen
    // User-Mode schreibt. Der LPC-Bootloader hat zuvor die 192-Byte-Vektor-
    // tabelle der Applikation an den RAM-Anfang (0x10000000 -> g_guest_ram)
    // kopiert. Wir suchen exakt diese 192 Bytes im 64-KB-Flash-Image, um den
    // Flash-Offset der Applikation zu bestimmen.
    auto*  img = reinterpret_cast<uint8_t*>(g_firmware_image);
    auto*  ram = reinterpret_cast<const uint8_t*>(g_guest_ram);
    constexpr uint32_t VEC_BYTES = 48 * 4;   // 192

    uint32_t app_off = 0;
    bool     found   = false;
    // 256-Byte-aligned absuchen (Vektortabellen sind so ausgerichtet); den
    // Offset 0 (Bootloader-Tabelle selbst) ueberspringen.
    for (uint32_t off = 0x100; off + VEC_BYTES <= emulator::LPC_LOAD_MAX_SIZE;
         off += 0x100) {
        if (std::memcmp(img + off, ram, VEC_BYTES) == 0) {
            app_off = off;
            found   = true;
            break;
        }
    }
    if (!found) {
        std::printf("[EMU] Handover: keine Applikations-Vektortabelle gefunden\n");
        return;
    }

    auto* dst = reinterpret_cast<uint32_t*>(img + app_off);

    // SP-Korrektur vormerken. dst[0] ist der Applikations-StackTop. Bei einem
    // vorab geladenen (kombinierten) Image wurde er bereits beim Laden per
    // relocate_ram_refs in den Gast-RAM-Puffer umgesetzt -> kein Fixup noetig.
    // Bei einer ueber den Bus eingespielten (OTA-)Applikation ist er dagegen
    // noch roh (z. B. 0x10002000) und zeigt ins RP2350-XIP-Fenster, das nicht
    // als Stack beschreibbar ist. Der Bootloader hat diesen rohen Wert bereits
    // gelesen und in ein Register geladen; der Fault-Handler ersetzt ihn vor
    // dem `mov SP, StackTop` durch die relocierte Gast-RAM-Adresse.
    {
        uint32_t raw_sp = dst[0];
        if (raw_sp >= 0x1000'0000u &&
            raw_sp <= 0x1000'0000u + emulator::LPC_GUEST_RAM_SIZE) {
            g_sp_fixup_raw.store(raw_sp, std::memory_order_relaxed);
            g_sp_fixup_reloc.store(
                reinterpret_cast<uint32_t>(g_guest_ram) + (raw_sp - 0x1000'0000u),
                std::memory_order_relaxed);
            g_sp_fixup_pending.store(true, std::memory_order_release);
        }
    }

    // Vektortabelle der Applikation im Image relocaten (Eintraege 1..47),
    // analog zum Loader. Geschieht NACH der checkApplication-CRC-Pruefung des
    // Bootloaders (diese laeuft vor dem Sprung), daher stoert die Modifikation
    // die CRC nicht mehr.
    for (uint32_t i = 1; i < 48; ++i) {
        dst[i] = relocate_vector(dst[i]);
    }
    // System-Fault-Handler + PendSV auf unsere Host-Handler setzen.
    dst[3]  = reinterpret_cast<uint32_t>(&isr_hardfault);
    dst[4]  = reinterpret_cast<uint32_t>(&isr_memmanage);
    dst[5]  = reinterpret_cast<uint32_t>(&isr_busfault);
    dst[6]  = reinterpret_cast<uint32_t>(&isr_usagefault);
    dst[14] = reinterpret_cast<uint32_t>(&isr_pendsv);
    // SysTick (Slot 15): Host-Shim wie im Loader (treibt LPC-Timer + ruft den
    // Applikations-SysTick-Handler). Der relozierte App-Handler wird gemerkt.
    g_guest_systick_handler.store(dst[15], std::memory_order_relaxed);
    dst[15] = reinterpret_cast<uint32_t>(&isr_systick_shim);
    // SVC-Dispatcher nur, wenn ein opt-in-Patch (WFI/PRIMASK) aktiv ist.
    // Hinweis: In der zweistufigen Variante bleibt der App-Bereich pristine
    // (nicht gepatcht), damit die Bootloader-CRC passt — die WFI/PRIMASK-
    // Patches wirken dann nur im Bootloader. dst[11] wird hier dennoch gesetzt,
    // ist aber wirkungslos, solange die Applikation kein SVC selbst nutzt.
    if (config::wfi_pin_wakeup() || config::primask_shadow()) {
        dst[11] = reinterpret_cast<uint32_t>(&isr_svc);
    }

    uint32_t base = reinterpret_cast<uint32_t>(img) + app_off;
    SCB->VTOR = base;
    __DSB(); __ISB();
    g_vtable_base.store(base, std::memory_order_release);

    std::printf("[EMU] Handover: Applikation @Flash-Offset 0x%04lx aktiviert "
                "(VTOR=0x%08lx)\n",
                static_cast<unsigned long>(app_off),
                static_cast<unsigned long>(base));
}

} // namespace emulator
