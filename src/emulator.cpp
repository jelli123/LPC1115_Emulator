#include "emulator.h"
#include "mmu.h"
#include "fault.h"
#include "peripherals.h"
#include "storage.h"
#include "config.h"
#include "hex_patcher.h"
#include "vnvic.h"

#include <atomic>
#include <cstdio>
#include <cstring>

#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include "RP2350.h"   // CMSIS, via cmsis_core lib

namespace {

std::atomic<emulator::State> g_state{emulator::State::Idle};
std::atomic<bool>            g_request_stop{false};
std::atomic<uint32_t>        g_pc{0};

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

// Naked Trampolin: setzt MPU, wechselt auf PSP/unprivileged Thread Mode,
// und springt mit BX in den Reset-Handler. Kehrt nie zurück (Gast läuft
// in Endlos-Loop wie eine echte MCU).
__attribute__((naked, noreturn))
void enter_guest(uint32_t /*initial_sp*/, uint32_t /*reset_handler*/) {
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

        // MPU für den Gast scharf schalten.
        mpu_setup::enable_for_guest();

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
    multicore_reset_core1();
    g_state.store(State::Idle);
    mpu_setup::disable();
    boot_core1();
}

void request_guest_reset() {
    // Vom WDT-Modell aufgerufen: nur Guest neustarten, RP2350 läuft weiter.
    // multicore_reset_core1 ist von beiden Cores aus sicher.
    multicore_reset_core1();
    g_state.store(State::Idle);
    mpu_setup::disable();
    boot_core1();
    g_state.store(State::Running, std::memory_order_release);
    __asm volatile ("sev");
}

State    state()    { return g_state.load(); }
uint32_t pc()       { return g_pc.load(); }
uint64_t mem_traps(){ return faultsys::stats().mem_traps; }

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
