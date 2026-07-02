#pragma once
//
// API zum Native-Execution-Emulator (läuft auf Core 1).
// Lädt LPC-Firmware in RP2350-SRAM, patcht Vektor-Table, aktiviert MPU,
// wechselt in unprivilegierten PSP-Thread-Mode und startet die Firmware
// mit BX in den Reset-Handler.
//

#include <cstdint>
#include <cstddef>

namespace emulator {

enum class State : uint8_t { Idle, Running, Halted, Faulted };

// Größen der Guest-Speicherbereiche.
constexpr uint32_t LPC_LOAD_MAX_SIZE  = 64 * 1024;
// LPC1115 hat 8 KB SRAM (0x10000000-0x10001FFF).
constexpr uint32_t LPC_GUEST_RAM_SIZE = 8 * 1024;

// Laufzeit-Adressen der Guest-Speicherbereiche im RP2350-SRAM.
// Werden in emulator.cpp als aligned Arrays allokiert.
// VTOR-Anforderung: LPC_LOAD_BASE muss 256-Byte-aligned sein.
uint32_t load_base();       // Adresse des Code-Image-Puffers
uint32_t guest_ram_base();  // Adresse des Guest-RAM-Puffers

// Basis-Adresse der aktuell aktiven Guest-Vektortabelle (absolute RP2350-
// Adresse). Nach dem Laden == load_base() (Vektortabelle am Flash-Anfang).
// Nach einem Bootloader->Applikation-Handover zeigt sie auf die Vektortabelle
// der gestarteten Applikation (load_base() + Flash-Offset). Die IRQ-Injektion
// liest hierueber den passenden LPC-Handler.
uint32_t vtable_base();

// Wird vom SYSCON-Modell aufgerufen, sobald der Gast SYSMEMREMAP in einen
// User-Mode schreibt (LPC-typischer Bootloader->Applikation-Uebergang).
// Lokalisiert die soeben nach Guest-RAM kopierte Applikations-Vektortabelle
// im Flash-Image, reloziert deren Eintraege + System-Handler, setzt VTOR und
// die IRQ-Vektorbasis auf die Applikation um. No-op, wenn kein passendes
// zweites Image gefunden wird (dann bleibt die bisherige Tabelle aktiv).
void activate_bootloader_handover();

// Liefert eine vom Handover vorgemerkte SP-Korrektur (rohe LPC-RAM-StackTop-
// Adresse raw -> relocierte Gast-RAM-Adresse reloc) und konsumiert sie.
// Liefert true, wenn eine Korrektur anstand. Der Fault-Handler patcht damit
// das Register, aus dem der Bootloader unmittelbar danach den SP laedt.
bool take_handover_sp_fixup(uint32_t& raw, uint32_t& reloc);

void boot_core1();
void load_and_start();
void stop();
// Asynchrones „Soft-Reset" nur des Guests (z. B. WDT-Ablauf, NVIC_SystemReset).
// Sicher von Core1 aufrufbar: stellt dann nur eine Anforderung an Core0 und
// parkt den Core, bis Core0 den eigentlichen Reset ausfuehrt.
void request_guest_reset();
// Von Core0 im Hauptloop konsumiert: true + Reset, wenn der Gast einen Reset
// angefordert hat (request_guest_reset von Core1). Fuehrt den Reset NICHT
// selbst aus — der Aufrufer ruft danach request_guest_reset() auf Core0.
bool guest_reset_pending();
// Vom Fault-Handler (Core1) bei nicht-emulierbarem Gast-Fault aufgerufen:
// Gast in State::Faulted halten statt das Silizium per Watchdog zu rebooten.
void notify_guest_faulted();
// --- Flash-Schreibschutz (nur Core0) --------------------------------------
// Ein Flash-Erase/Program blockiert XIP. Laeuft der Gast nativ auf Core1, wuerde
// dessen naechster Trap (unsere Fault-Handler liegen im XIP-Flash) waehrend der
// Operation crashen. Vor einem Core0-Flash-Schreibzugriff (CLI/USB-MSC) muss der
// Gast daher pausiert werden: stop() setzt Core1 per Reset in die sichere
// Spin-Schleife zurueck (SDK-VTOR -> Multicore-Lockout-Handler erreichbar, kein
// Gast-Code, der korrumpiert werden koennte). Auf Core1 (IAP-Pfad im Fault-
// Handler) sind diese Funktionen ein No-op; dort sperrt der FlashGuard statt-
// dessen Core0 per Multicore-Lockout aus.
bool pause_for_flash();               // true, wenn ein Gast lief
void resume_for_flash(bool was_running);
// RAII-Wrapper: pausiert im Konstruktor, setzt im Destruktor fort.
struct FlashPauseGuard {
    bool was_running;
    FlashPauseGuard()  : was_running(pause_for_flash()) {}
    ~FlashPauseGuard() { resume_for_flash(was_running); }
    FlashPauseGuard(const FlashPauseGuard&)            = delete;
    FlashPauseGuard& operator=(const FlashPauseGuard&) = delete;
};
State    state();
uint64_t mem_traps();
uint32_t pc();

// Anzahl der Gast-Starts seit Boot (Diagnose: steigt der Wert im Betrieb, wird
// der Gast wiederholt neu gestartet -> Blink-/LED-Aussetzer).
uint32_t start_count();

} // namespace emulator
