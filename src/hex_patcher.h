#pragma once
//
// Adress-Relocation einer LPC1115-Firmware ohne Re-Linken.
//
// Die LPC1115 platziert ihren SRAM bei 0x10000000-0x10001FFF (8 KB). Auf
// dem RP2350 ist dieser Adressbereich vom XIP belegt; wir ordnen den
// Gast-SRAM stattdessen dynamisch an (siehe emulator.h, emulator.cpp).
//
// Der Code-Generator von Keil/GCC legt SRAM-Adressen als 32-Bit-Konstanten
// in Literal-Pools ab (fuer `LDR Rx, =sym`). Der Patcher scannt das gesamte
// Firmware-Image nach 4-byte-aligned Woertern, deren Wert im Bereich
// [0x10000000, 0x10000000 + LPC_GUEST_RAM_SIZE] liegt (obere Grenze
// INKLUSIVE, um "one-past-end"-Zeiger wie _estack/_ebss abzudecken), und
// ersetzt sie durch guest_ram_base + (orig - 0x10000000).
//
// Robustheit / Backstop:
//   - Ein Fault-Handler-Pfad (src/fault.cpp) faengt RAM-Zugriffe ab, die die
//     Heuristik *verpasst* hat (z. B. zur Laufzeit berechnete Adressen), und
//     bedient sie korrekt auf dem Gast-RAM. Ein verpasster Treffer ist damit
//     nicht mehr fatal, sondern nur langsamer.
//
// Heuristik-Hinweis (im README dokumentiert):
//   - False-Positive-Wahrscheinlichkeit ~ 8 KB / 2^32 ~ 1 / 524288 pro Wort.
//     Eine 64 KB Firmware hat 16384 Woerter => Erwartungswert ~ 0.03 false
//     positives. Praktisch vernachlaessigbar.
//   - Werden ausschliesslich 4-byte-aligned Stellen geaendert; Befehlsstrom
//     im Thumb-Code ist 2-byte-aligned, daher kann eine BL-Instruktion
//     nicht versehentlich an ihrer 32-Bit-Form veraendert werden, wenn sie
//     nicht zufaellig 4-byte-aligned beginnt UND dieselbe Bitkombination
//     traegt. Das ist astronomisch unwahrscheinlich.
//

#include <cstdint>
#include <cstddef>

namespace hex_patcher {

struct Result {
    uint32_t patched_words;
    uint32_t scanned_words;
};

// Patcht das Firmware-Image *in place*. old_ram_base/old_ram_size beschreiben
// den LPC-SRAM-Bereich (0x10000000 / 8 KB), new_ram_base die Laufzeit-Basis
// des Gast-RAM-Puffers (emulator::guest_ram_base()).
Result relocate_ram_refs(uint8_t* image, std::size_t size_bytes,
                         uint32_t old_ram_base, uint32_t old_ram_size,
                         uint32_t new_ram_base);

// Ersetzt alle WFI-Instruktionen (Thumb 0xBF30) durch SVC #0 (0xDF00), damit
// eine reine WFI-Warteschleife der Gast-Firmware den Host-SVC-Trap auslöst
// (siehe src/emulator.cpp, isr_svc_wfi). Nur opt-in (config::wfi_pin_wakeup).
//
// Heuristik / Risiko:
//   - Gescannt werden 2-byte-aligned Halbwörter ab Offset `skip_bytes`
//     (üblicherweise hinter der Vektor-Tabelle), um Vektoren nicht zu treffen.
//   - Ein 16-Bit-Datenwort 0xBF30 im Code-Image würde fälschlich gepatcht.
//     Daher ist das Feature default AUS und nur für getestete Firmware
//     gedacht. Rückgabe = Anzahl ersetzter Instruktionen.
//   - WFI mit aktivierten Interrupts (PRIMASK=0) läuft über den SVC-Handler;
//     WFI mit gesperrten Interrupts (PRIMASK=1) eskaliert den SVC zu
//     HardFault und wird dort gesondert behandelt (src/fault.cpp).
uint32_t patch_wfi_to_svc(uint8_t* image, std::size_t size_bytes,
                          std::size_t skip_bytes);

// Ersetzt CPSID i (Thumb 0xB672) durch SVC #1 (0xDF01) und CPSIE i (0xB662)
// durch SVC #2 (0xDF02), damit der Host die Gast-Kritischen-Sektionen
// (__disable_irq()/__enable_irq()) als Schatten-PRIMASK nachfuehren kann
// (siehe src/emulator.cpp svc_dispatch_c, src/vnvic.cpp). Nur opt-in
// (config::primask_shadow). Gleiche Heuristik/Risiko wie patch_wfi_to_svc:
// gescannt werden 2-byte-aligned Halbwoerter ab `skip_bytes`; ein passendes
// 16-Bit-Datenwort im Image wuerde faelschlich gepatcht. Rueckgabe = Anzahl.
uint32_t patch_cps_to_svc(uint8_t* image, std::size_t size_bytes,
                          std::size_t skip_bytes);

} // namespace hex_patcher
