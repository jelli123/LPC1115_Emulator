#pragma once
//
// Adress-Relocation einer LPC1115-Firmware ohne Re-Linken.
//
// Die LPC1115 platziert ihren SRAM bei 0x10000000-0x10001FFF (8 KB). Auf
// dem RP2350 ist dieser Adressbereich vom XIP belegt; wir ordnen den
// Gast-SRAM stattdessen bei 0x20060000 an (siehe emulator.h).
//
// Der Code-Generator von Keil/GCC legt SRAM-Adressen als 32-Bit-Konstanten
// in Literal-Pools ab (für `LDR Rx, =sym`). Der Patcher scannt das gesamte
// Firmware-Image nach 4-byte-aligned Wörtern, deren Wert exakt im
// Bereich [0x10000000, 0x10002000) liegt, und ersetzt sie durch
// 0x20060000 + (orig - 0x10000000).
//
// Heuristik-Hinweis (im README dokumentiert):
//   - False-Positive-Wahrscheinlichkeit ~ 8 KB / 2^32 ≈ 1 / 524288 pro Wort.
//     Eine 64 KB Firmware hat 16384 Wörter ⇒ Erwartungswert ~ 0.03 false
//     positives. Praktisch vernachlässigbar, aber nicht 0.
//   - Werden ausschließlich 4-byte-aligned Stellen geändert; Befehlsstrom
//     im Thumb-Code ist 2-byte-aligned, daher kann eine BL-Instruktion
//     nicht versehentlich an ihrer 32-Bit-Form verändert werden, wenn sie
//     nicht zufällig 4-byte-aligned beginnt UND dieselbe Bitkombination
//     trägt. Das ist astronomisch unwahrscheinlich.
//

#include <cstdint>
#include <cstddef>

namespace hex_patcher {

struct Result {
    uint32_t patched_words;
    uint32_t scanned_words;
};

// Patcht das Firmware-Image *in place*. base_addr ist die Ladeadresse
// (i. d. R. 0x20040000 = LPC_LOAD_BASE).
Result relocate_ram_refs(uint8_t* image, std::size_t size_bytes,
                         uint32_t old_ram_base, uint32_t old_ram_size,
                         uint32_t new_ram_base);

} // namespace hex_patcher
