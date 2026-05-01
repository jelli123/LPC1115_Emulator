#include "clock.h"

#include <cstdio>
#include <cstring>
#include <cstdint>

// LPC1115-PLL: F_clkout = F_clkin * (MSEL+1)
// MSEL = SYSPLLCTRL[4:0], PSEL = SYSPLLCTRL[6:5] (für CCO-Bereich, beeinflusst
// F_clkout NICHT). SYSAHBCLKDIV teilt das Main-Clock auf den AHB-Bus.
// Eine reine statische Bytemuster-Suche kann die zur Laufzeit programmierten
// Werte nicht zuverlässig finden. Wir liefern daher einen sicheren Default
// und scannen nur nach klaren Marker-Konstanten („magic“ MOV/MOVW immediates),
// die bei aktivem Compiler-Output für die LPC1115-Standard-48-MHz-Sequenz
// erscheinen (M=3, P=1, DIV=1).

ClockConfig extract_clock_settings(const uint8_t* image, std::size_t image_size) {
    ClockConfig def{48'000'000, OscillatorType::IRC};
    if (!image || image_size < 8) return def;

    // Sehr defensive, ausschließlich lesende Heuristik.
    // Beispiel-Marker:  Sequenz 0x24 0x00 → MOVS R4, #36 (MSEL=3,PSEL=1 = 0x23?
    // — bewusst zu ungenau, daher nur Hinweis ausgeben).
    bool found_pll_marker = false;
    for (std::size_t i = 0; i + 1 < image_size; ++i) {
        // MOVS Rd, #imm8   — opcode 0010 0xxx iiii iiii
        uint16_t op = static_cast<uint16_t>(image[i] | (image[i + 1] << 8));
        if ((op & 0xF800) == 0x2000 && (op & 0xFFu) == 0x23u) {
            found_pll_marker = true;
            break;
        }
    }

    std::printf("[CLOCK] Heuristik: %sPLL-Marker gefunden, %u Hz default\n",
                found_pll_marker ? "" : "kein ",
                static_cast<unsigned>(def.frequency));
    return def;
}
