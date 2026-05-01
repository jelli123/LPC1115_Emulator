#pragma once
//
// Trap-Instruction-Decoder für Native-Execution-Modell.
//
// Wenn die ARMv8-M MPU einen Zugriff auf eine LPC-Peripherie-Adresse
// abfängt, ruft der BusFault/MemManage-Handler decode_mem_access() mit
// dem Stacked-PC auf. Wir decoden ausschließlich die LDR/STR-Familie
// von Thumb/Thumb-2, alles andere ist ein echter Fault.
//

#include <cstdint>

namespace trap_decoder {

enum class AccessSize : uint8_t { B = 1, H = 2, W = 4 };

struct Access {
    bool        is_load;
    bool        is_signed;     // LDRSB/LDRSH
    AccessSize  size;
    uint8_t     rt;            // Ziel-/Quellregister im gestackten Frame
    uint32_t    address;       // berechnete effektive Adresse
    uint32_t    instr_size;    // 2 oder 4 (Bytes), zum PC-Skip
};

struct StackedFrame {
    // Layout: r0,r1,r2,r3,r12,lr,pc,xpsr (Cortex-M Standard-Stacking)
    uint32_t r0,r1,r2,r3,r12,lr,pc,xpsr;
};

// Liest die Instruktion an pc, decodiert Speicher-Zugriff. Liefert false
// wenn unbekannte Instruktion (echter Fault). Berechnet Effektivadresse
// aus Frame-Werten plus banked-r4..r11/sp/lr (für die werden Caller-
// gespeicherte Werte aus dem aktiven Modus verwendet).
bool decode_mem_access(const StackedFrame* frame,
                       uint32_t r4_r11[8],   // Hilfregister, vom Asm-Wrapper geladen
                       uint32_t sp_main,
                       Access& out);

// Liefert für ein Register die Quelle (Pointer auf 32-bit-Speicher).
uint32_t* reg_ptr(StackedFrame* frame, uint32_t r4_r11[8],
                  uint32_t* sp_main_ptr, uint8_t rn);

// Liefert die Länge der Thumb-Instruktion am gegebenen 16-Bit-Halbwort
// (2 oder 4). Per ARMv7-M ARM A6.1: bei [15:11] in {0b11101, 0b11110,
// 0b11111} ist es eine 32-Bit-Instruktion. Auf reinem ARMv6-M existieren
// 32-Bit-Instruktionen ausschließlich als BL — auch dort gilt dasselbe
// Bitmuster.
inline uint32_t thumb_instr_size(uint16_t hw0) {
    uint32_t top5 = (hw0 >> 11) & 0x1Fu;
    return (top5 == 0x1Du || top5 == 0x1Eu || top5 == 0x1Fu) ? 4u : 2u;
}

} // namespace trap_decoder
