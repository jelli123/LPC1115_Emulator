#include "opcodes.h"

#include <cstdint>

// Trap-Decoder. Wir betrachten nur die Speicher-Zugriffs-Encodings, die
// ein für Cortex-M0 gebauter LPC1115-Compiler erzeugen kann. Alle anderen
// 16-Bit-Thumb-Encodings sind echte Faults und werden vom Aufrufer eskaliert.

namespace trap_decoder {

namespace {

uint32_t read_reg(const StackedFrame* f, const uint32_t r4_r11[8],
                  uint32_t sp_main, uint8_t r) {
    switch (r) {
        case 0:  return f->r0;
        case 1:  return f->r1;
        case 2:  return f->r2;
        case 3:  return f->r3;
        case 4:  return r4_r11[0];
        case 5:  return r4_r11[1];
        case 6:  return r4_r11[2];
        case 7:  return r4_r11[3];
        case 8:  return r4_r11[4];
        case 9:  return r4_r11[5];
        case 10: return r4_r11[6];
        case 11: return r4_r11[7];
        case 12: return f->r12;
        case 13: return sp_main;
        case 14: return f->lr;
        case 15: return f->pc + 4;        // PC liest sich +4 voraus (Thumb)
        default: return 0;
    }
}

bool decode16(uint16_t op, const StackedFrame* f, const uint32_t r4_r11[8],
              uint32_t sp_main, Access& a) {
    a.instr_size = 2;

    // 011 BL imm5 — LDR/STR/LDRB/STRB (immediate)
    if ((op & 0xE000u) == 0x6000u) {
        bool B = (op >> 12) & 1u;
        bool L = (op >> 11) & 1u;
        uint8_t imm5 = (op >> 6) & 0x1F;
        uint8_t rn   = (op >> 3) & 0x7;
        uint8_t rt   = op & 0x7;
        uint32_t base = read_reg(f, r4_r11, sp_main, rn);
        a.is_load = L;
        a.is_signed = false;
        a.size = B ? AccessSize::B : AccessSize::W;
        a.rt = rt;
        a.address = base + (B ? imm5 : (static_cast<uint32_t>(imm5) << 2));
        return true;
    }

    // 1000 L imm5 — LDRH/STRH (immediate)
    if ((op & 0xF000u) == 0x8000u) {
        bool L = (op >> 11) & 1u;
        uint8_t imm5 = (op >> 6) & 0x1F;
        uint8_t rn   = (op >> 3) & 0x7;
        uint8_t rt   = op & 0x7;
        a.is_load = L;
        a.is_signed = false;
        a.size = AccessSize::H;
        a.rt = rt;
        a.address = read_reg(f, r4_r11, sp_main, rn) + (static_cast<uint32_t>(imm5) << 1);
        return true;
    }

    // 0101 ooo Rm Rn Rt — LDR/STR (register)
    if ((op & 0xF000u) == 0x5000u) {
        uint8_t opc = (op >> 9) & 0x7;
        uint8_t rm  = (op >> 6) & 0x7;
        uint8_t rn  = (op >> 3) & 0x7;
        uint8_t rt  = op & 0x7;
        uint32_t addr = read_reg(f, r4_r11, sp_main, rn) +
                        read_reg(f, r4_r11, sp_main, rm);
        a.rt = rt;
        a.address = addr;
        switch (opc) {
            case 0: a.is_load=false; a.is_signed=false; a.size=AccessSize::W; return true; // STR
            case 1: a.is_load=false; a.is_signed=false; a.size=AccessSize::H; return true; // STRH
            case 2: a.is_load=false; a.is_signed=false; a.size=AccessSize::B; return true; // STRB
            case 3: a.is_load=true;  a.is_signed=true;  a.size=AccessSize::B; return true; // LDRSB
            case 4: a.is_load=true;  a.is_signed=false; a.size=AccessSize::W; return true; // LDR
            case 5: a.is_load=true;  a.is_signed=false; a.size=AccessSize::H; return true; // LDRH
            case 6: a.is_load=true;  a.is_signed=false; a.size=AccessSize::B; return true; // LDRB
            case 7: a.is_load=true;  a.is_signed=true;  a.size=AccessSize::H; return true; // LDRSH
        }
    }

    // LDR (literal): 01001 Rt imm8 — sollte nicht in Peripherieregion enden,
    // aber zur Vollständigkeit decoden wir es trotzdem.
    if ((op & 0xF800u) == 0x4800u) {
        uint8_t rt = (op >> 8) & 0x7;
        uint32_t imm = (op & 0xFFu) << 2;
        uint32_t base = (read_reg(f, r4_r11, sp_main, 15)) & ~3u;
        a.is_load = true; a.is_signed = false;
        a.size = AccessSize::W; a.rt = rt;
        a.address = base + imm;
        return true;
    }

    return false;
}

} // namespace

bool decode_mem_access(const StackedFrame* frame,
                       uint32_t r4_r11[8],
                       uint32_t sp_main,
                       Access& out) {
    uint16_t op = *reinterpret_cast<const uint16_t*>(frame->pc);
    return decode16(op, frame, r4_r11, sp_main, out);
}

uint32_t* reg_ptr(StackedFrame* frame, uint32_t r4_r11[8],
                  uint32_t* sp_main_ptr, uint8_t r) {
    switch (r) {
        case 0:  return &frame->r0;
        case 1:  return &frame->r1;
        case 2:  return &frame->r2;
        case 3:  return &frame->r3;
        case 4:  return &r4_r11[0];
        case 5:  return &r4_r11[1];
        case 6:  return &r4_r11[2];
        case 7:  return &r4_r11[3];
        case 8:  return &r4_r11[4];
        case 9:  return &r4_r11[5];
        case 10: return &r4_r11[6];
        case 11: return &r4_r11[7];
        case 12: return &frame->r12;
        case 13: return sp_main_ptr;
        case 14: return &frame->lr;
        default: return nullptr; // PC nicht beschreibbar im Trap-Kontext
    }
}

} // namespace trap_decoder
