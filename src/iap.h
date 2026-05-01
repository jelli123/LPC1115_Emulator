#pragma once
//
// LPC1115 IAP-ROM-Emulation.
// Die echte LPC-ROM stellt eine Funktion bei 0x1FFF1FF1 bereit, die u. a.
// Selfbus' Eeprom::commit() für Parameter-Persistenz nutzt. Da diese
// Adresse auf RP2350 nicht existiert, fängt fault.cpp:handle_memfault_c
// den Prefetch-Fault ab und ruft dispatch().
//
//   void iap_entry(unsigned param[5], unsigned result[5]);
//      param[0] = command, param[1..4] = args
//      result[0] = status, result[1..4] = result words
//
// Quelle: NXP UM10398 §20.

#include <cstdint>

namespace iap {

constexpr uint32_t ROM_ENTRY = 0x1FFF'1FF1;     // thumb-Bit gesetzt
constexpr uint32_t ROM_ENTRY_TARGET = 0x1FFF'1FF0;

constexpr uint32_t CMD_SUCCESS                = 0;
constexpr uint32_t CMD_INVALID_COMMAND        = 1;
constexpr uint32_t CMD_SRC_ADDR_ERROR         = 2;
constexpr uint32_t CMD_DST_ADDR_ERROR         = 3;
constexpr uint32_t CMD_SRC_ADDR_NOT_MAPPED    = 4;
constexpr uint32_t CMD_DST_ADDR_NOT_MAPPED    = 5;
constexpr uint32_t CMD_COUNT_ERROR            = 6;
constexpr uint32_t CMD_INVALID_SECTOR         = 7;
constexpr uint32_t CMD_SECTOR_NOT_BLANK       = 8;
constexpr uint32_t CMD_SECTOR_NOT_PREPARED    = 9;
constexpr uint32_t CMD_COMPARE_ERROR          = 10;

void init();
void dispatch(uint32_t* param, uint32_t* result);

struct Stats {
    uint32_t calls;
    uint32_t writes;
    uint32_t erases;
    uint32_t errors;
};
Stats stats();

} // namespace iap
