#pragma once
//
// time_us_64() ohne Funktionsaufruf: Die SDK-Variante liegt im XIP-Flash. Auf
// Core1 (MMIO-Trap, SysTick-Shim) kostet schon ein Cache-Miss dort mehrere
// hundert Takte, weil Core0 (USB, CLI) den Flash-Cache laufend verdraengt.
//

#include <cstdint>
#include "hardware/timer.h"      // timer_hw (Default-Instanz)

static inline __attribute__((always_inline)) uint64_t fast_time_us() {
    uint32_t hi = timer_hw->timerawh;
    uint32_t lo;
    for (;;) {
        lo = timer_hw->timerawl;
        const uint32_t hi2 = timer_hw->timerawh;
        if (hi2 == hi) break;
        hi = hi2;
    }
    return (static_cast<uint64_t>(hi) << 32) | lo;
}
