#pragma once
//
// Status-LED auf der Onboard-LED des Pi Pico 2 (PICO_DEFAULT_LED_PIN = 25).
//
//   Idle/Bereit  → 1 Hz Heartbeat (50 % duty, 500 ms an / 500 ms aus)
//   Running      → dauerhaft an
//   Halted       → kurzer Doppelblitz alle 2 s
//   Faulted      → schnelles Flackern (8 Hz)
//
// Aufruf-Stelle: einmal led::init(), dann led::poll() im Hauptloop. Alle
// Zustände werden anhand emulator::state() bestimmt.

#include <cstdint>

namespace led {

void init();
void poll();

// Diagnose: seit debug_reset() akkumulierte Sampling-Zaehler aus poll()
// (State-Verteilung + PIO0.7-Pegel). Zeigt, ob der Gast-Blink-Pin sauber
// toggelt oder haengt. In 'stats' ausgegeben.
void debug_counters(uint32_t& polls, uint32_t& run, uint32_t& idle,
                    uint32_t& halt, uint32_t& fault,
                    uint32_t& p07_high, uint32_t& p07_low, uint32_t& p07_noout);
void debug_reset();

} // namespace led
