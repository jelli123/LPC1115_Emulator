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

namespace led {

void init();
void poll();

} // namespace led
