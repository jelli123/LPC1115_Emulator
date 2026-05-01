#pragma once
//
// Heuristische CPU-Frequenz-Erkennung aus dem geladenen Firmware-Image.
// Das ist eine Hilfe, KEIN Ersatz für eine echte PLL-Konfig-Auswertung.
// Wir suchen nach Schreibsequenzen auf SYSPLLCTRL/MAINCLKSEL/SYSAHBCLKDIV
// und werten die *unmittelbar vorher in einem Register liegenden Werte*
// statisch nicht aus — daher wird im Zweifel der Default zurückgegeben.
//

#include <cstdint>
#include <cstddef>

enum class OscillatorType { IRC, EXT_CRYSTAL };

struct ClockConfig {
    uint32_t       frequency;
    OscillatorType oscillator;
};

ClockConfig extract_clock_settings(const uint8_t* image, std::size_t image_size);
