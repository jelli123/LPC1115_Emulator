#include "hex_patcher.h"

#include <cstring>

namespace hex_patcher {

Result relocate_ram_refs(uint8_t* image, std::size_t size_bytes,
                         uint32_t old_ram_base, uint32_t old_ram_size,
                         uint32_t new_ram_base) {
    Result r{0, 0};
    if (image == nullptr || size_bytes < 4) return r;

    const std::size_t end = size_bytes & ~static_cast<std::size_t>(3);
    const uint32_t    old_end = old_ram_base + old_ram_size;

    for (std::size_t off = 0; off + 4 <= end; off += 4) {
        uint32_t w;
        std::memcpy(&w, image + off, 4);
        ++r.scanned_words;

        // Den ersten Vector-Tabellen-Eintrag (Initial SP, Offset 0) lassen
        // wir aus, der wird vom Loader (emulator.cpp) gesondert behandelt.
        if (off == 0) continue;

        // Bereich [old_ram_base, old_end] INKLUSIVE der oberen Grenze:
        // Startup-Code (Scatter-Load/CRT) legt regelmaessig "one-past-end"-
        // Zeiger ab, z. B. _estack/_ebss/_edata == 0x10002000 (Top des 8-KB-
        // SRAM). Diese liegen exakt auf old_end; wuerden sie nicht reloziert,
        // faultet bereits die RAM-Initialisierung. Der gemappte Wert
        // new_ram_base + old_ram_size zeigt konsistent auf das Ende des
        // Gast-RAM-Puffers (vgl. Initial-SP-Behandlung in emulator.cpp).
        if (w >= old_ram_base && w <= old_end) {
            uint32_t mapped = new_ram_base + (w - old_ram_base);
            std::memcpy(image + off, &mapped, 4);
            ++r.patched_words;
        }
    }
    return r;
}

uint32_t patch_wfi_to_svc(uint8_t* image, std::size_t size_bytes,
                          std::size_t skip_bytes) {
    if (image == nullptr || size_bytes < 2) return 0;
    constexpr uint16_t WFI_OPC = 0xBF30;   // Thumb: WFI
    constexpr uint16_t SVC_OPC = 0xDF00;   // Thumb: SVC #0
    std::size_t start = skip_bytes & ~static_cast<std::size_t>(1);
    const std::size_t end = size_bytes & ~static_cast<std::size_t>(1);
    uint32_t count = 0;
    for (std::size_t off = start; off + 2 <= end; off += 2) {
        uint16_t h;
        std::memcpy(&h, image + off, 2);
        if (h == WFI_OPC) {
            std::memcpy(image + off, &SVC_OPC, 2);
            ++count;
        }
    }
    return count;
}

uint32_t patch_cps_to_svc(uint8_t* image, std::size_t size_bytes,
                          std::size_t skip_bytes) {
    if (image == nullptr || size_bytes < 2) return 0;
    constexpr uint16_t CPSID_I = 0xB672;   // Thumb: CPSID i (__disable_irq)
    constexpr uint16_t CPSIE_I = 0xB662;   // Thumb: CPSIE i (__enable_irq)
    constexpr uint16_t SVC_SET = 0xDF01;   // SVC #1 -> Schatten-PRIMASK setzen
    constexpr uint16_t SVC_CLR = 0xDF02;   // SVC #2 -> Schatten-PRIMASK loeschen
    std::size_t start = skip_bytes & ~static_cast<std::size_t>(1);
    const std::size_t end = size_bytes & ~static_cast<std::size_t>(1);
    uint32_t count = 0;
    for (std::size_t off = start; off + 2 <= end; off += 2) {
        uint16_t h;
        std::memcpy(&h, image + off, 2);
        if (h == CPSID_I) {
            std::memcpy(image + off, &SVC_SET, 2);
            ++count;
        } else if (h == CPSIE_I) {
            std::memcpy(image + off, &SVC_CLR, 2);
            ++count;
        }
    }
    return count;
}

} // namespace hex_patcher
