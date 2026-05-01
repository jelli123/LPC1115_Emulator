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

        if (w >= old_ram_base && w < old_end) {
            uint32_t mapped = new_ram_base + (w - old_ram_base);
            std::memcpy(image + off, &mapped, 4);
            ++r.patched_words;
        }
    }
    return r;
}

} // namespace hex_patcher
