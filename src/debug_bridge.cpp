#include "debug_bridge.h"

#include <atomic>
#include <cstring>

namespace debug_bridge {

namespace {

// Rollierender Ringpuffer der letzten CAP Bytes. SPSC: einziger Produzent ist
// der Gast (Core1, ueber den MMIO-Trap in peripherals::mmio_write8), einziger
// Konsument ist Core0 (CLI 'dbg' / USB-MSC DEBUG.TXT). Damit genuegt ein
// monoton steigender head-Zaehler ohne Lock.
constexpr uint32_t CAP = 4096;

char                  g_buf[CAP];
std::atomic<uint32_t> g_head{0};   // Gesamtzahl je geschriebener Bytes (monoton)

} // namespace

void init() {
    g_head.store(0, std::memory_order_relaxed);
    g_buf[0] = '\0';
}

void put_byte(uint8_t b) {
    // Nur der Gast-Core schreibt -> keine Konkurrenz auf head/buf. head zeigt
    // auf die naechste freie Position (modulo CAP).
    uint32_t h = g_head.load(std::memory_order_relaxed);
    g_buf[h % CAP] = static_cast<char>(b);
    g_head.store(h + 1u, std::memory_order_release);
}

uint32_t snapshot(char* out, uint32_t cap) {
    if (!out || cap == 0) return 0;
    uint32_t h = g_head.load(std::memory_order_acquire);
    uint32_t count = (h < CAP) ? h : CAP;           // im Ring verfuegbare Bytes
    if (count > cap - 1u) count = cap - 1u;          // Platz fuer NUL lassen
    // Aeltestes Byte: bei Ueberlauf steht es an (h % CAP), sonst bei 0.
    uint32_t start = (h < CAP) ? 0u : (h % CAP);
    for (uint32_t i = 0; i < count; ++i)
        out[i] = g_buf[(start + i) % CAP];
    out[count] = '\0';
    return count;
}

uint32_t total_bytes() {
    return g_head.load(std::memory_order_relaxed);
}

void clear() {
    g_head.store(0, std::memory_order_release);
    g_buf[0] = '\0';
}

} // namespace debug_bridge
