#include "cli.h"

#include "config.h"
#include "storage.h"
#include "hex_parser.h"
#include "emulator.h"
#include "peripherals.h"
#include "gdb_stub.h"
#include "swd_target.h"
#include "usb_msc.h"
#include "led.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "pico/stdlib.h"

namespace {

constexpr std::size_t LINE_MAX = 192;

// Stateful Hex-Upload-Modus
bool g_in_hex_upload = false;
hex::Parser* g_hex_parser = nullptr;

bool hex_writer(uint32_t offset, const uint8_t* data, std::size_t len) {
    return storage::firmware_write(offset, data, len);
}

void cmd_help() {
    static const char* lines[] = {
        "Befehle:",
        "  help                       diese Hilfe",
        "  status                     Emulatorstatus & Stats",
        "  config get <key>           Konfigwert lesen",
        "  config set <key> <value>   Konfigwert schreiben",
        "  config save                Konfig persistieren",
        "  config dump                alle Konfig-KVs ausgeben",
        "  pin set <lpc> <rp2350|-1>  Pin-Mapping setzen",
        "  pin show                   Pin-Mapping anzeigen",
        "  freq <Hz>                  Ziel-CPU-Frequenz",
        "  flash erase                Firmware-Slot löschen",
        "  flash hex                  Intel-Hex-Stream starten (mit ':' Zeilen, Ende = leere Zeile)",
        "  flash finalize <bytes>     Firmware abschließen + CRC-Marker",
        "  run                        Emulation starten",
        "  stop                       Emulation anhalten (Core1 reset)",
        "  reset                      Wie stop",
        "  gdb on|off|status          GDB Remote Stub (zweite USB-CDC)",
        "  swd start <clk> <dio>      SWD-Target auf RP-GPIOs aktivieren (Skelett)",
        "  swd stop                   SWD-Target deaktivieren",
        nullptr
    };
    for (int i = 0; lines[i]; ++i) std::puts(lines[i]);
}

void cmd_status() {
    auto s = peripherals::stats();
    static const char* names[] = {"Idle","Running","Halted","Faulted"};
    std::printf("State=%s PC=0x%08lx mmio-traps=%llu\n",
                names[static_cast<int>(emulator::state())],
                static_cast<unsigned long>(emulator::pc()),
                static_cast<unsigned long long>(emulator::mem_traps()));
    std::printf("MMIO R=%llu W=%llu  GPIO=%llu  PLL-cfg=%llu  NVIC-W=%llu\n",
                static_cast<unsigned long long>(s.mmio_reads),
                static_cast<unsigned long long>(s.mmio_writes),
                static_cast<unsigned long long>(s.gpio_writes),
                static_cast<unsigned long long>(s.pll_reconfigs),
                static_cast<unsigned long long>(s.nvic_writes));
    std::printf("CPU-target=%lu Hz  GDB=%s\n",
                static_cast<unsigned long>(peripherals::current_cpu_hz()),
                gdb_stub::active() ? "on" : "off");
}

bool parse_int(const char* s, long min_v, long max_v, long& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    long v = std::strtol(s, &end, 0);
    if (end == s || *end != '\0') return false;
    if (v < min_v || v > max_v) return false;
    out = v;
    return true;
}

void emit_line(const char* l) { std::puts(l); }

void handle_command(char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (!*line) return;

    char* tokens[6] = {};
    int   n = 0;
    char* p = line;
    while (*p && n < 6) {
        tokens[n++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) { *p = '\0'; ++p; while (*p == ' ' || *p == '\t') ++p; }
    }

    if (n == 0) return;

    if (std::strcmp(tokens[0], "help") == 0)   { cmd_help(); return; }
    if (std::strcmp(tokens[0], "status") == 0) { cmd_status(); return; }

    if (std::strcmp(tokens[0], "config") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "get") == 0 && n >= 3) {
            char buf[96];
            if (storage::config_get(tokens[2], buf, sizeof buf))
                std::printf("%s=%s\n", tokens[2], buf);
            else std::puts("(unset)");
            return;
        }
        if (std::strcmp(tokens[1], "set") == 0 && n >= 4) {
            std::puts(storage::config_set(tokens[2], tokens[3])
                      ? "ok" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "save") == 0) {
            std::puts(config::save() ? "saved" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "dump") == 0) {
            storage::config_dump(emit_line);
            return;
        }
    }

    if (std::strcmp(tokens[0], "pin") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "set") == 0 && n == 4) {
            long lpc, rp;
            if (!parse_int(tokens[2], 0, static_cast<long>(config::LPC_PIN_COUNT - 1), lpc) ||
                !parse_int(tokens[3], -1, 47, rp)) {
                std::puts("err: pin set <lpc 0..63> <rp -1..47>"); return;
            }
            std::puts(config::set_pin_map(static_cast<uint8_t>(lpc), static_cast<int>(rp))
                      ? "ok" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "show") == 0) {
            const auto& m = config::pin_map();
            for (std::size_t i = 0; i < config::LPC_PIN_COUNT; ++i) {
                if (m.lpc_to_rp[i] >= 0)
                    std::printf("LPC.P%u -> GP%d\n",
                                static_cast<unsigned>(i), m.lpc_to_rp[i]);
            }
            return;
        }
    }

    if (std::strcmp(tokens[0], "freq") == 0 && n == 2) {
        long hz;
        if (!parse_int(tokens[1], 1, 150'000'000, hz)) { std::puts("err"); return; }
        config::set_target_frequency_hz(static_cast<uint32_t>(hz));
        std::puts("ok (gespeichert in Konfig; eine Soft-PLL gibt es im\n     Native-Modus nicht — RP2350 läuft mit System-Clock)");
        return;
    }

    if (std::strcmp(tokens[0], "flash") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "erase") == 0) {
            std::puts(storage::firmware_erase() ? "erased" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "hex") == 0) {
            static hex::Parser parser(hex_writer, 0x0000'0000, 64u * 1024u);
            parser = hex::Parser(hex_writer, 0x0000'0000, 64u * 1024u);
            g_hex_parser = &parser;
            g_in_hex_upload = true;
            std::puts("hex: stream Intel-Hex Zeilen, Ende mit leerer Zeile");
            return;
        }
        if (std::strcmp(tokens[1], "finalize") == 0 && n == 3) {
            long len;
            if (!parse_int(tokens[2], 1, 64L * 1024, len)) { std::puts("err"); return; }
            std::puts(storage::firmware_finalize(static_cast<std::size_t>(len)) ? "ok" : "err");
            return;
        }
    }

    if (std::strcmp(tokens[0], "run") == 0)   { emulator::load_and_start(); return; }
    if (std::strcmp(tokens[0], "stop") == 0)  { emulator::stop();  return; }
    if (std::strcmp(tokens[0], "reset") == 0) { emulator::stop(); std::puts("reset"); return; }

    if (std::strcmp(tokens[0], "gdb") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "on") == 0)     { gdb_stub::start(); return; }
        if (std::strcmp(tokens[1], "off") == 0)    { gdb_stub::stop();  return; }
        if (std::strcmp(tokens[1], "status") == 0) {
            std::printf("gdb=%s port=CDC#%u\n",
                        gdb_stub::active() ? "on" : "off",
                        gdb_stub::port_index());
            return;
        }
    }

    if (std::strcmp(tokens[0], "swd") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "start") == 0 && n == 4) {
            long c, d;
            if (!parse_int(tokens[2], 0, 47, c) ||
                !parse_int(tokens[3], 0, 47, d)) { std::puts("err"); return; }
            std::puts(swd_target::start({static_cast<int8_t>(c),
                                         static_cast<int8_t>(d)})
                      ? "ok" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "stop") == 0)  { swd_target::stop(); return; }
        if (std::strcmp(tokens[1], "status") == 0) {
            std::printf("swd=%s\n", swd_target::active() ? "active" : "off");
            return;
        }
    }

    std::puts("unknown command — try 'help'");
}

void process_hex_line(const char* line) {
    if (*line == '\0') {
        // Leerzeile beendet Upload-Modus.
        std::printf("hex: %lu bytes empfangen\n",
                    static_cast<unsigned long>(g_hex_parser ? g_hex_parser->bytes_written() : 0));
        g_in_hex_upload = false;
        g_hex_parser = nullptr;
        return;
    }
    if (!g_hex_parser) return;
    for (const char* p = line; *p; ++p) g_hex_parser->feed(*p);
    auto r = g_hex_parser->feed('\n');
    using R = hex::Result;
    if (r == R::BadFormat || r == R::BadChecksum || r == R::OutOfRange ||
        r == R::Overflow) {
        std::printf("hex error: %d\n", static_cast<int>(r));
        g_in_hex_upload = false;
        g_hex_parser = nullptr;
    } else if (r == R::EndOfFile) {
        std::printf("hex eof: %lu bytes\n",
                    static_cast<unsigned long>(g_hex_parser->bytes_written()));
        g_in_hex_upload = false;
        g_hex_parser = nullptr;
    }
}

} // namespace

namespace cli {

void init() {
    std::puts("\nLPC1115 Emulator @ RP2350 — type 'help'");
}

void run() {
    char line[LINE_MAX];
    std::size_t len = 0;
    std::printf("> ");
    while (true) {
        led::poll();
        gdb_stub::poll();
        swd_target::poll();
        if (usb_msc::consume_pending_boot_request()) {
            std::printf("\n[BOOT] BOOT.HEX über USB-MSC erkannt → Start\n");
            emulator::load_and_start();
        }
        int c = getchar_timeout_us(50'000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r') continue;
        if (c == '\n') {
            line[len] = '\0';
            if (g_in_hex_upload) process_hex_line(line);
            else                 handle_command(line);
            len = 0;
            std::printf("> ");
            continue;
        }
        if (c == 0x7F /* DEL */ || c == 0x08 /* BS */) {
            if (len) { --len; std::printf("\b \b"); }
            continue;
        }
        if (len + 1 >= sizeof line) {
            // Schutz gegen Overlong-Lines
            len = 0;
            std::puts("\nerr: line too long");
            std::printf("> ");
            continue;
        }
        if (c >= 32 && c < 127) {
            line[len++] = static_cast<char>(c);
            std::putchar(c);
        }
    }
}

} // namespace cli