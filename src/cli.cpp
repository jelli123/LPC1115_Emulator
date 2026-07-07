#include "cli.h"

#include "config.h"
#include "storage.h"
#include "hex_parser.h"
#include "emulator.h"
#include "peripherals.h"
#include "gdb_stub.h"
#include "swd_target.h"
#include "target_halt.h"
#include "pio_glue.h"
#include "usb_msc.h"
#include "uart_bridge.h"
#include "led.h"
#include "xmodem.h"
#include "debug_bridge.h"
#include "irq_inject.h"
#include "usb_descriptors.h"

extern "C" void usb_stdio_task(void);
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "pico/stdlib.h"
#include "pico/version.h"

namespace {

constexpr std::size_t LINE_MAX = 192;

// Emulator-Versionsnummer (Software-Reife). Wird in 'info' und 'version'
// angezeigt. Semantik: 0.0x-alpha = fruehe, aktiv entwickelte Vorabstaende.
constexpr const char* EMU_VERSION = "0.03-alpha";

// Stateful Hex-Upload-Modus
bool g_in_hex_upload = false;
hex::Parser* g_hex_parser = nullptr;

bool hex_writer(uint32_t offset, const uint8_t* data, std::size_t len) {
    return storage::firmware_write(offset, data, len);
}

// Stoppt einen nativ laufenden Gast vor einem CLI-Flash-Schreibzugriff. Noetig,
// weil ein Erase/Program XIP stallt und Core1 sonst beim naechsten Trap (unsere
// Fault-Handler liegen im Flash) crasht. pause_for_flash() resettet Core1 in die
// sichere Spin-Schleife. Nach einem CLI-Reflash startet der Nutzer den Gast mit
// 'run' neu -> kein Auto-Resume.
void stop_guest_for_reflash() {
    if (emulator::pause_for_flash())
        std::puts("[CLI] Gast fuer Flash-Zugriff gestoppt ('run' zum Neustart)");
}

// Wendet ausgewaehlte Schluessel sofort auf die laufende Konfiguration an,
// damit ein direkt folgendes 'run' sie ohne Power-Cycle beruecksichtigt
// (andere Keys greifen erst nach erneutem config::load() beim Boot).
void apply_live_config_key(const char* key, const char* val) {
    if (std::strcmp(key, config::KEY_APP_START) == 0)
        config::set_app_start_addr(
            static_cast<uint32_t>(std::strtoul(val, nullptr, 0)));
    else if (std::strcmp(key, config::KEY_DESC_ADDR) == 0)
        config::set_descriptor_addr(
            static_cast<uint32_t>(std::strtoul(val, nullptr, 0)));
    else if (std::strcmp(key, config::KEY_AUTODESC) == 0)
        config::set_autodesc(val[0] == '1' || std::strcmp(val, "on") == 0);
}

// Persistiert eine soeben geaenderte Pinmap in den Flash-Config-Slot und baut
// die RAM-Disk-CONFIG.INI neu auf. Ohne das (a) ueberlebte die Zuordnung keinen
// Power-Cycle und (b) wuerde ein spaeteres MSC-Event die Aenderung aus der
// veralteten CONFIG.INI wieder zuruecksetzen. FlashPauseGuard pausiert einen
// laufenden Gast fuer den Flash-Schreibzugriff (und setzt ihn danach fort).
void persist_pinmap_change() {
    // Pinmap-Aenderung wirkt SOFORT live: config::set_pin_map() hat g_pin_map
    // bereits aktualisiert, und der GPIO-/Peripherie-Pfad liest config::pin_map()
    // bei jedem Zugriff -> ein laufender Gast nutzt die neue Zuordnung sofort.
    //
    // KEIN sofortiges config::save() hier: das liefe via FlashPauseGuard und
    // wuerde einen laufenden Gast STOPPEN + NEU LADEN (Core1 fuehrt Emulator-
    // Handler aus dem Flash aus + laeuft mit Gast-VTOR -> ein In-place-Freeze ist
    // nicht moeglich, daher der volle Neustart). Dieser Neustart blockiert Core0
    // kurz -> der CLI-CDC-Eingang verliert ein Zeichen am direkt folgenden
    // Kommando (aus 'pinmap set 0_1 -1' wurde 'pinmap set 0_1 -'). Stattdessen:
    // RAM-Disk-CONFIG.INI auf den Live-Zustand bringen (ohne Medienwechsel) und
    // die Flash-Persistenz vormerken. Sie wird beim naechsten gefahrlosen Anlass
    // geschrieben: wenn der Gast steht (usb_msc::poll, kein Neustall noetig),
    // per 'cfg save' (bewusst mit einmaligem Neustart) oder beim CONFIG.INI-Eject.
    usb_msc::refresh_config_volume(/*trigger_host_reread=*/false);
    usb_msc::request_config_persist();
    std::puts("ok (live; dauerhaft per 'cfg save' oder beim Gast-Stop)");
}

// --- XMODEM-Empfang in den HEX-Parser ---
struct XmodemHexCtx {
    hex::Parser* parser;
    bool         error;
    bool         done;   // EOF-Record erreicht; weiteres Padding ignorieren
};

bool xmodem_hex_sink(const uint8_t* data, std::size_t len, void* ctxv) {
    auto* c = static_cast<XmodemHexCtx*>(ctxv);
    for (std::size_t i = 0; i < len; ++i) {
        if (c->done) return true; // Padding (0x1A) nach EOF verwerfen
        hex::Result r = c->parser->feed(static_cast<char>(data[i]));
        using R = hex::Result;
        if (r == R::BadFormat || r == R::BadChecksum ||
            r == R::OutOfRange || r == R::Overflow) {
            c->error = true;
            return false;
        }
        if (r == R::EndOfFile) c->done = true;
    }
    return true;
}

void cmd_xmodem() {
    stop_guest_for_reflash();   // XIP-Schutz fuer den blockierenden Empfang
    std::puts("xmodem: Empfang (CRC/1K, additiv) bereit - Datei jetzt senden...");
    fflush(stdout);
    static hex::Parser parser(hex_writer, 0x0000'0000, 64u * 1024u);
    parser = hex::Parser(hex_writer, 0x0000'0000, 64u * 1024u);
    XmodemHexCtx ctx{&parser, false, false};
    uint32_t raw = 0;
    auto res = xmodem::receive(xmodem_hex_sink, &ctx, usb_stdio_task, raw);

    if (ctx.error) {
        std::puts("\n[xmodem] HEX-Fehler -> abgebrochen");
        return;
    }
    if (res == xmodem::Result::SyncFailed) {
        std::puts("\n[xmodem] kein Sender erkannt (timeout)");
        return;
    }
    if (res == xmodem::Result::Canceled) {
        std::puts("\n[xmodem] abgebrochen");
        return;
    }
    uint32_t bw = parser.bytes_written();
    if (bw > 0 && storage::firmware_finalize(bw)) {
        std::printf("\n[xmodem] %lu hex-bytes (%lu roh empfangen), CRC ok\n",
                    static_cast<unsigned long>(bw),
                    static_cast<unsigned long>(raw));
    } else {
        std::puts("\n[xmodem] keine gueltigen Daten");
    }
}

// Hilfe-Zeilen (file-scope, damit 'help' UND 'help <cmd>' darauf zugreifen).
// Eine Befehlszeile beginnt nach dem Einzug mit dem kanonischen Befehlsnamen;
// 'help <cmd>' filtert genau diese Zeilen heraus.
const char* const HELP_LINES[] = {
    "Befehle (abkuerzbar auf eindeutiges Praefix, z.B. 'cf sh' = 'cfg show'):",
    "  help [cmd]                 diese Hilfe / Hilfe zu einem Befehl ('?' = help)",
    "  version                    Build-Info",
    "  stats                      Emulatorstatus & Zaehler",
    "  dbg [clear]                Gast-Debug-Ausgabe zeigen/loeschen",
    "  dbg save                   Debug-Ausgabe als DEBUG.TXT aufs Laufwerk",
    "  dbg auto <sek|off>         DEBUG.TXT automatisch alle N s aktualisieren",
    "  reset                      Emulator-Core neu starten",
    "",
    "  upload                     Intel-Hex-Stream starten (alias: flash hex)",
    "  xmodem                     Intel-Hex per XMODEM-CRC/1K empfangen",
    "  info                       Reset-Vektor, Stack, Groesse, CRC",
    "  erase                      Firmware-Slot loeschen (alias: flash erase)",
    "  run                        Guest starten",
    "  halt                       Guest anhalten",
    "  step                       ein Befehl, dann halten",
    "  autostart on|off           nach Reset automatisch starten",
    "",
    "  cfg show                   alle KV-Paare ausgeben",
    "  cfg get <key>              Wert lesen",
    "  cfg set <key> <value>      Wert setzen",
    "  cfg save                   RAM-Snapshot in Flash schreiben",
    "  cfg clear                  alle Einstellungen auf Default zuruecksetzen",
    "  pinmap show                Tabelle LPC-Pin -> RP2350-GPIO",
    "  pinmap set <port_pin> <rp> Pin zuweisen (z.B. pinmap set 1_8 17)",
    "  pinmap reset               Default-Tabelle wiederherstellen",
    "",
    "  gdb on|off|status          GDB-Stub auf der GDB-CDC",
    "  bp <addr>                  SW-Breakpoint setzen",
    "  bp clr <addr>              Breakpoint loeschen",
    "  regs                       Register anzeigen (r0-r15, xPSR)",
    "  mem <addr> <len>           Hex-Dump Guest-Adressraum",
    "",
    "  swd start <swdio> <swclk>  SWD-Target aktivieren",
    "  swd stop                   SWD-Target deaktivieren",
    "  pio capture <pin> <count>  Edge-Capture-Trace (Zyklen)",
    "",
    "  cdc start <tx> <rx>        USB-Serial-Konverter (Serial-CDC <-> PIO-UART)",
    "  cdc stop                   CDC-Serial-Konverter stoppen",
    "  cdc status                 Pins, Baudrate, Datenfluss",
    "  uart pins <tx> <rx>|off    LPC-UART0 auf RP-UART-Pads routen",
    "  uart cdc on|off            LPC-UART0 virtuell an Serial-CDC koppeln",
    "  uart status                LPC-UART0-Routing anzeigen",
    "",
    "  i2c on <inst> <sda> <scl> [hz]  I2C-Bridge auf RP2350-HW (Neustart noetig)",
    "  i2c off                    I2C-Bridge deaktivieren",
    "  i2c status                 Instanz/Pins/Takt anzeigen",
    "",
    "  freq <Hz>                  Ziel-CPU-Frequenz",
    "  flash hex                  Intel-Hex-Stream (alias fuer upload)",
    "  flash erase                Firmware-Slot loeschen",
    "  flash finalize <bytes>     Firmware abschliessen + CRC-Marker",
    "",
    "Komfort: TAB vervollstaendigt, Pfeil-hoch/runter = History (nur im Terminal).",
    "Aliase: show=list=dump, stats=status, run=start, halt=stop, cfg=config, pinmap=pin.",
    nullptr
};

void cmd_help() {
    for (int i = 0; HELP_LINES[i]; ++i) std::puts(HELP_LINES[i]);
}

void cmd_status() {
    auto s = peripherals::stats();
    static const char* names[] = {"Idle","Running","Halted","Faulted"};
    std::printf("State=%s PC=0x%08lx mmio-traps=%llu\n",
                names[static_cast<int>(emulator::state())],
                static_cast<unsigned long>(emulator::pc()),
                static_cast<unsigned long long>(emulator::mem_traps()));
    {
        // Geladene Firmware-Datei (Langname der zuletzt via MSC geflashten HEX;
        // "(unbekannt)" = Autostart aus persistiertem Flash-Slot ohne Dateiname).
        const char* hexn = usb_msc::loaded_hex_name();
        std::printf("Firmware: %s\n", (hexn && hexn[0]) ? hexn : "(unbekannt)");
    }
    std::printf("Gast-Starts=%lu\n",
                static_cast<unsigned long>(emulator::start_count()));
    // Gast-PC-Samples (SysTick-Shim). Zeigt, wo ein "Running, mmio-traps
    // eingefroren"-Gast in einer reinen CPU-Schleife dreht (LPC-Offset).
    {
        uint32_t pcs[8];
        uint32_t n = emulator::pc_samples(pcs, 8);
        if (n) {
            std::printf("Gast-PC-Samples (LPC-Offset, neueste zuerst):");
            for (uint32_t i = 0; i < n; ++i)
                std::printf(" 0x%05lx", static_cast<unsigned long>(pcs[i]));
            std::putchar('\n');
        }
    }
    {
        uint32_t sh_enter, sh_exit, sh_age;
        emulator::shim_debug(sh_enter, sh_exit, sh_age);
        std::printf("SysTick-Shim: enter=%lu exit=%lu age=%lums%s\n",
                    static_cast<unsigned long>(sh_enter),
                    static_cast<unsigned long>(sh_exit),
                    static_cast<unsigned long>(sh_age),
                    (sh_enter != sh_exit) ? "  [HAENGT IM SHIM!]"
                    : (sh_age > 100u)     ? "  [Shim feuert nicht mehr!]" : "");
    }
    {
        uint32_t ct_ug; uint64_t ct_mt;
        peripherals::ct_advance_debug(ct_ug, ct_mt);
        std::printf("CT-Advance: underflow-guards=%lu max-ticks=%llu\n",
                    static_cast<unsigned long>(ct_ug),
                    static_cast<unsigned long long>(ct_mt));
    }
    {
        // Groesste IRQ-Injektions-Verschachtelung. 1 = keine Reentrancy (gesund);
        // >1 = ein injizierter Handler wurde rekursiv preemptet (frueherer
        // UART0-RX-Storm). Nach dem Reentrancy-Gate sollte der Wert 1 bleiben.
        std::printf("IRQ-Inject: max-depth=%lu live=%lu\n",
                    static_cast<unsigned long>(irq_inject::inject_depth_max()),
                    static_cast<unsigned long>(irq_inject::inject_depth_live()));
    }
    {
        // CT-Timer-Diagnose: pro Timer Konfiguration + Match-IRQ-Pend-Zaehler.
        // Ein Timer mit sehr hohem 'pends' bei kurzer Periode (kleiner MR0, pre=0)
        // = IRQ-Sturm -> Gast-Thread-Starvation. Nur belegte Timer zeigen.
        static const char* ctname[4] = { "CT16B0", "CT16B1", "CT32B0", "CT32B1" };
        for (int i = 0; i < 4; ++i) {
            bool en; uint32_t pre, mr0, mcr, tc, ir, pends, mr1, mr2, mr3;
            peripherals::ct_debug(i, en, pre, mr0, mcr, tc, ir, pends, mr1, mr2, mr3);
            if (!en && pends == 0u) continue;
            std::printf("%s: %s pre=%lu MR0=0x%lx MR1=0x%lx MR2=0x%lx MR3=0x%lx MCR=0x%lx TC=0x%lx IR=0x%lx pends=%lu\n",
                        ctname[i], en ? "an" : "aus",
                        static_cast<unsigned long>(pre),
                        static_cast<unsigned long>(mr0),
                        static_cast<unsigned long>(mr1),
                        static_cast<unsigned long>(mr2),
                        static_cast<unsigned long>(mr3),
                        static_cast<unsigned long>(mcr),
                        static_cast<unsigned long>(tc),
                        static_cast<unsigned long>(ir),
                        static_cast<unsigned long>(pends));
        }
    }
    {
        uint32_t ui_enter, ui_exit;
        peripherals::uart0_init_debug(ui_enter, ui_exit);
        std::printf("UART-init: enter=%lu exit=%lu%s\n",
                    static_cast<unsigned long>(ui_enter),
                    static_cast<unsigned long>(ui_exit),
                    (ui_enter != ui_exit) ? "  [HAENGT in uart_init!]" : "");
    }
    {
        // Letzter getrappter MMIO-Zugriff. Bei eingefrorenem mmio-traps + stehendem
        // SysTick-Shim zeigt diese Adresse das Register, an dem Core1 mitten im
        // Trap haengt (Handler-Mode, hoehere Prio als SysTick).
        uint32_t mmio_addr; bool mmio_wr;
        peripherals::last_mmio(mmio_addr, mmio_wr);
        std::printf("Letzter MMIO: 0x%08lx (%s)\n",
                    static_cast<unsigned long>(mmio_addr),
                    mmio_wr ? "Write" : "Read");
    }
    {
        // LPC-UART0-Aktivitaet (immer sichtbar, HW- wie CDC-Modus). rbr_reads
        // steigt = Gast liest echte Empfangsbytes; rx-live = LSR.DR gerade gesetzt.
        uint8_t u_ier; bool u_nvic;
        uint32_t u_pends, u_rbr, u_tx;
        peripherals::uart0_debug(u_ier, u_nvic, u_pends, u_rbr, u_tx);
        std::printf("UART0: IER=0x%02x NVIC=%s RX-live=%s | RX-IRQ-pends=%lu RBR-reads=%lu THR-writes=%lu\n",
                    u_ier, u_nvic ? "an" : "aus",
                    peripherals::uart0_rx_live() ? "JA" : "nein",
                    static_cast<unsigned long>(u_pends),
                    static_cast<unsigned long>(u_rbr),
                    static_cast<unsigned long>(u_tx));
    }
    {
        // REALER Pad-Zustand der HW-uart0-Bruecke (misst GP0/GP1 elektrisch).
        // txf/rxf=2 -> GPIO_FUNC_UART korrekt geroutet. rxlvl=0 -> RX-Leitung
        // LOW (erklaert Dauer-0x00). cr Bit0/8/9 = UARTEN/TXE/RXE.
        int p_hw, p_tx, p_rx, p_txf, p_rxf, p_rxl; uint32_t p_cr, p_lcrh;
        peripherals::uart0_pad_debug(p_hw, p_tx, p_rx, p_txf, p_rxf, p_rxl, p_cr, p_lcrh);
        const char* hwname = (p_hw == 0) ? "uart0" : (p_hw == 1) ? "uart1" : "keins";
        std::printf("UART0-PAD: hw=%s TX=GP%d(f%d) RX=GP%d(f%d) RX-Pegel=%s cr=0x%03lx lcr_h=0x%02lx\n",
                    hwname, p_tx, p_txf, p_rx, p_rxf,
                    (p_rxl < 0) ? "n/a" : (p_rxl ? "HIGH" : "LOW"),
                    static_cast<unsigned long>(p_cr),
                    static_cast<unsigned long>(p_lcrh));
    }
    std::printf("MMIO R=%llu W=%llu  GPIO=%llu  PLL-cfg=%llu  NVIC-W=%llu\n",
                static_cast<unsigned long long>(s.mmio_reads),
                static_cast<unsigned long long>(s.mmio_writes),
                static_cast<unsigned long long>(s.gpio_writes),
                static_cast<unsigned long long>(s.pll_reconfigs),
                static_cast<unsigned long long>(s.nvic_writes));
    std::printf("CPU-target=%lu Hz  GDB=%s\n",
                static_cast<unsigned long>(peripherals::current_cpu_hz()),
                gdb_stub::active() ? "on" : "off");
    // USB-CDC-Zuordnung (dynamisch, haengt von cli/gdb/serial_enable ab; MSC ist
    // immer vorhanden). Zeigt, welcher COM-Port am Host welche Rolle hat.
    {
        const int c_cli = usb_desc_cdc_cli();
        const int c_gdb = usb_desc_cdc_gdb();
        const int c_ser = usb_desc_cdc_serial();
        char line[128]; int p = 0;
        p += std::snprintf(line + p, sizeof line - p, "USB-CDC:");
        for (int i = 0; i < usb_desc_cdc_count(); ++i) {
            const char* nm = (i == c_cli) ? "CLI"
                           : (i == c_gdb) ? "GDB"
                           : (i == c_ser) ? "Serial-Adapter" : "?";
            p += std::snprintf(line + p, sizeof line - p, " CDC#%d=%s", i, nm);
        }
        if (usb_desc_cdc_count() == 0)
            p += std::snprintf(line + p, sizeof line - p, " (keine)");
        // deaktivierte Rollen benennen
        char off[48]; int op = 0;
        if (c_cli < 0) op += std::snprintf(off + op, sizeof off - op, " CLI");
        if (c_gdb < 0) op += std::snprintf(off + op, sizeof off - op, " GDB");
        if (c_ser < 0) op += std::snprintf(off + op, sizeof off - op, " Serial");
        if (op) std::snprintf(line + p, sizeof line - p, " | aus:%s", off);
        std::puts(line);
    }
    // SysTick-Diagnose (klaert, ob der Gast-SysTick getrappt + korrekt gesetzt wird).
    uint32_t st_r, st_w, st_csr, st_rvr, st_cvr;
    uint64_t st_ticks;
    peripherals::systick_debug(st_r, st_w, st_csr, st_rvr, st_cvr, st_ticks);
    std::printf("SysTick: trapR=%lu trapW=%lu CSR=0x%lx RVR=0x%lx CVR=0x%lx ticks=%llu\n",
                static_cast<unsigned long>(st_r), static_cast<unsigned long>(st_w),
                static_cast<unsigned long>(st_csr), static_cast<unsigned long>(st_rvr),
                static_cast<unsigned long>(st_cvr),
                static_cast<unsigned long long>(st_ticks));
    // Config-Persistenz (Bug-Diagnose): seq/keys + ob beim Boot aus Flash geladen.
    const auto ci = storage::config_persist_info();
    std::printf("Config: Flash=%luKiB seq=%lu keys=%lu %s\n",
                static_cast<unsigned long>(ci.flash_size_kib),
                static_cast<unsigned long>(ci.sequence),
                static_cast<unsigned long>(ci.key_count),
                ci.loaded_valid ? "(persistent)" : "(nur RAM/Defaults)");
    // PIO-Ressourcen (State-Machines + Instruktions-Slots) ueber alle Bloecke.
    // Zeigt, wieviel die CDC-Bridge / Timer-Capture-/Match-PIO belegen.
    uint32_t sm_u, sm_t, in_u, in_t;
    pio_glue::usage(sm_u, sm_t, in_u, in_t);
    std::printf("PIO used: SM %lu, Instr %lu | free: SM %lu, Instr %lu\n",
                static_cast<unsigned long>(sm_u), static_cast<unsigned long>(in_u),
                static_cast<unsigned long>(sm_t - sm_u),
                static_cast<unsigned long>(in_t - in_u));
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

void cmd_version() {
    std::printf("LPC1115-Emu  v%s  RP2350  SDK=%s\n",
                EMU_VERSION, PICO_SDK_VERSION_STRING);
}

void cmd_info() {
    std::printf("LPC1115-Emulator v%s\n", EMU_VERSION);
    const char* hexn = usb_msc::loaded_hex_name();
    std::printf("Firmware-Datei: %s\n", (hexn && hexn[0]) ? hexn : "(unbekannt)");
    std::size_t sz = storage::firmware_size();
    if (sz == 0) { std::puts("(kein Firmware-Image)"); return; }
    const uint8_t* fw = storage::firmware_data();
    uint32_t sp   = *reinterpret_cast<const uint32_t*>(fw);
    uint32_t rst  = *reinterpret_cast<const uint32_t*>(fw + 4);
    std::printf("Reset: 0x%08lX  Stack: 0x%08lX  Size: %lu\n",
                static_cast<unsigned long>(rst),
                static_cast<unsigned long>(sp),
                static_cast<unsigned long>(sz));
}

// Port_Pin-String (z.B. "1_8") in LPC-Pin-Index konvertieren.
// Bewusst OHNE sscanf: sscanf ist auf newlib-nano extrem stack-hungrig
// (haeufig 500-1000+ Bytes) und sprengt im tiefen handle_command-Aufrufkontext
// den knappen ~2 KB Core0-Stack -> stiller Hang ("CLI wartet auf Eingaben").
// Manuelles strtol-Parsing ist stack-arm und reentrant-sicher.
bool parse_port_pin(const char* s, long& out) {
    char* end = nullptr;
    long port = std::strtol(s, &end, 10);
    if (end != s && *end == '_') {
        const char* p2 = end + 1;
        char* end2 = nullptr;
        long pin = std::strtol(p2, &end2, 10);
        if (end2 != p2 && *end2 == '\0' && port >= 0 && pin >= 0 && pin < 12) {
            out = port * 12 + pin;
            return out >= 0 && out < static_cast<long>(config::LPC_PIN_COUNT);
        }
        return false;
    }
    // Fallback: reiner numerischer Index (z.B. "24").
    return parse_int(s, 0, static_cast<long>(config::LPC_PIN_COUNT - 1), out);
}

void emit_line(const char* l) { std::puts(l); }

// ===========================================================================
// CLI-Komfort: Praefix-Aufloesung, Alias-Kanonisierung, Vorschlaege.
//
// - Befehle UND Optionen duerfen auf ihr kuerzestes EINDEUTIGES Praefix
//   abgekuerzt werden ('cf sh' -> 'cfg show').
// - Aliase (list/dump=show, status=stats, config=cfg, pin=pinmap, ...) werden
//   auf ihren kanonischen Namen abgebildet; die Hilfe zeigt nur den kanonischen.
// - Unbekannter/mehrdeutiger Befehl -> passende Kandidaten in EINER Zeile.
// - Unbekannte/mehrdeutige Option -> moegliche Optionen des Befehls in EINER Zeile.
//
// Umsetzung: Vor der eigentlichen Dispatch-Kette werden tokens[0] (Befehl) und
// ggf. tokens[1] (Option) auf ihre kanonische Form umgeschrieben. Die bestehende
// strcmp-Kette bleibt dadurch unveraendert nutzbar.
// ===========================================================================
struct Word { const char* w; const char* canon; };   // w=Eingabewort, canon=kanonisch

// Top-Level-Befehle (inkl. Aliase). w==canon markiert den kanonischen Eintrag.
constexpr Word TOP[] = {
    {"help","help"}, {"?","help"}, {"version","version"},
    {"stats","stats"}, {"status","stats"},
    {"dbg","dbg"}, {"reset","reset"},
    {"upload","upload"}, {"xmodem","xmodem"}, {"info","info"},
    {"erase","erase"},
    {"run","run"}, {"start","run"},
    {"halt","halt"}, {"stop","halt"},
    {"step","step"}, {"autostart","autostart"},
    {"cfg","cfg"}, {"config","cfg"},
    {"pinmap","pinmap"}, {"pin","pinmap"},
    {"freq","freq"}, {"flash","flash"},
    {"gdb","gdb"}, {"bp","bp"}, {"regs","regs"}, {"mem","mem"},
    {"pio","pio"}, {"swd","swd"}, {"cdc","cdc"},
    {"uart","uart"}, {"i2c","i2c"},
};

// Sub-Optionen je Befehl (kanonisch + Aliase).
constexpr Word SUB_DBG[]    = {{"clear","clear"},{"save","save"},{"auto","auto"}};
constexpr Word SUB_CFG[]    = {{"show","show"},{"list","show"},{"dump","show"},
                               {"get","get"},{"set","set"},{"save","save"},{"clear","clear"}};
constexpr Word SUB_PINMAP[] = {{"show","show"},{"list","show"},{"set","set"},{"reset","reset"}};
constexpr Word SUB_GDB[]    = {{"on","on"},{"off","off"},{"status","status"}};
constexpr Word SUB_SWD[]    = {{"start","start"},{"stop","stop"},{"status","status"}};
constexpr Word SUB_CDC[]    = {{"start","start"},{"stop","stop"},{"status","status"}};
constexpr Word SUB_UART[]   = {{"pins","pins"},{"cdc","cdc"},{"status","status"}};
constexpr Word SUB_I2C[]    = {{"on","on"},{"off","off"},{"status","status"}};
constexpr Word SUB_PIO[]    = {{"capture","capture"}};
constexpr Word SUB_FLASH[]  = {{"hex","hex"},{"erase","erase"},{"finalize","finalize"}};
constexpr Word SUB_BP[]     = {{"clr","clr"}};   // blosse Adresse = Breakpoint setzen

template <int N> constexpr int wcount(const Word (&)[N]) { return N; }

// Liefert die Sub-Optionstabelle eines (kanonischen) Befehls, sonst false.
bool get_subtable(const char* cmd, const Word*& tbl, int& cnt) {
    if (!std::strcmp(cmd, "dbg"))    { tbl = SUB_DBG;    cnt = wcount(SUB_DBG);    return true; }
    if (!std::strcmp(cmd, "cfg"))    { tbl = SUB_CFG;    cnt = wcount(SUB_CFG);    return true; }
    if (!std::strcmp(cmd, "pinmap")) { tbl = SUB_PINMAP; cnt = wcount(SUB_PINMAP); return true; }
    if (!std::strcmp(cmd, "gdb"))    { tbl = SUB_GDB;    cnt = wcount(SUB_GDB);    return true; }
    if (!std::strcmp(cmd, "swd"))    { tbl = SUB_SWD;    cnt = wcount(SUB_SWD);    return true; }
    if (!std::strcmp(cmd, "cdc"))    { tbl = SUB_CDC;    cnt = wcount(SUB_CDC);    return true; }
    if (!std::strcmp(cmd, "uart"))   { tbl = SUB_UART;   cnt = wcount(SUB_UART);   return true; }
    if (!std::strcmp(cmd, "i2c"))    { tbl = SUB_I2C;    cnt = wcount(SUB_I2C);    return true; }
    if (!std::strcmp(cmd, "pio"))    { tbl = SUB_PIO;    cnt = wcount(SUB_PIO);    return true; }
    if (!std::strcmp(cmd, "flash"))  { tbl = SUB_FLASH;  cnt = wcount(SUB_FLASH);  return true; }
    if (!std::strcmp(cmd, "bp"))     { tbl = SUB_BP;     cnt = wcount(SUB_BP);     return true; }
    return false;
}

// Loest ein Eingabewort gegen eine Tabelle auf: exakter Treffer ODER eindeutiges
// Praefix. Aliase, die auf DENSELBEN kanonischen Namen zeigen, gelten NICHT als
// mehrdeutig. Rueckgabe: kanonischer Name oder nullptr. 'distinct' = Anzahl
// verschiedener kanonischer Treffer (0=keiner, 1=eindeutig, >1=mehrdeutig).
const char* resolve_word(const char* in, const Word* tbl, int cnt, int& distinct) {
    for (int i = 0; i < cnt; ++i)
        if (!std::strcmp(in, tbl[i].w)) { distinct = 1; return tbl[i].canon; }
    const std::size_t len = std::strlen(in);
    const char* canon = nullptr;
    distinct = 0;
    for (int i = 0; i < cnt; ++i) {
        if (std::strncmp(in, tbl[i].w, len) != 0) continue;
        if (!canon)                              { canon = tbl[i].canon; distinct = 1; }
        else if (std::strcmp(canon, tbl[i].canon) != 0) ++distinct;
    }
    return (distinct == 1) ? canon : nullptr;
}

// Gibt die (deduplizierten) kanonischen Namen einer Tabelle in EINER Zeile aus,
// optional gefiltert auf ein Praefix (nullptr = alle).
void print_matches(const Word* tbl, int cnt, const char* prefix) {
    const char* shown[40]; int ns = 0;
    const std::size_t len = prefix ? std::strlen(prefix) : 0;
    for (int i = 0; i < cnt && ns < 40; ++i) {
        if (prefix && std::strncmp(tbl[i].w, prefix, len) != 0) continue;
        bool dup = false;
        for (int j = 0; j < ns; ++j)
            if (!std::strcmp(shown[j], tbl[i].canon)) { dup = true; break; }
        if (!dup) shown[ns++] = tbl[i].canon;
    }
    for (int i = 0; i < ns; ++i) std::printf("%s%s", i ? " " : "", shown[i]);
    std::putchar('\n');
}

// Kontexthilfe zu einem Befehl: filtert aus HELP_LINES die Zeilen, deren erstes
// Wort (nach dem Einzug) dem kanonischen Befehl entspricht.
void cmd_help_topic(const char* cmd) {
    int d = 0;
    const char* canon = resolve_word(cmd, TOP, wcount(TOP), d);
    if (!canon) canon = cmd;
    const std::size_t cl = std::strlen(canon);
    bool any = false;
    for (int i = 0; HELP_LINES[i]; ++i) {
        const char* p = HELP_LINES[i];
        while (*p == ' ') ++p;
        if (std::strncmp(p, canon, cl) == 0 && (p[cl] == ' ' || p[cl] == '\0')) {
            std::puts(HELP_LINES[i]);
            any = true;
        }
    }
    if (!any) std::printf("keine Hilfe zu '%s' — 'help' fuer alle Befehle\n", cmd);
}

// --- TAB-Completion + History (nur im interaktiven Terminal nutzbar) --------

// Zeichnet die aktuelle Eingabezeile neu (Prompt + Inhalt). Nutzt ANSI
// "\r\033[K" (Zeilenanfang + bis Zeilenende loeschen) — von Terminals wie PuTTY
// unterstuetzt. TAB/Pfeiltasten funktionieren ohnehin nur mit solchen Terminals.
void redraw_line(const char* line, std::size_t len) {
    std::printf("\r\033[Kemu> ");
    for (std::size_t i = 0; i < len; ++i) std::putchar(line[i]);
    std::fflush(stdout);
}

// TAB-Vervollstaendigung auf dem aktuellen Eingabepuffer (immer am Zeilenende).
// Vervollstaendigt das Befehlswort (Feld 0) bzw. die Option (Feld 1) gegen die
// passende Tabelle: eindeutig -> ganz einsetzen (+Space); mehrere -> laengstes
// gemeinsames Praefix einsetzen und (falls kein Fortschritt) Kandidaten zeigen.
void complete_input(char* line, std::size_t& len) {
    const bool trailing = (len > 0 && (line[len-1] == ' ' || line[len-1] == '\t'));
    char cp[LINE_MAX];
    std::memcpy(cp, line, len); cp[len] = '\0';
    char* toks[6]; int nt = 0; char* p = cp;
    while (*p && nt < 6) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        toks[nt++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) *p++ = '\0';
    }

    const Word* tbl = nullptr; int cnt = 0;
    const char* frag = "";
    std::size_t frag_off = len;
    if (nt == 0) {
        tbl = TOP; cnt = wcount(TOP); frag = ""; frag_off = len;
    } else if (nt == 1 && !trailing) {
        tbl = TOP; cnt = wcount(TOP); frag = toks[0];
        frag_off = static_cast<std::size_t>(toks[0] - cp);
    } else {
        int d = 0;
        const char* canon = resolve_word(toks[0], TOP, wcount(TOP), d);
        if (!canon || !get_subtable(canon, tbl, cnt)) return;   // keine Completion
        if (trailing) { frag = ""; frag_off = len; }
        else { frag = toks[nt-1]; frag_off = static_cast<std::size_t>(toks[nt-1] - cp); }
    }

    const std::size_t fl = std::strlen(frag);
    const char* matches[40]; int nm = 0;
    for (int i = 0; i < cnt && nm < 40; ++i) {
        if (std::strncmp(tbl[i].w, frag, fl) != 0) continue;
        bool dup = false;
        for (int j = 0; j < nm; ++j)
            if (!std::strcmp(matches[j], tbl[i].canon)) { dup = true; break; }
        if (!dup) matches[nm++] = tbl[i].canon;
    }
    if (nm == 0) return;

    if (nm == 1) {
        const std::size_t ml = std::strlen(matches[0]);
        if (frag_off + ml + 2 >= LINE_MAX) return;
        std::memcpy(line + frag_off, matches[0], ml);
        len = frag_off + ml;
        line[len++] = ' ';
        line[len] = '\0';
        redraw_line(line, len);
        return;
    }

    // Mehrere: laengstes gemeinsames Praefix bestimmen.
    std::size_t lcp = std::strlen(matches[0]);
    for (int i = 1; i < nm; ++i) {
        std::size_t k = 0;
        while (k < lcp && matches[i][k] == matches[0][k]) ++k;
        lcp = k;
    }
    if (lcp > fl) {
        if (frag_off + lcp >= LINE_MAX) return;
        std::memcpy(line + frag_off, matches[0], lcp);
        len = frag_off + lcp;
        line[len] = '\0';
        redraw_line(line, len);
    } else {
        std::putchar('\n');
        for (int i = 0; i < nm; ++i) std::printf("%s%s", i ? " " : "", matches[i]);
        std::putchar('\n');
        redraw_line(line, len);
    }
}

// --- Kommando-History (Ringpuffer; Pfeil-hoch/runter im Terminal) ----------
constexpr int HIST_N = 8;
char g_hist[HIST_N][LINE_MAX];
int  g_hist_count = 0;   // Anzahl gueltiger Eintraege (<= HIST_N)
int  g_hist_head  = 0;   // naechster Schreibindex (Ring)

void history_push(const char* s) {
    if (!s || !*s) return;
    // Duplikat des zuletzt gespeicherten Kommandos nicht erneut ablegen.
    if (g_hist_count > 0) {
        int last = (g_hist_head - 1 + HIST_N) % HIST_N;
        if (!std::strcmp(g_hist[last], s)) return;
    }
    std::snprintf(g_hist[g_hist_head], LINE_MAX, "%s", s);
    g_hist_head = (g_hist_head + 1) % HIST_N;
    if (g_hist_count < HIST_N) ++g_hist_count;
}

// Liefert den Eintrag 'back' Schritte zurueck (1 = neuester), sonst nullptr.
const char* history_get(int back) {
    if (back < 1 || back > g_hist_count) return nullptr;
    int idx = (g_hist_head - back + HIST_N) % HIST_N;
    return g_hist[idx];
}

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

    // --- Praefix-/Alias-Aufloesung: Befehl (tokens[0]) und ggf. Option
    // (tokens[1]) auf ihre kanonische Form bringen, bevor die Dispatch-Kette
    // laeuft. Ermoeglicht Abkuerzungen ('cf sh' -> 'cfg show') und liefert bei
    // Unbekanntem/Mehrdeutigem hilfreiche Vorschlaege in einer Zeile.
    {
        int distinct = 0;
        const char* cmd = resolve_word(tokens[0], TOP, wcount(TOP), distinct);
        if (!cmd) {
            if (distinct > 1) {
                std::printf("mehrdeutig '%s' — passt auf: ", tokens[0]);
                print_matches(TOP, wcount(TOP), tokens[0]);
            } else {
                std::printf("unbekannt '%s'. Befehle: ", tokens[0]);
                print_matches(TOP, wcount(TOP), nullptr);
            }
            return;
        }
        tokens[0] = const_cast<char*>(cmd);

        const Word* sub = nullptr; int subn = 0;
        if (n >= 2 && get_subtable(cmd, sub, subn)) {
            int sd = 0;
            const char* opt = resolve_word(tokens[1], sub, subn, sd);
            if (opt) {
                tokens[1] = const_cast<char*>(opt);
            } else if (!std::strcmp(cmd, "bp")) {
                // 'bp <addr>': tokens[1] ist eine Adresse, keine Option -> lassen.
            } else {
                if (sd > 1) std::printf("mehrdeutige Option '%s' fuer '%s': ", tokens[1], cmd);
                else        std::printf("unbekannte Option '%s' fuer '%s'. Moeglich: ", tokens[1], cmd);
                print_matches(sub, subn, sd > 1 ? tokens[1] : nullptr);
                return;
            }
        }
    }

    // --- Allgemein ---
    if (std::strcmp(tokens[0], "help") == 0)    {
        if (n >= 2) cmd_help_topic(tokens[1]);
        else        cmd_help();
        return;
    }
    if (std::strcmp(tokens[0], "version") == 0) { cmd_version(); return; }
    if (std::strcmp(tokens[0], "stats") == 0 ||
        std::strcmp(tokens[0], "status") == 0)  { cmd_status(); return; }

    // --- Firmware ---
    if (std::strcmp(tokens[0], "upload") == 0 ||
        (std::strcmp(tokens[0], "flash") == 0 && n >= 2 &&
         std::strcmp(tokens[1], "hex") == 0)) {
        // Kein Auto-Erase: HEX wird additiv in den vorhandenen Slot gemischt,
        // damit z. B. ein zuvor geladener Bootloader erhalten bleibt. Zum
        // vollstaendigen Loeschen 'erase' verwenden.
        stop_guest_for_reflash();   // XIP-Schutz fuer den folgenden Stream
        static hex::Parser parser(hex_writer, 0x0000'0000, 64u * 1024u);
        parser = hex::Parser(hex_writer, 0x0000'0000, 64u * 1024u);
        g_hex_parser = &parser;
        g_in_hex_upload = true;
        std::puts("hex: stream Intel-Hex Zeilen (additiv), Ende mit leerer Zeile");
        return;
    }
    if (std::strcmp(tokens[0], "xmodem") == 0) {
        cmd_xmodem();
        return;
    }
    if (std::strcmp(tokens[0], "info") == 0)  { cmd_info(); return; }
    if (std::strcmp(tokens[0], "dbg") == 0) {
        if (n >= 2 && std::strcmp(tokens[1], "clear") == 0) {
            debug_bridge::clear();
            std::puts("dbg: geleert");
            return;
        }
        if (n >= 2 && std::strcmp(tokens[1], "save") == 0) {
            // Aktuellen Debug-Puffer als DEBUG.TXT aufs MSC-Laufwerk schreiben
            // und den Host zum Neu-Einlesen zwingen (Medienwechsel).
            usb_msc::flush_debug_volume();
            std::printf("dbg: DEBUG.TXT aktualisiert (%lu Bytes) - Laufwerk neu einlesen\n",
                        static_cast<unsigned long>(debug_bridge::total_bytes()));
            return;
        }
        if (n >= 2 && std::strcmp(tokens[1], "auto") == 0) {
            if (n >= 3) {
                if (std::strcmp(tokens[2], "off") == 0) {
                    usb_msc::set_debug_autoflush_ms(0);
                } else {
                    long s;
                    if (!parse_int(tokens[2], 0, 3600, s)) {
                        std::puts("err: dbg auto <sekunden|off>"); return;
                    }
                    usb_msc::set_debug_autoflush_ms(static_cast<uint32_t>(s) * 1000u);
                }
            }
            uint32_t ms = usb_msc::debug_autoflush_ms();
            if (ms == 0) std::puts("dbg auto: aus");
            else std::printf("dbg auto: alle %lu s\n",
                             static_cast<unsigned long>(ms / 1000u));
            return;
        }
        // Gast-Debug-Ausgabe (ueber die Debug-Bridge gesammelt) dumpen.
        static char buf[4096];
        uint32_t nb = debug_bridge::snapshot(buf, sizeof buf);
        std::printf("--- dbg (%lu Bytes, gesamt %lu) ---\n",
                    static_cast<unsigned long>(nb),
                    static_cast<unsigned long>(debug_bridge::total_bytes()));
        if (nb) std::fputs(buf, stdout);
        if (nb == 0 || buf[nb - 1] != '\n') std::putchar('\n');
        std::puts("--- ende ---");
        return;
    }
    if (std::strcmp(tokens[0], "erase") == 0 ||
        (std::strcmp(tokens[0], "flash") == 0 && n >= 2 &&
         std::strcmp(tokens[1], "erase") == 0)) {
        stop_guest_for_reflash();   // XIP-Schutz
        bool ok = storage::firmware_erase();
        if (ok) {
            // Persistierten Firmware-Namen loeschen -> 'info'/'stats' zeigen nach
            // dem Loeschen nicht mehr einen veralteten (nicht mehr geladenen) Namen.
            config::set_firmware_name("");
            config::save();
        }
        std::puts(ok ? "erased" : "err");
        return;
    }
    if (std::strcmp(tokens[0], "run") == 0 ||
        std::strcmp(tokens[0], "start") == 0) {
        // Ist der Gast kooperativ gehaltet ('stop'/'halt'), fortsetzen statt neu
        // zu laden — sonst meldet load_and_start() "bereits gestartet" (State
        // bleibt beim Halt Running). Sonst regulaer laden + starten.
        if (target_halt::is_halted()) {
            target_halt::request_resume();
            std::puts("fortgesetzt");
        } else {
            emulator::load_and_start();
        }
        return;
    }
    if (std::strcmp(tokens[0], "halt") == 0 ||
        std::strcmp(tokens[0], "stop") == 0) {
        if (emulator::state() == emulator::State::Running) {
            target_halt::request_halt();
            // Kurz warten bis der Gast im PendSV hängt
            for (int w = 0; w < 100 && !target_halt::is_halted(); ++w)
                sleep_ms(1);
            if (target_halt::is_halted()) {
                auto* s = target_halt::snapshot();
                std::printf("halted at PC=0x%08lX\n",
                            static_cast<unsigned long>(s ? s->r[15] : 0));
            } else {
                emulator::stop();
                std::puts("halted (forced core reset)");
            }
        } else if (emulator::state() == emulator::State::Faulted) {
            // Gast steht nach nicht-emulierbarem Fault -> Core hart resetten.
            emulator::stop();
            std::puts("Gast war Faulted -> gestoppt ('run' zum Neustart)");
        } else {
            std::puts("nicht gestartet");
        }
        return;
    }
    if (std::strcmp(tokens[0], "step") == 0) {
        if (target_halt::is_halted()) {
            target_halt::request_step();
            // Warten bis erneut gehaltet
            for (int w = 0; w < 200 && !target_halt::is_halted(); ++w)
                sleep_ms(1);
            auto* s = target_halt::snapshot();
            std::printf("step -> PC=0x%08lX\n",
                        static_cast<unsigned long>(s ? s->r[15] : 0));
        } else if (emulator::state() == emulator::State::Running) {
            target_halt::request_halt();
            for (int w = 0; w < 100 && !target_halt::is_halted(); ++w)
                sleep_ms(1);
            auto* s = target_halt::snapshot();
            std::printf("halted at PC=0x%08lX (use 'step' again)\n",
                        static_cast<unsigned long>(s ? s->r[15] : 0));
        } else {
            std::puts("Guest nicht aktiv");
        }
        return;
    }
    if (std::strcmp(tokens[0], "reset") == 0) {
        emulator::stop();
        std::puts("reset");
        return;
    }
    if (std::strcmp(tokens[0], "autostart") == 0 && n >= 2) {
        emulator::FlashPauseGuard _fp;   // XIP-Schutz; Gast laeuft danach weiter
        if (std::strcmp(tokens[1], "on") == 0) {
            config::set_autostart(true);
            config::save();
            std::puts("autostart=on (saved)");
        } else {
            config::set_autostart(false);
            config::save();
            std::puts("autostart=off (saved)");
        }
        return;
    }

    // --- Konfiguration: cfg show/get/set/save/clear (USERGUIDE-kompatibel) ---
    if (std::strcmp(tokens[0], "cfg") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "show") == 0) {
            storage::config_dump(emit_line);
            return;
        }
        if (std::strcmp(tokens[1], "get") == 0 && n >= 3) {
            char buf[96];
            if (storage::config_get(tokens[2], buf, sizeof buf))
                std::printf("%s=%s\n", tokens[2], buf);
            else std::puts("(unset)");
            return;
        }
        if (std::strcmp(tokens[1], "set") == 0 && n >= 4) {
            bool ok = storage::config_set(tokens[2], tokens[3]);
            if (ok) apply_live_config_key(tokens[2], tokens[3]);
            std::puts(ok ? "ok" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "save") == 0) {
            emulator::FlashPauseGuard _fp;   // XIP-Schutz; Gast laeuft danach weiter
            bool ok = config::save();
            usb_msc::note_config_persisted();  // deferred-Flush erledigt
            std::puts(ok ? "saved" : "err");
            return;
        }
        if (std::strcmp(tokens[1], "clear") == 0) {
            // Kompletter Reset: Config-Slot leeren + Live-Zustand auf Default.
            // Loest festgefahrene/veraltete Eintraege ("alles durcheinander").
            emulator::FlashPauseGuard _fp;
            storage::config_clear();
            storage::config_commit();     // leerer Config-Slot im Flash
            config::load();               // laedt leeren Slot -> reine Defaults
            usb_msc::refresh_config_volume();
            std::puts("cfg cleared (Defaults gesetzt + gespeichert)");
            return;
        }
    }

    // --- Pinmap: pinmap show/set/reset (USERGUIDE) ---
    if (std::strcmp(tokens[0], "pinmap") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "show") == 0) {
            const auto& m = config::pin_map();
            for (std::size_t i = 0; i < config::LPC_PIN_COUNT; ++i) {
                if (m.lpc_to_rp[i] >= 0)
                    std::printf("LPC P%u_%u -> GP%d\n",
                                static_cast<unsigned>(i / 12),
                                static_cast<unsigned>(i % 12),
                                m.lpc_to_rp[i]);
            }
            return;
        }
        if (std::strcmp(tokens[1], "set") == 0 && n >= 4) {
            long lpc, rp;
            if (!parse_port_pin(tokens[2], lpc) ||
                !parse_int(tokens[3], -1, 47, rp)) {
                std::puts("err: pinmap set <port_pin> <rp-gpio>"); return;
            }
            if (!config::set_pin_map(static_cast<uint8_t>(lpc), static_cast<int>(rp))) {
                std::puts("err"); return;
            }
            persist_pinmap_change();
            return;
        }
        if (std::strcmp(tokens[1], "reset") == 0) {
            config::apply_default_pinmap();
            std::puts("pinmap defaults restored");
            return;
        }
    }

    if (std::strcmp(tokens[0], "freq") == 0 && n == 2) {
        long hz;
        if (!parse_int(tokens[1], 1, 150'000'000, hz)) { std::puts("err"); return; }
        config::set_target_frequency_hz(static_cast<uint32_t>(hz));
        std::puts("ok");
        return;
    }

    if (std::strcmp(tokens[0], "flash") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "finalize") == 0 && n == 3) {
            long len;
            if (!parse_int(tokens[2], 1, 64L * 1024, len)) { std::puts("err"); return; }
            stop_guest_for_reflash();   // XIP-Schutz
            std::puts(storage::firmware_finalize(static_cast<std::size_t>(len)) ? "ok" : "err");
            return;
        }
    }

    // --- Debugger ---
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

    if (std::strcmp(tokens[0], "bp") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "clr") == 0 && n >= 3) {
            long a;
            if (!parse_int(tokens[2], 0, 0x7FFFFFFF, a)) {
                std::puts("err: bp clr <addr>"); return;
            }
            std::puts(target_halt::clear_breakpoint(static_cast<uint32_t>(a))
                      ? "bp cleared" : "err: bp not found");
        } else {
            long a;
            if (!parse_int(tokens[1], 0, 0x7FFFFFFF, a)) {
                std::puts("err: bp <addr>"); return;
            }
            std::puts(target_halt::set_breakpoint(static_cast<uint32_t>(a))
                      ? "bp set" : "err: max 8 bp or invalid addr");
        }
        return;
    }

    if (std::strcmp(tokens[0], "regs") == 0) {
        auto* s = target_halt::snapshot();
        if (!s) {
            std::puts("(Guest nicht gehaltet — zuerst 'halt')");
            return;
        }
        for (int i = 0; i < 16; i += 4) {
            std::printf("r%-2d=0x%08lX  r%-2d=0x%08lX  r%-2d=0x%08lX  r%-2d=0x%08lX\n",
                        i,   static_cast<unsigned long>(s->r[i]),
                        i+1, static_cast<unsigned long>(s->r[i+1]),
                        i+2, static_cast<unsigned long>(s->r[i+2]),
                        i+3, static_cast<unsigned long>(s->r[i+3]));
        }
        std::printf("xPSR=0x%08lX  SP=0x%08lX  LR=0x%08lX  PC=0x%08lX\n",
                    static_cast<unsigned long>(s->xpsr),
                    static_cast<unsigned long>(s->r[13]),
                    static_cast<unsigned long>(s->r[14]),
                    static_cast<unsigned long>(s->r[15]));
        return;
    }

    if (std::strcmp(tokens[0], "mem") == 0 && n >= 3) {
        long addr, len;
        if (!parse_int(tokens[1], 0, 0x7FFFFFFF, addr) ||
            !parse_int(tokens[2], 1, 256, len)) {
            std::puts("err: mem <addr> <len 1..256>"); return;
        }
        uint32_t a = static_cast<uint32_t>(addr);
        uint32_t l = static_cast<uint32_t>(len);
        uint8_t buf[256];
        if (!target_halt::read_memory(a, buf, l)) {
            std::puts("err: read failed"); return;
        }
        for (uint32_t i = 0; i < l; i += 16) {
            std::printf("%08lX: ", static_cast<unsigned long>(a + i));
            for (uint32_t j = 0; j < 16 && i + j < l; ++j)
                std::printf("%02X ", buf[i + j]);
            std::putchar('\n');
        }
        return;
    }

    if (std::strcmp(tokens[0], "pio") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "capture") == 0 && n >= 4) {
            long pin_arg, count;
            if (!parse_int(tokens[2], 0, 47, pin_arg) ||
                !parse_int(tokens[3], 1, 1000, count)) {
                std::puts("err: pio capture <pin> <count>"); return;
            }
            uint16_t h = pio_glue::setup_capture(static_cast<uint8_t>(pin_arg), true);
            if (h == 0xFFFF) { std::puts("err: PIO voll"); return; }
            std::printf("[PIO] capturing %ld edges on GP%ld...\n",
                        count, pin_arg);
            for (long i = 0; i < count; ++i) {
                uint32_t val = 0;
                int timeout = 5000;
                while (!pio_glue::capture_read(h, val) && --timeout > 0)
                    sleep_ms(1);
                if (timeout <= 0) {
                    std::printf("  [%ld] timeout\n", i); break;
                }
                std::printf("  [%ld] %lu cycles\n", i,
                            static_cast<unsigned long>(val));
            }
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

    // --- CDC-Serial-Konverter (eigenstaendige USB-Serial-Bruecke, Serial-CDC <-> PIO-UART) ---
    // Voellig unabhaengig vom Gast: macht den RP2350 zu einem USB<->UART-Adapter
    // auf frei waehlbaren Pins (PIO). 'uart' betrifft dagegen NUR den LPC-UART0
    // des Gasts.
    if (std::strcmp(tokens[0], "cdc") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "start") == 0) {
            if (n == 4) {
                long tx, rx;
                if (!parse_int(tokens[2], 0, 47, tx) ||
                    !parse_int(tokens[3], 0, 47, rx)) {
                    std::puts("err: cdc start <tx-gpio> <rx-gpio>"); return;
                }
                uart_bridge::set_tx_pin(static_cast<int>(tx));
                uart_bridge::set_rx_pin(static_cast<int>(rx));
                config::set_uart_bridge_tx_pin(static_cast<int>(tx));
                config::set_uart_bridge_rx_pin(static_cast<int>(rx));
            }
            std::puts(uart_bridge::start() ? "ok" : "err: Pins nicht gesetzt oder PIO voll");
            return;
        }
        if (std::strcmp(tokens[1], "stop") == 0) {
            uart_bridge::stop();
            return;
        }
        if (std::strcmp(tokens[1], "status") == 0) {
            std::printf("cdc-bridge=%s TX=GP%d RX=GP%d baud=%lu\n",
                        uart_bridge::active() ? "active" : "off",
                        uart_bridge::tx_pin(), uart_bridge::rx_pin(),
                        static_cast<unsigned long>(uart_bridge::baud_rate()));
            uint32_t crx, ptx, prx, ctx;
            uart_bridge::debug_counts(crx, ptx, prx, ctx);
            std::printf("  Fluss: CDC-RX=%lu -> PIO-TX=%lu -> PIO-RX=%lu -> CDC-TX=%lu\n",
                        static_cast<unsigned long>(crx), static_cast<unsigned long>(ptx),
                        static_cast<unsigned long>(prx), static_cast<unsigned long>(ctx));
            return;
        }
        std::puts("err: cdc start <tx> <rx> | cdc stop | cdc status");
        return;
    }

    // --- LPC-UART0 des Gasts: HW-Pin-Routing ODER virtuelle Serial-CDC-Kopplung ---
    if (std::strcmp(tokens[0], "uart") == 0 && n >= 2) {
        // LPC-UART0 virtuell an die Serial-CDC koppeln (kein Draht/Pin). Schliesst die
        // CDC-Bridge auf der Serial-CDC aus (beide koennen sie nicht gleichzeitig nutzen).
        if (std::strcmp(tokens[1], "cdc") == 0 && n >= 3) {
            bool on = (std::strcmp(tokens[2], "on") == 0 || std::strcmp(tokens[2], "1") == 0);
            if (on) uart_bridge::stop();     // CDC-Bridge gibt die Serial-CDC frei
            config::set_uart0_cdc_enabled(on);
            { emulator::FlashPauseGuard _fp; config::save(); }
            usb_msc::refresh_config_volume();
            if (on && usb_desc_cdc_serial() < 0)
                std::puts("[warn] Serial-CDC ist per Config deaktiviert (serial_enable=0) "
                          "-> Kopplung wirkt erst nach Reset mit aktiver Serial-CDC.");
            std::printf("uart0-cdc=%s (LPC-UART0 %s Serial-CDC)\n",
                        on ? "on" : "off",
                        on ? "<->" : "getrennt von");
            return;
        }
        // LPC-UART0 auf echte RP2350-UART-Pads routen (Hardwareentwurf).
        // 'uart pins <tx> <rx>' (RP-GPIOs) oder 'uart pins off'. TX/RX muessen
        // zum selben RP-Peripheral gehoeren (uart0 GP0/12/16 + GP1/13/17;
        // uart1 GP4/8/20/24 + GP5/9/21/25; + Alt-Funktion GP2/3/6/7/10/11/…).
        if (std::strcmp(tokens[1], "pins") == 0 && n >= 3) {
            if (std::strcmp(tokens[2], "off") == 0) {
                config::set_uart0_tx_gpio(-1);
                config::set_uart0_rx_gpio(-1);
            } else if (n >= 4) {
                long tx, rx;
                if (!parse_int(tokens[2], 0, 47, tx) ||
                    !parse_int(tokens[3], 0, 47, rx)) {
                    std::puts("err: uart pins <tx-gpio> <rx-gpio> | off"); return;
                }
                config::set_uart0_tx_gpio(static_cast<int>(tx));
                config::set_uart0_rx_gpio(static_cast<int>(rx));
            } else {
                std::puts("err: uart pins <tx-gpio> <rx-gpio> | off"); return;
            }
            { emulator::FlashPauseGuard _fp; config::save(); }
            usb_msc::refresh_config_volume();
            std::printf("uart0-pins TX=GP%d RX=GP%d (wirkt beim naechsten UART-Zugriff/Start)\n",
                        config::uart0_tx_gpio(), config::uart0_rx_gpio());
            return;
        }
        if (std::strcmp(tokens[1], "status") == 0) {
            std::printf("LPC-UART0: HW-Pads TX=GP%d RX=GP%d | Serial-CDC-virtuell=%s\n",
                        config::uart0_tx_gpio(), config::uart0_rx_gpio(),
                        config::uart0_cdc_enabled() ? "on" : "off");
            if (config::uart0_cdc_enabled()) {
                uint32_t p2g, g2p;
                uart_bridge::uart0_cdc_counts(p2g, g2p);
                std::printf("  Fluss: PC->Gast=%lu Bytes  Gast->PC=%lu Bytes  (Serial-CDC=%s)\n",
                            static_cast<unsigned long>(p2g),
                            static_cast<unsigned long>(g2p),
                            usb_desc_cdc_serial() >= 0 ? "aktiv" : "AUS (serial_enable=0!)");
                uint8_t ier; bool nvic_en;
                uint32_t rx_pends, rbr_reads, tx_writes;
                peripherals::uart0_debug(ier, nvic_en, rx_pends, rbr_reads, tx_writes);
                std::printf("  Gast-UART: IER=0x%02x (RBR-IRQ=%s) NVIC-UART0=%s | "
                            "RX-IRQ-pends=%lu RBR-reads=%lu THR-writes=%lu\n",
                            ier, (ier & 0x01u) ? "an" : "AUS",
                            nvic_en ? "an" : "AUS",
                            static_cast<unsigned long>(rx_pends),
                            static_cast<unsigned long>(rbr_reads),
                            static_cast<unsigned long>(tx_writes));
                uint32_t mmio_addr; bool mmio_wr;
                peripherals::last_mmio(mmio_addr, mmio_wr);
                std::printf("  Letzter MMIO-Zugriff: 0x%08lx (%s)  [aendert er sich nicht, "
                            "haengt der Gast hier]\n",
                            static_cast<unsigned long>(mmio_addr),
                            mmio_wr ? "Write" : "Read");
            }
            return;
        }
        std::puts("err: uart pins <tx> <rx>|off | uart cdc on|off | uart status");
        return;
    }

    // --- I2C-Bridge (LPC-I2C-Master → RP2350-Hardware-I2C) ---
    if (std::strcmp(tokens[0], "i2c") == 0 && n >= 2) {
        if (std::strcmp(tokens[1], "on") == 0) {
            if (n < 5) { std::puts("err: i2c on <inst 0|1> <sda> <scl> [hz]"); return; }
            long inst, sda, scl, hz = 100000;
            if (!parse_int(tokens[2], 0, 1, inst) ||
                !parse_int(tokens[3], 0, 47, sda) ||
                !parse_int(tokens[4], 0, 47, scl) ||
                (n >= 6 && !parse_int(tokens[5], 1000, 1000000, hz))) {
                std::puts("err: i2c on <inst 0|1> <sda> <scl> [hz]"); return;
            }
            config::set_i2c_bridge_instance(static_cast<int>(inst));
            config::set_i2c_bridge_sda_pin(static_cast<int>(sda));
            config::set_i2c_bridge_scl_pin(static_cast<int>(scl));
            config::set_i2c_bridge_hz(static_cast<uint32_t>(hz));
            config::set_i2c_bridge_enabled(true);
            std::puts("ok (wird beim naechsten Start/Reset aktiv)");
            return;
        }
        if (std::strcmp(tokens[1], "off") == 0) {
            config::set_i2c_bridge_enabled(false);
            std::puts("ok (wird beim naechsten Start/Reset wirksam)");
            return;
        }
        if (std::strcmp(tokens[1], "status") == 0) {
            std::printf("i2c-bridge=%s inst=i2c%d SDA=GP%d SCL=GP%d hz=%lu\n",
                        config::i2c_bridge_enabled() ? "on" : "off",
                        config::i2c_bridge_instance(),
                        config::i2c_bridge_sda_pin(), config::i2c_bridge_scl_pin(),
                        static_cast<unsigned long>(config::i2c_bridge_hz()));
            return;
        }
    }

    // Befehl wurde erkannt (tokens[0] ist kanonisch), aber es fehlt eine Option
    // oder ein Argument. Hat der Befehl Sub-Optionen, diese in einer Zeile zeigen.
    {
        const Word* sub = nullptr; int subn = 0;
        if (get_subtable(tokens[0], sub, subn)) {
            std::printf("Optionen fuer '%s': ", tokens[0]);
            print_matches(sub, subn, nullptr);
        } else {
            std::puts("unvollstaendig — 'help' fuer die Syntax");
        }
    }
}

void process_hex_line(const char* line) {
    if (*line == '\0') {
        // Leerzeile beendet Upload-Modus.
        uint32_t bw = g_hex_parser ? g_hex_parser->bytes_written() : 0;
        if (bw > 0) {
            storage::firmware_finalize(bw);
            std::printf("[upload] %lu bytes, CRC ok\n",
                        static_cast<unsigned long>(bw));
        } else {
            std::puts("[upload] abgebrochen (keine Daten)");
        }
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
        uint32_t bw = g_hex_parser->bytes_written();
        storage::firmware_finalize(bw);
        std::printf("[upload] %lu bytes, CRC ok\n",
                    static_cast<unsigned long>(bw));
        g_in_hex_upload = false;
        g_hex_parser = nullptr;
    }
}

} // namespace

namespace cli {

void init() {
    if (config::cli_enabled())
        std::puts("\nLPC1115 Emulator @ RP2350 — type 'help'");
    else
        std::puts("\n[CLI deaktiviert (cli_enable=0) — Housekeeping laeuft weiter]");
}

void run() {
    // CLI-Interaktion per Config abschaltbar (cli_enable=0). Auch dann muss der
    // Hauptloop weiterlaufen (USB-Task, GDB, MSC, UART-Bridge, Boot-Requests) —
    // nur Prompt + Tastatureingabe entfallen.
    const bool cli_on = config::cli_enabled();
    char line[LINE_MAX];
    std::size_t len = 0;
    int  hist_browse = 0;    // 0 = aktuelle Eingabe; >0 = n Schritte zurueck
    bool last_cr     = false; // fuer CRLF-Entprellung (siehe Enter-Behandlung)
    if (cli_on) std::printf("emu> ");
    while (true) {
        usb_stdio_task();
        led::poll();
        gdb_stub::poll();
        swd_target::poll();
        usb_msc::poll();
        uart_bridge::poll();
        uart_bridge::uart0_cdc_poll();   // virtuelle LPC-UART0 <-> Serial-CDC
        if (emulator::guest_reset_pending()) {
            // Vom Gast angeforderter Soft-Reset (NVIC_SystemReset/WDT). Core1
            // hat geparkt; Core0 fuehrt den eigentlichen Core-Reset aus.
            emulator::request_guest_reset();
        }
        if (usb_msc::consume_pending_boot_request()) {
            std::printf("\n[BOOT] BOOT.HEX ueber USB-MSC erkannt -> Start\n");
            emulator::load_and_start();
            if (cli_on) std::printf("emu> ");
        }
        if (!cli_on) {
            // Keine CLI: Eingabe/Prompt ueberspringen, nur Loop-Pacing.
            sleep_us(500);
            continue;
        }
        int c = getchar_timeout_us(1'000);
        if (c == PICO_ERROR_TIMEOUT) continue;

        // --- Enter: terminal-unabhaengig. PuTTY/serielle Terminals senden bei
        // Enter oft NUR CR ('\r'), andere LF ('\n'), manche CRLF. Alle drei als
        // Zeilenende akzeptieren; ein LF unmittelbar nach CR (das CRLF-Paar) wird
        // entprellt, damit nicht zwei Kommandos ausgeloest werden.
        if (c == '\r' || c == '\n') {
            if (c == '\n' && last_cr) { last_cr = false; continue; }
            last_cr = (c == '\r');
            std::putchar('\n');
            line[len] = '\0';
            if (g_in_hex_upload) process_hex_line(line);
            else { history_push(line); handle_command(line); }
            len = 0;
            hist_browse = 0;
            std::printf("emu> ");
            // Ausgabe des Kommandos sofort ueber USB rausschieben, damit die
            // Antwort nicht erst beim naechsten Tastendruck sichtbar wird.
            std::fflush(stdout);
            usb_stdio_task();
            continue;
        }
        last_cr = false;   // jedes Nicht-CR-Zeichen beendet ein evtl. CRLF-Paar

        // --- ESC-Sequenzen (Pfeiltasten): ESC '[' 'A'/'B' = History hoch/runter.
        // Nur im Nicht-Upload-Modus; die Folgebytes kommen zusammen (kurzer poll).
        if (c == 27) {
            int c1 = getchar_timeout_us(3'000);
            int c2 = (c1 == '[') ? getchar_timeout_us(3'000) : PICO_ERROR_TIMEOUT;
            if (!g_in_hex_upload && c1 == '[' && (c2 == 'A' || c2 == 'B')) {
                if (c2 == 'A') {                       // hoch: aelter
                    if (hist_browse < g_hist_count) ++hist_browse;
                } else {                               // runter: neuer
                    if (hist_browse > 0) --hist_browse;
                }
                const char* h = (hist_browse > 0) ? history_get(hist_browse) : "";
                len = 0;
                for (const char* q = h; q && *q && len + 1 < sizeof line; ++q)
                    line[len++] = *q;
                line[len] = '\0';
                redraw_line(line, len);
            }
            continue;   // andere ESC-Sequenzen ignorieren
        }

        // --- TAB: Befehl/Option vervollstaendigen (nicht im Upload-Modus).
        if (c == '\t') {
            if (!g_in_hex_upload) complete_input(line, len);
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