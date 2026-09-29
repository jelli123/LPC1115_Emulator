//
// HTML-Kurzanleitung fuer das USB-MSC-Laufwerk (HELP.HTM).
//
// Diese Datei enthaelt AUSSCHLIESSLICH den HTML-Inhalt als Raw-String, damit
// er ohne Ruecksicht auf C++-Escaping direkt editiert werden kann. Struktur:
//   * Self-contained: Inline-CSS, keine externen Ressourcen -> direkt vom
//     Laufwerk im Browser lesbar.
//   * Umlaute bewusst als ae/oe/ue geschrieben (robust ueber Editoren/Encodings).
//   * Der Raw-String-Delimiter ist )HTML" — im HTML selbst nicht verwenden.
//
// Nach Aenderungen genuegt ein normaler Rebuild; usb_msc.cpp bindet den Text
// ueber help_html.h ein und legt ihn beim Volume-Aufbau als HELP.HTM ab.
//

#include "help_html.h"

const char HELP_HTML[] =
R"HTML(<!DOCTYPE html><html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LPC1115-Emulator - Kurzanleitung</title><style>
body{font-family:system-ui,Arial,sans-serif;max-width:60em;margin:1em auto;padding:0 1em;color:#222;line-height:1.45}
h1{border-bottom:3px solid #0a7;padding-bottom:.2em}h2{margin-top:1.6em;color:#066;border-bottom:1px solid #ccc}
code,kbd{background:#f2f2f2;padding:.1em .35em;border-radius:3px;font-family:Consolas,monospace}
table{border-collapse:collapse;width:100%;margin:.6em 0}th,td{border:1px solid #ccc;padding:.3em .5em;text-align:left;vertical-align:top}
th{background:#e8f5f2}tr:nth-child(even){background:#fafafa}.mono{font-family:Consolas,monospace}
.note{background:#fffbe6;border-left:4px solid #e0a800;padding:.5em .8em;margin:.8em 0}
.files code{font-weight:bold}small{color:#666}</style></head><body>
<h1>LPC1115-Emulator auf RP2350</h1>
<p>Selfbus-/LPC1115-Firmware laeuft <b>nativ</b> auf einem Cortex-M33-Kern des RP2350.
Diese Seite liegt auf dem USB-Laufwerk und fasst Bedienung, Konfiguration und die
Fehleranzeige zusammen. Ausfuehrliche Doku: <span class="mono">README.md</span>,
<span class="mono">docs/USERGUIDE.md</span>, <span class="mono">docs/TECHNICAL.md</span>.</p>

<h2>Dateien auf diesem Laufwerk</h2>
<table class="files"><tr><th>Datei</th><th>Bedeutung</th></tr>
<tr><td><code>BOOT.HEX</code></td><td>Hierher kopierte Intel-HEX-Firmware (max. 64&nbsp;KiB). Wird beim Auswerfen geladen und danach automatisch vom Laufwerk entfernt (wie beim RP2350-UF2-Bootloader). Jede <code>*.HEX</code> wird akzeptiert; <code>BOOT.HEX</code> hat Vorrang.</td></tr>
<tr><td><code>CONFIG.INI</code></td><td>Aktuelle Einstellungen + auskommentierte Optionen. Editierbar; wird beim Auswerfen uebernommen.</td></tr>
<tr><td><code>HELP.HTM</code></td><td>Diese Kurzanleitung.</td></tr>
<tr><td><code>FAULT.TXT</code></td><td>Erscheint nur nach einem Gast-Fehler: kompletter Fehlerbericht (siehe unten).</td></tr>
</table>
<div class="note"><b>Firmware laden ohne CLI:</b> <code>BOOT.HEX</code> (und optional <code>CONFIG.INI</code>)
aufs Laufwerk kopieren, dann <b>auswerfen</b> ("sicher entfernen"). Das Laden ist <b>additiv</b>
(Merge) &ndash; zum vollstaendigen Ersetzen vorher <code>flash_erase=on</code> in <code>CONFIG.INI</code>
setzen oder in der CLI <code>erase</code> ausfuehren. Nach dem Flashen verschwindet die HEX-Datei
vom Laufwerk (Medienwechsel); die uebrigen Dateien bleiben.</div>

<h2>Status-LED (Onboard, GP25)</h2>
<table><tr><th>Muster</th><th>Bedeutung</th></tr>
<tr><td>1&nbsp;Hz Herzschlag</td><td>Idle / bereit, keine Firmware aktiv</td></tr>
<tr><td>dauerhaft an</td><td>Gast-Firmware laeuft</td></tr>
<tr><td>Doppelblitz alle 2&nbsp;s</td><td>Gast angehalten (<code>halt</code>/Breakpoint)</td></tr>
<tr><td>schnelles Flackern (8&nbsp;Hz)</td><td>Gast-Fehler (State=Faulted) &ndash; siehe <code>FAULT.TXT</code></td></tr>
</table>

<h2>CLI-Befehle (USB-CLI-CDC, 115200&nbsp;8N1)</h2>
<table><tr><th>Befehl</th><th>Wirkung</th></tr>
<tr><td><code>help [cmd]</code> / <code>version</code></td><td>Hilfe (auch zu einzelnem Befehl) / Build-Info. Befehle sind abkuerzbar, TAB vervollstaendigt.</td></tr>
<tr><td><code>reboot</code> / <code>bootsel</code></td><td>RP2350 neu starten / in den USB-Bootloader (Update per <code>picotool load</code> oder UF2)</td></tr>
<tr><td><code>stats</code></td><td>Status &amp; Zaehler (siehe unten)</td></tr>
<tr><td><code>run</code> / <code>stop</code> / <code>reset</code></td><td>Gast starten / anhalten / neu starten</td></tr>
<tr><td><code>step</code></td><td>ein Befehl, dann anhalten</td></tr>
<tr><td><code>upload</code></td><td>Intel-HEX per XMODEM laden (additiv; Alias <code>xmodem</code>)</td></tr>
<tr><td><code>erase</code></td><td>Firmware-Slot komplett leeren</td></tr>
<tr><td><code>info</code></td><td>Reset-Vektor, Stack, Groesse</td></tr>
<tr><td><code>autostart on|off</code></td><td>nach Reset automatisch starten</td></tr>
<tr><td><code>cfg show|get|set|save|clear</code></td><td>Konfiguration; <code>set</code> wirkt sofort (on/off erlaubt), dauerhaft per <code>save</code></td></tr>
<tr><td><code>dbg [clear|save|auto]</code></td><td>Debug-Ausgabe des Gasts (Debug-Bridge) / als <code>DEBUG.TXT</code></td></tr>
<tr><td><code>pinmap show|set|reset</code></td><td>LPC-Pin &rarr; RP2350-GPIO</td></tr>
<tr><td><code>gdb on|off|status</code></td><td>GDB-Stub auf der GDB-CDC (Adressen in LPC-Sicht, passend zur ELF)</td></tr>
<tr><td><code>halt</code>, <code>step</code>, <code>regs</code>, <code>bp [clr] &lt;addr&gt;</code>, <code>mem &lt;addr&gt; &lt;len&gt;</code></td><td>Gast anhalten, Einzelschritt, Register, Breakpoints, Speicher (Flash/RAM)</td></tr>
<tr><td><code>swd start &lt;dio&gt; &lt;clk&gt;</code></td><td>externes SWD-Target (clk = dio+1)</td></tr>
<tr><td><code>cdc start|stop|status</code></td><td>USB&harr;UART-Adapter (Serial-CDC &harr; PIO-UART, beliebige GPIOs)</td></tr>
<tr><td><code>uart pins|cdc|status</code></td><td>LPC-UART0 des Gasts: echte RP-Pads bzw. virtuell an Serial-CDC</td></tr>
<tr><td><code>isp</code>, <code>isp enter|exit</code></td><td>ISP-Bootloader (FlashMagic: ISP-COM-Port, Option &bdquo;Use DTR and RTS&ldquo;)</td></tr>
<tr><td><code>resetpin on|off</code></td><td>P0_0 als RESET-Eingang des Gasts</td></tr>
<tr><td><code>i2c on|off|status</code></td><td>I2C-Bridge auf echte RP2350-Hardware</td></tr>
<tr><td><code>ncn on|off|loopback|status</code></td><td>virtueller NCN5130 (KNX-Transceiver) an LPC-SSP0/1</td></tr>
<tr><td><code>pio capture &lt;pin&gt; &lt;n&gt;</code></td><td>Flanken-Logger (Richtung + Abstand in &micro;s), liest den Pin rueckwirkungsfrei mit</td></tr>
<tr><td><code>gpio &lt;n&gt; [up|down|none|inv|noinv]</code></td><td>RP2350-Pin: Funktion, Richtung, Pegel, Pull; Eingang invertieren</td></tr>
</table>

<h2>Ausgabe von <code>stats</code></h2>
<table><tr><th>Feld</th><th>Bedeutung</th></tr>
<tr><td class="mono">State</td><td><b>Idle</b> bereit &middot; <b>Running</b> Gast laeuft &middot; <b>Halted</b> angehalten (Debugger) &middot; <b>Faulted</b> Gast-Fehler</td></tr>
<tr><td class="mono">PC</td><td>Reset-PC (nur beim Start gesetzt) &ndash; <b>kein</b> Live-Programmzaehler. Aendert sich waehrend der Ausfuehrung nicht.</td></tr>
<tr><td class="mono">mmio-traps</td><td>Summe aller erfolgreich emulierten Trap-Zugriffe (MMIO + RAM-Fallback + Flash-Lesevorgaenge). Steigt, solange der Gast mit LPC-Peripherie arbeitet.</td></tr>
<tr><td class="mono">MMIO R / W</td><td>Lese-/Schreibzugriffe auf modellierte LPC-Peripherie</td></tr>
<tr><td class="mono">GPIO</td><td>GPIO-Schreibzugriffe (Pin-Ausgaben)</td></tr>
<tr><td class="mono">PLL-cfg</td><td>Anzahl PLL-/Takt-Rekonfigurationen des Gastes</td></tr>
<tr><td class="mono">NVIC-W</td><td>Schreibzugriffe auf den Schatten-NVIC (ISER/ICER/ISPR/ICPR/IPR)</td></tr>
<tr><td class="mono">CPU-target</td><td>uebernommene LPC-Soll-Frequenz (Zeitbasis der Timer). Der reale RP2350-Takt bleibt bei 150&nbsp;MHz.</td></tr>
<tr><td class="mono">GDB</td><td>GDB-Stub aktiv (on/off)</td></tr>
<tr><td class="mono">USB-CDC</td><td>dynamische CDC-Zuordnung je COM-Port (z.&nbsp;B. <code>CDC#0=CLI CDC#1=GDB CDC#2=Serial-Adapter CDC#3=ISP</code>); <code>| aus:</code> listet per Config deaktivierte Rollen</td></tr>
<tr><td class="mono">PIO used</td><td>belegte/freie PIO-State-Machines + Instruktions-Slots</td></tr>
</table>
<div class="note">Die vier USB-CDCs sind einzeln abschaltbar: <code>cli_enable</code>,
<code>gdb_enable</code>, <code>serial_enable</code>, <code>isp_enable</code> in der CONFIG.INI (Default <b>on</b>).
Eine deaktivierte CDC verschwindet komplett vom USB (ein COM-Port weniger); das
MSC-Laufwerk bleibt immer aktiv. Aenderung wirkt erst nach einem <b>Reset</b>.</div>
<div class="note">Laeuft der Gast und die Zaehler steigen bei erneutem <code>stats</code>
nicht mehr, dreht er meist eine Warteschleife (z.&nbsp;B. KNX-Bus-Idle) &ndash; das ist normal.</div>

<h2>Fehlerbericht <code>FAULT.TXT</code> / Stacktrace</h2>
<p>Bei einem nicht emulierbaren Fehler <b>haelt der Gast an</b> (kein Chip-Reset). Der Bericht
erscheint seriell <i>und</i> als <code>FAULT.TXT</code>. Aufbau:</p>
<p class="mono">[FAULT] &lt;Typ&gt; @PC=0x&hellip; (LPC 0x&hellip;) CFSR=0x&hellip; HFSR=0x&hellip;</p>
<ul>
<li><b>Typ</b> &ndash; <code>HardFault</code>, <code>UsageFault</code>, <code>non-decodable</code> (unbekannte Instruktion am Trap) oder <code>mmio rejected</code> (Zugriff auf nicht modellierte Peripherie).</li>
<li><b>PC</b> &ndash; Programmzaehler beim Fehler. <b>LPC&nbsp;0x&hellip;</b> = Offset im LPC-Flash-Abbild; <code>0xffffffff</code> = Adresse nicht im Flash-Abbild.</li>
<li><b>CFSR/HFSR</b> &ndash; ARM-Fault-Statusregister; die gesetzten Bits werden darunter im Klartext aufgeschluesselt.</li>
</ul>
<p>Register-Zeilen: <span class="mono">r0&ndash;r12, LR (+LPC-Offset), xPSR, SP, r4&ndash;r7</span>,
sowie <span class="mono">instr@PC</span> (die zwei Instruktions-Halbworte am PC) und ggf.
<span class="mono">MMFAR/BFAR</span> (Fehleradresse).</p>
<table><tr><th>Abkuerzung</th><th>Bedeutung</th></tr>
<tr><td class="mono">IACCVIOL</td><td>Code-Ausfuehrung an gesperrter Adresse (wilder Sprung / Funktionspointer)</td></tr>
<tr><td class="mono">DACCVIOL</td><td>Datenzugriff an gesperrter Adresse (MPU)</td></tr>
<tr><td class="mono">MSTKERR / MUNSTKERR</td><td>Fehler beim Exception-Stacking / -Unstacking</td></tr>
<tr><td class="mono">IBUSERR</td><td>Bus-Fehler beim Instruktions-Fetch</td></tr>
<tr><td class="mono">PRECISERR / IMPRECISERR</td><td>praeziser / verzoegerter Datenbus-Fehler</td></tr>
<tr><td class="mono">UNDEFINSTR</td><td>illegale/unbekannte Instruktion</td></tr>
<tr><td class="mono">INVSTATE</td><td>ungueltiger Thumb-/EPSR-Zustand (z.&nbsp;B. Sprung ohne Thumb-Bit)</td></tr>
<tr><td class="mono">INVPC</td><td>ungueltiger EXC_RETURN / PC bei Exception-Rueckkehr</td></tr>
<tr><td class="mono">NOCP</td><td>Coprozessor/FPU angesprochen, aber nicht verfuegbar</td></tr>
<tr><td class="mono">STKOF</td><td>Stack-Overflow (Stack-Limit verletzt)</td></tr>
<tr><td class="mono">UNALIGNED</td><td>nicht ausgerichteter Speicherzugriff</td></tr>
<tr><td class="mono">DIVBYZERO</td><td>Division durch Null</td></tr>
<tr><td class="mono">FORCED</td><td>eskalierter Fault (haeufig: Fehler in kritischer Sektion mit gesperrten IRQs)</td></tr>
<tr><td class="mono">VECTTBL</td><td>Fehler beim Lesen der Vektortabelle</td></tr>
</table>
<p><small>Weiterlaufen nach einem Fehler: CLI <code>reset</code> bzw. <code>run</code>, oder neue Firmware aufspielen.</small></p>

<h2>Pinmap (LPC &rarr; RP2350-GPIO)</h2>
<p>Ab Werk ist <b>kein</b> LPC-Pin gemappt; die Zuordnung legt man selbst fest:
<code>CONFIG.INI</code> (<code>pin.&lt;port&gt;_&lt;pin&gt;=&lt;gpio&gt;</code>) oder CLI <code>pinmap set 1_8 17</code>.
GP25 = Status-LED; GP26&ndash;29 sind die ADC-Eingaenge.</p>
<table><tr><th>Funktion</th><th>Konfiguration</th></tr>
<tr><td>GPIO des Gasts</td><td><code>pin.&lt;port&gt;_&lt;pin&gt;=&lt;gpio&gt;</code></td></tr>
<tr><td>LPC-UART0</td><td><code>uart0_tx</code>/<code>uart0_rx</code> (RP-UART-Pads) oder <code>uart0_cdc=on</code> (virtuell)</td></tr>
<tr><td>KNX-Bus (Timer)</td><td><code>tcap.&lt;t&gt;=&lt;gpio&gt;</code> (RX), <code>tmat.&lt;t&gt;.&lt;m&gt;=&lt;gpio&gt;</code> (TX); <code>tcap_pio</code>/<code>tmatch_pio=on</code> (Default) fuer flankengenaues Timing</td></tr>
<tr><td>RESET / ISP</td><td><code>pin.0_0</code> + <code>reset_in=on</code>, <code>pin.0_1</code> + <code>isp_pins=on</code></td></tr>
</table>
<p><small>LPC1115-Emulator &middot; generiert auf dem Geraet &middot; Details in CONFIG.INI</small></p>
</body></html>)HTML";

const unsigned HELP_HTML_LEN = sizeof(HELP_HTML) - 1u;
