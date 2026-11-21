#!/usr/bin/env python3
"""
mkimage.py - Flash-Image fuer den LPC1115-Emulator erzeugen (ohne USB).

Erzeugt die Speicherbereiche, die der Emulator beim Booten aus dem RP2350-
Flash liest, als Intel-HEX (fuer SWD/OpenOCD) und/oder UF2:

  * INI-Import-Sektor: CONFIG.INI-Text (gleiche Syntax wie auf dem USB-
    Laufwerk). Der Emulator uebernimmt ihn beim naechsten Boot EINMALIG und
    speichert ihn (config-Key ini_crc); spaetere CLI-Aenderungen bleiben.
  * LPC-Firmware-Slot (64 KiB) + Marker: Bootloader und/oder Applikation aus
    einer oder mehreren Intel-HEX-Dateien (spaetere ueberschreiben fruehere).

Layout (muss zu src/storage.cpp passen), F = Flash-Groesse:
  config  = F - 32 KiB      (8 Sektoren, vom Emulator verwaltet - wird NICHT geschrieben)
  fw-slot = config - 64 KiB
  marker  = fw-slot - 4 KiB (magic 0xF00D1115, seq, laenge, crc32)
  import  = marker - 4 KiB  (magic 0x1A1C1115, laenge, crc32, 0xFFFFFFFF, Text)

Beispiele:
  # Board-Profil + eigene Ergaenzungen, Bootloader + App, Emulator dazu:
  scripts/mkimage.py --board selfbus_io8_rp2354 --ini meine.ini \\
      --hex bootloader.hex --hex app_0x3000.hex \\
      --emulator build-io8/emulator.hex -o io8_komplett.hex

  # Flashen per SWD (CMSIS-DAP / Picoprobe):
  openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \\
      -c "adapter speed 5000" -c "program io8_komplett.hex verify reset exit"
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path

XIP_BASE = 0x1000_0000
SECTOR = 4096
CONFIG_REGION = 8 * SECTOR
FW_SLOT = 64 * 1024

MAGIC_FIRMWARE = 0xF00D_1115
MAGIC_INI = 0x1A1C_1115

INI_LINE_MAX = 95          # Zeilenpuffer des Emulator-Parsers (96 inkl. NUL)

UF2_MAGIC0 = 0x0A32_4655
UF2_MAGIC1 = 0x9E5D_5157
UF2_MAGIC_END = 0x0AB1_6F30
UF2_FLAG_FAMILY = 0x0000_2000
UF2_FAMILY_ABSOLUTE = 0xE48B_FF57   # RP2350 "absolute": feste Flash-Adresse

REPO = Path(__file__).resolve().parent.parent


def die(msg: str) -> None:
    print(f"mkimage: Fehler: {msg}", file=sys.stderr)
    sys.exit(1)


def parse_size(s: str) -> int:
    s = s.strip().upper()
    mult = 1
    if s.endswith("K"):
        mult, s = 1024, s[:-1]
    elif s.endswith("M"):
        mult, s = 1024 * 1024, s[:-1]
    return int(s, 0) * mult


# ---------------------------------------------------------------------------
# Intel-HEX
# ---------------------------------------------------------------------------
def read_hex(path: Path) -> dict[int, int]:
    """Liest Intel-HEX, liefert {adresse: byte}."""
    data: dict[int, int] = {}
    base = 0
    for no, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":"):
            die(f"{path}:{no}: keine Intel-HEX-Zeile")
        rec = bytes.fromhex(line[1:])
        if len(rec) < 5 or len(rec) != rec[0] + 5:
            die(f"{path}:{no}: falsche Satzlaenge")
        if sum(rec) & 0xFF:
            die(f"{path}:{no}: Pruefsumme falsch")
        n, addr, typ = rec[0], (rec[1] << 8) | rec[2], rec[3]
        payload = rec[4:4 + n]
        if typ == 0x00:
            for i, b in enumerate(payload):
                data[base + addr + i] = b
        elif typ == 0x01:
            break
        elif typ == 0x02:
            base = int.from_bytes(payload, "big") << 4
        elif typ == 0x04:
            base = int.from_bytes(payload, "big") << 16
        elif typ in (0x03, 0x05):
            pass                      # Startadresse - irrelevant
        else:
            die(f"{path}:{no}: unbekannter Satztyp {typ:#04x}")
    return data


def write_hex(path: Path, regions: list[tuple[int, bytes]]) -> None:
    out = []
    upper = None
    for start, blob in sorted(regions):
        for off in range(0, len(blob), 16):
            addr = start + off
            chunk = blob[off:off + 16]
            if addr >> 16 != upper:
                upper = addr >> 16
                rec = bytes([2, 0, 0, 4]) + upper.to_bytes(2, "big")
                out.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
            rec = bytes([len(chunk), (addr >> 8) & 0xFF, addr & 0xFF, 0]) + chunk
            out.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
    out.append(":00000001FF")
    path.write_text("\n".join(out) + "\n")


def write_uf2(path: Path, regions: list[tuple[int, bytes]]) -> None:
    blocks = []
    for start, blob in sorted(regions):
        for off in range(0, len(blob), 256):
            blocks.append((start + off, blob[off:off + 256].ljust(256, b"\xff")))
    out = bytearray()
    for i, (addr, payload) in enumerate(blocks):
        hdr = struct.pack("<8I", UF2_MAGIC0, UF2_MAGIC1, UF2_FLAG_FAMILY, addr,
                          256, i, len(blocks), UF2_FAMILY_ABSOLUTE)
        out += hdr + payload + bytes(476 - 256) + struct.pack("<I", UF2_MAGIC_END)
    path.write_bytes(bytes(out))


# ---------------------------------------------------------------------------
# Bereiche
# ---------------------------------------------------------------------------
def build_ini_sector(texts: list[tuple[str, str]], keep_comments: bool) -> bytes:
    lines: list[str] = []
    for name, text in texts:
        lines.append(f"# --- {name} ---")
        lines.extend(text.replace("\r", "").split("\n"))
    for no, ln in enumerate(lines, 1):
        s = ln.strip()
        if s and not s.startswith(("#", ";")) and len(ln) > INI_LINE_MAX:
            die(f"INI-Zeile laenger als {INI_LINE_MAX} Zeichen: {ln[:40]}...")
    body = "\n".join(lines) + "\n"
    room = SECTOR - 16
    if keep_comments and len(body.encode()) > room:
        print("mkimage: INI zu gross fuer einen Sektor - Kommentare werden entfernt",
              file=sys.stderr)
        keep_comments = False
    if not keep_comments:
        body = "\n".join(l.strip() for l in lines
                         if l.strip() and not l.strip().startswith(("#", ";"))) + "\n"
    raw = body.encode("ascii", errors="strict")
    if len(raw) > room:
        die(f"INI belegt {len(raw)} Bytes, maximal {room}")
    hdr = struct.pack("<4I", MAGIC_INI, len(raw), zlib.crc32(raw), 0xFFFF_FFFF)
    return (hdr + raw).ljust(SECTOR, b"\xff")


def build_firmware(hexfiles: list[Path]) -> tuple[bytes, bytes, int]:
    slot = bytearray(b"\xff" * FW_SLOT)
    for path in hexfiles:
        data = read_hex(path)
        bad = [a for a in data if a >= FW_SLOT]
        if bad:
            die(f"{path}: Adresse {min(bad):#x} ausserhalb des LPC-Flash (64 KiB)")
        for a, b in data.items():
            slot[a] = b
    length = FW_SLOT
    while length and slot[length - 1] == 0xFF:
        length -= 1
    if length == 0:
        die("Firmware ist leer")
    marker = struct.pack("<4I", MAGIC_FIRMWARE, 1, length, zlib.crc32(slot[:length]))
    return bytes(slot), marker.ljust(SECTOR, b"\xff"), length


def hex_to_regions(path: Path) -> list[tuple[int, bytes]]:
    """Emulator-HEX in zusammenhaengende Bereiche zerlegen."""
    data = read_hex(path)
    regions: list[tuple[int, bytes]] = []
    start = prev = None
    buf = bytearray()
    for a in sorted(data):
        if prev is not None and a == prev + 1:
            buf.append(data[a])
        else:
            if buf:
                regions.append((start, bytes(buf)))
            start, buf = a, bytearray([data[a]])
        prev = a
    if buf:
        regions.append((start, bytes(buf)))
    return regions


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Flash-Image (INI-Import + LPC-Firmware) fuer den LPC1115-Emulator",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--flash-size", default="2M",
                    help="Flash-Groesse des RP2350 (Default 2M = RP2354; Pico 2: 4M)")
    ap.add_argument("--board", help="Board-Profil boards/<BOARD>.ini voranstellen")
    ap.add_argument("--ini", action="append", default=[], type=Path,
                    help="INI-Datei (mehrfach; spaetere Werte gewinnen)")
    ap.add_argument("--hex", action="append", default=[], type=Path,
                    help="LPC-Firmware als Intel-HEX (mehrfach, z. B. Bootloader + App)")
    ap.add_argument("--emulator", type=Path,
                    help="Emulator-Firmware (emulator.hex) mit in die HEX-Ausgabe aufnehmen")
    ap.add_argument("--strip-comments", action="store_true",
                    help="Kommentare/Leerzeilen nicht in den Import-Sektor schreiben")
    ap.add_argument("-o", "--output", type=Path, help="Intel-HEX-Ausgabe (SWD/OpenOCD)")
    ap.add_argument("--uf2", type=Path,
                    help="UF2-Ausgabe (nur INI/Firmware, BOOTSEL bzw. picotool)")
    args = ap.parse_args()

    if not args.output and not args.uf2:
        die("keine Ausgabe angegeben (-o und/oder --uf2)")
    if args.emulator and not args.output:
        die("--emulator nur zusammen mit -o (HEX)")

    fsize = parse_size(args.flash_size)
    config_off = fsize - CONFIG_REGION
    fw_off = config_off - FW_SLOT
    marker_off = fw_off - SECTOR
    ini_off = marker_off - SECTOR

    texts: list[tuple[str, str]] = []
    if args.board:
        p = REPO / "boards" / f"{args.board}.ini"
        if not p.exists():
            die(f"Board-Profil {p} nicht gefunden")
        texts.append((p.name, p.read_text()))
    for p in args.ini:
        texts.append((p.name, p.read_text()))
    if not texts and not args.hex:
        die("nichts zu tun: --board/--ini und/oder --hex angeben")

    regions: list[tuple[int, bytes]] = []
    if texts:
        sec = build_ini_sector(texts, keep_comments=not args.strip_comments)
        regions.append((XIP_BASE + ini_off, sec))
        n = struct.unpack_from("<I", sec, 4)[0]
        crc = struct.unpack_from("<I", sec, 8)[0]
        print(f"INI-Import  @ {XIP_BASE + ini_off:#010x}: {n} Bytes, crc32={crc:#010x}"
              f" ({', '.join(t[0] for t in texts)})")
    if args.hex:
        slot, marker, length = build_firmware(args.hex)
        regions.append((XIP_BASE + marker_off, marker))
        regions.append((XIP_BASE + fw_off, slot))
        print(f"Marker      @ {XIP_BASE + marker_off:#010x}")
        print(f"Firmware    @ {XIP_BASE + fw_off:#010x}: {length} Bytes, "
              f"crc32={zlib.crc32(slot[:length]):#010x} ({', '.join(p.name for p in args.hex)})")
    print(f"(Config-Bereich @ {XIP_BASE + config_off:#010x} bleibt unberuehrt)")

    if args.uf2:
        write_uf2(args.uf2, regions)
        print(f"UF2  -> {args.uf2}")
    if args.output:
        all_regions = list(regions)
        if args.emulator:
            emu = hex_to_regions(args.emulator)
            end = max(a + len(b) for a, b in emu)
            if end > XIP_BASE + ini_off:
                die(f"Emulator-Image reicht bis {end:#x} und ueberlappt den Storage-Bereich")
            all_regions += emu
            print(f"Emulator    @ {emu[0][0]:#010x}..{end:#010x} ({args.emulator.name})")
        write_hex(args.output, all_regions)
        print(f"HEX  -> {args.output}")


if __name__ == "__main__":
    main()
