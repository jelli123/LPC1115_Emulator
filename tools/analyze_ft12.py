import sys

PATH = r"examples/ft12_release_v0.02_libv2.10.hex"

def parse_ihex(path):
    mem = {}
    base = 0
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln[0] != ':':
                continue
            n = int(ln[1:3], 16)
            addr = int(ln[3:7], 16)
            rectype = int(ln[7:9], 16)
            data = bytes(int(ln[9+2*i:11+2*i], 16) for i in range(n))
            if rectype == 0:
                for i, b in enumerate(data):
                    mem[base + addr + i] = b
            elif rectype == 4:
                base = (data[0] << 8 | data[1]) << 16
            elif rectype == 2:
                base = (data[0] << 8 | data[1]) << 4
            elif rectype == 1:
                break
    return mem

mem = parse_ihex(PATH)
lo = min(mem)
hi = max(mem)
size = hi - lo + 1
print(f"image: 0x{lo:08x}..0x{hi:08x}  size={size} ({size} bytes)")

# Build a flat array (fill gaps with 0xFF)
img = bytearray(0xFF for _ in range(size))
for a, b in mem.items():
    img[a - lo] = b

def rd32(off):
    return img[off] | img[off+1] << 8 | img[off+2] << 16 | img[off+3] << 24

def rd16(off):
    return img[off] | img[off+1] << 8

# Vector table
print("\n-- Vektortabelle (erste 16 Worte) --")
names = ["SP","Reset","NMI","HardFault","r4","r5","r6","SVC?","r8","r9","r10","SVC","r12","r13","PendSV","SysTick"]
for i in range(16):
    print(f"  [{i:2}] {names[i]:9} = 0x{rd32(i*4):08x}")

# Scan words the relocator WOULD relocate: [0x10000000, 0x10002000]
RAM_LO = 0x10000000
RAM_HI = 0x10002000
print("\n-- Worte in [0x10000000, 0x10002000] (Relokationskandidaten) --")
cands = []
for off in range(0, size - 3, 4):
    w = rd32(off)
    if RAM_LO <= w <= RAM_HI:
        cands.append((off, w))
print(f"  Anzahl: {len(cands)}")
for off, w in cands:
    print(f"    off=0x{off:05x}  w=0x{w:08x}")

# Search for any reference to 0x10003820 / 0x10003821 / 0x1820 patterns
print("\n-- Suche nach 0x10003820 / 0x10003821 als 32-bit-Wort --")
for off in range(0, size - 3):
    w = rd32(off)
    if w in (0x10003820, 0x10003821, 0x10003800, 0x10003824):
        print(f"    off=0x{off:05x} (aligned={off%4==0})  w=0x{w:08x}")

# Search any word in extended RAM (0x10002000, 0x10010000) - beyond 8K
print("\n-- Worte in (0x10002000, 0x10010000] (RAM jenseits 8K) --")
cnt = 0
for off in range(0, size - 3, 4):
    w = rd32(off)
    if 0x10002000 < w <= 0x10010000:
        print(f"    off=0x{off:05x}  w=0x{w:08x}")
        cnt += 1
print(f"  Anzahl: {cnt}")
