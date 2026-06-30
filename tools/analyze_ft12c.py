import struct

PATH = r"examples/ft12_release_v0.02_libv2.10.hex"

def parse_ihex(path):
    mem = {}; base = 0
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln[0] != ':': continue
            n=int(ln[1:3],16); addr=int(ln[3:7],16); rt=int(ln[7:9],16)
            data=bytes(int(ln[9+2*i:11+2*i],16) for i in range(n))
            if rt==0:
                for i,b in enumerate(data): mem[base+addr+i]=b
            elif rt==4: base=(data[0]<<8|data[1])<<16
            elif rt==2: base=(data[0]<<8|data[1])<<4
            elif rt==1: break
    return mem

mem = parse_ihex(PATH)
size = max(mem)+1
img = bytearray(0xFF for _ in range(size))
for a,b in mem.items(): img[a]=b
def rd32(o): return struct.unpack_from('<I', img, o)[0]

print("=== Region-Tabelle roh 0xc0..0xe0 ===")
for o in range(0xc0, 0xe0, 4):
    print(f"  [0x{o:04x}] = 0x{rd32(o):08x}")

print("\n=== Vektoren 0..0x40 ===")
for o in range(0, 0x40, 4):
    print(f"  vec[{o//4:2d}] @0x{o:02x} = 0x{rd32(o):08x}")

# Suche: ist 0x1820/0x1821 eine echte Funktion? Zeige Bytes drumherum
print("\n=== Bytes um 0x1818..0x1830 ===")
for o in range(0x1818, 0x1830, 2):
    print(f"  0x{o:04x}: {img[o]:02x}{img[o+1]:02x}  (hw=0x{struct.unpack_from('<H',img,o)[0]:04x})")

# Suche alle Worte == 0x10003820 / 0x10003821 (sollte 0 sein)
for target in (0x10003820, 0x10003821, 0x10001820, 0x10001821):
    hits=[o for o in range(0,size-3) if rd32(o)==target]
    print(f"\nWorte == 0x{target:08x}: {[hex(h) for h in hits]}")

# Finde Vtable-Kandidaten: 3+ aufeinanderfolgende Worte, die ungerade Thumb-Code-Ptr sind (0x100..0x3233)
print("\n=== Vtable-Kandidaten (>=3 aufeinanderf. Thumb-Ptr, 4-aligned) ===")
def is_code(w): return (w & 1)==1 and 0x100 <= (w & ~1) <= 0x3232
o=0
while o < size-3:
    if is_code(rd32(o)):
        run=[]; p=o
        while p<size-3 and is_code(rd32(p)):
            run.append(rd32(p)); p+=4
        if len(run)>=3:
            print(f"  @0x{o:04x}: "+" ".join(f"0x{w:06x}" for w in run))
        o=p
    else:
        o+=4
