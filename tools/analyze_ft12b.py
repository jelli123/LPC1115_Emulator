import struct

PATH = r"examples/ft12_release_v0.02_libv2.10.hex"

def parse_ihex(path):
    mem = {}
    base = 0
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln[0] != ':':
                continue
            n = int(ln[1:3], 16); addr = int(ln[3:7], 16); rt = int(ln[7:9], 16)
            data = bytes(int(ln[9+2*i:11+2*i], 16) for i in range(n))
            if rt == 0:
                for i, b in enumerate(data): mem[base+addr+i] = b
            elif rt == 4: base = (data[0]<<8|data[1])<<16
            elif rt == 2: base = (data[0]<<8|data[1])<<4
            elif rt == 1: break
    return mem

mem = parse_ihex(PATH)
size = max(mem)+1
img = bytearray(0xFF for _ in range(size))
for a,b in mem.items(): img[a]=b
def rd32(o): return struct.unpack_from('<I', img, o)[0]

print("Literale der Scatter-Region-Tabelle:")
for o in (0x248,0x24c,0x250,0x254,0x258):
    print(f"  [0x{o:04x}] = 0x{rd32(o):08x}")

# Region table is between [0x248] and [0x24c]
rt_start = rd32(0x248)
rt_end   = rd32(0x24c)
print(f"\nRegion-Tabelle: 0x{rt_start:04x}..0x{rt_end:04x}  ({(rt_end-rt_start)//16} Eintraege a 16B)")
o = rt_start
while o + 16 <= rt_end:
    src, dst, sz, fn = rd32(o), rd32(o+4), rd32(o+8), rd32(o+12)
    print(f"  @0x{o:04x}: src=0x{src:08x} dst=0x{dst:08x} size=0x{sz:08x} fn=0x{fn:08x}")
    o += 16

# Relocated offsets from previous scan
RAM_LO, RAM_HI = 0x10000000, 0x10002000
reloc = [o for o in range(0, size-3, 4) if RAM_LO <= rd32(o) <= RAM_HI and o != 0]
print(f"\nRelozierte Offsets gesamt: {len(reloc)}")
# How many fall at/after 0x2654 (likely .data init image start)?
for thresh in (rt_start, 0x2654, 0x2c00):
    after = [o for o in reloc if o >= thresh]
    print(f"  >= 0x{thresh:04x}: {len(after)} -> {[hex(x) for x in after]}")
