import struct
PATH = r"examples/ft12_release_v0.02_libv2.10.hex"
def parse_ihex(path):
    mem={};base=0
    for ln in open(path):
        ln=ln.strip()
        if not ln or ln[0]!=':':continue
        n=int(ln[1:3],16);addr=int(ln[3:7],16);rt=int(ln[7:9],16)
        data=bytes(int(ln[9+2*i:11+2*i],16) for i in range(n))
        if rt==0:
            for i,b in enumerate(data):mem[base+addr+i]=b
        elif rt==4:base=(data[0]<<8|data[1])<<16
        elif rt==2:base=(data[0]<<8|data[1])<<4
        elif rt==1:break
    return mem
mem=parse_ihex(PATH);size=max(mem)+1
img=bytearray(0xFF for _ in range(size))
for a,b in mem.items():img[a]=b
def rd32(o):return struct.unpack_from('<I',img,o)[0]

print("=== .data Init-Image: Flash 0x31c4..0x3234 -> RAM 0x10000000..0x10000070 ===")
for o in range(0x31c4, 0x31c4+0x70, 4):
    ram = 0x10000000 + (o-0x31c4)
    w = rd32(o)
    note=""
    if 0x10000000 <= w <= 0x10002000: note=" <-- RAM-Ptr (würde reloziert)"
    elif 0x100<= (w&~1) <=0x3232 and (w&1): note=" (Thumb-Code-Ptr?)"
    print(f"  flash 0x{o:04x} = RAM 0x{ram:08x}: 0x{w:08x}{note}")

# Was steht bei 0x3000..0x31c4 (zwischen Code-Ende und .data-Quelle)?
print("\n=== Vtables/Rodata 0x3000..0x31c4 (Wortweise, Thumb-Ptr markiert) ===")
for o in range(0x3000, 0x31c4, 4):
    w=rd32(o)
    tag=""
    if (w&1) and 0x100<=(w&~1)<=0x3232: tag=" code"
    elif 0x10000000<=w<=0x10002000: tag=" RAMptr"
    print(f"  0x{o:04x}: 0x{w:08x}{tag}")
