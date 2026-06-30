import struct
PATH=r"examples/ft12_release_v0.02_libv2.10.hex"
def parse(p):
    m={};b=0
    for ln in open(p):
        ln=ln.strip()
        if not ln or ln[0]!=':':continue
        n=int(ln[1:3],16);a=int(ln[3:7],16);t=int(ln[7:9],16)
        d=bytes(int(ln[9+2*i:11+2*i],16) for i in range(n))
        if t==0:
            for i,x in enumerate(d):m[b+a+i]=x
        elif t==4:b=(d[0]<<8|d[1])<<16
        elif t==2:b=(d[0]<<8|d[1])<<4
        elif t==1:break
    return m
m=parse(PATH);size=max(m)+1
img=bytearray(0xFF for _ in range(size))
for a,x in m.items():img[a]=x
def r32(o):return struct.unpack_from('<I',img,o)[0]

print("=== Allocator-Literale (Heap-/Pool-Symbole) ===")
for o in (0x2cb4,0x2cc8,0x2d0c,0x2e0c,0x2e1c,0x2e2c,0x2e50,0x2f44,0x2f64,0x2f68,0x2f8c,0x2fcc,0x2fd0,0x2ff4):
    print(f"  [0x{o:04x}] = 0x{r32(o):08x}")

# Heap-Symbole sind oft die hoechsten RAM-Adressen. Zeige alle relozierten
# Worte >= 0x10002000 (jenseits 8KB) -- sollte KEINE geben:
print("\n=== Worte mit RAM-Wert >= 0x10002000 (jenseits 8KB) ===")
hits=[(o,r32(o)) for o in range(0,size-3) if 0x10002000<=r32(o)<0x10010000]
for o,w in hits: print(f"  0x{o:04x}: 0x{w:08x}")
print(f"  -> {len(hits)} Treffer")

# Zeige alle distinkten RAM-Pointer-Werte, die irgendwo im Image stehen,
# sortiert -- die hoechsten zeigen die RAM-Obergrenze, die die FW annimmt:
vals=sorted({r32(o) for o in range(0,size-3,1) if 0x10000000<=r32(o)<=0x10002000})
print(f"\n=== Hoechste angenommene RAM-Adressen (Top 12) ===")
for v in vals[-12:]: print(f"  0x{v:08x}")
