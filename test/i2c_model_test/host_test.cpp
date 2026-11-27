// Host-Test der I2C-Zustandsmaschine (src/peripherals.cpp) gegen die Master-
// Logik der sblib (handleMasterXferState, Polling ohne Timeout) mit Fake-Slave.
// model_part.inc erzeugt run.sh aus src/peripherals.cpp.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
constexpr uint32_t I2C_BASE=0x40000000, I2C_CONSET=I2C_BASE, I2C_STAT=I2C_BASE+4, I2C_DAT=I2C_BASE+8, I2C_CONCLR=I2C_BASE+0x18;
constexpr uint32_t I2C_AA=4,I2C_SI=8,I2C_STO=0x10,I2C_STA=0x20,I2C_I2EN=0x40;
namespace lpc_irq { constexpr int I2C0=15; }
namespace irq_inject { inline void pend(int){} }
struct I2cModel { uint32_t conset, conclr, stat, dat; uint8_t addr7; bool is_read, in_txn, pending, addr_done, rfirst; uint8_t wbuf[64]; uint8_t wlen; };
I2cModel g_i2c{};
bool g_i2c_ready=true;
// Fake-Slave
int slave_addr=0x44; bool slave_present=true; std::vector<uint8_t> writes; uint8_t rdata[6]={1,2,3,4,5,6}; int rpos=0; int busy_reads=0;
bool i2c_flush_write(bool){ uint8_t n=g_i2c.wlen; g_i2c.wlen=0; if(!n) return true; if(!slave_present||g_i2c.addr7!=slave_addr) return false; writes.insert(writes.end(),g_i2c.wbuf,g_i2c.wbuf+n); return true; }
bool i2c_hw_read_byte(bool last, uint8_t& out){ bool first=g_i2c.rfirst; g_i2c.rfirst=false;
  if(!slave_present||g_i2c.addr7!=slave_addr) return false;
  if(first){ if(busy_reads>0){busy_reads--; return false;} rpos=0; }
  out=rdata[rpos++]; (void)last; return true; }
void i2c_hw_release(){}
#include "model_part.inc"
void wr(uint32_t a,uint32_t v){ for(int i=0;i<4;i++) i2c_write_byte(a+i,(v>>(8*i))&0xFF); }
uint32_t rd(uint32_t a){ uint32_t v=0; for(int i=0;i<4;i++) v|=uint32_t(i2c_read_byte(a+i))<<(8*i); return v; }
// --- sblib (1:1 Logik aus i2c.cpp) ---
enum { BUSY, DONE, NAK, ARBLOST, BUSERR };
struct Xfer { uint8_t slaveAddr; const uint8_t* txBuff; int txSz; uint8_t* rxBuff; int rxSz; int status; };
#define I2C_CON_FLAGS (I2C_AA|I2C_SI|I2C_STO|I2C_STA)
int handle(Xfer* x){ uint32_t cclr=I2C_CON_FLAGS;
 switch(rd(I2C_STAT)&0xF8){
 case 0x08: case 0x10: wr(I2C_DAT,(x->slaveAddr<<1)|(x->txSz==0)); break;
 case 0x18: case 0x28: if(!x->txSz) cclr&=~(x->rxSz?I2C_STA:I2C_STO); else { wr(I2C_DAT,*x->txBuff++); x->txSz--; } break;
 case 0x58: cclr&=~I2C_STO; /*fall*/
 case 0x50: *x->rxBuff++=rd(I2C_DAT); x->rxSz--; /*fall*/
 case 0x40: if(x->rxSz>1) cclr&=~I2C_AA; break;
 case 0x20: case 0x30: case 0x48: x->status=NAK; cclr&=~I2C_STO; break;
 case 0x38: x->status=ARBLOST; break;
 case 0x00: x->status=BUSERR; cclr&=~I2C_STO; break; }
 wr(I2C_CONSET,cclr^I2C_CON_FLAGS); wr(I2C_CONCLR,cclr);
 if(!(cclr&I2C_STO)||x->status==ARBLOST){ if(x->status==BUSY) x->status=DONE; return 0;} return 1; }
int transfer(Xfer* x){ x->status=BUSY;
 wr(I2C_CONCLR,I2C_SI|I2C_STO|I2C_STA|I2C_AA); wr(I2C_CONSET,I2C_I2EN|I2C_STA);
 long guard=0; while(x->status==BUSY){ if(rd(I2C_CONSET)&I2C_SI) handle(x); if(++guard>100000){ puts("HANG"); return -1; } }
 guard=0; while(rd(I2C_CONSET)&I2C_STO){ if(++guard>100000){ puts("HANG STO"); return -1; } }
 return x->status; }
int send(uint8_t a,const uint8_t* b,int n){ Xfer x{a,b,n,nullptr,0,0}; transfer(&x); return n-x.txSz; }
int read(uint8_t a,uint8_t* b,int n){ Xfer x{a,nullptr,0,b,n,0}; transfer(&x); return n-x.rxSz; }
int cmdread(uint8_t a,uint8_t c,uint8_t* b,int n){ Xfer x{a,&c,1,b,n,0}; int s=transfer(&x); return s==DONE? n-x.rxSz : -s; }
int fails=0; void ok(const char*n,bool c){ printf("%s %s\n",c?"ok  ":"FAIL",n); if(!c) fails++; }
int main(){
 g_i2c.stat=0xF8; i2c_reset_txn();
 uint8_t cmd=0x94; ok("send mit Slave", send(0x44,&cmd,1)==1 && writes.size()==1 && writes[0]==0x94);
 uint8_t b[6]={}; ok("read 6 mit Slave", read(0x44,b,6)==6 && !memcmp(b,rdata,6));
 busy_reads=3; int tries=0,n; do { n=read(0x44,b,6); tries++; } while(n==0 && tries<10);
 ok("read NAK während Messung, dann Daten", n==6 && tries==4);
 ok("cmdread (write+RepStart+read)", cmdread(0x44,0xFD,b,6)==6 && writes.back()==0xFD);
 slave_present=false;
 ok("send ohne Slave hängt nicht", send(0x44,&cmd,1)>=0);
 ok("read ohne Slave -> 0 Bytes", read(0x44,b,6)==0);
 ok("cmdread ohne Slave -> NAK", cmdread(0x44,0xFD,b,6)==-NAK);
 g_i2c_ready=false;
 ok("ohne Bridge: send -> NAK, kein Hängen", send(0x44,&cmd,1)==0);
 ok("ohne Bridge: read -> 0", read(0x44,b,6)==0);
 ok("Ruhezustand F8 am Ende", (rd(I2C_STAT)&0xF8)==0xF8 && !(rd(I2C_CONSET)&(I2C_SI|I2C_STO|I2C_STA)));
 printf(fails?"%d FAIL\n":"alle ok\n",fails); return fails; }
