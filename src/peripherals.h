#pragma once
//
// LPC1115-Peripherie-Modelle. Alle MMIO-Zugriffe der Gast-Firmware werden
// über mmio_read8/mmio_write8 geleitet (vom Trap-Handler aus aufgerufen).
//

#include <cstdint>

namespace peripherals {

void init();
void reset();

// (Re-)Initialisiert die I²C-Hardware-Bridge anhand der aktuellen config.
// Wird nach dem Einlesen von CONFIG.INI aufgerufen, damit die Bridge auch
// ohne Power-Cycle aktiv wird.
void i2c_bridge_reinit();

// (Re-)Initialisiert die SPI- bzw. ADC-Hardware-Bridge anhand der aktuellen
// config (analog zu i2c_bridge_reinit, nach dem Einlesen von CONFIG.INI).
void spi_bridge_reinit();
void adc_bridge_reinit();
void ct_bridge_reinit();

// Vom Trap-Handler nach erfolgter Emulation aufgerufen — Stats/PLL-Folgen.
void on_post_write_hook();

bool mmio_read8 (uint32_t addr, uint8_t&  out);
bool mmio_write8(uint32_t addr, uint8_t   val);

// Aktuelle resultierende CPU-Frequenz, die per SYSCON-Schreibvorgängen
// des Gastes auf dem RP2350 eingestellt wurde (0 = Default 48 MHz IRC).
uint32_t current_cpu_hz();

// Liest echte Eingänge, erkennt Flanken und pendet PINT-/GINT-IRQs.
// Normalerweise aus dem MMIO-Trap aufgerufen; bei aktivem WFI-Pin-Wakeup
// auch aus dem SVC-Warte-Handler (src/emulator.cpp), damit eine reine
// WFI-Warteschleife durch echte Pin-Flanken geweckt werden kann.
void sample_pin_interrupts();

// Treibt die zeitbasierten Modelle (CT16/CT32-Timer, WWDT) lazy weiter und
// pendet fällige IRQs. Wird aus dem WFI-Warte-Handler periodisch gepollt,
// da diese Modelle sonst nur bei MMIO-Zugriffen voranschreiten.
void poll_timed_sources();

// True, sobald mindestens ein Timer-Capture-Kanal scharf ist (CCR-Flanke
// aktiv und Pin gebunden). Der WFI-Warte-Handler pollt dann eng (ohne die
// 50-µs-Pause), damit KNX-Busflanken (~104 µs/Bit) zeitlich aufgelöst werden.
bool capture_armed();

struct Stats {
    uint64_t mmio_writes;
    uint64_t mmio_reads;
    uint64_t gpio_writes;
    uint64_t pll_reconfigs;
    uint64_t nvic_writes;
};
Stats stats();

// SysTick-Diagnose: Trap-Zaehler (Reads/Writes auf 0xE000E010-01F) + aktuelle
// emulierte Registerwerte. Zeigt, ob der unprivilegierte Gast-SysTick-Zugriff
// ueberhaupt getrappt wird und ob CTRL (ENABLE|TICKINT) korrekt gesetzt ist.
void systick_debug(uint32_t& reads, uint32_t& writes,
                   uint32_t& csr, uint32_t& rvr, uint32_t& cvr,
                   uint64_t& ticks);

// Host-SysTick-"Alarm" (adaptiver Core1-SysTick als Zeitbasis). Vom Host-Shim
// (emulator.cpp isr_systick_shim) genutzt:
//   systick_hw_rearm()        - realen SysTick auf naechsten Deadline programmieren
//   systick_take_guest_ticks() - faellige Gast-SysTick-Perioden abholen (Handler-Nachhol)
void     systick_hw_rearm();
uint32_t systick_take_guest_ticks();

// Liefert true, wenn der LPC-Pin (port 0..3, pin 0..11) vom Gast als AUSGANG
// konfiguriert ist; dann steht in 'level' der zuletzt geschriebene Pegel (aus
// dem GPIO-Schatten g_gpio). Fuer Core0 (LED-Poll), um die Onboard-LED die
// Gast-Blink-LED (PIO0.7) spiegeln zu lassen. Cross-Core-Read ist benign
// (32-bit-aligned, hoechstens ein Update Verzug).
bool guest_output_level(uint8_t port, uint8_t pin, bool& level);

// Core0-Seite der virtuellen UART0<->CDC#2-Kopplung (config uart0_cdc). Von der
// USB-Seite (uart_bridge::uart0_cdc_poll auf Core0) aufgerufen.
bool uart0_cdc_tx_pop(uint8_t& b);   // Gast-TX-Byte holen (false = leer)
void uart0_cdc_rx_push(uint8_t b);   // von CDC#2 empfangenes Byte an Gast-RX

// UART0-Diagnose (via 'uart status'). Zeigt, WO der RX-Interrupt-Pfad bricht:
//   ier        - aktueller IER-Schatten (Bit0=RBR-IRQ, Bit1=THRE-IRQ). 0 = Gast
//                hat serial.begin() noch nicht erreicht.
//   nvic_en    - UART0 (IRQ21) im vNVIC enabled (Gast rief NVIC_EnableIRQ).
//   rx_irq_pends - wie oft UART0-RX-IRQ gepended wurde (Datenzustellung angestossen).
//   rbr_reads  - wie oft der Gast das RBR gelesen hat (=Handler holt Bytes ab).
//   tx_writes  - wie oft der Gast ins THR geschrieben hat (=Handler sendet).
void uart0_debug(uint8_t& ier, bool& nvic_en, uint32_t& rx_irq_pends,
                 uint32_t& rbr_reads, uint32_t& tx_writes);

// Letzter getrappter MMIO-Zugriff (Adresse + R/W). Die Adresse identifiziert das
// zuletzt beruehrte LPC-Peripherie-Register. Aendert sie sich zwischen zwei
// Abfragen nicht, steht der Gast in einer Nicht-MMIO-Schleife an dieser Stelle.
void last_mmio(uint32_t& addr, bool& is_write);

// Diagnose des CT-Timer-Advance: Anzahl gegriffener Underflow-Schutzfaelle
// (last_us>now, z. B. nach time_us_64-Diskontinuitaet durch Gast-PLL-Reconfig)
// + groesster je berechneter Tick-Wert. underflow_guards>0 erklaert einen
// fruehreren ct_advance-Endlos-Loop.
void ct_advance_debug(uint32_t& underflow_guards, uint64_t& max_ticks);

// Diagnose der HW-uart0-Initialisierung: Enter/Exit-Zaehler um den (potenziell
// blockierenden) SDK-uart_init(). init_enter>init_exit = uart_init haengt gerade
// (Busy-Wait im Fault-Handler-Kontext -> Core1 blockiert, SysTick-Shim steht).
// Zusaetzlich reale Boot-Taktwerte + uart_init-Rueckgabe, um den cr=0x000-Fall
// (UART HW-deaktiviert) zu erklaeren: clk_peri_hz/ctrl (Bit11 ENABLE) zeigen, ob
// der UART-Takt real laeuft; uart0_init_ret=0 = uart_init nahm den Early-Return.
void uart0_init_debug(uint32_t& init_enter, uint32_t& init_exit,
                      uint32_t& clk_sys_hz, uint32_t& clk_peri_hz,
                      uint32_t& clk_peri_ctrl, uint32_t& uart0_init_ret);

// uart0->cr / ->lcr_h wie sie DIREKT nach uart_init(uart0) im Boot waren, plus
// das LIVE RESETS-Register (Bit26=uart0, Bit27=uart1; 1 = Block wird IM RESET
// gehalten -> Registerschreibzugriffe verpuffen, cr/lcr_h lesen 0). u0_cr_wb =
// uart0->cr nach explizitem Direktschreiben (Test: nimmt uart0 Writes an?);
// u1_cr_boot = uart1->cr (Referenz, uart1 laeuft).
void uart0_boot_regs(uint32_t& cr_boot, uint32_t& lcrh_boot, uint32_t& resets_now,
                     uint32_t& u0_cr_wb, uint32_t& u1_cr_boot);

// Live-RX-Zustand der LPC-UART0 (LSR.DR): true = meldet gerade Empfangsdaten.
// Zeigt, ob eine haengende serial.begin()-Drain-Schleife echte Bytes sieht.
bool uart0_rx_live();

// Pad-Diagnose der HW-UART0-Bruecke: liest den REALEN RP2350-Pad-Zustand der
// gerouteten uart0-Pins (statt Emulator-Schatten). Damit laesst sich MESSEN, ob
// TX/RX korrekt auf GPIO_FUNC_UART (=2) liegen und welchen Pegel die RX-Leitung
// fuehrt — entscheidend fuer die Loopback-/FT12-Diagnose.
//   hw_sel  : 0=uart0, 1=uart1, -1=kein HW-Backing (idle/CDC)
//   tx_gpio/rx_gpio: aktuell geroutete RP2350-Pads (-1 = keins)
//   tx_func/rx_func: reale Pad-Funktion (gpio_get_function); 2=GPIO_FUNC_UART
//   rx_level: realer Eingangspegel des RX-Pads (0=LOW -> Dauer-0x00-Frames)
//   uart_cr : uart0/1-Control-Register (Bit0 UARTEN, Bit8 TXE, Bit9 RXE)
//   uart_lcr_h: uart0/1-Leitungsformat (WLEN Bits6:5, STP2 Bit3, PEN Bit1,
//               EPS Bit2, FEN Bit4). 8E1+FIFO = 0x76.
void uart0_pad_debug(int& hw_sel, int& tx_gpio, int& rx_gpio,
                     int& tx_func, int& rx_func, int& rx_level,
                     uint32_t& uart_cr, uint32_t& uart_lcr_h);

} // namespace peripherals
