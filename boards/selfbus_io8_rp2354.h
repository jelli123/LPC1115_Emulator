/*
 * Board definition: Selfbus IO-8x fused 35x25 FM with RP2354A
 * (RP2350A die + 2 MB stacked flash, 12 MHz crystal) - LPC1115-Emulator.
 *
 * Abgeleitet vom Board-Header der IO-8x-Testsoftware. Die Standard-UART
 * (stdio/CLI des Emulators) waehlt boards/selfbus_io8_rp2354.cmake per
 * EMU_CLI_UART: jp7 (Default, ISP-Pads GPIO16/17, uart0) oder jp6 (DBG,
 * GPIO4/5, uart1); die Defines unten sind nur der Fallback (jp6).
 * Keine Status-LED: GPIO25 ist auf diesem Board EIB_IN.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _BOARDS_SELFBUS_IO8_RP2354_H
#define _BOARDS_SELFBUS_IO8_RP2354_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define SELFBUS_IO8_RP2354
#define PICO_RP2350A 1

// --- stdio/CLI-Fallback: uart1 auf JP6 (EMU_CLI_UART setzt die Werte) -------
//   GPIO4 = DBG_TXD -> JP6 pin 1
//   GPIO5 = DBG_RXD -> JP6 pin 2
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 1
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 4
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 5
#endif
#ifndef PICO_DEFAULT_UART_BAUD_RATE
#define PICO_DEFAULT_UART_BAUD_RATE 115200
#endif

// --- I2C1: GPIO18 = SDA, GPIO19 = SCL (FRAM U1, JP1, JP5) -----------------
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 18
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 19
#endif

// Keine PICO_DEFAULT_LED_PIN: Prog-LED (GPIO2) gehoert dem Gast (P2_0),
// GPIO25 ist EIB_IN.

// --- Flash: 2 MB in package --------------------------------------------------
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif
pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// ABM8 crystal: give the XOSC a bit more start-up time
#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
