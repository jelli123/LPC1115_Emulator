# Board-spezifische Build-Optionen: Selfbus IO-8x fused 35x25 FM (RP2354A).
# Kein USB am Modul -> CLI zusaetzlich auf einer UART (per -DEMU_STDIO_UART=OFF
# abschaltbar).
if(NOT DEFINED EMU_STDIO_UART)
    set(EMU_STDIO_UART ON)
endif()

# Lage der CLI-UART:
#   jp7 (Default): ISP_TXD/ISP_RXD auf JP7 (GPIO16/17, uart0) - gleicher 10-pol.
#                  Stecker wie SWD, eine Leitung zur Debug-Probe (z. B. dap42).
#                  Gast-UART0 und ISP ueber Pins sind dann nicht verfuegbar
#                  (Firmware per 'upload' (XMODEM) oder SWD/scripts/mkimage.py).
#   jp6:           DBG_TXD/DBG_RXD auf JP6 (GPIO4/5, uart1). JP7 bleibt fuer
#                  Gast-UART0 und den emulierten LPC-ISP frei.
set(EMU_CLI_UART "jp7" CACHE STRING "CLI-UART des IO-8x: jp7 (GPIO16/17) oder jp6 (GPIO4/5)")
set_property(CACHE EMU_CLI_UART PROPERTY STRINGS jp7 jp6)
if(EMU_CLI_UART STREQUAL "jp7")
    list(APPEND EMU_BOARD_DEFINITIONS
        PICO_DEFAULT_UART=0 PICO_DEFAULT_UART_TX_PIN=16 PICO_DEFAULT_UART_RX_PIN=17)
elseif(EMU_CLI_UART STREQUAL "jp6")
    list(APPEND EMU_BOARD_DEFINITIONS
        PICO_DEFAULT_UART=1 PICO_DEFAULT_UART_TX_PIN=4 PICO_DEFAULT_UART_RX_PIN=5)
else()
    message(FATAL_ERROR "EMU_CLI_UART muss jp7 oder jp6 sein (ist: ${EMU_CLI_UART})")
endif()
message(STATUS "IO-8x: CLI-UART auf ${EMU_CLI_UART}")
