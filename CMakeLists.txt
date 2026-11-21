# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.21)

# ---------------------------------------------------------------------------
# LPC1115 Emulator on RP2350 (Raspberry Pi Pico 2 family / RP2354)
#
# Voraussetzung: arm-none-eabi GCC im PATH (>= 13).
# Das Pico-SDK wird, falls PICO_SDK_PATH nicht gesetzt ist, automatisch
# via FetchContent aus dem offiziellen Repo geladen. Dies funktioniert
# nur bei einer ersten Konfiguration mit Internetzugriff.
# ---------------------------------------------------------------------------

set(PICO_PLATFORM rp2350 CACHE STRING "")
set(PICO_BOARD    pico2  CACHE STRING "RP2350-basiertes Board (Pico 2 / RP2354).")
# Eigene Board-Header (boards/<name>.h), z. B. -DPICO_BOARD=selfbus_io8_rp2354.
# Optional je Board: boards/<name>.cmake (Build-Optionen) und boards/<name>.ini
# (Board-Profil = Default-Konfiguration, wird ins Image eingebettet).
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

if(NOT DEFINED ENV{PICO_SDK_PATH} AND NOT PICO_SDK_PATH)
    include(FetchContent)
    FetchContent_Declare(
        pico_sdk
        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
        GIT_TAG        2.1.0
        GIT_SUBMODULES_RECURSE TRUE
    )
    FetchContent_MakeAvailable(pico_sdk)
    set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE PATH "Pico SDK path")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(lpc1115_emulator C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

pico_sdk_init()

# --- Board-Optionen + Board-Profil ------------------------------------------
set(EMU_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR}/boards)
if(EXISTS ${EMU_BOARD_DIR}/${PICO_BOARD}.cmake)
    include(${EMU_BOARD_DIR}/${PICO_BOARD}.cmake)
endif()
option(EMU_STDIO_UART "CLI/stdio zusaetzlich auf PICO_DEFAULT_UART (Boards ohne USB)" OFF)

# Board-Profil-INI als C-String einbetten (leer, wenn keine vorhanden).
set(EMU_BOARD_INI_FILE ${EMU_BOARD_DIR}/${PICO_BOARD}.ini)
if(EXISTS ${EMU_BOARD_INI_FILE})
    file(READ ${EMU_BOARD_INI_FILE} EMU_BOARD_INI)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${EMU_BOARD_INI_FILE})
    if(EMU_BOARD_INI MATCHES "\\)EMUINI\"")
        message(FATAL_ERROR "${EMU_BOARD_INI_FILE} enthaelt den Raw-String-Begrenzer )EMUINI\"")
    endif()
    message(STATUS "Board-Profil: ${EMU_BOARD_INI_FILE}")
else()
    set(EMU_BOARD_INI "")
endif()
configure_file(${CMAKE_CURRENT_LIST_DIR}/src/board_profile.cpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/board_profile.cpp @ONLY)

add_executable(emulator
    src/main.cpp
    src/cli.cpp
    src/xmodem.cpp
    src/config.cpp
    src/hex_parser.cpp    src/hex_patcher.cpp
    src/storage.cpp
    src/mmu.cpp
    src/fault.cpp
    src/emulator.cpp
    src/peripherals.cpp
    src/ncn5130.cpp
    src/clock.cpp
    src/opcodes.cpp
    src/pio_glue.cpp
    src/vnvic.cpp
    src/gdb_stub.cpp
    src/usb_descriptors.cpp
    src/irq_inject.cpp
    src/swd_target.cpp
    src/target_halt.cpp
    src/iap.cpp
    src/usb_msc.cpp
    src/help_html.cpp
    src/usb_stdio.cpp
    src/uart_bridge.cpp
    src/led.cpp
    src/debug_bridge.cpp
    src/isp.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/board_profile.cpp
)

target_include_directories(emulator PRIVATE src)
# Board-spezifische Defines aus boards/<board>.cmake (z. B. Lage der CLI-UART).
if(EMU_BOARD_DEFINITIONS)
    target_compile_definitions(emulator PRIVATE ${EMU_BOARD_DEFINITIONS})
endif()

# PIO-Programm fuer flankengenaues Capture-Timestamping (KNX-Empfang).
# pioasm erzeugt den Header timer_edge_ts.pio.h ins Build-Verzeichnis.
pico_generate_pio_header(emulator ${CMAKE_CURRENT_LIST_DIR}/src/timer_edge_ts.pio)
pico_generate_pio_header(emulator ${CMAKE_CURRENT_LIST_DIR}/src/match_pulse.pio)


# tusb_config.h aus src/ verwenden statt SDK-Default.
target_compile_definitions(emulator PRIVATE
    PICO_STDIO_USB_USE_USB_CDC=1
)

# Strenge Warnungen, hilft Sicherheitsprobleme früh aufzudecken.
# -Wno-pedantic auf Header-Ebene des SDK über -isystem; eigene Files bleiben
# strikt. -Wpedantic global wirkt sonst auch in TinyUSB-Headern.
target_compile_options(emulator PRIVATE
    -Wall -Wextra
    -Wshadow
    -Wnull-dereference -Wdouble-promotion -Wformat=2 -Wno-format-nonliteral
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Release>:-O2>
    $<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

target_link_libraries(emulator PRIVATE
    pico_stdlib
    pico_multicore
    pico_unique_id
    pico_sync
    cmsis_core
    hardware_flash
    hardware_sync
    hardware_gpio
    hardware_uart
    hardware_i2c
    hardware_spi
    hardware_adc
    hardware_timer
    hardware_watchdog
    hardware_pio
    hardware_clocks
    hardware_powman
    tinyusb_device
    tinyusb_board
)

# stdio läuft über unseren eigenen TinyUSB-CDC-Bridge (src/usb_stdio.cpp);
# pico_stdio_usb wird bewusst NICHT verwendet, da es eine eigene
# tusb_config.h mitbringen würde, die mit unseren 2×CDC + MSC-Descriptoren
# kollidiert. UART-stdio bleibt aus.
pico_enable_stdio_usb(emulator 0)
if(EMU_STDIO_UART)
    # Zusaetzlicher stdio-Treiber auf PICO_DEFAULT_UART (Board-Header). Die CLI
    # liest/schreibt ueber stdio und ist damit auch ohne USB erreichbar.
    pico_enable_stdio_uart(emulator 1)
    target_compile_definitions(emulator PRIVATE EMU_STDIO_UART=1)
else()
    pico_enable_stdio_uart(emulator 0)
endif()

pico_add_extra_outputs(emulator)
