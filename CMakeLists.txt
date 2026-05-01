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

add_executable(emulator
    src/main.cpp
    src/cli.cpp
    src/config.cpp
    src/hex_parser.cpp
    src/hex_patcher.cpp
    src/storage.cpp
    src/mmu.cpp
    src/fault.cpp
    src/emulator.cpp
    src/peripherals.cpp
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
)

target_include_directories(emulator PRIVATE src)

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
    hardware_timer
    hardware_watchdog
    hardware_pio
    hardware_clocks
    tinyusb_device
    tinyusb_board
)

# stdio über USB-CDC; UART-stdio abschalten (UART-Pins gehören dem Emulator-Mapping).
pico_enable_stdio_usb(emulator 1)
pico_enable_stdio_uart(emulator 0)

pico_add_extra_outputs(emulator)
