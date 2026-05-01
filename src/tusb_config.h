#pragma once
//
// TinyUSB-Konfiguration für 2 CDC-Interfaces (CLI + GDB).
// Diese Datei MUSS im Include-Pfad von TinyUSB als `tusb_config.h`
// gefunden werden — wir setzen das in CMakeLists.txt mit
// target_include_directories(... PRIVATE src/) und stellen sicher, dass
// das Pico-SDK-Default-tusb_config.h überschrieben wird.
//

#define CFG_TUSB_MCU              OPT_MCU_RP2040  /* RP2350-Treiber teilt USB-IP */
#define CFG_TUSB_OS               OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE

#define CFG_TUD_ENABLED           1
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_CDC               2     /* zwei CDC-Endpoints */
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    256
#define CFG_TUD_CDC_EP_BUFSIZE    64

#define CFG_TUD_HID               0
#define CFG_TUD_MSC               1
#define CFG_TUD_MSC_EP_BUFSIZE    512
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0
