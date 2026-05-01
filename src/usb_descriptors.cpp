#include "tusb.h"
#include "pico/unique_id.h"

#include <cstring>

// Eigene USB-Descriptoren mit ZWEI CDC-Interfaces:
//   CDC #0 → stdio / CLI
//   CDC #1 → GDB Remote Serial Protocol (gdb_stub.cpp)
//
// Pico-SDK lässt mit TINYUSB_OPT_USE_CUSTOM_USBD_DESCRIPTORS=1 (in
// CMakeLists per pico_enable_stdio_usb(...) und Override) eigene
// Descriptoren zu. Wir folgen dem TinyUSB-CDC-Dual-Beispiel.

#define USB_VID   0xCAFE
#define USB_PID   0x4011
#define USB_BCD   0x0200

enum {
    ITF_NUM_CDC0_NOTIF = 0,
    ITF_NUM_CDC0_DATA,
    ITF_NUM_CDC1_NOTIF,
    ITF_NUM_CDC1_DATA,
    ITF_NUM_MSC,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC0_NOTIF  0x81
#define EPNUM_CDC0_OUT    0x02
#define EPNUM_CDC0_IN     0x82
#define EPNUM_CDC1_NOTIF  0x83
#define EPNUM_CDC1_OUT    0x04
#define EPNUM_CDC1_IN     0x84
#define EPNUM_MSC_OUT     0x05
#define EPNUM_MSC_IN      0x85

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const* tud_descriptor_device_cb(void) {
    return reinterpret_cast<uint8_t const*>(&desc_device);
}

static uint8_t const desc_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0_NOTIF, 4, EPNUM_CDC0_NOTIF, 8,
                       EPNUM_CDC0_OUT, EPNUM_CDC0_IN, 64),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1_NOTIF, 5, EPNUM_CDC1_NOTIF, 8,
                       EPNUM_CDC1_OUT, EPNUM_CDC1_IN, 64),

    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 6, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t /*idx*/) {
    return desc_fs_configuration;
}

static char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },        // 0: en-US
    "Selfbus",                            // 1: Manufacturer
    "LPC1115-Emulator on RP2350",         // 2: Product
    nullptr,                              // 3: Serial (dynamisch)
    "LPC-Emu CLI",                        // 4: CDC #0 name
    "LPC-Emu GDB",                        // 5: CDC #1 name
    "LPC-Emu MSC",                        // 6: MSC name
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t /*langid*/) {
    uint8_t chr_count = 0;

    if (index == 0) {
        std::memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == 3) {
        char serial[32];
        pico_get_unique_board_id_string(serial, sizeof serial);
        size_t len = std::strlen(serial);
        if (len > 31) len = 31;
        for (size_t i = 0; i < len; ++i) _desc_str[1 + i] = serial[i];
        chr_count = static_cast<uint8_t>(len);
    } else if (index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])
               && string_desc_arr[index] != nullptr) {
        const char* s = string_desc_arr[index];
        size_t len = std::strlen(s);
        if (len > 31) len = 31;
        for (size_t i = 0; i < len; ++i) _desc_str[1 + i] = s[i];
        chr_count = static_cast<uint8_t>(len);
    } else {
        return nullptr;
    }

    _desc_str[0] = static_cast<uint16_t>(
        (TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
