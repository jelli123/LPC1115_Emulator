#include "tusb.h"
#include "pico/unique_id.h"
#include "usb_descriptors.h"
#include "config.h"

#include <cstring>

// Composite-USB mit bis zu DREI CDC-Interfaces + MSC. Welche CDCs erscheinen,
// bestimmt die CONFIG.INI (cli_enable/gdb_enable/serial_enable). Deaktivierte
// CDCs fallen komplett aus dem Konfigurations-Deskriptor -> der Host zeigt einen
// COM-Port weniger. Das MSC-Laufwerk ist IMMER dabei (Recovery-Pfad ueber
// CONFIG.INI). Der Deskriptor wird zur Bootzeit dynamisch gebaut (usb_desc_build,
// vor tusb_init). Die Reihenfolge der aktiven CDCs ist fix: CLI, GDB, Serial;
// ihre tud_cdc_n_*-Instanzindizes vergibt der Builder entsprechend.

#define USB_VID   0xCAFE
#define USB_PID   0x4012
#define USB_BCD   0x0200

// String-Indizes (Reihenfolge in string_desc_arr unten).
enum {
    STRID_LANG = 0, STRID_MANUF, STRID_PRODUCT, STRID_SERIAL,
    STRID_CLI, STRID_GDB, STRID_MSC, STRID_SERIALADP,
};

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

// --- Dynamischer Konfigurations-Deskriptor --------------------------------
namespace {

// Puffer gross genug fuer 3x CDC + MSC + Header.
uint8_t  g_cfg_desc[TUD_CONFIG_DESC_LEN + 3 * TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN];
uint16_t g_cfg_len = 0;

// CDC-Instanzindizes je Rolle (-1 = deaktiviert). MSC ist immer dabei.
int g_cdc_cli    = -1;
int g_cdc_gdb    = -1;
int g_cdc_serial = -1;
int g_cdc_count  =  0;
bool g_built = false;

// Fuegt einen CDC-Funktionsblock an und vergibt Interface-/Endpoint-Nummern.
// itf/ep_num werden fortgeschrieben. Endpoint-Schema wie zuvor: notif = 0x80|n,
// dann n++, data-out = n, data-in = 0x80|n, dann n++.
void append_cdc(uint32_t& pos, uint8_t& itf, uint8_t& ep_num, uint8_t str_idx) {
    uint8_t ep_notif = static_cast<uint8_t>(0x80u | ep_num); ++ep_num;
    uint8_t ep_out   = ep_num;
    uint8_t ep_in    = static_cast<uint8_t>(0x80u | ep_num); ++ep_num;
    uint8_t blk[TUD_CDC_DESC_LEN] = {
        TUD_CDC_DESCRIPTOR(itf, str_idx, ep_notif, 8, ep_out, ep_in, 64)
    };
    std::memcpy(g_cfg_desc + pos, blk, sizeof blk);
    pos += TUD_CDC_DESC_LEN;
    itf += 2;
}

} // namespace

void usb_desc_build() {
    g_cdc_cli = g_cdc_gdb = g_cdc_serial = -1;
    g_cdc_count = 0;

    const bool cli = config::cli_enabled();
    const bool gdb = config::gdb_enabled();
    const bool ser = config::serial_cdc_enabled();
    int idx = 0;
    if (cli) g_cdc_cli    = idx++;
    if (gdb) g_cdc_gdb    = idx++;
    if (ser) g_cdc_serial = idx++;
    g_cdc_count = idx;

    const uint8_t itf_total = static_cast<uint8_t>(g_cdc_count * 2 + 1); // +MSC
    uint32_t pos = TUD_CONFIG_DESC_LEN;   // Header spaeter (braucht total_len)
    uint8_t  itf = 0;
    uint8_t  ep  = 1;

    if (cli) append_cdc(pos, itf, ep, STRID_CLI);
    if (gdb) append_cdc(pos, itf, ep, STRID_GDB);
    if (ser) append_cdc(pos, itf, ep, STRID_SERIALADP);

    // MSC: OUT = ep, IN = 0x80|ep.
    {
        uint8_t msc_out = ep;
        uint8_t msc_in  = static_cast<uint8_t>(0x80u | ep);
        uint8_t blk[TUD_MSC_DESC_LEN] = {
            TUD_MSC_DESCRIPTOR(itf, STRID_MSC, msc_out, msc_in, 64)
        };
        std::memcpy(g_cfg_desc + pos, blk, sizeof blk);
        pos += TUD_MSC_DESC_LEN;
    }

    g_cfg_len = static_cast<uint16_t>(pos);
    // Header mit endgueltiger Gesamtlaenge + Interface-Anzahl schreiben.
    uint8_t hdr[TUD_CONFIG_DESC_LEN] = {
        TUD_CONFIG_DESCRIPTOR(1, itf_total, 0, g_cfg_len, 0x00, 100)
    };
    std::memcpy(g_cfg_desc, hdr, sizeof hdr);
    g_built = true;
}

int usb_desc_cdc_cli()    { return g_cdc_cli; }
int usb_desc_cdc_gdb()    { return g_cdc_gdb; }
int usb_desc_cdc_serial() { return g_cdc_serial; }
int usb_desc_cdc_count()  { return g_cdc_count; }

uint8_t const* tud_descriptor_configuration_cb(uint8_t /*idx*/) {
    if (!g_built) usb_desc_build();   // Fallback (sollte vor tusb_init erfolgen)
    return g_cfg_desc;
}

static char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },        // 0: en-US
    "Selfbus",                            // 1: Manufacturer
    "LPC1115-Emulator on RP2350",         // 2: Product
    nullptr,                              // 3: Serial (dynamisch)
    "LPC-Emu CLI",                        // 4: CDC CLI
    "LPC-Emu GDB",                        // 5: CDC GDB
    "LPC-Emu MSC",                        // 6: MSC
    "LPC-Emu Serial",                     // 7: CDC Serial-Adapter
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
