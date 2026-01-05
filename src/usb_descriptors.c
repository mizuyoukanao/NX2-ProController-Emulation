#include "tusb.h"
#include <string.h>

// Nintendo Switch2 Pro コントローラーの VID/PID/Bcd デバイス値。
// 参考資料: handheldlegend/docs の Pro Controller 2 / USB initialization ノート。
#define USB_VID 0x057E
#define USB_PID 0x2069
#define USB_BCD 0x0300

// HID report descriptor: NX2 Pro コントローラーの USB 入力/出力/Feature を
// switch2_controller_research の記述に合わせて複数レポート構成で定義する。
// - 入力レポート ID 0x30: 16 ボタン + ハット + 16bit スティック x2 + トリガー + IMU/リザーブで 64B
// - 出力レポート ID 0x21: ベンダー定義 64B (ランブル/LED/モーション設定受信用)
// - Feature レポート ID 0x80: ベンダー定義 64B (初期ハンドシェイク応答用)
uint8_t const desc_hid_report[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)

    // --- Input report (0x30) ---
    0x85, 0x30,       //   Report ID (0x30)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x35, 0x00,       //   Physical Minimum (0)
    0x45, 0x01,       //   Physical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x10,       //   Report Count (16 buttons)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x10,       //   Usage Maximum (Button 16)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Hat (d-pad)
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x25, 0x07,       //   Logical Maximum (7)
    0x46, 0x3B, 0x01, //   Physical Maximum (315)
    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x65, 0x14,       //   Unit (Eng Rot:Angular Pos)
    0x09, 0x39,       //   Usage (Hat switch)
    0x81, 0x42,       //   Input (Data,Var,Abs,Null)
    0x65, 0x00,       //   Unit (None)
    0x95, 0x01,       //   Report Count (1) padding nibble
    0x75, 0x04,       //   Report Size (4)
    0x81, 0x01,       //   Input (Const,Array,Abs)

    // Analog sticks (16-bit signed)
    0x16, 0x00, 0x80, //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, //   Logical Maximum (32767)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x04,       //   Report Count (4 axes)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x35,       //   Usage (Rz)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Triggers (0-255)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0xFF,       //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2 triggers)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // IMU + reserved bytes to align to 64-byte report size
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0)
    0x09, 0x20,
    0x95, 0x18,       //   24 bytes (12 for IMU placeholder + 12 reserve)
    0x75, 0x08,
    0x81, 0x02,
    0x09, 0x21,
    0x95, 0x0E,       //   Remaining 14 bytes to make total 63 data bytes
    0x75, 0x08,
    0x81, 0x02,

    // --- Output report (0x21) ---
    0x85, 0x21,       //   Report ID (0x21)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined)
    0x09, 0x21,       //   Usage
    0x95, 0x3F,       //   Report Count (63 bytes payload)
    0x75, 0x08,       //   Report Size (8)
    0x91, 0x02,       //   Output (Data,Var,Abs)

    // --- Feature report (0x80) ---
    0x85, 0x80,       //   Report ID (0x80)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined)
    0x09, 0x80,       //   Usage
    0x95, 0x3F,       //   Report Count (63 bytes payload)
    0x75, 0x08,       //   Report Size (8)
    0xB1, 0x02,       //   Feature (Data,Var,Abs)
    0xC0              // End Collection
};

// Device descriptor (Switch2 Pro Controller)
static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = TUSB_CLASS_HUMAN_INTERFACE,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// HID endpoint configuration descriptor
enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID 0x81

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface, string index, protocol, report descriptor len, EP In address, size, polling interval
    TUD_HID_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 5)
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// String descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},        // 0: is supported language is English (0x0409)
    "Nintendo Co., Ltd.",            // 1: Manufacturer
    "Nintendo Switch2 Pro Controller", // 2: Product
    "0001",                           // 3: Serial
    "HID Interface",                  // 4: HID string
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = string_desc_arr[0][0] | (string_desc_arr[0][1] << 8);
        chr_count = 1;
    } else {
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string descriptor type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
    (void)itf;
    return desc_hid_report;
}
