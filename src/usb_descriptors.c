#include "tusb.h"
#include <string.h>

// Nintendo Switch2 Pro コントローラーの VID/PID/Bcd デバイス値。
// 参考資料: handheldlegend/docs の Pro Controller 2 / USB initialization ノート。
#define USB_VID 0x057E
#define USB_PID 0x2069
#define USB_BCD 0x0200

// HID report descriptor: NX2 Pro コントローラーの USB 入力/出力/Feature を
// switch2_controller_research の記述に合わせて複数レポート構成で定義する。
// - 入力レポート ID 0x30: 16 ボタン + ハット + 16bit スティック x2 + トリガー + IMU/リザーブで 64B
// - 出力レポート ID 0x21: ベンダー定義 64B (ランブル/LED/モーション設定受信用)
// - Feature レポート ID 0x80: ベンダー定義 64B (初期ハンドシェイク応答用)
uint8_t const desc_hid_report[] = {
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x05,        // Usage (Game Pad)
0xA1, 0x01,        // Collection (Application)
0x85, 0x05,        //   Report ID (5)
0x05, 0xFF,        //   Usage Page (Reserved 0xFF)
0x09, 0x01,        //   Usage (0x01)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x95, 0x3F,        //   Report Count (63)
0x75, 0x08,        //   Report Size (8)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x85, 0x09,        //   Report ID (9)
0x09, 0x01,        //   Usage (0x01)
0x95, 0x02,        //   Report Count (2)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x09,        //   Usage Page (Button)
0x19, 0x01,        //   Usage Minimum (0x01)
0x29, 0x15,        //   Usage Maximum (0x15)
0x25, 0x01,        //   Logical Maximum (1)
0x95, 0x15,        //   Report Count (21)
0x75, 0x01,        //   Report Size (1)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x95, 0x01,        //   Report Count (1)
0x75, 0x03,        //   Report Size (3)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x09, 0x30,        //     Usage (X)
0x09, 0x31,        //     Usage (Y)
0x09, 0x33,        //     Usage (Rx)
0x09, 0x35,        //     Usage (Rz)
0x26, 0xFF, 0x0F,  //     Logical Maximum (4095)
0x95, 0x04,        //     Report Count (4)
0x75, 0x0C,        //     Report Size (12)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0x05, 0xFF,        //   Usage Page (Reserved 0xFF)
0x09, 0x02,        //   Usage (0x02)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x95, 0x34,        //   Report Count (52)
0x75, 0x08,        //   Report Size (8)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x85, 0x02,        //   Report ID (2)
0x09, 0x01,        //   Usage (0x01)
0x95, 0x3F,        //   Report Count (63)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0xC0,              // End Collection
};

// Device descriptor (Switch2 Pro Controller)
static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = 0xEF,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0200,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// HID endpoint configuration descriptor
//enum {
//    ITF_NUM_HID,
//    ITF_NUM_TOTAL
//};

//#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
//#define EPNUM_HID 0x81

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    //TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface, string index, protocol, report descriptor len, EP In address, size, polling interval
    //TUD_HID_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 5)
0x09,        // bLength
0x02,        // bDescriptorType (Configuration)
0x0C, 0x01,  // wTotalLength 268
0x05,        // bNumInterfaces 5
0x01,        // bConfigurationValue
0x04,        // iConfiguration (String Index)
0xC0,        // bmAttributes Self Powered
0xFA,        // bMaxPower 500mA

0x08,        // bLength
0x0B,        // bDescriptorType (Interface Association)
0x00,        // bFirstInterface
0x01,        // bInterfaceCount
0x03,        // bFunctionClass
0x00,        // bFunctionSubClass
0x00,        // bFunctionProtocol
0x00,        // iFunction

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x00,        // bInterfaceNumber 0
0x00,        // bAlternateSetting
0x02,        // bNumEndpoints 2
0x03,        // bInterfaceClass
0x00,        // bInterfaceSubClass
0x00,        // bInterfaceProtocol
0x05,        // iInterface (String Index)

0x09,        // bLength
0x21,        // bDescriptorType (HID)
0x11, 0x01,  // bcdHID 1.11
0x00,        // bCountryCode
0x01,        // bNumDescriptors
0x22,        // bDescriptorType[0] (HID)
0x61, 0x00,  // wDescriptorLength[0] 97

0x07,        // bLength
0x05,        // bDescriptorType (Endpoint)
0x81,        // bEndpointAddress (IN/D2H)
0x03,        // bmAttributes (Interrupt)
0x40, 0x00,  // wMaxPacketSize 64
0x04,        // bInterval 4 (unit depends on device speed)

0x07,        // bLength
0x05,        // bDescriptorType (Endpoint)
0x01,        // bEndpointAddress (OUT/H2D)
0x03,        // bmAttributes (Interrupt)
0x40, 0x00,  // wMaxPacketSize 64
0x04,        // bInterval 4 (unit depends on device speed)

0x08,        // bLength
0x0B,        // bDescriptorType (Interface Association)
0x01,        // bFirstInterface
0x01,        // bInterfaceCount
0xFF,        // bFunctionClass
0x00,        // bFunctionSubClass
0x00,        // bFunctionProtocol
0x00,        // iFunction

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x01,        // bInterfaceNumber 1
0x00,        // bAlternateSetting
0x02,        // bNumEndpoints 2
0xFF,        // bInterfaceClass
0x00,        // bInterfaceSubClass
0x00,        // bInterfaceProtocol
0x06,        // iInterface (String Index)

0x07,        // bLength
0x05,        // bDescriptorType (Endpoint)
0x02,        // bEndpointAddress (OUT/H2D)
0x02,        // bmAttributes (Bulk)
0x40, 0x00,  // wMaxPacketSize 64
0x00,        // bInterval 0 (unit depends on device speed)

0x07,        // bLength
0x05,        // bDescriptorType (Endpoint)
0x82,        // bEndpointAddress (IN/D2H)
0x02,        // bmAttributes (Bulk)
0x40, 0x00,  // wMaxPacketSize 64
0x00,        // bInterval 0 (unit depends on device speed)

0x08,        // bLength
0x0B,        // bDescriptorType (Interface Association)
0x02,        // bFirstInterface
0x03,        // bInterfaceCount
0x01,        // bFunctionClass
0x01,        // bFunctionSubClass
0x00,        // bFunctionProtocol
0x00,        // iFunction

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x02,        // bInterfaceNumber 2
0x00,        // bAlternateSetting
0x00,        // bNumEndpoints 0
0x01,        // bInterfaceClass (Audio)
0x01,        // bInterfaceSubClass (Audio Control)
0x00,        // bInterfaceProtocol
0x00,        // iInterface (String Index)

0x0A,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x01,        // bDescriptorSubtype (CS_INTERFACE -> HEADER)
0x00, 0x01,  // bcdADC 1.00
0x47, 0x00,  // wTotalLength 71
0x02,        // binCollection 0x02
0x03,        // baInterfaceNr 3
0x04,        // baInterfaceNr 4

0x0C,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x02,        // bDescriptorSubtype (CS_INTERFACE -> INPUT_TERMINAL)
0x01,        // bTerminalID
0x01, 0x01,  // wTerminalType (USB Streaming)
0x00,        // bAssocTerminal
0x02,        // bNrChannels 2
0x03, 0x00,  // wChannelConfig (Left and Right Front)
0x00,        // iChannelNames
0x00,        // iTerminal

0x0A,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x06,        // bDescriptorSubtype (CS_INTERFACE -> FEATURE_UNIT)
0x02,        // bUnitID
0x01,        // bSourceID
0x01,        // bControlSize 1
0x03, 0x00,  // bmaControls[0] (Mute,Volume)
0x00, 0x00,  // bmaControls[1] (None)

0x09,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x03,        // bDescriptorSubtype (CS_INTERFACE -> OUTPUT_TERMINAL)
0x03,        // bTerminalID
0x02, 0x03,  // wTerminalType (Headphones)
0x00,        // bAssocTerminal
0x02,        // bSourceID
0x00,        // iTerminal

0x0C,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x02,        // bDescriptorSubtype (CS_INTERFACE -> INPUT_TERMINAL)
0x04,        // bTerminalID
0x01, 0x02,  // wTerminalType (Microphone)
0x00,        // bAssocTerminal
0x01,        // bNrChannels 1
0x00, 0x00,  // wChannelConfig
0x00,        // iChannelNames
0x00,        // iTerminal

0x09,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x06,        // bDescriptorSubtype (CS_INTERFACE -> FEATURE_UNIT)
0x05,        // bUnitID
0x04,        // bSourceID
0x01,        // bControlSize 1
0x03, 0x00,  // bmaControls[0] (Mute,Volume)
0x00,        // iFeature

0x09,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x03,        // bDescriptorSubtype (CS_INTERFACE -> OUTPUT_TERMINAL)
0x06,        // bTerminalID
0x01, 0x01,  // wTerminalType (USB Streaming)
0x00,        // bAssocTerminal
0x05,        // bSourceID
0x00,        // iTerminal

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x03,        // bInterfaceNumber 3
0x00,        // bAlternateSetting
0x00,        // bNumEndpoints 0
0x01,        // bInterfaceClass (Audio)
0x02,        // bInterfaceSubClass (Audio Streaming)
0x00,        // bInterfaceProtocol
0x00,        // iInterface (String Index)

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x03,        // bInterfaceNumber 3
0x01,        // bAlternateSetting
0x01,        // bNumEndpoints 1
0x01,        // bInterfaceClass (Audio)
0x02,        // bInterfaceSubClass (Audio Streaming)
0x00,        // bInterfaceProtocol
0x00,        // iInterface (String Index)

0x07,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x01,        // bDescriptorSubtype (CS_INTERFACE -> AS_GENERAL)
0x01,        // bTerminalLink
0x00,        // bDelay 0
0x01, 0x00,  // wFormatTag (PCM)

0x0B,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x02,        // bDescriptorSubtype (CS_INTERFACE -> FORMAT_TYPE)
0x01,        // bFormatType 1
0x02,        // bNrChannels (Stereo)
0x02,        // bSubFrameSize 2
0x10,        // bBitResolution 16
0x01,        // bSamFreqType 1
0x80, 0xBB, 0x00,  // tSamFreq[1] 48000 Hz

0x07,        // bLength
0x05,        // bDescriptorType (See Next Line)
0x03,        // bEndpointAddress (OUT/H2D)
0x0D,        // bmAttributes (Isochronous, Sync, Data EP)
0xC0, 0x00,  // wMaxPacketSize 192
0x01,        // bInterval 1 (unit depends on device speed)

0x07,        // bLength
0x25,        // bDescriptorType (See Next Line)
0x01,        // bDescriptorSubtype (CS_ENDPOINT -> EP_GENERAL)
0x00,        // bmAttributes (None)
0x00,        // bLockDelayUnits
0x00, 0x00,  // wLockDelay 0

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x04,        // bInterfaceNumber 4
0x00,        // bAlternateSetting
0x00,        // bNumEndpoints 0
0x01,        // bInterfaceClass (Audio)
0x02,        // bInterfaceSubClass (Audio Streaming)
0x00,        // bInterfaceProtocol
0x00,        // iInterface (String Index)

0x09,        // bLength
0x04,        // bDescriptorType (Interface)
0x04,        // bInterfaceNumber 4
0x01,        // bAlternateSetting
0x01,        // bNumEndpoints 1
0x01,        // bInterfaceClass (Audio)
0x02,        // bInterfaceSubClass (Audio Streaming)
0x00,        // bInterfaceProtocol
0x00,        // iInterface (String Index)

0x07,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x01,        // bDescriptorSubtype (CS_INTERFACE -> AS_GENERAL)
0x06,        // bTerminalLink
0x00,        // bDelay 0
0x01, 0x00,  // wFormatTag (PCM)

0x0B,        // bLength
0x24,        // bDescriptorType (See Next Line)
0x02,        // bDescriptorSubtype (CS_INTERFACE -> FORMAT_TYPE)
0x01,        // bFormatType 1
0x02,        // bNrChannels (Stereo)
0x02,        // bSubFrameSize 2
0x10,        // bBitResolution 16
0x01,        // bSamFreqType 1
0x80, 0xBB, 0x00,  // tSamFreq[1] 48000 Hz

0x07,        // bLength
0x05,        // bDescriptorType (See Next Line)
0x83,        // bEndpointAddress (IN/D2H)
0x0D,        // bmAttributes (Isochronous, Sync, Data EP)
0xC0, 0x00,  // wMaxPacketSize 192
0x01,        // bInterval 1 (unit depends on device speed)

0x07,        // bLength
0x25,        // bDescriptorType (See Next Line)
0x01,        // bDescriptorSubtype (CS_ENDPOINT -> EP_GENERAL)
0x00,        // bmAttributes (None)
0x00,        // bLockDelayUnits
0x00, 0x00,  // wLockDelay 0
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// String descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04, 0x02, 0x04, 0x01, 0x08},        // 0: is supported language is English (0x0409)
    "Nintendo",            // 1: Manufacturer
    "Switch 2 Pro Controller", // 2: Product
    "00",                           // 3: Serial
    "Config_0",
    "If_Hid",
    "Switch 2 Pro Controller",
};

static uint16_t _desc_str[(sizeof(string_desc_arr) / 2)];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = string_desc_arr[0][0] | (string_desc_arr[0][1] << 8);
        _desc_str[2] = string_desc_arr[0][2] | (string_desc_arr[0][3] << 8);
        _desc_str[3] = string_desc_arr[0][4] | (string_desc_arr[0][5] << 8);
        chr_count = 3;
    } else if (index == 0xEE) {
        // Microsoft OS 1.0 Descriptors
        const char ms_os_desc[] = "MSFT100";
        chr_count = (uint8_t)strlen(ms_os_desc);
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = ms_os_desc[i];
        }
        _desc_str[1 + chr_count] = 0x01;
        chr_count += 1;
    }  else {
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        //if (chr_count > 31) {
        //    chr_count = 31;
        //}
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
