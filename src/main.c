#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
//#include "pico/mutex.h"
#include "bsp/board.h"
#include "tusb.h"
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "AES_128_ECB.h"

#define CHANGE_DESC 1

#if CHANGE_DESC == 1
#define NX2_INPUT_REPORT_ID 0x08
#define NX2_CONTROLLER_TYPE 0x01
#elif CHANGE_DESC == 2
#define NX2_INPUT_REPORT_ID 0x07
#define NX2_CONTROLLER_TYPE 0x00
#else
#define NX2_INPUT_REPORT_ID 0x09
#define NX2_CONTROLLER_TYPE 0x02
#endif
#define NX2_OUTPUT_REPORT_ID 0x02
#define NX2_COMMAND_HEADER_DIRECTION_REQUEST 0x91
#define NX2_COMMAND_HEADER_DIRECTION_RESPONSE 0x01
#define NX2_TRANSPORT_BT 0x01
#define NX2_TRANSPORT_USB 0x00
#define NX2_ACK1 0x78
#define NX2_ACK2 0xF8

//static const uint8_t NX2_FEATURE_REPORT_ID = 0x80;

#define EN_AUDIO 0
#if EN_AUDIO
// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];      // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];// +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio20_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];// Volume range state
audio20_control_range_4_n_t(1) sampleFreqRng;                                    // Sample frequency range state

// Audio test data
uint16_t test_buffer_audio[CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX / 2];
uint16_t startVal = 0;
#endif

// commands.md にあるコマンド ID 一覧 (HID デスクリプタとは異なるベンダー独自領域)
typedef enum {
    NX2_CMD_NFC = 0x01,
    NX2_CMD_FLASH_MEMORY = 0x02,
    NX2_CMD_INIT = 0x03,
    NX2_CMD_UNKNOWN04 = 0x04,
    NX2_CMD_UNKNOWN05 = 0x05,
    NX2_CXD_UNKNOWN06 = 0x06,
    NX2_CMD_UNKNOWN07 = 0x07,
    NX2_CMD_CHRGRIP = 0x08,
    NX2_CMD_PLAYER_LED = 0x09,
    NX2_CMD_VIBRATION = 0x0A,
    NX2_CMD_BATTERY = 0x0B,
    NX2_CMD_FEATURE = 0x0C,
    NX2_CMD_FW = 0x0D,
    NX2_CMD_UNKNOWN0E = 0x0E,
    NX2_CMD_UNKNOWN0F = 0x0F,
    NX2_CMD_FW_INFO = 0x10,
    NX2_CMD_UNKNOWN11 = 0x11,
    NX2_CMD_UNKNOWN12 = 0x12,
    NX2_CMD_UNKNOWN13 = 0x13,
    NX2_CMD_UNKNOWN14 = 0x14,
    NX2_CMD_BT_PAIR = 0x15,
    NX2_CMD_UNKNOWN16 = 0x16,
    NX2_CMD_UNKNOWN17 = 0x17,
    NX2_CMD_UNKNOWN18 = 0x18,
    NX2_CMD_UNKNOWN = 0xFF,
} nx2_command_id_t;

// Input report payload (Report ID は別引数で付加)
typedef struct __attribute__((packed)) {
    uint8_t counter;
    uint8_t connection_info;
    uint8_t buttons[3];
    uint8_t lstick[3];
    uint8_t rstick[3];
    uint8_t unknown1[2];
    uint8_t headset_flag;
    uint8_t imu_data_len;
    uint8_t imu[0x28];
    uint8_t unknown2[8];
} nx2_input_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t counter;
    uint8_t connection_info;
    uint8_t buttons[2];
    uint8_t unknown;
    uint8_t rstick[3];
    uint8_t unknown2;
    uint8_t mouse_data[5];
    uint8_t unknown3;
    uint8_t imu_data_len;
    uint8_t imu[0x28];
    uint8_t unknown4[7];
} nx2_joycon_r_input_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t counter;
    uint8_t connection_info;
    uint8_t buttons[2];
    uint8_t unknown;
    uint8_t lstick[3];
    uint8_t unknown2;
    uint8_t mouse_data[5];
    uint8_t unknown3;
    uint8_t imu_data_len;
    uint8_t imu[0x28];
    uint8_t unknown4[7];
} nx2_joycon_l_input_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t left_rumble[0x10];
    uint8_t right_rumble[0x10];
    uint8_t reserved[0x09];
} nx2_output_payload_t;

#if CHANGE_DESC == 1
static nx2_joycon_r_input_payload_t input_payload = {
    .counter = 0,
    .connection_info = 0x23,
    .buttons = {0},
    .unknown = 0x07,
    .rstick = {0x00, 0x08, 0x80},
    .unknown2 = 0x00,
    .mouse_data = {0},
    .unknown3 = 0x00,
    .imu_data_len = 0x00,
    .imu = {0},
    .unknown4 = {0},
};
#elif CHANGE_DESC == 2
static nx2_joycon_l_input_payload_t input_payload = {
    .counter = 0,
    .connection_info = 0x23,
    .buttons = {0},
    .unknown = 0x07,
    .lstick = {0x00, 0x08, 0x80},
    .unknown2 = 0x00,
    .mouse_data = {0},
    .unknown3 = 0x00,
    .imu_data_len = 0x00,
    .imu = {0},
    .unknown4 = {0},
};
#else
static nx2_input_payload_t input_payload = {
    .counter = 0,
    .connection_info = 0x23,
    .buttons = {0},
    .lstick = {0x00, 0x08, 0x80},
    .rstick = {0x00, 0x08, 0x80},
    .unknown1 = {0x38, 0x00},
    .headset_flag = 0x00,
    .imu_data_len = 0x00,
    .imu = {0},
    .unknown2 = {0},
};
#endif

typedef enum {
    SetupRequestId_DeviceInfo  = 0x02,
    SetupRequestId_FactoryData = 0x03,
} SetupRequestId;

typedef struct __attribute__((packed)) {
    uint8_t major;
    uint8_t minor;
    uint8_t micro;
} FirmwareVersion;

typedef struct __attribute__((packed)) {
    FirmwareVersion fw_version;
    uint32_t bt_patch_version;
    FirmwareVersion dsp_fw_version;
    uint8_t bt_address_reversed[6];
} DeviceInfo;

// commands.md で規定されている「実レポート構成」を模した 64 バイトの応答バッファ。
// HID レポートディスクリプタとは異なり、先頭にコマンド ID とステータスを入れる。
static uint8_t command_reply[640] = {0};
static uint16_t command_reply_len = 0;
//static uint8_t feature_payload[63] = {0x01};
static uint8_t last_host_output[64] = {0};
static uint16_t last_host_output_len = 0;
static absolute_time_t next_report_at;
static bool polling_enabled = false;
static uint8_t host_address[6] = {0};
static uint8_t bt_ltk[16] = {0};
static uint8_t dev_key[16] = {0x10, 0x5f, 0x1a, 0xc4, 0x25, 0x63, 0x2b, 0xba, 0xe1, 0x05, 0xdf, 0x2c, 0x79, 0xee, 0xf6, 0x5c};
static nx2_output_payload_t output_payload = {
    .left_rumble = {0},
    .right_rumble = {0},
    .reserved = {0},
};
AES_CTX ctx;
DeviceInfo device_info = {
    .fw_version = { .major = 0x02, .minor = 0x01, .micro = 0x04 },
    .bt_patch_version = 12,
    .dsp_fw_version = { .major = 0x00, .minor = 0x02, .micro = 0x03 },
    .bt_address_reversed = { 0xe8, 0x20, 0x17, 0x05, 0x48, 0xc8 },
};
//mutex_t __usb_mutex;

static uint8_t spi_flash_0x00013080[] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0xbc, 0x97, 0x85, 0x4f, 0x16, 0x5f, 0x2e, 0x96,
    0x61, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x000130c0[] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0x73, 0x38, 0x82, 0xf5, 0x55, 0x60, 0x2a, 0x56,
    0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x00013040[] = {
    0x85, 0x2e, 0xec, 0x41, 0x49, 0xec, 0x20, 0xbc, 0xc5, 0x5d, 0x3f, 0xbc, 0xad, 0x11, 0x42, 0x3b
};
static uint8_t spi_flash_0x00013100[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xd2, 0x7e, 0xbd,
    0x33, 0x8a, 0xbe, 0xbe, 0xaa, 0x82, 0x20, 0x41
};
static uint8_t amiibo_header[] = {
    0x01, 0x58, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x07
};
static uint8_t amiibo_header2[] = {
    0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x00, 0x07
};
static uint8_t amiibo_header3[] = {
    0x00, 0x00, 0x00, 0x00, 0x71, 0x4E, 0x3C, 0x11, 0xCE, 0xEE, 0x3A, 0xCE,0x8E, 0x49, 0xEA,0xB0, 0x71, 0x51, 0x30, 0xCF, 0xED, 0xE4, 0x89, 0x00, 0x9F, 0xB7, 0x96, 0x14, 0x88, 0x72, 0x2B, 0x7A, 0x7F, 0xB0, 0xF4, 0x7D, 0x03, 0x00, 0x3B, 0x3C, 0x77, 0x78, 0x86, 0x00, 0x00
};
static uint8_t amiibo_data[540] = {0};
static uint8_t charging_grip_data[] = {
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x48, 0x45, 0x4a, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32,
    0x31, 0x32, 0x34, 0x37, 0x00, 0x00, 0x7e, 0x05, 0x69, 0x20, 0x01, 0x06, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x00013000[] = {
0x01, 0x00,
0x48, 0x45, 0x4A, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32, 0x31, 0x32, 0x34, 0x37, 0x00, 0x00,
0x7E, 0x05,
#if CHANGE_DESC == 1
0x66, 0x20,
#elif CHANGE_DESC == 2
0x67, 0x20,
#else
0x69, 0x20,
#endif
0x01, 0x06, 0x01,
0x23, 0x23, 0x23,
0xA0, 0xA0, 0xA0,
0xE6, 0xE6, 0xE6,
0x32, 0x32, 0x32,
};

static void set_command_reply(nx2_command_id_t cmd, uint8_t subcommand, uint8_t unknown, uint8_t ACK, const uint8_t *payload, uint16_t payload_len) {
    memset(command_reply, 0, sizeof(command_reply));
    command_reply[0] = (uint8_t)cmd;
    command_reply[1] = NX2_COMMAND_HEADER_DIRECTION_RESPONSE;
    command_reply[2] = NX2_TRANSPORT_USB;
    command_reply[3] = subcommand;
    command_reply[4] = unknown; // unknown
    command_reply[5] = ACK;
    command_reply[6] = 0x00; // reserved
    command_reply[7] = 0x00; // reserved

    command_reply_len = 8 + payload_len;

    if (payload && payload_len > 0) {
        //if (payload_len > sizeof(command_reply) - 2) {
        //    payload_len = sizeof(command_reply) - 2;
        //}
        memcpy(command_reply+8, payload, payload_len);
    }
}

static void respond_nfc(void) {
    // commands.md: 0x01 プロトコルバージョン要求 -> 0x0003 を返す
    //const uint8_t proto[2] = {0x03, 0x00};
    //set_command_reply(NX2_CMD_GET_PROTOCOL_VERSION, 0x00, proto, sizeof(proto));

    //memset(feature_payload, 0, sizeof(feature_payload));
    //feature_payload[0] = 0x81; // ACK
    //feature_payload[1] = NX2_CMD_GET_PROTOCOL_VERSION;
    //feature_payload[2] = proto[0];
    //feature_payload[3] = proto[1];
    switch (last_host_output[3]) {
        case 0x0c:
            uint8_t buf[] = {0x61, 0x12, 0x50, 0x10};
            set_command_reply(NX2_CMD_NFC, 0x0c, 0x0, NX2_ACK2, buf, sizeof(buf));
            break;
        case 0x15:
            uint8_t buf2[sizeof(amiibo_header) + sizeof(amiibo_data) + 71] = {0};
            memcpy(buf2, amiibo_header, sizeof(amiibo_header));
            for (int i = 0; i < 3; i++) {
                buf2[sizeof(amiibo_header) + i] = amiibo_data[i];
            }
            for (int i = 0; i < 4; i++) {
                buf2[sizeof(amiibo_header) + 3 + i] = amiibo_data[4 + i];
            }
            memcpy(buf2 + sizeof(amiibo_header) + 7, amiibo_header3, sizeof(amiibo_header3));
            memcpy(buf2 + sizeof(amiibo_header) + 7 + sizeof(amiibo_header3), amiibo_data, sizeof(amiibo_data));
            set_command_reply(NX2_CMD_NFC, 0x15, 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        case 0x05:
            uint8_t buf3[sizeof(amiibo_header2) + 7] = {0};
            memcpy(buf3, amiibo_header2, sizeof(amiibo_header2));
            for (int i = 0; i < 3; i++) {
                buf3[sizeof(amiibo_header2) + i] = amiibo_data[i];
            }
            for (int i = 0; i < 4; i++) {
                buf3[sizeof(amiibo_header2) + 3 + i] = amiibo_data[4 + i];
            }
            set_command_reply(NX2_CMD_NFC, 0x05, 0x00, NX2_ACK2, buf3, sizeof(buf3));
            break;
        default:
            set_command_reply(NX2_CMD_NFC, last_host_output[3], 0x00, NX2_ACK2, NULL, 0);
            break;
    }
}

static uint8_t read_memory_block(uint32_t address, uint8_t* buffer, uint8_t length) {
    uint8_t len = length;
    switch (address) {
        case 0x00013080:
            memcpy(buffer, spi_flash_0x00013080, sizeof(spi_flash_0x00013080));
            len = sizeof(spi_flash_0x00013080) + 8;
            break;
        case 0x000130c0:
            memcpy(buffer, spi_flash_0x000130c0, sizeof(spi_flash_0x000130c0));
            len = sizeof(spi_flash_0x000130c0) + 8;
            break;
        case 0x00013100:
            memcpy(buffer, spi_flash_0x00013100, sizeof(spi_flash_0x00013100));
            len = sizeof(spi_flash_0x00013100) + 8;
            //memset(buffer + sizeof(spi_flash_0x00013100), 0x00, length - sizeof(spi_flash_0x00013100));
            break;
        case 0x00013040:
            memcpy(buffer, spi_flash_0x00013040, sizeof(spi_flash_0x00013040));
            len = sizeof(spi_flash_0x00013040) + 8;
            //memset(buffer + sizeof(spi_flash_0x00013040), 0x00, length - sizeof(spi_flash_0x00013040));
            break;
        case 0x00013060:
            len = 40;
            memset(buffer, 0xFF, 32);
            break;
        default:
            memset(buffer, 0xFF, length);
            len = length + 8;
            break;
    }
    return len;
}

static void respond_flash_memory(void) {
    // commands.md: 0x02 デバイス情報。PID/VID/バージョンなどを 8byte で返す想定。
    //uint8_t info[8] = {0};
    //info[0] = 0x69; // PID LSB
    //info[1] = 0x20; // PID MSB
    //info[2] = 0x7E; // VID LSB
    //info[3] = 0x05; // VID MSB
    //info[4] = 0x00; // hw major
    //info[5] = 0x03; // hw minor (= bcdUSB)
    //info[6] = 0x00; // fw major placeholder
    //info[7] = 0x01; // fw minor placeholder
//
    //set_command_reply(NX2_CMD_GET_DEVICE_INFO, 0x00, info, sizeof(info));
//
    //memset(feature_payload, 0, sizeof(feature_payload));
    //feature_payload[0] = 0x81;
    //feature_payload[1] = NX2_CMD_GET_DEVICE_INFO;
    //memcpy(&feature_payload[2], info, sizeof(info));
    switch (last_host_output[3]) {
        case 0x01:
        case 0x04:
            uint32_t address = last_host_output[12] | (last_host_output[13] << 8) | (last_host_output[14] << 16) | (last_host_output[15] << 24);
            uint8_t buf[72] = {0};
            memset(buf, 0xFF, sizeof(buf));
            buf[0] = last_host_output[8];
            buf[1] = 0x00;
            buf[2] = 0x00;
            buf[3] = 0x00;
            buf[4] = last_host_output[12];
            buf[5] = last_host_output[13];
            buf[6] = last_host_output[14];
            buf[7] = last_host_output[15];
            uint8_t send_len = read_memory_block(address, buf+8, 64);
            set_command_reply(NX2_CMD_FLASH_MEMORY, last_host_output[3], 0x0, NX2_ACK2, buf, send_len);
            break;
        case 0x02:
            //uint32_t address2 = last_host_output[12] | (last_host_output[13] << 8) | (last_host_output[14] << 16) | (last_host_output[15] << 24);
            //const uint8_t* data = &last_host_output[16];
            //write_memory_block(address2, data, 64);
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x02, 0x0, NX2_ACK2, &last_host_output[8], 8);
            break;
        case 0x03:
            uint8_t buf2[4] = {0};
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x03, 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        case 0x05:
            uint8_t buf3[8] = {0};
            buf3[4] = last_host_output[12];
            buf3[5] = last_host_output[13];
            buf3[6] = last_host_output[14];
            buf3[7] = last_host_output[15];
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x05, 0x0, NX2_ACK2, buf3, sizeof(buf3));
            break;
        default:
            set_command_reply(NX2_CMD_FLASH_MEMORY, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
            break;
    }
}

static void respond_initialisation(void) {
    switch (last_host_output[3]) {
        case 0x07:
            for (int i = 0; i < 6; i++) {
                host_address[i] = last_host_output[13 - i];
            }
            for (int i = 0; i < 16; i++) {
                bt_ltk[i] = last_host_output[29 - i];
            }
            set_command_reply(NX2_CMD_INIT, last_host_output[3], 0x00, NX2_ACK2, NULL, 0);
            break;
        case 0x0D:
            polling_enabled = true;
            for (int i = 0; i < 6; i++) {
                host_address[i] = last_host_output[15 - i];
            }
            uint8_t buf[] = {0x01, 0x00, 0x00, 0x00};
            set_command_reply(NX2_CMD_INIT, last_host_output[3], 0x00, NX2_ACK2, buf, sizeof(buf));
            break;
        case 0x0F:
            uint8_t buf2[] = {0x05, 0x00, 0x00, 0x00};
            set_command_reply(NX2_CMD_INIT, last_host_output[3], 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        default:
            set_command_reply(NX2_CMD_INIT, last_host_output[3], 0x00, NX2_ACK2, NULL, 0);
            break;
    }
}

static void respond_charging_grip(void) {
    switch (last_host_output[3]) {
        case 0x01:
            uint8_t buf[36] = {0};
            memcpy(buf, charging_grip_data, sizeof(buf));
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output[3], 0x00, NX2_ACK2, buf, sizeof(buf));
            break;
        case 0x03:
            uint8_t buf2[68] = {0};
            memcpy(buf2, charging_grip_data, sizeof(buf2));
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output[3], 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        default:
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output[3], 0x00, NX2_ACK2, NULL, 0);
            break;
    }
}

static void respond_battery(void) {
    switch (last_host_output[3]) {
        case 0x03:
            {
                uint8_t buf[] = {0xa5, 0x0e, 0x00, 0x00};
                set_command_reply(NX2_CMD_BATTERY, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
            }
            break;
        case 0x04:
            {
                uint8_t buf2[] = {0x34, 0x00, 0x83, 0x00};
                set_command_reply(NX2_CMD_BATTERY, last_host_output[3], 0x0, NX2_ACK2, buf2, sizeof(buf2));
            }
            break;
        case 0x06:
            {
                uint8_t buf3[] = {0x11, 0x00, 0x00, 0x00};
                set_command_reply(NX2_CMD_BATTERY, last_host_output[3], 0x0, NX2_ACK2, buf3, sizeof(buf3));
            }
            break;
        default:
            set_command_reply(NX2_CMD_BATTERY, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
            break;
    }
}

static void respond_feature(void) {
    switch (last_host_output[3]) {
        case 0x01:
            {
                uint8_t buf[12] = {0};
                if (last_host_output[8] & 0x01) {
                    buf[4] = 0x07;
                }
                if (last_host_output[8] & 0x02) {
                    buf[5] = 0x07;
                }
                if (last_host_output[8] & 0x04) {
#if CHANGE_DESC
                    buf[6] = 0x03;
#else
                    buf[6] = 0x01;
#endif
                }
                if (last_host_output[8] & 0x80) {
#if CHANGE_DESC
                    buf[7] = 0x03;
#else
                    buf[7] = 0x01;
#endif
                }
                if (last_host_output[8] & 0x10) {
#if CHANGE_DESC
                    buf[8] = 0x03;
#else
                    buf[8] = 0x01;
#endif
                }
                if (last_host_output[8] & 0x20) {
                    buf[9] = 0x03;
                }
                set_command_reply(NX2_CMD_FEATURE, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
            }
            break;
        case 0x02:
            {
                if (last_host_output[8] & 0x01 && last_host_output[8] & 0x02) {
                    polling_enabled = true;
                }
                //if need more feature, add here
                uint8_t buf2[] = {0x00, 0x00, 0x00, 0x00};
                set_command_reply(NX2_CMD_FEATURE, last_host_output[3], 0x0, NX2_ACK2, buf2, sizeof(buf2));
            }
            break;
        case 0x03:
            polling_enabled = false;
        case 0x04:
        case 0x05:
            uint8_t buf3[] = {0x00, 0x00, 0x00, 0x00};
            set_command_reply(NX2_CMD_FEATURE, last_host_output[3], 0x0, NX2_ACK2, buf3, sizeof(buf3));
            break;
        default:
            set_command_reply(NX2_CMD_FEATURE, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
            break;
    }
}

static void respond_bt_pair(void) {
    switch (last_host_output[3]) {
        case 0x01:
            {
                uint8_t buf[] = {0x01, 0x04, 0x01, 0xe8, 0x20, 0x17, 0x05, 0x48, 0xc8};
                set_command_reply(NX2_CMD_BT_PAIR, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
            }
            break;
        case 0x02:
            {
                AES_EncryptInit(&ctx, bt_ltk);
                uint8_t data[16] = {0};
                for (int i = 0; i < 16; i++) {
                    data[i] = last_host_output[24 - i];
                }
                AES_Encrypt(&ctx, data, 16, data);
                uint8_t buf2[17] = {0};
                buf2[0] = 0x01;
                memcpy(&buf2[1], data, 16);
                set_command_reply(NX2_CMD_BT_PAIR, last_host_output[3], 0x0, NX2_ACK2, buf2, sizeof(buf2));
                AES_CTX_Free(&ctx);
            }
            break;
        case 0x03:
            {
                uint8_t buf3[1] = {0x01};
                set_command_reply(NX2_CMD_BT_PAIR, last_host_output[3], 0x0, NX2_ACK2, buf3, sizeof(buf3));
            }
            break;
        case 0x04:
            {
                uint8_t host_key[16] = {0};
                for (int i = 0; i < 16; i++) {
                    host_key[i] = last_host_output[24 - i];
                }
                for (int i = 0; i < 16; i++) {
                    bt_ltk[i] = host_key[i] ^ dev_key[i];
                }
                uint8_t buf4[17] = {0};
                buf4[0] = 0x01;
                for (int i = 0; i < 16; i++) {
                    buf4[1 + i] = dev_key[15 - i];
                }
                set_command_reply(NX2_CMD_BT_PAIR, last_host_output[3], 0x0, NX2_ACK2, buf4, sizeof(buf4));
            }
            break;
        default:
            set_command_reply(NX2_CMD_BT_PAIR, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
            break;
    }
}

static void hid_task(void) {
    if (!tud_mounted()) {
        return;
    }

    if (absolute_time_diff_us(get_absolute_time(), next_report_at) > 0) {
        return;
    }

    if (tud_hid_ready() && polling_enabled) {
        tud_hid_report(NX2_INPUT_REPORT_ID, (uint8_t *)&input_payload, sizeof(input_payload));
    }

    // Send report at ~200Hz to match console polling expectations
    next_report_at = delayed_by_us(get_absolute_time(), 5000);
    input_payload.counter++;
}

void tud_mount_cb(void) {
    // Reset neutral state on mount
    //idle_payload.buttons = 0;
    //idle_payload.hat = 0x08;
    //idle_payload.lx = idle_payload.ly = idle_payload.rx = idle_payload.ry = 0;
    //idle_payload.lt = idle_payload.rt = 0x00;
    polling_enabled = false;
    memset(command_reply, 0, sizeof(command_reply));
}

// Nintendo Switch2 からの GET_REPORT/SET_REPORT に応答
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance;

    // コントロール転送(GET_REPORT)では入力/Feature レポートを返す
    //if (report_type == HID_REPORT_TYPE_INPUT && (report_id == NX2_INPUT_REPORT_ID || report_id == 0)) {
    //    uint16_t copy_len = (uint16_t)(sizeof(command_reply) + 1);
    //    if (copy_len > reqlen) {
    //        copy_len = reqlen;
    //    }
    //    buffer[0] = NX2_INPUT_REPORT_ID;
    //    memcpy(buffer + 1, command_reply, copy_len - 1);
    //    return copy_len;
    //}

    //if (report_type == HID_REPORT_TYPE_FEATURE && report_id == NX2_FEATURE_REPORT_ID) {
    //    uint16_t copy_len = (uint16_t)(sizeof(feature_payload) + 1);
    //    if (copy_len > reqlen) {
    //        copy_len = reqlen;
    //    }
    //    buffer[0] = NX2_FEATURE_REPORT_ID;
    //    memcpy(buffer + 1, feature_payload, copy_len - 1);
    //    return copy_len;
    //}

    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;
    if (report_id == 0x02) {
        memcpy(&output_payload, buffer, 0x29);
    }
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request->bmRequestType & 0x80) {
        switch (request->bRequest) {
            case SetupRequestId_DeviceInfo:
                tud_control_xfer(rhport, request, (void*)&device_info, sizeof(device_info));
                return true;
            case SetupRequestId_FactoryData:
                uint8_t buf[64];
                memset(buf, 0xFF, sizeof(buf));
                memcpy(buf, spi_flash_0x00013000, sizeof(spi_flash_0x00013000));
                tud_control_xfer(rhport, request, (void*)buf, sizeof(buf));
                return true;
            default:
                return false;
        }
    } else if (request->bRequest == 0x04) {
        //Is the maximum length of data read/write set?
        //0x276 = 630 bytes
            tud_control_xfer(rhport, request, NULL, 0);
            return true;
    }
    return false;
}

void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buf, uint32_t bufs) {

    uint8_t buffer[tud_vendor_n_available(idx)];
    uint32_t bufsize = tud_vendor_n_read(idx, buffer, sizeof(buffer));

    //printf("Vendor RX CB: idx=%d, bufsize=%d\n", idx, bufsize);
    //for (int i = 0; i < bufsize; i++) {
    //    printf("%02X ", buffer[i]);
    //}
    //printf("\n");

    // 出力レポートは commands.md の実コマンドに従って解釈する
    //if (report_type == HID_REPORT_TYPE_OUTPUT && report_id == NX2_OUTPUT_REPORT_ID && buffer && bufsize) {
    if (buffer[1] == NX2_COMMAND_HEADER_DIRECTION_REQUEST && buffer[2] == NX2_TRANSPORT_USB) {
        last_host_output_len = bufsize;// < sizeof(last_host_output) ? bufsize : (uint16_t)sizeof(last_host_output);
        //memcpy(last_host_output, buffer, last_host_output_len);
        memcpy(last_host_output, buffer, bufsize);

        nx2_command_id_t cmd = (bufsize > 0) ? (nx2_command_id_t)buffer[0] : NX2_CMD_UNKNOWN;
        switch (cmd) {
            case NX2_CMD_NFC:
                respond_nfc();
                break;
            case NX2_CMD_FLASH_MEMORY:
                respond_flash_memory();
                break;
            case NX2_CMD_INIT:
                respond_initialisation();
                break;
            case NX2_CMD_UNKNOWN07:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[1] = {0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                break;
            case NX2_CMD_CHRGRIP:
                respond_charging_grip();
                break;
            case NX2_CMD_PLAYER_LED:
                //respond_player_led();
                set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                break;
            case NX2_CMD_VIBRATION:
                //respond_vibration();
                set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                break;
            case NX2_CMD_BATTERY:
                respond_battery();
                break;
            case NX2_CMD_FEATURE:
                respond_feature();
                break;
            case NX2_CMD_FW:
                break;
            case NX2_CMD_FW_INFO:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[] = {0x02, 0x01, 0x04, NX2_CONTROLLER_TYPE, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                break;
            case NX2_CMD_UNKNOWN11:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[] = {0x01, 0x00, 0x00, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else if (last_host_output[3] == 0x03) {
                    uint8_t buf[] = {0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c, 0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c, 0x41, 0x58, 0xa0, 0x0b, 0x41};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                break;
#if CHANGE_DESC
            case NX2_CMD_UNKNOWN13:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[] = {0x01, 0x00, 0x00, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    uint8_t buf[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                }
                break;
#endif
            case NX2_CMD_BT_PAIR:
                respond_bt_pair();
                break;
            case NX2_CMD_UNKNOWN16:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[0x18] = {0};
                    buf[12] = 0x7c;
                    buf[13] = 0x06;
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                break;
            case NX2_CMD_UNKNOWN18:
                if (last_host_output[3] == 0x01) {
                    uint8_t buf[] = {0x00, 0x00, 0x40, 0xf0, 0x00, 0x00, 0x60, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else if (last_host_output[3] == 0x03) {
                    uint8_t buf[1] = {last_host_output[8]};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                break;
            default:
                set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                break;
        }

        // コマンド応答をすぐに IN トランザクションで返す
        //if (tud_hid_ready()) {
        //    tud_hid_report(0x00, command_reply, sizeof(command_reply_len));
        //}
        tud_vendor_n_write(idx, command_reply, command_reply_len);
        tud_vendor_n_write_flush(idx);
    }
}

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
#if PICO_RP2040
    #define CS_BIT (1u << 1)
#else
    #define CS_BIT SIO_GPIO_HI_IN_QSPI_CSN_BITS
#endif
    bool button_state = !(sio_hw->gpio_hi_in & CS_BIT);

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

#if EN_AUDIO
void audio_task(void);
#endif

int main(void) {
    //stdio_init_all();
    board_init();
    //mutex_init(&__usb_mutex);
    tusb_init(0);
    board_init_after_tusb();
    //printf("Nintendo Switch2 Emulation Started\n");

#if EN_AUDIO
    sampFreq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    clkValid = 1;

    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;
#endif

    next_report_at = get_absolute_time();

    //while (!tud_connected()) {tud_task();}
    while (true) {
        //if (tud_ready()) {
            //printf("USB ready\n");
            //tud_task();
            //try {
            //    tud_task();
                //if (mutex_try_enter(&__usb_mutex, NULL)) {
                    tud_task();
                    if (tud_suspended()) {
                        tud_remote_wakeup();
                    }
                    if (tud_hid_ready()) {
                        hid_task();
                    }
                    //mutex_exit(&__usb_mutex);
                //}
            //} catch (int e) {
            //    tud_task();
            //    if (tud_suspended()) {
            //        tud_remote_wakeup();
            //    }
            //}
            //hid_task();
            if (get_bootsel_button()) {
                input_payload.buttons[0] |= 2;
            } else {
                input_payload.buttons[0] &= ~2;
            }
        //}
#if EN_AUDIO
        audio_task();
#endif
    }
    return 0;
}
#if EN_AUDIO
void audio_task(void) {
  static uint32_t start_ms = 0;
  uint32_t curr_ms = board_millis();
  if (start_ms == curr_ms) {
    return; // not enough time
  }
  start_ms = curr_ms;
  for (size_t cnt = 0; cnt < sizeof(test_buffer_audio) / 2; cnt++) {
    test_buffer_audio[cnt] = startVal++;
  }
  tud_audio_write((uint8_t *) test_buffer_audio, sizeof(test_buffer_audio));
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO20_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) ep;

  return false;// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO20_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) itf;

  return false;// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  (void) itf;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO20_CS_REQ_CUR);

  // If request is for our feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
      case AUDIO20_FU_CTRL_MUTE:
        // Request uses format layout 1
        TU_VERIFY(p_request->wLength == sizeof(audio20_control_cur_1_t));

        mute[channelNum] = ((audio20_control_cur_1_t *) pBuff)->bCur;

        TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
        return true;

      case AUDIO20_FU_CTRL_VOLUME:
        // Request uses format layout 2
        TU_VERIFY(p_request->wLength == sizeof(audio20_control_cur_2_t));

        volume[channelNum] = (uint16_t) ((audio20_control_cur_2_t *) pBuff)->bCur;

        TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
        return true;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }
  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) ep;

  //	return tud_control_xfer(rhport, p_request, &tmp, 1);

  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) itf;

  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // Input terminal (Microphone input)
  if (entityID == 1) {
    switch (ctrlSel) {
      case AUDIO20_TE_CTRL_CONNECTOR: {
        // The terminal connector control only has a get request with only the CUR attribute.
        audio20_desc_channel_cluster_t ret;

        // Those are dummy values for now
        ret.bNrChannels = 1;
        ret.bmChannelConfig = (audio20_channel_config_t) 0;
        ret.iChannelNames = 0;

        TU_LOG2("    Get terminal connector\r\n");

        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));
      } break;

        // Unknown/Unsupported control selector
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
      case AUDIO20_FU_CTRL_MUTE:
        // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
        // There does not exist a range parameter block for mute
        TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[channelNum], 1);

      case AUDIO20_FU_CTRL_VOLUME:
        switch (p_request->bRequest) {
          case AUDIO20_CS_REQ_CUR:
            TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

          case AUDIO20_CS_REQ_RANGE:
            TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

            // Copy values - only for testing - better is version below
            audio20_control_range_2_n_t(1)
                ret;

            ret.wNumSubRanges = 1;
            ret.subrange[0].bMin = -90;// -90 dB
            ret.subrange[0].bMax = 90; // +90 dB
            ret.subrange[0].bRes = 1;  // 1 dB steps

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
        break;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Clock Source unit
  if (entityID == 4) {
    switch (ctrlSel) {
      case AUDIO20_CS_CTRL_SAM_FREQ:
        // channelNum is always zero in this case
        switch (p_request->bRequest) {
          case AUDIO20_CS_REQ_CUR:
            TU_LOG2("    Get Sample Freq.\r\n");
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

          case AUDIO20_CS_REQ_RANGE:
            TU_LOG2("    Get Sample Freq. range\r\n");
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
        break;

      case AUDIO20_CS_CTRL_CLK_VALID:
        // Only cur attribute exists for this request
        TU_LOG2("    Get Sample Freq. valid\r\n");
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

      // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  TU_LOG2("  Unsupported entity: %d\r\n", entityID);
  return false;// Yet not implemented
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  startVal = 0;

  return true;
}
#endif
