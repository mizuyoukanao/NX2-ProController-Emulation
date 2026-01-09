#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "bsp/board.h"
#include "tusb.h"
#include <string.h>

#define NX2_INPUT_REPORT_ID 0x09
#define NX2_OUTPUT_REPORT_ID 0x02
#define NX2_COMMAND_HEADER_DIRECTION_REQUEST 0x91
#define NX2_COMMAND_HEADER_DIRECTION_RESPONSE 0x01
#define NX2_TRANSPORT_BT 0x01
#define NX2_TRANSPORT_USB 0x00
#define NX2_ACK1 0x78
#define NX2_ACK2 0xF8
//static const uint8_t NX2_FEATURE_REPORT_ID = 0x80;

// commands.md にあるコマンド ID 一覧 (HID デスクリプタとは異なるベンダー独自領域)
typedef enum {
    NX2_CMD_NFC = 0x01,
    NX2_CMD_FLASH_MEMORY = 0x02,
    NX2_CMD_INIT = 0x03,
    NX2_CMD_CHRGRIP = 0x08,
    NX2_CMD_PLAYER_LED = 0x09,
    NX2_CMD_VIBRATION = 0x0A,
    NX2_CMD_BATTERY = 0x0B,
    NX2_CMD_FEATURE = 0x0C,
    NX2_CMD_FW = 0x0D,
    NX2_CMD_FW_INFO = 0x10,
    NX2_CMD_BT_PAIR = 0x15,
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
    uint8_t left_rumble[0x10];
    uint8_t right_rumble[0x10];
    uint8_t reserved[0x09];
} nx2_output_payload_t;

static nx2_input_payload_t input_payload = {
    .counter = 0,
    .connection_info = 0x91,
    .buttons = {0},
    .lstick = {0x00, 0x08, 0x80},
    .rstick = {0x00, 0x08, 0x80},
    .unknown1 = {0x38, 0x00},
    .headset_flag = 0x00,
    .imu_data_len = 0x00,
    .imu = {0},
    .unknown2 = {0},
};

// commands.md で規定されている「実レポート構成」を模した 64 バイトの応答バッファ。
// HID レポートディスクリプタとは異なり、先頭にコマンド ID とステータスを入れる。
static uint8_t command_reply[640] = {0};
static uint16_t command_reply_len = 0;
static uint8_t feature_payload[63] = {0x01};
static uint8_t last_host_output[64] = {0};
static uint16_t last_host_output_len = 0;
static absolute_time_t next_report_at;
static bool polling_enabled = false;
static uint8_t host_address[6] = {0};
static uint8_t bt_ltk[16] = {0};

static uint8_t spi_flash_0x00013080[] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0xa3, 0xa7, 0x87, 0x35, 0x06, 0x5f, 0x1c, 0xc6,
    0x5c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x000130c0[] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0xb1, 0xa8, 0x83, 0xb6, 0x35, 0x5e, 0x27, 0x26,
    0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x00013040[] = {
    0x16, 0xf4, 0xd3, 0x41, 0x48, 0xce, 0x85, 0xba, 0xf1, 0x05, 0x71, 0xba, 0x1f, 0x27, 0xcb, 0x3b
};
static uint8_t spi_flash_0x00013100[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x10, 0xa7, 0x3d,
    0xe7, 0x49, 0x35, 0x3c, 0xa4, 0x2d, 0x20, 0x41
};
static uint8_t amiibo_header[] = {
    0x01, 0x58, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x07
};
static uint8_t amiibo_header2[] = {
    0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x00, 0x07
};
static uint8_t amiibo_data[540] = { 0 };
static uint8_t charging_grip_data[] = {
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x48, 0x45, 0x4a, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32,
    0x31, 0x32, 0x34, 0x37, 0x00, 0x00, 0x7e, 0x05, 0x69, 0x20, 0x01, 0x06, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff
} 

static void set_command_reply(nx2_command_id_t cmd, uint8_t subcommand, uint8_t unknown, uint8_t ACK, const uint8_t *payload, uint8_t payload_len) {
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
            uint8_t buf[] = {0x61, 0x12, 0x50, 0x0d};
            set_command_reply(NX2_CMD_NFC, 0x0c, 0x10, NX2_ACK1, buf, sizeof(buf));
            break;
        case 0x15:
            uint8_t buf2[sizeof(amiibo_header) + sizeof(amiibo_data) + 71] = {0};
            memcpy(buf2, amiibo_header, sizeof(amiibo_header));
            memcpy(buf2 + sizeof(amiibo_header), amiibo_data, sizeof(amiibo_data));
            set_command_reply(NX2_CMD_NFC, 0x15, 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        case 0x05:
            uint8_t buf3[sizeof(amiibo_header2) + 8] = {0};
            memcpy(buf3, amiibo_header2, sizeof(amiibo_header2));
            memcpy(buf3 + sizeof(amiibo_header2), amiibo_data, 8);
            set_command_reply(NX2_CMD_NFC, 0x05, 0x00, NX2_ACK2, buf3, sizeof(buf3));
            break;
        default:
            set_command_reply(NX2_CMD_NFC, last_host_output+3, 0x10, NX2_ACK1, NULL, 0);
            break;
    }
}

static uint8_t read_memory_block(uint32_t address, uint8_t* buffer, uint8_t length) {
    uint8_t len = length;
    switch (address) {
        case 0x00013080:
            memcpy(buffer, spi_flash_0x00013080, sizeof(spi_flash_0x00013080));
            break;
        case 0x000130c0:
            memcpy(buffer, spi_flash_0x000130c0, sizeof(spi_flash_0x000130c0));
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
            //memset(buffer + 32, 0x00, length - 32);
        default:
            memset(buffer, 0xFF, length);
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
            buf[0] = 0x40;
            buf[1] = 0x00;
            buf[2] = 0x00;
            buf[3] = 0x00;
            buf[4] = last_host_output[12];
            buf[5] = last_host_output[13];
            buf[6] = last_host_output[14];
            buf[7] = last_host_output[15];
            uint8_t send_len = read_memory_block(address, buf+8, 64);
            set_command_reply(NX2_CMD_FLASH_MEMORY, last_host_output+3, 0x10, NX2_ACK1, buf, send_len);
            break;
        case 0x02:
            //uint32_t address2 = last_host_output[12] | (last_host_output[13] << 8) | (last_host_output[14] << 16) | (last_host_output[15] << 24);
            //const uint8_t* data = &last_host_output[16];
            //write_memory_block(address2, data, 64);
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x02, 0x10, NX2_ACK1, &last_host_output[8], 8);
            break;
        case 0x03:
            uint8_t buf[4] = {0};
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x03, 0x00, NX2_ACK1, buf, sizeof(buf));
            break;
        case 0x05:
            uint8_t buf3[8] = {0};
            buf3[4] = last_host_output[12];
            buf3[5] = last_host_output[13];
            buf3[6] = last_host_output[14];
            buf3[7] = last_host_output[15];
            set_command_reply(NX2_CMD_FLASH_MEMORY, 0x05, 0x10, NX2_ACK1, buf3, sizeof(buf3));
            break;
        default:
            set_command_reply(NX2_CMD_FLASH_MEMORY, last_host_output+3, 0x10, NX2_ACK1, NULL, 0);
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
            set_command_reply(NX2_CMD_INIT, last_host_output+3, 0x00, NX2_ACK2, NULL, 0);
            break;
        case 0x0D:
            polling_enabled = true;
            for (int i = 0; i < 6; i++) {
                host_address[i] = last_host_output[15 - i];
            }
            uint8_t buf[] = {0x01, 0x00, 0x00, 0x00};
            set_command_reply(NX2_CMD_INIT, last_host_output+3, 0x00, NX2_ACK2, buf, sizeof(buf));
            break;
        case 0x0F:
            uint8_t buf2[] = {0x05, 0x00, 0x00, 0x00};
            set_command_reply(NX2_CMD_INIT, last_host_output+3, 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        default:
            set_command_reply(NX2_CMD_INIT, last_host_output+3, 0x00, NX2_ACK1, NULL, 0);
            break;
    }
}

static void respond_charging_grip(void) {
    switch (last_host_output[3]) {
        case 0x01:
            uint8_t buf[36] = {0};
            memcpy(buf, charging_grip_data, sizeof(buf));
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output+3, 0x00, NX2_ACK2, buf, sizeof(buf));
            break;
        case 0x03:
            uint8_t buf2[68] = {0};
            memcpy(buf2, charging_grip_data, sizeof(buf2));
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output+3, 0x00, NX2_ACK2, buf2, sizeof(buf2));
            break;
        default:
            set_command_reply(NX2_CMD_CHRGRIP, last_host_output+3, 0x00, NX2_ACK2, NULL, 0);
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
        tud_hid_report(NX2_INPUT_REPORT_ID, command_reply, sizeof(command_reply));
    }

    // Send report at ~200Hz to match console polling expectations
    next_report_at = delayed_by_us(get_absolute_time(), 5000);
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
            case NX2_CMD_CHRGRIP:
                respond_charging_grip();
                break;
            case NX2_CMD_PLAYER_LED:
                //respond_player_led();
                set_command_reply(cmd, last_host_output+3, 0x10, NX2_ACK1, NULL, 0);
                break;
            case NX2_CMD_VIBRATION:
                //respond_vibration();
                set_command_reply(cmd, last_host_output+3, 0x10, NX2_ACK1, NULL, 0);
                break;
            default:
                set_command_reply(cmd, 0x7F, NULL, 0); // 未知コマンド
                memset(feature_payload, 0, sizeof(feature_payload));
                feature_payload[0] = 0x81;
                feature_payload[1] = (uint8_t)cmd;
                feature_payload[2] = 0x7F;
                break;
        }

        // コマンド応答をすぐに IN トランザクションで返す
        if (tud_hid_ready()) {
            tud_hid_report(0x00, command_reply, sizeof(command_reply_len));
        }
    }
}

int main(void) {
    stdio_init_all();
    board_init();
    tusb_init();

    next_report_at = get_absolute_time();

    while (true) {
        tud_task();
        hid_task();
    }
    return 0;
}
