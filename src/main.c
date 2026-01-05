#include <stdio.h>
#include "pico/stdlib.h"
#include "bsp/board.h"
#include "tusb.h"
#include <string.h>

static const uint8_t NX2_INPUT_REPORT_ID = 0x30;
static const uint8_t NX2_OUTPUT_REPORT_ID = 0x21;
static const uint8_t NX2_FEATURE_REPORT_ID = 0x80;

// commands.md にあるコマンド ID 一覧 (HID デスクリプタとは異なるベンダー独自領域)
typedef enum {
    NX2_CMD_GET_PROTOCOL_VERSION = 0x01,
    NX2_CMD_GET_DEVICE_INFO = 0x02,
    NX2_CMD_GET_UNIQUE_ID = 0x03,
    NX2_CMD_SET_SHIP_MODE = 0x04,
    NX2_CMD_START_POLLING = 0x10,
    NX2_CMD_STOP_POLLING = 0x11,
    NX2_CMD_SET_PLAYER_LED = 0x12,
    NX2_CMD_SET_HOME_LED = 0x13,
    NX2_CMD_SET_RUMBLE = 0x14,
    NX2_CMD_SET_IMU_ENABLE = 0x15,
    NX2_CMD_CALIBRATION = 0x20,
    NX2_CMD_UNKNOWN = 0xFF,
} nx2_command_id_t;

// Input report payload (Report ID は別引数で付加)
typedef struct __attribute__((packed)) {
    uint16_t buttons;  // 16 digital buttons
    uint8_t hat;       // 0-7 directions, 8 = released
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t lt;
    uint8_t rt;
    uint8_t imu[12];      // IMU placeholder
    uint8_t reserved[38]; // 64B 全体の残りをゼロ埋め
} nx2_input_payload_t;

static nx2_input_payload_t idle_payload = {
    .buttons = 0,
    .hat = 0x08, // center
    .lx = 0,
    .ly = 0,
    .rx = 0,
    .ry = 0,
    .lt = 0x00,
    .rt = 0x00,
    .imu = {0},
    .reserved = {0},
};

// commands.md で規定されている「実レポート構成」を模した 64 バイトの応答バッファ。
// HID レポートディスクリプタとは異なり、先頭にコマンド ID とステータスを入れる。
static uint8_t command_reply[63] = {0};
static uint8_t feature_payload[63] = {0x01};
static uint8_t last_host_output[63] = {0};
static uint16_t last_host_output_len = 0;
static absolute_time_t next_report_at;
static bool polling_enabled = false;

static void set_command_reply(nx2_command_id_t cmd, uint8_t status, const uint8_t *payload, uint8_t payload_len) {
    memset(command_reply, 0, sizeof(command_reply));
    command_reply[0] = (uint8_t)cmd;
    command_reply[1] = status; // 0x00 = OK

    if (payload && payload_len > 0) {
        if (payload_len > sizeof(command_reply) - 2) {
            payload_len = sizeof(command_reply) - 2;
        }
        memcpy(&command_reply[2], payload, payload_len);
    }
}

static void respond_protocol_version(void) {
    // commands.md: 0x01 プロトコルバージョン要求 -> 0x0003 を返す
    const uint8_t proto[2] = {0x03, 0x00};
    set_command_reply(NX2_CMD_GET_PROTOCOL_VERSION, 0x00, proto, sizeof(proto));

    memset(feature_payload, 0, sizeof(feature_payload));
    feature_payload[0] = 0x81; // ACK
    feature_payload[1] = NX2_CMD_GET_PROTOCOL_VERSION;
    feature_payload[2] = proto[0];
    feature_payload[3] = proto[1];
}

static void respond_device_info(void) {
    // commands.md: 0x02 デバイス情報。PID/VID/バージョンなどを 8byte で返す想定。
    uint8_t info[8] = {0};
    info[0] = 0x69; // PID LSB
    info[1] = 0x20; // PID MSB
    info[2] = 0x7E; // VID LSB
    info[3] = 0x05; // VID MSB
    info[4] = 0x00; // hw major
    info[5] = 0x03; // hw minor (= bcdUSB)
    info[6] = 0x00; // fw major placeholder
    info[7] = 0x01; // fw minor placeholder

    set_command_reply(NX2_CMD_GET_DEVICE_INFO, 0x00, info, sizeof(info));

    memset(feature_payload, 0, sizeof(feature_payload));
    feature_payload[0] = 0x81;
    feature_payload[1] = NX2_CMD_GET_DEVICE_INFO;
    memcpy(&feature_payload[2], info, sizeof(info));
}

static void respond_start_polling(nx2_command_id_t cmd) {
    polling_enabled = (cmd == NX2_CMD_START_POLLING);
    uint8_t payload[sizeof(idle_payload)] = {0};
    memcpy(payload, &idle_payload, sizeof(idle_payload));
    set_command_reply(cmd, 0x00, payload, (uint8_t)sizeof(payload));
    memset(feature_payload, 0, sizeof(feature_payload));
    feature_payload[0] = 0x81;
    feature_payload[1] = (uint8_t)cmd;
    feature_payload[2] = polling_enabled ? 0x01 : 0x00;
}

static void respond_led_or_rumble(nx2_command_id_t cmd, uint8_t const *buffer, uint16_t bufsize) {
    uint8_t ack[4] = {0};
    ack[0] = (bufsize > 1) ? buffer[1] : 0x00; // sub parameter echo
    ack[1] = (bufsize > 2) ? buffer[2] : 0x00;
    ack[2] = (bufsize > 3) ? buffer[3] : 0x00;
    ack[3] = (bufsize > 4) ? buffer[4] : 0x00;
    set_command_reply(cmd, 0x00, ack, sizeof(ack));

    memset(feature_payload, 0, sizeof(feature_payload));
    feature_payload[0] = 0x81;
    feature_payload[1] = (uint8_t)cmd;
    memcpy(&feature_payload[2], ack, sizeof(ack));
}

static void respond_calibration(void) {
    // 簡易キャリブレーション: ゼロ値を返す
    uint8_t calib[6] = {0};
    set_command_reply(NX2_CMD_CALIBRATION, 0x00, calib, sizeof(calib));

    memset(feature_payload, 0, sizeof(feature_payload));
    feature_payload[0] = 0x81;
    feature_payload[1] = NX2_CMD_CALIBRATION;
    memcpy(&feature_payload[2], calib, sizeof(calib));
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
    idle_payload.buttons = 0;
    idle_payload.hat = 0x08;
    idle_payload.lx = idle_payload.ly = idle_payload.rx = idle_payload.ry = 0;
    idle_payload.lt = idle_payload.rt = 0x00;
    polling_enabled = false;
    memset(command_reply, 0, sizeof(command_reply));
}

// Nintendo Switch2 からの GET_REPORT/SET_REPORT に応答
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance;

    // コントロール転送(GET_REPORT)では入力/Feature レポートを返す
    if (report_type == HID_REPORT_TYPE_INPUT && (report_id == NX2_INPUT_REPORT_ID || report_id == 0)) {
        uint16_t copy_len = (uint16_t)(sizeof(command_reply) + 1);
        if (copy_len > reqlen) {
            copy_len = reqlen;
        }
        buffer[0] = NX2_INPUT_REPORT_ID;
        memcpy(buffer + 1, command_reply, copy_len - 1);
        return copy_len;
    }

    if (report_type == HID_REPORT_TYPE_FEATURE && report_id == NX2_FEATURE_REPORT_ID) {
        uint16_t copy_len = (uint16_t)(sizeof(feature_payload) + 1);
        if (copy_len > reqlen) {
            copy_len = reqlen;
        }
        buffer[0] = NX2_FEATURE_REPORT_ID;
        memcpy(buffer + 1, feature_payload, copy_len - 1);
        return copy_len;
    }

    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;

    // 出力レポートは commands.md の実コマンドに従って解釈する
    if (report_type == HID_REPORT_TYPE_OUTPUT && report_id == NX2_OUTPUT_REPORT_ID && buffer && bufsize) {
        last_host_output_len = bufsize < sizeof(last_host_output) ? bufsize : (uint16_t)sizeof(last_host_output);
        memcpy(last_host_output, buffer, last_host_output_len);

        nx2_command_id_t cmd = (bufsize > 0) ? (nx2_command_id_t)buffer[0] : NX2_CMD_UNKNOWN;
        switch (cmd) {
            case NX2_CMD_GET_PROTOCOL_VERSION:
                respond_protocol_version();
                break;
            case NX2_CMD_GET_DEVICE_INFO:
                respond_device_info();
                break;
            case NX2_CMD_GET_UNIQUE_ID:
            case NX2_CMD_SET_SHIP_MODE:
                // 0x03/0x04 は現状ペイロード無しの ACK のみ
                set_command_reply(cmd, 0x00, NULL, 0);
                memset(feature_payload, 0, sizeof(feature_payload));
                feature_payload[0] = 0x81;
                feature_payload[1] = (uint8_t)cmd;
                break;
            case NX2_CMD_START_POLLING:
            case NX2_CMD_STOP_POLLING:
                respond_start_polling(cmd);
                break;
            case NX2_CMD_SET_PLAYER_LED:
            case NX2_CMD_SET_HOME_LED:
            case NX2_CMD_SET_RUMBLE:
            case NX2_CMD_SET_IMU_ENABLE:
                respond_led_or_rumble(cmd, buffer, bufsize);
                break;
            case NX2_CMD_CALIBRATION:
                respond_calibration();
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
            tud_hid_report(NX2_INPUT_REPORT_ID, command_reply, sizeof(command_reply));
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
