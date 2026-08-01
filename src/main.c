#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
//#include "pico/mutex.h"
#include "bsp/board.h"
#include "tusb.h"
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "AES_128_ECB.h"
#include <hardware/flash.h>
#include "pico/flash.h"

#define CHANGE_DESC 0

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

#define EN_AUDIO CFG_TUD_AUDIO
#if EN_AUDIO
// Audio controls
// Current states
// UAC1 Feature Unit 2 (speaker) and 5 (microphone) expose controls only on
// master channel 0. Keep their state separately because both units are queried by
// Windows during audio-driver startup.
static uint8_t audio_mute[2];
static int16_t audio_volume[2];

// Audio test data
uint16_t test_buffer_audio[CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX / 2];
uint8_t audio_out_buffer[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX];
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
static bool nfc_readed = false;
static uint8_t host_address[6] = {0};
static uint8_t bt_ltk[16] = {0};
static uint8_t dev_key[16] = {0x10, 0x5f, 0x1a, 0xc4, 0x25, 0x63, 0x2b, 0xba, 0xe1, 0x05, 0xdf, 0x2c, 0x79, 0xee, 0xf6, 0x5c};
static nx2_output_payload_t output_payload = {
    .left_rumble = {0},
    .right_rumble = {0},
    .reserved = {0},
};

#define STDIO_UART_ID uart0
#define STDIO_UART_TX_PIN 0
#define STDIO_UART_RX_PIN 1
#define GPIO_UART_BAUDRATE 9600
#define GPIO_UART_MARKER 0xAB
#define UART_MIN_PACKET_SIZE 11
#define UART_MAX_PACKET_SIZE 64

typedef struct {
    uint16_t buttons;
    uint8_t hat;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
} uart_controller_state_t;

static volatile uart_controller_state_t g_uart_state = {
    .buttons = 0,
    .hat = 0x08,
    .lx = 0x80,
    .ly = 0x80,
    .rx = 0x80,
    .ry = 0x80,
};

static inline void pack_stick_8bit_to_12bit(uint8_t x8, uint8_t y8, uint8_t out[3]) {
    uint16_t x12 = ((uint16_t)x8) << 4;
    uint16_t y12 = ((uint16_t)y8) << 4;
    out[0] = (uint8_t)(x12 & 0xFF);
    out[1] = (uint8_t)(((x12 >> 8) & 0x0F) | ((y12 & 0x0F) << 4));
    out[2] = (uint8_t)((y12 >> 4) & 0xFF);
}

static void apply_uart_state_to_input_payload(void) {
    uart_controller_state_t state = g_uart_state;
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    uint8_t b2 = 0;

    // data[1] | data[2] << 8 のビット定義:
    // Y,B,A,X,L,R,ZL,ZR,-,+,LClick,RClick,Home,Capture
    if (state.buttons & 0x0002) b0 |= 0x01; // B
    if (state.buttons & 0x0004) b0 |= 0x02; // A
    if (state.buttons & 0x0001) b0 |= 0x04; // Y
    if (state.buttons & 0x0008) b0 |= 0x08; // X
    if (state.buttons & 0x0020) b0 |= 0x10; // R
    if (state.buttons & 0x0080) b0 |= 0x20; // ZR
    if (state.buttons & 0x0200) b0 |= 0x40; // Plus
    if (state.buttons & 0x0800) b0 |= 0x80; // Right Stick Click

    if (state.buttons & 0x0010) b1 |= 0x10; // L
    if (state.buttons & 0x0040) b1 |= 0x20; // ZL
    if (state.buttons & 0x0100) b1 |= 0x40; // Minus
    if (state.buttons & 0x0400) b1 |= 0x80; // Left Stick Click

    if (state.buttons & 0x1000) b2 |= 0x01; // Home
    if (state.buttons & 0x2000) b2 |= 0x02; // Capture

    // HAT(0-7,8=neutral) -> dpad bits
    switch (state.hat & 0x0F) {
        case 0x00: b1 |= 0x08; break;                   // Up
        case 0x01: b1 |= (0x08 | 0x02); break;          // Up + Right
        case 0x02: b1 |= 0x02; break;                   // Right
        case 0x03: b1 |= (0x01 | 0x02); break;          // Down + Right
        case 0x04: b1 |= 0x01; break;                   // Down
        case 0x05: b1 |= (0x01 | 0x04); break;          // Down + Left
        case 0x06: b1 |= 0x04; break;                   // Left
        case 0x07: b1 |= (0x08 | 0x04); break;          // Up + Left
        default: break;                                 // Neutral
    }

    input_payload.buttons[0] = b0;
    input_payload.buttons[1] = b1;
    input_payload.buttons[2] = b2;
    pack_stick_8bit_to_12bit(state.lx, (uint8_t)(0xFF - state.ly), input_payload.lstick);
    pack_stick_8bit_to_12bit(state.rx, (uint8_t)(0xFF - state.ry), input_payload.rstick);
}

static size_t uart_receive_packet(uint8_t *packet, size_t max_len) {
    int ch = 0;
    while (true) {
        if (!uart_is_readable(STDIO_UART_ID)) {
            tight_loop_contents();
            continue;
        }
        ch = uart_getc(STDIO_UART_ID);
        if ((uint8_t)ch == GPIO_UART_MARKER) {
            packet[0] = (uint8_t)ch;
            break;
        }
    }

    size_t len = 1;
    const uint32_t inter_byte_timeout_us = 3000;
    while (len < max_len) {
        if (!uart_is_readable_within_us(STDIO_UART_ID, inter_byte_timeout_us)) {
            break;
        }
        packet[len++] = uart_getc(STDIO_UART_ID);
    }
    return len;
}
uint8_t amiibo_data[540];
static void process_amiibo_chunk(const uint8_t *packet, size_t packet_len) {
    static size_t amiibo_write_index = 0;
    static uint8_t amiibo_frame_count = 0;

    if (packet_len != UART_MAX_PACKET_SIZE) {
        return;
    }

    const size_t chunk_offset = 10; // 11バイト目(1-based)
    const size_t chunk_size = UART_MAX_PACKET_SIZE - chunk_offset; // 54 bytes

    if ((amiibo_write_index + chunk_size) <= sizeof(amiibo_data)) {
        memcpy(&amiibo_data[amiibo_write_index], &packet[chunk_offset], chunk_size);
        amiibo_write_index += chunk_size;
    }
    amiibo_frame_count++;

    if (amiibo_frame_count >= 10) {
        if (amiibo_write_index != sizeof(amiibo_data)) {
            memset(amiibo_data, 0, sizeof(amiibo_data));
        }
        amiibo_write_index = 0;
        amiibo_frame_count = 0;
    }
}

static void core1_entry(void) {
    // core0 がフラッシュを書き換える間、この core を止められるようにする。
    // これを呼ばないと XIP 停止中に core1 が命令フェッチして両コアごとハングする。
    flash_safe_execute_core_init();

    uart_init(STDIO_UART_ID, GPIO_UART_BAUDRATE);
    gpio_set_function(STDIO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(STDIO_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(STDIO_UART_ID, false, false);
    uart_set_format(STDIO_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(STDIO_UART_ID, false);

    uint8_t packet[UART_MAX_PACKET_SIZE] = {0};
    while (true) {
        size_t packet_len = uart_receive_packet(packet, sizeof(packet));
        if (packet_len < UART_MIN_PACKET_SIZE) {
            continue;
        }
        g_uart_state.buttons = (uint16_t)packet[1] | ((uint16_t)packet[2] << 8);
        g_uart_state.hat = packet[3];
        g_uart_state.lx = packet[4];
        g_uart_state.ly = packet[5];
        g_uart_state.rx = packet[6];
        g_uart_state.ry = packet[7];
        process_amiibo_chunk(packet, packet_len);
    }
}
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
uint8_t amiibo_header2[] = {
    0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x00, 0x07
};
static uint8_t amiibo_header3[] = {
    0x00, 0x00, 0x00, 0x00, 0x71, 0x4E, 0x3C, 0x11, 0xCE, 0xEE, 0x3A, 0xCE,0x8E, 0x49, 0xEA,0xB0, 0x71, 0x51, 0x30, 0xCF, 0xED, 0xE4, 0x89, 0x00, 0x9F, 0xB7, 0x96, 0x14, 0x88, 0x72, 0x2B, 0x7A, 0x7F, 0xB0, 0xF4, 0x7D, 0x03, 0x00, 0x3B, 0x3C, 0x77, 0x78, 0x86, 0x00, 0x00
};
uint8_t amiibo_data[540] = {
    0
};
static uint8_t charging_grip_data[] = {
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x48, 0x45, 0x4a, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32,
    0x31, 0x32, 0x34, 0x37, 0x00, 0x00, 0x7e, 0x05, 0x69, 0x20, 0x01, 0x06, 0x01, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff
};
static uint8_t spi_flash_0x00013000[] = {
0x01, 0x00,
0x48, 0x45, 0x4A, 0x31, 0x30, 0x30, 0x30, 0x30, 0x33, 0x31, 0x34, 0x33, 0x37, 0x32, 0x00, 0x00,
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

const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;

// core1 のロックアウト待ち時間。core1 は SIO 割り込みで止まるので即座に応答する。
#define FLASH_SAFE_TIMEOUT_MS 1000

// 消去済みのセクタは全 0xFF なので、そのまま読むと LTK が 0xFF で埋まってしまう。
// マジックを先頭に置いて「保存済みかどうか」を判定する。
#define FLASH_SETTING_MAGIC 0x324B544Cu // 'LTK2'

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t bt_ltk[16];
    uint8_t host_address[6];
} flash_setting_t;

_Static_assert(sizeof(flash_setting_t) <= FLASH_PAGE_SIZE, "setting does not fit in one flash page");

// flash_safe_execute() から呼ばれるワーカー。
// ここに来た時点で割り込みは禁止され、core1 はロックアウトされている。
static void flash_write_worker(void *param)
{
    const uint8_t *write_data = (const uint8_t *)param;
    // Flash消去。
    //  消去単位はflash.hで定義されている FLASH_SECTOR_SIZE(4096Byte) の倍数とする
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    // Flash書き込み。
    //  書込単位はflash.hで定義されている FLASH_PAGE_SIZE(256Byte) の倍数とする
    flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);
}

static void save_setting_to_flash(void)
{
    // W25Q16JVの最終ブロック(Block31)のセクタ0の先頭アドレス = 0x1F0000
    // W25Q16JVの書き込み最小単位 = FLASH_PAGE_SIZE(256Byte)
    // FLASH_PAGE_SIZE(256Byte)はflash.hで定義済
    // flash_range_program() の転送元は RAM 上にある必要がある (スタックで可)
    uint8_t write_data[FLASH_PAGE_SIZE];
    memset(write_data, 0xFF, sizeof(write_data));

    // 保存データのセット
    flash_setting_t setting = { .magic = FLASH_SETTING_MAGIC };
    memcpy(setting.bt_ltk, bt_ltk, sizeof(setting.bt_ltk));
    memcpy(setting.host_address, host_address, sizeof(setting.host_address));
    memcpy(write_data, &setting, sizeof(setting));

    // core1 は XIP フラッシュ上のコードを実行し続けているため、
    // save_and_disable_interrupts() だけでは不十分。イレースで XIP が止まった
    // 瞬間に core1 が命令フェッチできなくなり、両コアごとハングする。
    // flash_safe_execute() が core1 のロックアウトと割り込み禁止をまとめて行う。
    // (core1 側で flash_safe_execute_core_init() を呼んでおくこと)
    flash_safe_execute(flash_write_worker, write_data, FLASH_SAFE_TIMEOUT_MS);
}

// フラッシュ書き込みは 4KB イレースで数十 ms かかり、その間は割り込みを止める。
// USB コールバックの中で実行するとコマンド応答を返す前にバスが止まってしまうので、
// フラグだけ立てておいてメインループ側で処理する。
static volatile bool flash_save_pending = false;
static absolute_time_t flash_save_not_before; // 応答がバスへ出るまでの最低待ち時間
static absolute_time_t flash_save_give_up_at; // ホストが IN を出さない場合の打ち切り

static void request_flash_save(void)
{
    flash_save_pending = true;
    flash_save_not_before = delayed_by_ms(get_absolute_time(), 5);
    flash_save_give_up_at = delayed_by_ms(get_absolute_time(), 200);
}

static void flash_save_task(void)
{
    if (!flash_save_pending) {
        return;
    }
    if (absolute_time_diff_us(get_absolute_time(), flash_save_not_before) > 0) {
        return;
    }
    // コマンド応答を TX FIFO から吐き出し切るまで待つ。
    // ただしホストが IN トークンを出さなくなった場合に保存が永久に走らないよう、
    // 上限時間を過ぎたら待たずに書き込む。
    if (tud_vendor_n_write_available(0) < CFG_TUD_VENDOR_TX_BUFSIZE &&
        absolute_time_diff_us(get_absolute_time(), flash_save_give_up_at) > 0) {
        return;
    }

    flash_save_pending = false;
    save_setting_to_flash();
}

void load_setting_from_flash(void)
{
    // W25Q16JVの最終ブロック(Block31)のセクタ0の先頭アドレス = 0x1F0000
    // XIP_BASE(0x10000000)はflash.hで定義済み
    const flash_setting_t *stored = (const flash_setting_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    if (stored->magic != FLASH_SETTING_MAGIC) {
        // 未保存 (消去済みセクタ)。初期値のままにする
        return;
    }
    memcpy(bt_ltk, stored->bt_ltk, sizeof(bt_ltk));
    memcpy(host_address, stored->host_address, sizeof(host_address));
}

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

    if (payload_len > sizeof(command_reply) - 8) {
        payload_len = sizeof(command_reply) - 8;
    }
    command_reply_len = 8 + payload_len;

    if (payload && payload_len > 0) {
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
            nfc_readed = true;
            break;
        case 0x05:
            uint8_t buf3[61] = {0};
            memcpy(buf3, amiibo_header2, sizeof(amiibo_header2));
            for (int i = 0; i < 3; i++) {
                buf3[sizeof(amiibo_header2) + i] = amiibo_data[i];
            }
            for (int i = 0; i < 4; i++) {
                buf3[sizeof(amiibo_header2) + 3 + i] = amiibo_data[4 + i];
            }
            set_command_reply(NX2_CMD_NFC, 0x05, 0x00, NX2_ACK2, buf3, 61);
            //if (nfc_readed) {
            //    nfc_readed = false;
            //    amiibo_header2[0] = 0x09;
            //}
            break;
        case 0x03:
            //if (last_host_output[5] == 5 && last_host_output[9] == 0x00 && last_host_output[10] == 0x00 && last_host_output[11] == 0x2c && last_host_output[12] == 0x01) {
            //    set_command_reply(NX2_CMD_NFC, 0x03, 0x00, NX2_ACK2, NULL, 0);
            //    input_payload.unknown1[1]++;
            //} else if (last_host_output[5] == 5 && last_host_output[9] == 0xe8 && last_host_output[10] == 0x03 && last_host_output[11] == 0x2c && last_host_output[12] == 0x01) {
            //    set_command_reply(NX2_CMD_NFC, 0x03, 0x00, NX2_ACK2, NULL, 0);
            //    input_payload.unknown1[1]++;
            //}
            if (last_host_output[5] == 5 && last_host_output[11] == 0x2c && last_host_output[12] == 0x01) {
                set_command_reply(NX2_CMD_NFC, 0x03, 0x00, NX2_ACK2, NULL, 0);
                if (input_payload.unknown1[1] == 7) {
                    input_payload.unknown1[1] = 0;
                } else {
                    input_payload.unknown1[1]+=1;
                }
            } else {
                set_command_reply(NX2_CMD_NFC, 0x03, 0x00, NX2_ACK2, NULL, 0);
            }
            if (nfc_readed) {
                nfc_readed = false;
                amiibo_header2[0] = 0x09;
            }
            break;
        case 0x06:
            set_command_reply(NX2_CMD_NFC, 0x06, 0x00, NX2_ACK2, NULL, 0);
            if (input_payload.unknown1[1] == 7) {
                input_payload.unknown1[1] = 0;
            } else {
                input_payload.unknown1[1]+=1;
            }
            amiibo_header2[0] = 0x04;
            break;
        case 0x04:
            set_command_reply(NX2_CMD_NFC, 0x04, 0x00, NX2_ACK2, NULL, 0);
            //if (nfc_readed) {
            //    nfc_readed = false;
            //    amiibo_header2[0] = 0x09;
            //    //input_payload.unknown1[1] = 1;
            //}
            //if (input_payload.unknown1[1] != 0 && input_payload.unknown1[1] != 1) {
            //    input_payload.unknown1[1] = 0;
            //}
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
            request_flash_save();
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
                //for (int i = 0; i < 16; i++) {
                //    buf2[1 + i] = data[15 - i];
                //}
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
                request_flash_save();
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

    // 可変長配列だと受信が 0 バイトのときに buffer[1] が範囲外アクセスになり、
    // 64 バイトを超えると last_host_output を溢れさせるため固定長で受ける
    uint8_t buffer[sizeof(last_host_output)] = {0};
    uint32_t bufsize = tud_vendor_n_read(idx, buffer, sizeof(buffer));

    // コマンドヘッダ (8 バイト) に満たないものは無視する
    if (bufsize < 8) {
        return;
    }

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
                //{
                if (last_host_output[3] == 0x02) {
                    uint8_t buf[] = {last_host_output[8], 0x00, 0x00, 0x00};
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, buf, sizeof(buf));
                } else {
                    set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
                }
                //}
                //set_command_reply(cmd, last_host_output[3], 0x0, NX2_ACK2, NULL, 0);
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

#if EN_AUDIO
void audio_task(void);
#endif

int main(void) {
    stdio_init_all();
    board_init();
    //mutex_init(&__usb_mutex);
    tusb_init(0);
    board_init_after_tusb();
    load_setting_from_flash();
    //printf("Nintendo Switch2 Emulation Started\n");

    next_report_at = get_absolute_time();
    multicore_launch_core1(core1_entry);

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
            apply_uart_state_to_input_payload();
            // 保留中のフラッシュ保存を、USB 応答を返し終えてから実行する
            flash_save_task();
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
  // OUT FIFO を常に読み捨て、ホスト側の再生ストリームを停止させない。
  tud_audio_read(audio_out_buffer, sizeof(audio_out_buffer));

  for (size_t cnt = 0; cnt < sizeof(test_buffer_audio) / 2; cnt++) {
    test_buffer_audio[cnt] = startVal++;
  }
  tud_audio_write((uint8_t *) test_buffer_audio, sizeof(test_buffer_audio));
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// This device advertises Audio Class 1.0 (bcdADC 1.00). UAC1 uses GET_CUR
// (0x81), GET_MIN (0x82), GET_MAX (0x83), and GET_RES (0x84), rather than
// UAC2's CUR/RANGE requests. Stalling these requests makes Windows fail to
// start usbaudio.sys and report Device Manager error Code 10.
enum {
  UAC1_SET_CUR = 0x01,
  UAC1_GET_CUR = 0x81,
  UAC1_GET_MIN = 0x82,
  UAC1_GET_MAX = 0x83,
  UAC1_GET_RES = 0x84,
  UAC1_FU_MUTE = 0x01,
  UAC1_FU_VOLUME = 0x02,
};

static int audio_feature_index(uint8_t entity_id) {
  if (entity_id == 2) return 0; // speaker Feature Unit
  if (entity_id == 5) return 1; // microphone Feature Unit
  return -1;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *request, uint8_t *buffer) {
  (void)rhport;
  (void)request;
  (void)buffer;
  return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *request, uint8_t *buffer) {
  (void)rhport;
  (void)request;
  (void)buffer;
  return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *request, uint8_t *buffer) {
  (void)rhport;
  const uint8_t channel = TU_U16_LOW(request->wValue);
  const uint8_t control = TU_U16_HIGH(request->wValue);
  const int feature = audio_feature_index(TU_U16_HIGH(request->wIndex));

  TU_VERIFY(feature >= 0);
  TU_VERIFY(channel == 0);
  TU_VERIFY(request->bRequest == UAC1_SET_CUR);

  if (control == UAC1_FU_MUTE) {
    TU_VERIFY(request->wLength == 1);
    audio_mute[feature] = buffer[0];
    return true;
  }
  if (control == UAC1_FU_VOLUME) {
    TU_VERIFY(request->wLength == sizeof(int16_t));
    memcpy(&audio_volume[feature], buffer, sizeof(int16_t));
    return true;
  }
  return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *request) {
  (void)rhport;
  (void)request;
  return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *request) {
  (void)rhport;
  (void)request;
  return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *request) {
  const uint8_t channel = TU_U16_LOW(request->wValue);
  const uint8_t control = TU_U16_HIGH(request->wValue);
  const int feature = audio_feature_index(TU_U16_HIGH(request->wIndex));

  TU_VERIFY(feature >= 0);
  TU_VERIFY(channel == 0);

  if (control == UAC1_FU_MUTE && request->bRequest == UAC1_GET_CUR) {
    return tud_audio_buffer_and_schedule_control_xfer(
        rhport, request, &audio_mute[feature], sizeof(audio_mute[feature]));
  }

  if (control == UAC1_FU_VOLUME) {
    static const int16_t volume_min = -90 * 256;
    static const int16_t volume_max = 0;
    static const int16_t volume_res = 256;
    const int16_t *value = NULL;

    switch (request->bRequest) {
      case UAC1_GET_CUR: value = &audio_volume[feature]; break;
      case UAC1_GET_MIN: value = &volume_min; break;
      case UAC1_GET_MAX: value = &volume_max; break;
      case UAC1_GET_RES: value = &volume_res; break;
      default: return false;
    }
    return tud_audio_buffer_and_schedule_control_xfer(
        rhport, request, (void *)value, sizeof(*value));
  }

  return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  startVal = 0;

  return true;
}
#endif
