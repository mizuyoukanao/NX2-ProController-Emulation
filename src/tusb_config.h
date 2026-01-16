#pragma once

// TinyUSB configuration for RP2040 Pro Controller emulation

#define CHANGE_DESC 1

#define CFG_TUSB_MCU OPT_MCU_RP2040
#define BOARD_DEVICE_RHPORT_NUM 0
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO

#define CFG_TUSB_DEBUG 0

#define CFG_TUD_ENABLED 1

#define CFG_TUD_ENDPOINT0_SIZE 64

// HID only device configuration
#define CFG_TUD_HID 1
#define CFG_TUD_HID_EP_BUFSIZE 64
#if CHANGE_DESC
#define CFG_TUD_AUDIO 0
#else
#define CFG_TUD_AUDIO 1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 1
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX TU_MAX(CFG_TUD_AUDIO10_FUNC_1_FORMAT_1_EP_SZ_OUT, TU_MAX(CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_OUT, CFG_TUD_AUDIO20_FUNC_1_FORMAT_2_EP_SZ_OUT))
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE                              48000
#define CFG_TUD_AUDIO_ENABLE_EP_IN                                    1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX                    2                                       // Driver gets this info from the descriptors - we define it here to use it to setup the descriptors and to do calculations with it below
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                            1                                       // Driver gets this info from the descriptors - we define it here to use it to setup the descriptors and to do calculations with it below - be aware: for different number of channels you need another descriptor!
#define CFG_TUD_AUDIO_EP_SZ_IN                                        TUD_AUDIO_EP_SIZE(TUD_OPT_HIGH_SPEED, CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX                             CFG_TUD_AUDIO_EP_SZ_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ                          (TUD_OPT_HIGH_SPEED ? 32 : 4) * CFG_TUD_AUDIO_EP_SZ_IN // Example write FIFO every 1ms, so it should be 8 times larger for HS device
#endif

// No CDC/MIDI etc.
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_CUSTOM_CLASS 0
#define CFG_TUD_VENDOR 1
#define CFG_TUD_VENDOR_RX_BUFSIZE 1024
#define CFG_TUD_VENDOR_TX_BUFSIZE 1024

// Disable USB high speed since RP2040 is FS only
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
