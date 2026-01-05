#pragma once

// TinyUSB configuration for RP2040 Pro Controller emulation

#define CFG_TUSB_MCU OPT_MCU_RP2040
#define BOARD_DEVICE_RHPORT_NUM 0
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO

#define CFG_TUD_ENDPOINT0_SIZE 64

// HID only device configuration
#define CFG_TUD_HID 1
#define CFG_TUD_HID_EP_BUFSIZE 64

// No CDC/MIDI etc.
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_NET 0
#define CFG_TUD_CUSTOM_CLASS 0

// Disable USB high speed since RP2040 is FS only
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
