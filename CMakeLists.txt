cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(nx2_pro_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(nx2_pro_controller
    src/main.c
    src/usb_descriptors.c
)

target_include_directories(nx2_pro_controller PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_compile_definitions(nx2_pro_controller PRIVATE
    PICO_TINYUSB_DEVICE=1
    CFG_TUD_ENABLED=1
    CFG_TUH_ENABLED=0
    CFG_TUSB_MCU=OPT_MCU_RP2040
)

target_link_libraries(nx2_pro_controller PRIVATE
    pico_stdlib
    tinyusb_device
    tinyusb_board
)

pico_add_extra_outputs(nx2_pro_controller)

pico_enable_stdio_usb(nx2_pro_controller 0)
pico_enable_stdio_uart(nx2_pro_controller 1)
