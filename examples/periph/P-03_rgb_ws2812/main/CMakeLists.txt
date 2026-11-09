# SPDX-License-Identifier: MIT
# P-03_rgb_ws2812 main 组件：RMT 驱动 WS2812 + 自研 HSV 色轮
#
# 双数据源（编译期宏 P_SIM_LED，默认 1）：
#   - P_SIM_LED=1：合成灯带（相位→色轮渐变 + 单灯定点，内存缓冲零硬件，
#   - P_SIM_LED=0：真 RMT 发 GPIO13（真板 + J7 灯带；esp_driver_rmt 自带零外部依赖）
#
#   0（默认）= USB-Serial-JTAG（真板 USB-C 直连）

# P_SIM_LED：1（默认）= 合成灯带；0 = 真 RMT
if(NOT DEFINED P_SIM_LED)
    set(P_SIM_LED 1)
endif()


set(P03_SRCS "p03_rgb_ws2812_main.c")
set(P03_INCS ".")

# REQUIRES 必须无条件全列（历史实测教训）：条件化 REQUIRES 破坏组件发现。
# RMT 用 esp_driver_rmt（v6.1 自带组件）；SIM 模式未用的组件仅链接，无害。
idf_component_register(
    SRCS ${P03_SRCS}
    INCLUDE_DIRS ${P03_INCS}
    REQUIRES esp_driver_rmt esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_LED=${P_SIM_LED})