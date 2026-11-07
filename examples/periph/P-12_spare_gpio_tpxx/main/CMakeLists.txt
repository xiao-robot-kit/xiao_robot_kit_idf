# SPDX-License-Identifier: MIT
# P-12_spare_gpio_tpxx main 组件：TP20/TP21（MTDI/MTDO）备用 GPIO 双向互测
#
# 双数据源（编译期宏 P_SIM_SPARE，默认 1）：
#   - P_SIM_SPARE=1：合成 GPIO 回环（内存电平表模拟 TP20↔TP21 飞线互联，
#     QEMU 可测：QEMU 的 GPIO 无外部飞线电平）
#   - P_SIM_SPARE=0：真硬件 GPIO41/42（真板 + TP20/21 飞线，读回真实电平）
#
# 输出层 EXP_QEMU_UART：1 = UART0（QEMU 第 1 个 -serial 槽有 chardev 出口）；
#   0（默认）= USB-Serial-JTAG（真板 USB-C 直连）

# P_SIM_SPARE：1（默认）= 合成回环；0 = 真硬件
if(NOT DEFINED P_SIM_SPARE)
    set(P_SIM_SPARE 1)
endif()

# EXP_QEMU_UART：1 = UART0 文本输出（QEMU）；0（默认）= USB-Serial-JTAG
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()

set(P12_SRCS "p12_spare_main.c")
set(P12_INCS ".")

# REQUIRES 必须无条件全列（同 exp2_calib 实测教训）：条件化 REQUIRES 破坏组件发现。
# SIM 模式未用的组件仅链接，无害。
idf_component_register(
    SRCS ${P12_SRCS}
    INCLUDE_DIRS ${P12_INCS}
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_SPARE=${P_SIM_SPARE} EXP_QEMU_UART=${EXP_QEMU_UART})