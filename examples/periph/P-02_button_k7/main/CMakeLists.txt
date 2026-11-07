# SPDX-License-Identifier: MIT
# P-02_button_k7 main 组件：D11(GPIO11) 按键消抖 + 短按/长按事件
#
# 双数据源（编译期宏 P_SIM_KEY，默认 1）：
#   - P_SIM_KEY=1：合成按键序列（内存事件表：短按×3 + 长按×2 + 抖动注入，
#     5.6s 跑完；QEMU 可测：QEMU 的 GPIO 无外部按键驱动）
#   - P_SIM_KEY=0：真 GPIO11（内部上拉，按下=低；真板 + 手按）
#
# 输出层 EXP_QEMU_UART：1 = UART0（QEMU 第 1 个 -serial 槽有 chardev 出口）；
#   0（默认）= USB-Serial-JTAG（真板 USB-C 直连）

# P_SIM_KEY：1（默认）= 合成按键序列；0 = 真 GPIO11
if(NOT DEFINED P_SIM_KEY)
    set(P_SIM_KEY 1)
endif()

# EXP_QEMU_UART：1 = UART0 文本输出（QEMU）；0（默认）= USB-Serial-JTAG
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()

set(P02_SRCS "p02_button_main.c")
set(P02_INCS ".")

# REQUIRES 必须无条件全列（同 exp2_calib 实测教训）：条件化 REQUIRES 破坏组件发现。
# SIM 模式未用的组件仅链接，无害。
idf_component_register(
    SRCS ${P02_SRCS}
    INCLUDE_DIRS ${P02_INCS}
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_KEY=${P_SIM_KEY} EXP_QEMU_UART=${EXP_QEMU_UART})