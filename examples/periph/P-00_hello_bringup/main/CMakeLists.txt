# SPDX-License-Identifier: MIT
# P-00_hello_bringup main 组件：纯日志例程（无组件依赖，最小脚手架）
# 输出层：默认 USB-Serial-JTAG（真板）；EXP_QEMU_UART=1 走 UART0（QEMU chardev 出口）
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()

set(P00_SRCS "hello_bringup_main.c")
set(P00_INCS ".")

idf_component_register(
    SRCS ${P00_SRCS}
    INCLUDE_DIRS ${P00_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EXP_QEMU_UART=${EXP_QEMU_UART})