# SPDX-License-Identifier: MIT
# P-00_hello_bringup main 组件：纯日志例程（无组件依赖，最小脚手架）

set(P00_SRCS "hello_bringup_main.c")
set(P00_INCS ".")

idf_component_register(
    SRCS ${P00_SRCS}
    INCLUDE_DIRS ${P00_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

