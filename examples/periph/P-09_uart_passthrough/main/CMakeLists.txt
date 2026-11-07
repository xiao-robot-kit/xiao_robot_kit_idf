# SPDX-License-Identifier: MIT
# P-09_uart_passthrough main 组件：UART 环回/透传（J10 UART0 115200 8N1）
if(NOT DEFINED P_SIM_UART)
    set(P_SIM_UART 1)
endif()
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()
set(P09_SRCS "p09_uart_main.c")
set(P09_INCS ".")
idf_component_register(
    SRCS ${P09_SRCS}
    INCLUDE_DIRS ${P09_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_UART=${P_SIM_UART} EXP_QEMU_UART=${EXP_QEMU_UART})
