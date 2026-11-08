# SPDX-License-Identifier: MIT
# P-05 motor_gc9112 main component: dual DC bridge (M1/M2) + synthetic HAL
# (QEMU-testable).
#
# Component wiring (public-repo layout):
#   - xiao_motor library target from repository root (add_subdirectory below)
#     carries motor.c / motor_cfg_presets.c / pid.c / pwm_timer.c
#   - _shared/motor_hal_sim.c  synthetic PWM HAL (QEMU: no LEDC model)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../_shared)

# P_SIM_MOTOR (1 = synthetic HAL, QEMU-testable; 0 = real HW LEDC HAL - not
# provided, rejected by #error in C): GC9112 real-board output needs LEDC
# timers (D1/D2, D19/D18 <-> 20kHz); port layer only ships motor_hal_sim.c
# (QEMU esp32s3 has no LEDC model). Setting 0 is explicitly rejected in C
# (never silently build "fake hardware" firmware, ERR-004 discipline).
set(P_SIM_MOTOR 1 CACHE STRING "1=synthetic HAL (default, QEMU-testable); 0=real LEDC HAL (not provided, build rejected)")

# EXP_QEMU_UART (1 = output on UART0, QEMU first -serial slot; 0 = USB-Serial-JTAG)
set(EXP_QEMU_UART 1 CACHE STRING "1=telemetry UART0 (QEMU); 0=USB-Serial-JTAG (real board)")

idf_component_register(
    SRCS
        "p05_motor_main.c"
        "${SHARED_DIR}/motor_hal_sim.c"
    INCLUDE_DIRS
        "."
        "${SHARED_DIR}"
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_motor)

# dual-macro compile contract (both 0/1 states explicitly defined; C #if picks;
# undefined = rejected by default)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    P_SIM_MOTOR=${P_SIM_MOTOR} EXP_QEMU_UART=${EXP_QEMU_UART})