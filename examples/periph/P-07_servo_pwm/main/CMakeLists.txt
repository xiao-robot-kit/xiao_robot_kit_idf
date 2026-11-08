# SPDX-License-Identifier: MIT
# P-07 servo_pwm main component: 4x 50Hz servo + voltage-linked alarm
# (QEMU-testable).
#
# Component wiring (public-repo layout):
#   - xiao_motor library target from repository root (add_subdirectory below)
#   - _shared/motor_hal_sim.c  synthetic PWM HAL (QEMU: no LEDC model)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../_shared)

# P_SIM_MOTOR (1 = synthetic HAL, QEMU-testable; 0 = real HW LEDC HAL - not
# provided, rejected by #error in C): real-board output needs LEDC timers
# (D15/D9/D0/D8 <-> 50Hz); port layer only ships motor_hal_sim.c. Setting 0 is
# explicitly rejected in C (ERR-004 discipline).
set(P_SIM_MOTOR 1 CACHE STRING "1=synthetic HAL (default, QEMU-testable); 0=real LEDC HAL (not provided, build rejected)")

# EXP_QEMU_UART (1 = output on UART0, QEMU first -serial slot; 0 = USB-Serial-JTAG)
set(EXP_QEMU_UART 1 CACHE STRING "1=telemetry UART0 (QEMU); 0=USB-Serial-JTAG (real board)")

# P_ADC_RAW (1 = synthetic voltage demo curve, no battery needed, QEMU can
#            assert WARN lines; 0 = real ADC1_CH5 GPIO16 + 100K:30K divider)
set(P_ADC_RAW 1 CACHE STRING "1=synthetic voltage (default); 0=real ADC sampling (real board)")

idf_component_register(
    SRCS
        "p07_servo_main.c"
        "${SHARED_DIR}/motor_hal_sim.c"
    INCLUDE_DIRS
        "."
        "${SHARED_DIR}"
    REQUIRES esp_adc esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_motor)

# triple-macro compile contract (all 0/1 states explicitly defined; C #if picks)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    P_SIM_MOTOR=${P_SIM_MOTOR} EXP_QEMU_UART=${EXP_QEMU_UART} P_ADC_RAW=${P_ADC_RAW})