# SPDX-License-Identifier: MIT
# P-06 encoder_read main component: N20 encoder + synthetic HAL (QEMU-testable).
#
# Component wiring (public-repo layout):
#   - xiao_motor library target from repository root (add_subdirectory below)
#   - _shared/motor_hal_sim.c  synthetic PWM/PCNT HAL (QEMU: no LEDC/PCNT model)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../_shared)

# P_SIM_MOTOR (1 = synthetic HAL, QEMU-testable; 0 = real HW LEDC HAL - not
# provided, rejected by #error in C): motor.enc.n20 real-board output needs
# LEDC timers (D1/D2 <-> 20kHz) + PCNT readback; port layer only ships
# motor_hal_sim.c. Setting 0 is explicitly rejected in C (never silently build
# "fake hardware" firmware, ERR-004 discipline).
set(P_SIM_MOTOR 1 CACHE STRING "1=synthetic HAL (default, QEMU-testable); 0=real LEDC HAL (not provided, build rejected)")

# EXP_QEMU_UART (1 = output on UART0, QEMU first -serial slot; 0 = USB-Serial-JTAG)
set(EXP_QEMU_UART 1 CACHE STRING "1=telemetry UART0 (QEMU); 0=USB-Serial-JTAG (real board)")

idf_component_register(
    SRCS
        "p06_enc_main.c"
        "${SHARED_DIR}/motor_hal_sim.c"
    INCLUDE_DIRS
        "."
        "${SHARED_DIR}"
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_motor)

# dual-macro compile contract (states 0/1 both explicitly defined; C #if picks)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    P_SIM_MOTOR=${P_SIM_MOTOR} EXP_QEMU_UART=${EXP_QEMU_UART})