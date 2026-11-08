# SPDX-License-Identifier: MIT
# P-10_imu_heater main component: constant-temperature heater closed loop
# (imu_heater control layer + temp_comp compensation hookup).
#
# Component wiring (public-repo layout):
#   - xiao_imu_heater library (control state machine)
#   - xiao_calibration library (temp_comp)
#   - xiao_imu library (imu_types.h, transitively required by temp_comp.h)
#   all pulled in via add_subdirectory(repo root) below.
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

# P_SIM_HEAT: 1 (default) = synthetic temperature scenario (zero peripheral
#             deps, QEMU-testable, exercises control+protection chain);
#             0 = real hardware mode (LEDC heater PWM + IMU temp readout - HAL
#             not provided, rejected by #error in main)
# QEMU builds must use P_SIM_HEAT=1
if(NOT DEFINED P_SIM_HEAT)
    set(P_SIM_HEAT 1)
endif()
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()

set(P10_SRCS "p10_imu_heater_main.c")
set(P10_INCS ".")

idf_component_register(
    SRCS ${P10_SRCS}
    INCLUDE_DIRS ${P10_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_timer
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC
    xiao_imu_heater xiao_calibration xiao_imu)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_HEAT=${P_SIM_HEAT} EXP_QEMU_UART=${EXP_QEMU_UART})