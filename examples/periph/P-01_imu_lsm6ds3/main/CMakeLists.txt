# SPDX-License-Identifier: MIT
# P-01_imu_lsm6ds3 main component: poll LSM6DS3 over I2C (WHO_AM_I + 100Hz stream,
# 5min continuous read + bus scan).
#
# Component wiring (public-repo layout):
#   - xiao_imu library target  from repository root (add_subdirectory, below)
#   - examples/common/sim/imu_sim.c    synthetic data source (P_SIM_IMU=1)
#   - port/esp_idf/imu_hal_i2c.c       real I2C platform endpoint (P_SIM_IMU=0)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(PORT_ESP_IDF_DIR ${LIBS_ROOT}/port/esp_idf)
set(SIM_DIR ${LIBS_ROOT}/examples/common/sim)

#              WHO/SCAN are synthesized echoes)
#             0 = real I2C sensor (LSM6DS3TR-C @0x6A via port/esp_idf I2C endpoint)
if(NOT DEFINED P_SIM_IMU)
    set(P_SIM_IMU 1)
endif()

#   path; UART0 = first -serial slot has a chardev exit);
#   0 (default) = USB-Serial-JTAG (real board)

set(P01_SRCS "p01_imu_lsm6ds3_main.c")
set(P01_INCS
    "."
    "${PORT_ESP_IDF_DIR}"
    "${SIM_DIR}"
)

if(P_SIM_IMU)
    list(APPEND P01_SRCS "${SIM_DIR}/imu_sim.c")
else()
    # Real sensor stream: I2C HAL endpoint + driver + preset table come from
    # the xiao_imu library (SRCS via target link, below).
    list(APPEND P01_SRCS "${PORT_ESP_IDF_DIR}/imu_hal_i2c.c")
endif()

# REQUIRES must list all components unconditionally (IDF component discovery
# runs before this file's cmake -D cache evaluation; conditional REQUIRES would
# record a dep set missing esp_driver_i2c -> include propagation breaks on
# P_SIM_IMU=0). Unused components are just linked, harmless.
idf_component_register(
    SRCS ${P01_SRCS}
    INCLUDE_DIRS ${P01_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_driver_i2c esp_timer
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_imu)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_IMU=${P_SIM_IMU})