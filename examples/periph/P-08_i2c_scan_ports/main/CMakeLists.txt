# SPDX-License-Identifier: MIT
# P-08_i2c_scan_ports main component: I2C bus scan + address-table management.
#
# Shared wiring (public-repo layout):
#   - _shared/port_i2c.h + imu_hal_i2c.c  I2C platform endpoint (real mode
#     P_SIM_SCAN=0); SIM mode is a zero-I2C synthetic echo
set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../_shared)

#              dev=0x6A sim echo + NEW 0x40); 0 = real I2C scan via _shared
#              endpoint, probe any 0x03-0x77
if(NOT DEFINED P_SIM_SCAN)
    set(P_SIM_SCAN 1)
endif()

#   0 (default) = USB-Serial-JTAG (real board)

set(P08_SRCS "p08_i2c_scan_main.c")
set(P08_INCS ".")

if(NOT P_SIM_SCAN)
    # Real scan mode: I2C HAL endpoint from _shared
    list(APPEND P08_SRCS "${SHARED_DIR}/imu_hal_i2c.c")
    list(APPEND P08_INCS "${SHARED_DIR}")
endif()

# REQUIRES must list all components unconditionally (IDF component discovery
# runs before cmake -D cache evaluation; conditional REQUIRES would drop
# esp_driver_i2c from the recorded dep set -> real-mode build fails).
idf_component_register(
    SRCS ${P08_SRCS}
    INCLUDE_DIRS ${P08_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_driver_i2c esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    P_SIM_SCAN=${P_SIM_SCAN})