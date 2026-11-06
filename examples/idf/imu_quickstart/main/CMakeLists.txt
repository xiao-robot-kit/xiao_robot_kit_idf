# SPDX-License-Identifier: MIT
# imu_quickstart main component.
#
# Pulls the C core library from the repository root (three levels up) through
# its library-mode CMakeLists. The non-standalone branch there skips
# install/export/tests, so only the four static libraries are built.
# xiao_imu carries its own PUBLIC include path (src/imu, flat layout).

set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

idf_component_register(
    SRCS
        "main.c"
        "bus_i2c_idf.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_i2c
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_imu)
