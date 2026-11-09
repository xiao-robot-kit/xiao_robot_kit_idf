# SPDX-License-Identifier: MIT
# P-11_pstore_params main component: pstore component + flash raw-partition
# backend (media) + parameter read/write.
#
# Component wiring (public-repo layout):
#   - xiao_pstore library target (pstore.c) from repository root
#   - port/esp_idf/pstore_flash.c  raw partition media backend (ESP partition API)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(PORT_ESP_IDF_DIR ${LIBS_ROOT}/port/esp_idf)


set(P11_SRCS
    "p11_pstore_main.c"
    "${PORT_ESP_IDF_DIR}/pstore_flash.c")
set(P11_INCS
    "."
    "${PORT_ESP_IDF_DIR}")

# REQUIRES listed unconditionally (IDF component discovery runs before cmake -D
# cache evaluation)
idf_component_register(
    SRCS ${P11_SRCS}
    INCLUDE_DIRS ${P11_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_partition nvs_flash
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_pstore)