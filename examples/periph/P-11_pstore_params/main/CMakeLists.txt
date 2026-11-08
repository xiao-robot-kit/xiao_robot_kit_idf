# SPDX-License-Identifier: MIT
# P-11_pstore_params main component: pstore component + flash raw-partition
# backend (media) + parameter read/write.
#
# Component wiring (public-repo layout):
#   - xiao_pstore library target (pstore.c) from repository root
#   - _shared/pstore_flash.c  raw partition media backend (ESP partition API)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../_shared)

# EXP_QEMU_UART: 1 = output on UART0 (QEMU chardev path); 0 (default) = USB-Serial-JTAG
if(NOT DEFINED EXP_QEMU_UART)
    set(EXP_QEMU_UART 0)
endif()

set(P11_SRCS
    "p11_pstore_main.c"
    "${SHARED_DIR}/pstore_flash.c")
set(P11_INCS
    "."
    "${SHARED_DIR}")

# REQUIRES listed unconditionally (IDF component discovery runs before cmake -D
# cache evaluation, same rule as store_demo)
idf_component_register(
    SRCS ${P11_SRCS}
    INCLUDE_DIRS ${P11_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_partition nvs_flash
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_pstore)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EXP_QEMU_UART=${EXP_QEMU_UART})