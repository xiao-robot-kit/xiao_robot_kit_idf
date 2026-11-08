# SPDX-License-Identifier: MIT
# store_demo main component (public-repo library layout).
#
# Library code lives in the repository root CMakeLists (the xiao_* targets);
# only this project's own sources and the shared port layer / synthetic data
# sources from examples/_shared are compiled in here.
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)   # repository root
set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../_shared)

# EXP_SIM is unused by this example (kept so the shared frame path stays uniform)
set(EXP_SIM 1 CACHE STRING "unused")
# EXP_QEMU_UART (1 = frames on UART0; 0 = USB-Serial-JTAG)
set(EXP_QEMU_UART 1 CACHE STRING "1=frames on UART0 (QEMU, console); 0=USB-Serial-JTAG")

set(STORE_SRCS
        "store_demo_main.c"
        "store_demo.c"
        "${SHARED_DIR}/pstore_flash.c"
)
set(STORE_INCS
        "."
        "${SHARED_DIR}"
)

# REQUIRES is unconditionally complete: IDF discovers components before
# cmake -D cache evaluation, so a conditional REQUIRES loses the include path.
idf_component_register(
    SRCS ${STORE_SRCS}
    INCLUDE_DIRS ${STORE_INCS}
    REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_partition nvs_flash
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_pstore xiao_proto)

# Both macro states are defined explicitly (0/1); the C side rejects anything else.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    EXP_SIM=${EXP_SIM} EXP_QEMU_UART=${EXP_QEMU_UART})

# EXP_USE_SIM is a *presence* flag: the sources test it with #if defined(...),
# so it must exist only when EXP_SIM is on (EXP_USE_SIM=0 would still read as set).
if(EXP_SIM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE EXP_USE_SIM=1)
endif()
