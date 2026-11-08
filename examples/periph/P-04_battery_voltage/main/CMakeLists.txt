# SPDX-License-Identifier: MIT
# P-04_battery_voltage main component: compute battery voltage / SOC curve.
#
# Component wiring (public-repo layout):
#   - xiao_battery library target from repository root (add_subdirectory below)
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root

# P_ADC_RAW: 1 = synthetic ADC raw value (exercise conversion chain without a
#            battery, QEMU/SIM); 0 = real ADC1_CH5 GPIO16 sample (real board,
#            100K:30K divider)
if(NOT DEFINED P_ADC_RAW)
    set(P_ADC_RAW 1)
endif()

# Frame output on UART0 (QEMU) vs default USB-Serial-JTAG (real board)
if(NOT DEFINED P_QEMU_UART)
    set(P_QEMU_UART 0)
endif()

set(P04_SRCS "p04_battery_main.c")
set(P04_INCS ".")

# REQUIRES listed unconditionally (component discovery runs before cmake cache
# eval; ADC uses esp_adc oneshot API, console for printf/console)
idf_component_register(
    SRCS ${P04_SRCS}
    INCLUDE_DIRS ${P04_INCS}
    REQUIRES esp_adc esp_driver_gpio esp_driver_uart console
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_battery)

if(P_ADC_RAW)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC P_ADC_RAW=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC P_ADC_RAW=0)
endif()
if(P_QEMU_UART)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC P_QEMU_UART=1)
endif()