# SPDX-License-Identifier: MIT
# exp5_trajectory main component (public-repo library layout).
#
# Library code lives in the repository root CMakeLists (the xiao_* targets);
# only this project's own sources and the shared port layer / synthetic data
# sources from examples/_shared are compiled in here.
set(LIBS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)   # repository root
set(SHARED_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../_shared)
set(IMU_COMMON ${CMAKE_CURRENT_LIST_DIR}/../../common)   # shared by the IMU experiments

# EXP_SIM (1 = synthetic IMU source, no sensor needed; 0 = real LSM6DS3 over I2C)
set(EXP_SIM 1 CACHE STRING "1=synthetic IMU source (default); 0=real LSM6DS3")
# EXP_QEMU_UART (1 = frames on UART0; 0 = USB-Serial-JTAG)
set(EXP_QEMU_UART 1 CACHE STRING "1=frames on UART0 (QEMU, console); 0=USB-Serial-JTAG")

set(EXP5_SRCS
        "exp5_main.c"
        "att_cfg_presets.c"
        "zero_cal.c"
        "traj_scen.c"
        "${IMU_COMMON}/cal_cmd.c"
        "${SHARED_DIR}/pstore_nvs.c"
)
set(EXP5_INCS
        "."
        "${IMU_COMMON}"
        "${SHARED_DIR}"
)

if(EXP_SIM)
    list(APPEND EXP5_SRCS "${SHARED_DIR}/imu_sim.c")
else()
    list(APPEND EXP5_SRCS "${SHARED_DIR}/imu_hal_i2c.c")
endif()

# REQUIRES is unconditionally complete: IDF discovers components before
# cmake -D cache evaluation, so a conditional REQUIRES loses the include path.
idf_component_register(
    SRCS ${EXP5_SRCS}
    INCLUDE_DIRS ${EXP5_INCS}
    REQUIRES esp_driver_uart esp_driver_i2c esp_driver_usb_serial_jtag nvs_flash
)

add_subdirectory(${LIBS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/xiao_libs)
target_link_libraries(${COMPONENT_LIB} PUBLIC xiao_attitude xiao_imu xiao_calibration xiao_pstore xiao_proto)

# Both macro states are defined explicitly (0/1); the C side rejects anything else.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    EXP_SIM=${EXP_SIM} EXP_QEMU_UART=${EXP_QEMU_UART})

# EXP_USE_SIM is a *presence* flag: the sources test it with #if defined(...),
# so it must exist only when EXP_SIM is on (EXP_USE_SIM=0 would still read as set).
if(EXP_SIM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE EXP_USE_SIM=1)
endif()
