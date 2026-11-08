# SPDX-License-Identifier: MIT
# Top-level CMake build for the XIAO Robot Kit C core libraries.
#
# Independent, platform-free C libraries (headers and sources are laid out flat
# inside each library directory, so a consumer only needs that one directory on
# its include path):
#
#   xiao_imu       - LSM6DS3TR-C 6-axis IMU driver (I2C bus table + preset registry)
#   xiao_attitude  - quaternion math + Mahony complementary filter (uses xiao_imu types)
#   xiao_motor     - PWM timer / PID / S-curve profile / motor instances (HAL tables)
#   xiao_pstore    - parameter persistence (item registry + framed serialization + CRC32)
#   xiao_battery   - battery voltage divider -> cell voltage / SoC percent
#   xiao_calibration - IMU bias calibration window + temperature drift compensation
#   xiao_imu_heater - on-die heater duty state machine (thermal window control)
#   xiao_proto     - serial frame codec shared by firmware examples and host app
#                    (SOF/LEN/TYPE/SEQ/PAYLOAD/CRC8; frame type + CMD/ACK table)
#
# Standalone build (library mode + optional tests):
#   cmake -S . -B build
#   cmake --build build
#   ctest --test-dir build            (requires -DXIAO_LIBS_BUILD_TESTS=ON, default ON)
#
# Tests live in tests/<component>/ (outside src/ on purpose: the Arduino
# library build compiles everything under src/ recursively, and the test
# executables each define main()).
#
# Consumed from a parent build (e.g. ESP-IDF project via add_subdirectory or
# extra component): link the four targets below; include paths are PUBLIC,
# e.g.  target_link_libraries(app PRIVATE xiao_motor)  then #include "motor.h".

cmake_minimum_required(VERSION 3.13)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(XIAO_LIBS_STANDALONE ON)
    project(xiao_robot_kit_libs VERSION 1.0.0 LANGUAGES C)
else()
    set(XIAO_LIBS_STANDALONE OFF)
endif()

option(XIAO_LIBS_BUILD_TESTS "Build the per-library unit tests" ON)

# ---------------------------------------------------------------- xiao_imu
add_library(xiao_imu STATIC
    src/imu/lsm6ds3.c
    src/imu/imu_cfg_presets.c)
target_include_directories(xiao_imu PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/imu>
    $<INSTALL_INTERFACE:include/imu>)
target_compile_features(xiao_imu PUBLIC c_std_11)

# ----------------------------------------------------------- xiao_attitude
add_library(xiao_attitude STATIC
    src/attitude/quat.c
    src/attitude/mahony.c)
target_include_directories(xiao_attitude PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/attitude>
    $<INSTALL_INTERFACE:include/attitude>)
target_compile_features(xiao_attitude PUBLIC c_std_11)
# mahony.h includes imu_types.h from xiao_imu (sample type contract)
target_link_libraries(xiao_attitude PUBLIC xiao_imu)

# -------------------------------------------------------------- xiao_motor
add_library(xiao_motor STATIC
    src/motor/pwm_timer.c
    src/motor/pid.c
    src/motor/scurve.c
    src/motor/motor.c
    src/motor/motor_cfg_presets.c)
target_include_directories(xiao_motor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/motor>
    $<INSTALL_INTERFACE:include/motor>)
target_compile_features(xiao_motor PUBLIC c_std_11)

# ------------------------------------------------------------- xiao_pstore
add_library(xiao_pstore STATIC
    src/pstore/pstore.c)
target_include_directories(xiao_pstore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/pstore>
    $<INSTALL_INTERFACE:include/pstore>)
target_compile_features(xiao_pstore PUBLIC c_std_11)

# ------------------------------------------------------------ xiao_battery
add_library(xiao_battery STATIC
    src/battery/battery.c)
target_include_directories(xiao_battery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/battery>
    $<INSTALL_INTERFACE:include/battery>)
target_compile_features(xiao_battery PUBLIC c_std_11)

# -------------------------------------------------------- xiao_calibration
add_library(xiao_calibration STATIC
    src/calibration/imu_calib.c
    src/calibration/temp_comp.c)
target_include_directories(xiao_calibration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/calibration>
    $<INSTALL_INTERFACE:include/calibration>)
target_compile_features(xiao_calibration PUBLIC c_std_11)
# imu_calib.h includes imu_types.h from xiao_imu (sample type contract)
target_link_libraries(xiao_calibration PUBLIC xiao_imu)

# ---------------------------------------------------------- xiao_imu_heater
add_library(xiao_imu_heater STATIC
    src/imu_heater/imu_heater.c)
target_include_directories(xiao_imu_heater PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/imu_heater>
    $<INSTALL_INTERFACE:include/imu_heater>)
target_compile_features(xiao_imu_heater PUBLIC c_std_11)

# ---------------------------------------------------------------- xiao_proto
# Serial frame contract: firmware examples and the host app must agree byte for
# byte, so this codec is the single source of the wire format (SOF 0xA5,
# CRC8 poly 0x07 init 0x00 over LEN..PAYLOAD). payload <= PROTO_PAYLOAD_MAX (96).
add_library(xiao_proto STATIC
    src/proto/frame.c)
target_include_directories(xiao_proto PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/proto>
    $<INSTALL_INTERFACE:include/proto>)
target_compile_features(xiao_proto PUBLIC c_std_11)

set(XIAO_LIBS_TARGETS xiao_imu xiao_attitude xiao_motor xiao_pstore xiao_battery xiao_calibration xiao_imu_heater xiao_proto)

# ---------------------------------------------------------------- install +
# target export (only meaningful for a standalone build; when this tree is
# pulled in by a parent build - e.g. ESP-IDF - consumers use the targets
# directly and the install/export rules below are skipped).
if(XIAO_LIBS_STANDALONE)

    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(TARGETS ${XIAO_LIBS_TARGETS}
        EXPORT xiao-libs-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    install(FILES
        src/imu/imu_types.h
        src/imu/imu_cfg.h
        src/imu/imu_hal_i2c.h
        src/imu/lsm6ds3.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/imu)
    install(FILES
        src/attitude/quat.h
        src/attitude/mahony.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/attitude)
    install(FILES
        src/motor/motor_types.h
        src/motor/motor.h
        src/motor/motor_hal.h
        src/motor/pwm_timer.h
        src/motor/pwm_hal.h
        src/motor/pid.h
        src/motor/scurve.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/motor)
    install(FILES
        src/pstore/pstore.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/pstore)
    install(FILES
        src/battery/battery.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/battery)
    install(FILES
        src/calibration/imu_calib.h
        src/calibration/temp_comp.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/calibration)
    install(FILES
        src/imu_heater/imu_heater.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/imu_heater)
    install(FILES
        src/proto/frame.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/proto)

    set(XIAO_LIBS_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/xiao-libs)
    install(EXPORT xiao-libs-targets
        FILE xiao-libs-targets.cmake
        NAMESPACE xiao::
        DESTINATION ${XIAO_LIBS_INSTALL_CMAKEDIR})
    # Build-tree export for sibling projects via add_subdirectory + find_package
    export(EXPORT xiao-libs-targets
        FILE ${CMAKE_CURRENT_BINARY_DIR}/xiao-libs-targets.cmake
        NAMESPACE xiao::)

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/xiao-libs-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/xiao-libs-config-version.cmake
        DESTINATION ${XIAO_LIBS_INSTALL_CMAKEDIR})

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/xiao-libs-config.cmake
        "# SPDX-License-Identifier: MIT\n"
        "include(\"\${CMAKE_CURRENT_LIST_DIR}/xiao-libs-targets.cmake\")\n")
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/xiao-libs-config.cmake
        DESTINATION ${XIAO_LIBS_INSTALL_CMAKEDIR})

    # ---------------------------------------------------------------- tests
    if(XIAO_LIBS_BUILD_TESTS)
        enable_testing()
        foreach(t test_imu)
            add_executable(${t} tests/imu/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_imu)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_attitude)
            add_executable(${t} tests/attitude/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_attitude)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_pid test_pwm_timer test_scurve test_motor)
            add_executable(${t} tests/motor/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_motor)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_pstore)
            add_executable(${t} tests/pstore/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_pstore)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_battery)
            add_executable(${t} tests/battery/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_battery)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_calibration)
            add_executable(${t} tests/calibration/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_calibration)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_imu_heater)
            add_executable(${t} tests/imu_heater/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_imu_heater)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
        foreach(t test_proto)
            add_executable(${t} tests/proto/${t}.c)
            target_link_libraries(${t} PRIVATE xiao_proto)
            add_test(NAME ${t} COMMAND ${t})
        endforeach()
    endif()

endif()
