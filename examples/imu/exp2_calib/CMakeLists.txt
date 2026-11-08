# SPDX-License-Identifier: MIT
# exp2_calib — IMU bias calibration + temperature compensation, persisted across reboots
cmake_minimum_required(VERSION 3.16)
# xtensa-gcc 15.2 ICEs while compiling esp_lcd; these projects do not use it.
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(exp2_calib)
