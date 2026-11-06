# SPDX-License-Identifier: MIT
# imu_quickstart — minimal ESP-IDF project: stream six-axis samples over the
# console UART. Uses the library directly from this repository (no component
# manager involved), see main/CMakeLists.txt.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(imu_quickstart)
