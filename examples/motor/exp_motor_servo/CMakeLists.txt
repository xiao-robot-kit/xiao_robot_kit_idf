# SPDX-License-Identifier: MIT
# exp_motor_servo — four servos: angle sweep on the PWM timer
cmake_minimum_required(VERSION 3.16)
# xtensa-gcc 15.2 ICEs while compiling esp_lcd; these projects do not use it.
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(exp_motor_servo)
