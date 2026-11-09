# SPDX-License-Identifier: MIT
# P-05_motor_gc9112 — 双路 GC9112 电机驱动例程（正转/反转/停/调速 + 占空比-转速曲线）
# 链路：xiao_motor 组件（预设 motor.dc.generic @ pwm.motor.20k）
cmake_minimum_required(VERSION 3.16)
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p05_motor_gc9112)