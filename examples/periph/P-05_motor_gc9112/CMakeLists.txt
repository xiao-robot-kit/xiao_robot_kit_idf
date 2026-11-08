# SPDX-License-Identifier: MIT
# P-05_motor_gc9112 — 双路 GC9112 电机驱动例程（正转/反转/停/调速 + 占空比-转速曲线）
# 链路：xiao_motor 组件（预设 motor.dc.generic @ pwm.motor.20k）
#      + 合成 PWM/方向桥 HAL（port/esp_idf/motor_hal_sim.c，QEMU 无 LEDC 模型）
#      → 500Hz 控制环 → 文本遥测 → UART0(QEMU 档) / USB-Serial-JTAG(真板档)
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 exp_motor_qemu/其余 periph 例程）：xtensa-gcc 15.2 对 esp_lcd ICE
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p05_motor_gc9112)