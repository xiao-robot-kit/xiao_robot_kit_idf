# SPDX-License-Identifier: MIT
# P-06_encoder_read — N20 编码器计数例程（手转模拟 + 电转闭环）
# 链路：xiao_motor 组件（预设 motor.enc.n20 @ pwm.motor.20k）
#      + 合成编码器 HAL/plant（port/esp_idf/motor_hal_sim.c，QEMU 无 PCNT/编码器模型）
#      → 文本遥测 → UART0(QEMU 档) / USB-Serial-JTAG(真板档)
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-05/其余 periph 例程）：xtensa-gcc 15.2 对 esp_lcd ICE
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p06_encoder_read)