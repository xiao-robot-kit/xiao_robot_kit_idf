# SPDX-License-Identifier: MIT
# P-07_servo_pwm — 4 路舵机 PWM 例程（0°/90°/180° 定位 + 电压联动告警）
# 链路：xiao_motor 组件（预设 motor.servo.sg90 @ pwm.servo.50）
#      + 合成 PWM HAL（port/esp_idf/motor_hal_sim.c，QEMU 无 LEDC 模型）
#      → 四槽错相扫角表 → 文本遥测 → UART0(QEMU 档) / USB-Serial-JTAG(真板档)
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-05/P-06/其余 periph 例程）：xtensa-gcc 15.2 对 esp_lcd ICE
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p07_servo_pwm)