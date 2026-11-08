# SPDX-License-Identifier: MIT
# P-10_imu_heater — IMU 恒温加热闭环（D12 PWM → 目标 ±2°C 恒温 + 超温保护 + 温补对接）
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-10 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p10_imu_heater)
