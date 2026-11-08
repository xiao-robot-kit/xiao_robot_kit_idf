# SPDX-License-Identifier: MIT
# P-04_battery_voltage — 电池电压 / 剩余电量例程（XIAO Robot Board V1.0）
# 任务书 P-04：电池 ADC → 电压换算 → SOC% → 剩余 mAh，低电告警。
# D16 电池采样（板上 100K:30K 分压）→ ESP32-S3 ADC2_CH5 (GPIO16)。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 exp2/store_demo）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# project() 名禁含连字符（IDF 内部以其做变量名拼接）；目录名保留任务书 P-04 前缀
project(p04_battery_voltage)