# SPDX-License-Identifier: MIT
# P-01_imu_lsm6ds3 — I2C 轮询 LSM6DS3TR-C(0x6A)：WHO_AM_I + 100Hz 连读 + 总线扫描
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-01 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p01_imu_lsm6ds3)