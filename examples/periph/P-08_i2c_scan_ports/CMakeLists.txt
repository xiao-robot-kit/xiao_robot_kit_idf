# SPDX-License-Identifier: MIT
# P-08_i2c_scan_ports — I2C 总线扫描 + 地址表管理（J11/J12 Grove I2C ×2 + 板载 IMU 同总线）
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-08 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p08_i2c_scan_ports)
