# SPDX-License-Identifier: MIT
# P-11_pstore_params — 参数持久化（pstore 组件：电机校准/舵机中位写入断电重启读回一致）
# 注意：project() 名禁连字符；目录名保留任务书 P-11 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p11_pstore_params)
