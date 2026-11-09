# SPDX-License-Identifier: MIT
# P-00_hello_bringup — 工程脚手架 + USB 串口日志（XIAO Robot Board V1.0）
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-00 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p00_hello_bringup)