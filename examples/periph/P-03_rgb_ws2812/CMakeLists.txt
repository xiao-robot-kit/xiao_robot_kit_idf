# SPDX-License-Identifier: MIT
# P-03_rgb_ws2812 — D13→J7 外接 WS2812 灯带：RMT 驱动 + 自研色轮（XIAO Robot Board V1.0）
# 任务书 P-03：全彩渐变 + 单灯定点；GPIO13 → R79 22R → U4 缓冲 → R52 22R → J7
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-03 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p03_rgb_ws2812)