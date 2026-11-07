# SPDX-License-Identifier: MIT
# P-12_spare_gpio_tpxx — 备用 GPIO（TP20/TP21 飞线验证 MTDI/MTDO 双向 IO）
# 任务书 P-12：TP20=MTDI(GPIO41)、TP21=MTDO(GPIO42)（U3 pad24/25 = 唯一备用 GPIO）
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-12 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p12_spare_gpio_tpxx)