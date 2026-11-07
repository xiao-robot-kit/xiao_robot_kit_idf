# SPDX-License-Identifier: MIT
# P-02_button_k7 — 按键消抖与短按/长按事件例程（XIAO Robot Board V1.0）
# 任务书 P-02：D11 按键（GPIO11，内部上拉，按下=低）→ 软件消抖 → 短按/长按事件
# 注意：project() 名禁含连字符（IDF 内部以其做变量名拼接），目录名保留任务书 P-02 前缀。
cmake_minimum_required(VERSION 3.16)
# 工具链规避（同 P-00 / exp 系列）：xtensa-gcc 15.2 对 esp_lcd/rgb ICE，整件排除
set(EXCLUDE_COMPONENTS esp_lcd)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p02_button_k7)