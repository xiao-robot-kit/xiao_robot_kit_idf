/* SPDX-License-Identifier: MIT */
/**
 * @file    att_cfg_presets.c
 * @brief   Mahony 参数预设注册表（追加预设只改本文件，不动算法）
 *
 * @note    命名空间：att.<实验>.<场景>。dt 必须与所配 imu 预设的
 *          sample_period_ms/1000 相等（exp3 配 imu.attitude.208 = 5ms）。
 */

#include <string.h>
#include "att_cfg.h"

const att_cfg_preset_t att_cfg_presets[] = {
    /* 实验三：教学档——与 att.exp4.mahony 同值（两实验唯一变量 = 表示法）：
     * kp 中等收敛、ki 小量补陀螺零偏 */
    { "att.exp3.mahony", {
        .kp = 2.0f,
        .ki = 0.2f,
        .dt = 0.005f,
    }},
};

const int att_cfg_preset_count =
    sizeof(att_cfg_presets) / sizeof(att_cfg_presets[0]);

const att_cfg_t *att_cfg_find(const char *name)
{
    for (int i = 0; i < att_cfg_preset_count; i++)
        if (name && strcmp(att_cfg_presets[i].name, name) == 0)
            return &att_cfg_presets[i].cfg;
    return 0;
}
