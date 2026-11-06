/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_cfg_presets.c
 * @brief   IMU 配置预设注册表（追加预设只改本文件，不动驱动）
 *
 * @note    命名空间：imu.<场景>.<参数>。档位值必须在驱动查表范围内，
 *          表外值会在 lsm6ds3_configure 返回 -2（写不进硬件）。
 */

#include <string.h>
#include "imu_cfg.h"

const imu_cfg_preset_t imu_cfg_presets[] = {
    /* 裸数据直出（104 Hz 常用档，量程温和） */
    { "imu.raw.104", {
        .accel_odr_hz = 104, .accel_fs_g = 2,
        .gyro_odr_hz  = 104, .gyro_fs_dps = 250,
        .i2c_speed_hz = 400000,
        .sample_period_ms = 10,
    }},
    /* 零偏标定（与 raw 同档；零偏统计窗口由上层算法定） */
    { "imu.calib.104", {
        .accel_odr_hz = 104, .accel_fs_g = 2,
        .gyro_odr_hz  = 104, .gyro_fs_dps = 250,
        .i2c_speed_hz = 400000,
        .sample_period_ms = 10,
    }},
    /* 姿态解算：208 Hz 档，留解算余量 */
    { "imu.attitude.208", {
        .accel_odr_hz = 208, .accel_fs_g = 4,
        .gyro_odr_hz  = 208, .gyro_fs_dps = 500,
        .i2c_speed_hz = 400000,
        .sample_period_ms = 5,
    }},
};

const int imu_cfg_preset_count =
    sizeof(imu_cfg_presets) / sizeof(imu_cfg_presets[0]);

const imu_cfg_t *imu_cfg_find(const char *name)
{
    for (int i = 0; i < imu_cfg_preset_count; i++)
        if (name && strcmp(imu_cfg_presets[i].name, name) == 0)
            return &imu_cfg_presets[i].cfg;
    return 0;
}
