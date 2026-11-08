/* SPDX-License-Identifier: MIT */
#ifndef IMU_TYPES_H
#define IMU_TYPES_H

/**
 * @file    imu_types.h
 * @brief   IMU 跨层公共数据契约（驱动输出、算法消费，两侧只依赖本文件）
 *
 * @note    单位约定（全库统一，换算只发生在驱动边界）：
 *          - 加速度：g（重力加速度倍数，1g = 9.80665 m/s^2）
 *          - 角速度：dps（度/秒）
 *          - 温度：  °C
 *          零偏与采样同单位，去零偏可直接逐分量相减。
 */

/** @brief 一次六轴采样（物理量，已过灵敏度换算） */
typedef struct {
    float ax, ay, az;   /**< 加速度，单位 g */
    float gx, gy, gz;   /**< 角速度，单位 dps */
} imu_sample_t;

/** @brief IMU 零偏（标定结果；与 imu_sample_t 同单位） */
typedef struct {
    float gx, gy, gz;   /**< 陀螺仪零偏，dps */
    float ax, ay, az;   /**< 加速度计零偏，g */
} imu_bias_t;

/** @brief 一次带温度的采样（温补实验数据通道） */
typedef struct {
    imu_sample_t six;   /**< 六轴物理量 */
    float        temp;  /**< 芯片温度，°C */
} imu_sample_temp_t;

#endif /* IMU_TYPES_H */
