/* SPDX-License-Identifier: MIT */
#ifndef ATT_MAHONY_H
#define ATT_MAHONY_H

#include "att/quat.h"
#include "imu/imu_types.h"

/**
 * @file    mahony.h
 * @brief   Mahony 互补滤波姿态解算（float-only）
 *
 * @note    算法：陀螺仪积分保高频动态，加速度计观测重力方向、
 *          叉积误差经 PI 修正陀螺（补偿漂移）。
 * @note    加速度模长 < 1e-6f（自由落体/无效）时该帧不更新。
 * @note    消费 imu_sample_t（g / dps），输出四元数（无死锁、可插值）
 *          或欧拉角（教学对照，exp3 专用）。
 */

/** @brief Mahony 滤波器状态 */
typedef struct {
    quat_t q;               /**< 姿态四元数 */
    float  integral[3];     /**< 误差积分项（x/y/z，补偿陀螺零偏） */
    float  kp, ki;          /**< 比例/积分增益 */
    float  dt;              /**< 采样周期，秒（可运行时更新） */
} mahony_t;

/**
 * @brief   初始化（单位四元数 + 积分清零）
 * @param   kp  比例增益（典型 2.0：越大收敛越快，过大放大加速度噪声）
 * @param   ki  积分增益（典型 0.0~0.5：补偿陀螺零偏漂移）
 * @param   dt  初始采样周期，秒
 */
void mahony_init(mahony_t *m, float kp, float ki, float dt);

/** @brief 更新采样周期（秒）——主循环按实测间隔调用 */
void mahony_set_dt(mahony_t *m, float dt);

/** @brief 用一次采样更新姿态（g / dps 进；加速度无效帧跳过） */
void mahony_update(mahony_t *m, const imu_sample_t *s);

/** @brief 取欧拉角（弧度，ZYX 约定；3D 应用请用 mahony_get_quat） */
void mahony_get_angles(const mahony_t *m, float *roll, float *pitch, float *yaw);

/** @brief 取四元数（w,x,y,z）——渲染主通道（|q|=1 保持） */
void mahony_get_quat(const mahony_t *m, float *w, float *x, float *y, float *z);

#endif /* ATT_MAHONY_H */
