/* SPDX-License-Identifier: MIT */
/**
 * @file    mahony.c
 * @brief   Mahony 互补滤波实现（提炼自旧式实现实机验证代码，float-only）
 *
 * @note    七步：1 归一化加速度 2 估计重力 3 叉积误差 4 误差积分
 *          5 陀螺转弧度+PI 修正 6 一阶积分 7 归一化。
 */

#include <math.h>
#include "att/mahony.h"

void mahony_init(mahony_t *m, float kp, float ki, float dt)
{
    quat_identity(&m->q);
    m->kp = kp;
    m->ki = ki;
    m->integral[0] = m->integral[1] = m->integral[2] = 0.0f;
    m->dt = dt;
}

void mahony_set_dt(mahony_t *m, float dt)
{
    m->dt = dt;
}

void mahony_update(mahony_t *m, const imu_sample_t *s)
{
    const float dt    = m->dt;
    const float twoKp = 2.0f * m->kp;
    const float twoKi = 2.0f * m->ki;

    /* 1. 归一化加速度（无效帧直接丢弃） */
    float ax = s->ax, ay = s->ay, az = s->az;
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return;
    ax /= n; ay /= n; az /= n;

    /* 2. 从四元数估计重力方向（机体坐标） */
    float q0 = m->q.w, q1 = m->q.x, q2 = m->q.y, q3 = m->q.z;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 3. 误差 = 测量重力 × 估计重力（叉积） */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* 4. 误差积分（陀螺零偏补偿通道） */
    m->integral[0] += twoKi * ex * dt;
    m->integral[1] += twoKi * ey * dt;
    m->integral[2] += twoKi * ez * dt;

    /* 5. 陀螺（dps → rad/s）+ PI 修正 */
    const float d2r = ATT_PI_F / 180.0f;
    float gx = s->gx * d2r + twoKp * ex + m->integral[0];
    float gy = s->gy * d2r + twoKp * ey + m->integral[1];
    float gz = s->gz * d2r + twoKp * ez + m->integral[2];

    /* 6. 四元数一阶积分 */
    float hdt = 0.5f * dt;
    m->q.w += hdt * (-q1 * gx - q2 * gy - q3 * gz);
    m->q.x += hdt * ( q0 * gx + q2 * gz - q3 * gy);
    m->q.y += hdt * ( q0 * gy - q1 * gz + q3 * gx);
    m->q.z += hdt * ( q0 * gz + q1 * gy - q2 * gx);

    /* 7. 归一化（防数值漂移，|q|=1 是渲染通道的依赖） */
    quat_normalize(&m->q);
}

void mahony_get_angles(const mahony_t *m, float *roll, float *pitch, float *yaw)
{
    quat_to_euler(&m->q, roll, pitch, yaw);
}

void mahony_get_quat(const mahony_t *m, float *w, float *x, float *y, float *z)
{
    *w = m->q.w; *x = m->q.x; *y = m->q.y; *z = m->q.z;
}
