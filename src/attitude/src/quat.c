/* SPDX-License-Identifier: MIT */
/**
 * @file    quat.c
 * @brief   三维向量 + 四元数运算实现（提炼自旧式实现实机验证代码，重组注释）
 */

#include "att/quat.h"

float vec3_norm(const vec3_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void vec3_normalize(vec3_t *v)
{
    float n = vec3_norm(v);
    if (n < 1e-6f) return;                 /* 零向量不修改（策略见头注） */
    v->x /= n; v->y /= n; v->z /= n;
}

void quat_identity(quat_t *q)
{
    q->w = 1.0f; q->x = q->y = q->z = 0.0f;
}

void quat_normalize(quat_t *q)
{
    float n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n < 1e-6f) { quat_identity(q); return; }
    q->w /= n; q->x /= n; q->y /= n; q->z /= n;
}

void quat_multiply(quat_t *out, const quat_t *a, const quat_t *b)
{
    quat_t r;
    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    *out = r;                               /* 拷贝落地，支持 out 与输入同址 */
}

void quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw)
{
    /* roll（绕 X） */
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch（绕 Y）：|sinp|>=1 为万向锁区，钉到 ±90° */
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(ATT_PI_F / 2.0f, sinp);
    else
        *pitch = asinf(sinp);

    /* yaw（绕 Z） */
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void quat_from_euler(quat_t *q, float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}
