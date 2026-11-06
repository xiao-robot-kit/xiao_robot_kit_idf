/* SPDX-License-Identifier: MIT */
#ifndef ATT_QUAT_H
#define ATT_QUAT_H

#include <math.h>

/**
 * @file    quat.h
 * @brief   三维向量 + 四元数数学（float 版，纯标准 C，零平台依赖）
 *
 * @note    所有角度均为弧度。四元数 w + xi + yj + zk 表示旋转，
 *          避免欧拉角万向锁；欧拉换算为 ZYX 内旋约定，互为逆。
 * @note    零输入策略（有意不同）：vec3_normalize 对零向量不修改；
 *          quat_normalize 对零模长重置为单位四元数。
 */

/** @brief 圆周率（float 精度） */
#define ATT_PI_F  3.14159265358979f

/** @brief 三维向量 */
typedef struct { float x, y, z; } vec3_t;

/** @brief 四元数，分量顺序 w + xi + yj + zk */
typedef struct { float w, x, y, z; } quat_t;

/** @brief 求向量模长（零向量返回 0） */
float vec3_norm(const vec3_t *v);

/** @brief 原地归一化（模长 < 1e-6f 直接返回不修改，不产出 NaN） */
void  vec3_normalize(vec3_t *v);

/** @brief 置单位四元数（w=1，无旋转） */
void  quat_identity(quat_t *q);

/** @brief 原地归一化（模长 < 1e-6f 重置为单位四元数） */
void  quat_normalize(quat_t *q);

/** @brief 四元数乘法 out = a * b（out 与 a/b 可同为一路） */
void  quat_multiply(quat_t *out, const quat_t *a, const quat_t *b);

/** @brief 四元数 → 欧拉角（弧度；pitch 到 ±90° 为万向锁退化区） */
void  quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw);

/** @brief 欧拉角（弧度，ZYX 内旋）→ 四元数（与 quat_to_euler 互为逆） */
void  quat_from_euler(quat_t *q, float roll, float pitch, float yaw);

#endif /* ATT_QUAT_H */
