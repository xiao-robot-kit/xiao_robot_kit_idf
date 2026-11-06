/* SPDX-License-Identifier: MIT */
/**
 * @file    test_attitude.c
 * @brief   姿态库无硬件单测（合成 IMU 采样喂滤波器，纯数学闭环）
 *
 * @note    自研极简断言（不引第三方测试框架），风格对齐 test_imu.c。
 * @note    验证范围：欧拉↔四元数往返 / 单位态性质 / 乘法合成 /
 *          归一化零向量边界 / Mahony 静止收敛与无效帧 / |q| 全程有界
 *          （用例 8 由 feed_frames 每 100 帧核查）。
 *          不覆盖：真实传感器噪声/温漂/陀螺零偏（需实机验证）。
 * @note    轴向约定（与 quat.h 一致，ZYX 内旋）：roll 绕 X、pitch 绕 Y、
 *          yaw 绕 Z。静止加速度读数 = 世界 +Z 轴在机体系的投影，
 *          绕 X 倾角 θ 的读数为 (0, sinθ, cosθ)，绕 Y 倾角 θ 为 (-sinθ, 0, cosθ)。
 *          故用例 6a 的 (0, 0.5, 0.866) 是绕 X 倾斜 30°（倾角在 roll），
 *          6b 的 (-0.5, 0, 0.866) 是绕 Y 倾斜 30°（倾角在 pitch）。
 */

#include <stdio.h>
#include <math.h>
#include "quat.h"
#include "mahony.h"

/* ---------- 极简断言（对齐 test_imu.c） ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)

/* ---------- 测试参数表（可调参数集中注册，代码内零写死） ---------- */
typedef struct {
    float kp;             /**< Mahony 比例增益 */
    float ki;             /**< Mahony 积分增益 */
    float dt;             /**< 采样周期，秒 */
    int   frames;         /**< 收敛喂帧数 */
    int   norm_every;     /**< |q| 核查间隔，帧（用例 8） */
    float roundtrip_tol;  /**< 欧拉往返容差，rad（1e-4） */
    float identity_tol;   /**< 单位态分量容差 */
    float quat_norm_tol;  /**< |q|-1 容差（用例 8：1e-4） */
    float level_deg;      /**< 水平收敛容差，°（1°） */
    float tilt_lo_deg;    /**< 倾斜收敛下限，°（28°） */
    float tilt_hi_deg;    /**< 倾斜收敛上限，°（32°） */
    float flat_deg;       /**< 倾斜姿态中非倾斜轴的残余容差，° */
    float perturb_deg[3]; /**< 用例 5b 受扰初值 (roll, pitch, yaw)，° */
} att_test_cfg_t;

static const att_test_cfg_t T = {
    .kp            = 2.0f,
    .ki            = 0.0f,
    .dt            = 0.01f,
    .frames        = 2000,
    .norm_every    = 100,
    .roundtrip_tol = 1e-4f,
    .identity_tol  = 1e-6f,
    .quat_norm_tol = 1e-4f,
    .level_deg     = 1.0f,
    .tilt_lo_deg   = 28.0f,
    .tilt_hi_deg   = 32.0f,
    .flat_deg      = 2.0f,
    .perturb_deg   = { 10.0f, -8.0f, 5.0f },
};

/* ---------- 合成数据与公用工具 ---------- */
#define DEG2RAD(d) ((d) * ATT_PI_F / 180.0f)

/** 合成一次静止采样：加速度 g（模长由调用方保证）、角速度 0 dps */
static imu_sample_t still_sample(float ax, float ay, float az)
{
    imu_sample_t s = { ax, ay, az, 0.0f, 0.0f, 0.0f };
    return s;
}

/** 四元数模长 |q| */
static float quat_mag(const quat_t *q)
{
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

/**
 * @brief   喂 n 帧恒定采样（静止场景）
 * @note    用例 8 的落点：每 norm_every 帧核查 |q|-1 < quat_norm_tol，
 *          覆盖全部 Mahony 用例的喂帧全程。
 */
static void feed_frames(mahony_t *m, const imu_sample_t *s, int n)
{
    for (int i = 0; i < n; i++) {
        mahony_update(m, s);
        if ((i + 1) % T.norm_every == 0)
            CHECK(fabsf(quat_mag(&m->q) - 1.0f) < T.quat_norm_tol,
                  "case8 |q|-1 < 1e-4 every 100 frames");
    }
}

int main(void)
{
    /* ---- 1. 欧拉(0.3,-0.5,1.2) → 四元数 → 欧拉 往返误差 < 1e-4 ---- */
    {
        quat_t q;
        quat_from_euler(&q, 0.3f, -0.5f, 1.2f);
        CHECK(fabsf(quat_mag(&q) - 1.0f) < T.roundtrip_tol, "case1 from_euler |q|~1");
        float r, p, y;
        quat_to_euler(&q, &r, &p, &y);
        CHECK(fabsf(r - 0.3f) < T.roundtrip_tol, "case1 roll roundtrip <1e-4");
        CHECK(fabsf(p + 0.5f) < T.roundtrip_tol, "case1 pitch roundtrip <1e-4");
        CHECK(fabsf(y - 1.2f) < T.roundtrip_tol, "case1 yaw roundtrip <1e-4");
    }

    /* ---- 2. identity：单位性、|q|≈1、欧拉角≈0 ---- */
    {
        quat_t q;
        quat_identity(&q);
        CHECK(fabsf(q.w - 1.0f) < T.identity_tol, "case2 identity w=1");
        CHECK(fabsf(q.x) < T.identity_tol && fabsf(q.y) < T.identity_tol
              && fabsf(q.z) < T.identity_tol, "case2 identity x=y=z=0");
        CHECK(fabsf(quat_mag(&q) - 1.0f) < T.identity_tol, "case2 |q|~1");
        float r, p, y;
        quat_to_euler(&q, &r, &p, &y);
        CHECK(fabsf(r) < T.identity_tol && fabsf(p) < T.identity_tol
              && fabsf(y) < T.identity_tol, "case2 euler angles ~0");
    }

    /* ---- 3. 绕 Z 90°×90°=180°（from_euler 构造再取角） ---- */
    {
        quat_t qa, qb, qr;
        quat_from_euler(&qa, 0.0f, 0.0f, DEG2RAD(90.0f));
        quat_from_euler(&qb, 0.0f, 0.0f, DEG2RAD(90.0f));
        quat_multiply(&qr, &qa, &qb);
        CHECK(fabsf(quat_mag(&qr) - 1.0f) < T.roundtrip_tol, "case3 product |q|~1");
        float r, p, y;
        quat_to_euler(&qr, &r, &p, &y);
        CHECK(fabsf(fabsf(y) - ATT_PI_F) < T.roundtrip_tol, "case3 yaw = 180deg");
        CHECK(fabsf(r) < T.roundtrip_tol && fabsf(p) < T.roundtrip_tol,
              "case3 roll/pitch ~0");
    }

    /* ---- 4. vec3_normalize 零向量：不修改、不产 NaN ---- */
    {
        vec3_t v = { 0.0f, 0.0f, 0.0f };
        vec3_normalize(&v);
        CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f, "case4 zero vector untouched");
        CHECK(!isnan(v.x) && !isnan(v.y) && !isnan(v.z), "case4 no NaN on zero vector");
        /* 非零向量走正路径：归一化到单位长度 */
        vec3_t u = { 3.0f, 4.0f, 0.0f };
        vec3_normalize(&u);
        CHECK(fabsf(u.x - 0.6f) < T.identity_tol && fabsf(u.y - 0.8f) < T.identity_tol,
              "case4 (3,4,0)->(0.6,0.8)");
        CHECK(fabsf(vec3_norm(&u) - 1.0f) < T.identity_tol, "case4 unit norm after");
    }

    /* ---- 5. Mahony 静止水平：acc=(0,0,1) gyro=0 喂 2000 帧 → |roll|,|pitch|<1° ---- */
    {
        mahony_t m;
        float r, p, y;
        imu_sample_t s = still_sample(0.0f, 0.0f, 1.0f);

        mahony_init(&m, T.kp, T.ki, T.dt);
        feed_frames(&m, &s, T.frames);
        mahony_get_angles(&m, &r, &p, &y);
        CHECK(fabsf(r) < DEG2RAD(T.level_deg), "case5 level roll <1deg");
        CHECK(fabsf(p) < DEG2RAD(T.level_deg), "case5 level pitch <1deg");

        /* 5b：受扰初值（白盒置 q）→ 同样收敛回水平，验证真收敛而非原地不动 */
        mahony_init(&m, T.kp, T.ki, T.dt);
        quat_from_euler(&m.q, DEG2RAD(T.perturb_deg[0]),
                        DEG2RAD(T.perturb_deg[1]), DEG2RAD(T.perturb_deg[2]));
        feed_frames(&m, &s, T.frames);
        mahony_get_angles(&m, &r, &p, &y);
        CHECK(fabsf(r) < DEG2RAD(T.level_deg), "case5b perturbed roll converges <1deg");
        CHECK(fabsf(p) < DEG2RAD(T.level_deg), "case5b perturbed pitch converges <1deg");
    }

    /* ---- 6. Mahony 静止倾斜 30°（两个正交子用例） ---- */
    {
        mahony_t m;
        float r, p, y;

        /* 6a：acc=(0, sin30°, cos30°) = (0, 0.5, 0.866)，模=1。
         * ZYX 约定下这是「绕 X 倾斜 30°」的重力投影 → 倾角收敛于 roll；
         * 实测该读数不会体现到 pitch（推导见文件头注）。 */
        mahony_init(&m, T.kp, T.ki, T.dt);
        imu_sample_t s = still_sample(0.0f, 0.5f, 0.866f);
        feed_frames(&m, &s, T.frames);
        mahony_get_angles(&m, &r, &p, &y);
        CHECK(fabsf(r) > DEG2RAD(T.tilt_lo_deg) && fabsf(r) < DEG2RAD(T.tilt_hi_deg),
              "case6a roll converges 28~32 deg (tilt about X)");
        CHECK(fabsf(p) < DEG2RAD(T.flat_deg), "case6a pitch stays flat");

        /* 6b：acc=(-sin30°, 0, cos30°) = (-0.5, 0, 0.866)，模=1——
         * 「绕 Y 倾斜 30°」→ 倾角收敛于 pitch */
        mahony_init(&m, T.kp, T.ki, T.dt);
        s = still_sample(-0.5f, 0.0f, 0.866f);
        feed_frames(&m, &s, T.frames);
        mahony_get_angles(&m, &r, &p, &y);
        CHECK(fabsf(p) > DEG2RAD(T.tilt_lo_deg) && fabsf(p) < DEG2RAD(T.tilt_hi_deg),
              "case6b pitch converges 28~32 deg (tilt about Y)");
        CHECK(fabsf(r) < DEG2RAD(T.flat_deg), "case6b roll stays flat");
    }

    /* ---- 7. 无效帧：acc=(0,0,0) 不更新姿态（保持 identity） ---- */
    {
        mahony_t m;
        mahony_init(&m, T.kp, T.ki, T.dt);
        imu_sample_t s = still_sample(0.0f, 0.0f, 0.0f);
        feed_frames(&m, &s, T.norm_every);   /* 100 帧全无效，足以证明逐帧跳过 */
        CHECK(m.q.w == 1.0f, "case7 w stays exactly 1");
        CHECK(m.q.x == 0.0f && m.q.y == 0.0f && m.q.z == 0.0f,
              "case7 x/y/z stay exactly 0");
        CHECK(fabsf(quat_mag(&m.q) - 1.0f) < T.quat_norm_tol, "case7 |q|~1");
    }

    printf("== test_attitude: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
