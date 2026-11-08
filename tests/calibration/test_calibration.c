/* SPDX-License-Identifier: MIT */
/**
 * @file    test_calibration.c
 * @brief   标定组件无硬件单测（合成静止样本 + 已知真值零偏/温漂）
 *
 * @note    自研极简断言（原创律：不引第三方测试框架）。
 * @note    覆盖（对应任务单 a/b/c/d/e）：
 *          a) 合成静止样本 + 已知零偏 → 标定精度（误差 < 0.1）；
 *          b) 同实例连续两轮标定（不同姿态）→ 第二轮与第一轮一致
 *             （旧式实现「start 不清 raw 累加」雷的防回归测试）；
 *          c) 带零偏样本减标定 bias 后残差 < 0.05；
 *          d) 两温度点合成数据 → 温标 → 中间温度补偿误差显著小于不补偿。
 *          e) 残差累加（自动补偿法）：从偏差很大的初值起标，结果是残差而非
 *             绝对零偏；`imu_calib_accumulate` 累加后回真值；连续两轮累加收敛。
 *          附加：NULL 拒绝 / 未就绪拒绝 / 幂等 start / 阶梯保持与跨档 /
 *          disable 回退 / 温补双判 gate / 钳制 / 预设查表 / 无效拟合路径。
 * @note    不覆盖：真实芯片温漂特性与温度传感器噪声（需实机，exp2 板上验证）。
 */

#include <stdio.h>
#include <math.h>
#include "imu/imu_types.h"
#include "cal/imu_calib.h"
#include "cal/temp_comp.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)
#define CHECK_EQ(a, b, msg) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a == _b) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s: %ld != %ld (line %d)\n", msg, _a, _b, __LINE__); } \
} while (0)
#define CHECK_NEAR(a, b, tol, msg) do { \
    double _d = (double)(a) - (double)(b); \
    if (_d < (double)(tol) && _d > -(double)(tol)) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s: %g != %g (line %d)\n", msg, (double)(a), (double)(b), __LINE__); } \
} while (0)
static int feq(float a, float b) { float d = a - b; return d < 0.000001f && d > -0.000001f; }

/* ---------- 合成数据 ---------- */

/** 确定性零均值"噪声"：5 点循环 ±0.004，N 为 5 的倍数时均值恰 0 */
static float noise(int i) { return 0.002f * (float)((i % 5) - 2); }

/* 已知真值零偏（陀螺 dps / 加速度 g）——任务单 a 的"gx 恒 2.5dps 等" */
static const imu_bias_t TRUTH = {
    .gx = 2.5f,  .gy = -1.25f, .gz = 0.75f,
    .ax = 0.06f, .ay = -0.03f, .az = 0.02f,
};

/** 合成一次静止原始样本：z 轴朝上（az 原始 = 1g + 零偏），含确定性噪声 */
static imu_sample_t synth_z_up(int i)
{
    imu_sample_t s;
    s.gx = TRUTH.gx + noise(i);
    s.gy = TRUTH.gy - noise(i);
    s.gz = TRUTH.gz + noise(i);
    s.ax = TRUTH.ax + noise(i);
    s.ay = TRUTH.ay - noise(i);
    s.az = 1.0f + TRUTH.az + noise(i);
    return s;
}

/** 合成一次静止原始样本：y 轴朝上（ay 原始 = 1g + 零偏）——任务单 b 的"不同姿态" */
static imu_sample_t synth_y_up(int i)
{
    imu_sample_t s;
    s.gx = TRUTH.gx + noise(i);
    s.gy = TRUTH.gy - noise(i);
    s.gz = TRUTH.gz + noise(i);
    s.ax = TRUTH.ax + noise(i);
    s.ay = 1.0f + TRUTH.ay - noise(i);
    s.az = TRUTH.az + noise(i);
    return s;
}

int main(void)
{
    imu_calib_t   c;
    imu_bias_t    bias1 = {0}, bias2 = {0}, b = {0};
    imu_sample_t  s;
    float         kg[3], ka[3];   /* 温标拟合结果输出缓冲（非 const，供 result 写入） */

    /* ============ 0. 参数校验与预设查表 ============ */
    const imu_calib_cfg_t *bcfg = imu_calib_cfg_find("cal.exp2.bias");
    CHECK(bcfg != 0, "find preset cal.exp2.bias");
    CHECK_EQ(bcfg->target_n, 500, "preset target_n = 500");
    CHECK_EQ(imu_calib_cfg_preset_count, 1, "imu_calib preset count");
    CHECK(imu_calib_cfg_find("no.such.preset") == 0, "find miss");
    CHECK(imu_calib_cfg_find(NULL) == 0, "find NULL name");

    const temp_comp_cfg_t *dcfg_p = temp_comp_cfg_find("cal.exp2.drift");
    CHECK(dcfg_p != 0, "find preset cal.exp2.drift");
    CHECK(feq(dcfg_p->step_c, 0.5f), "preset step_c = 0.5");
    CHECK_EQ(dcfg_p->calib_target_n, 3000, "preset calib_target_n = 3000");
    CHECK_EQ(temp_comp_cfg_preset_count, 1, "temp_comp preset count");

    CHECK_EQ(imu_calib_init(NULL, bcfg), -1, "init NULL instance rejected");
    CHECK_EQ(imu_calib_init(&c, NULL), -1, "init NULL cfg rejected");
    imu_calib_cfg_t bad_cfg = { 0 };   /* target_n=0 非法 */
    CHECK_EQ(imu_calib_init(&c, &bad_cfg), -1, "init target_n<1 rejected");
    CHECK_EQ(imu_calib_start(NULL), -1, "start NULL rejected");
    CHECK_EQ(imu_calib_ready(NULL), -1, "ready NULL rejected");
    CHECK_EQ(imu_calib_init(&c, bcfg), 0, "init ok");
    CHECK_EQ(imu_calib_ready(&c), 0, "not ready before start");
    CHECK_EQ(imu_calib_result(&c, &bias1), -1, "result rejected before ready");
    s = synth_z_up(0);
    CHECK_EQ(imu_calib_feed(&c, &s, &s), -1, "feed rejected when idle");
    CHECK_EQ(imu_calib_feed(NULL, &s, &s), -1, "feed NULL instance rejected");

    /* ============ a. 静止样本 + 已知零偏 → 标定精度 ============ */
    CHECK_EQ(imu_calib_start(&c), 0, "start round 1");
    CHECK_EQ(c.state, IMU_CALIB_COLLECTING, "state COLLECTING");
    for (int i = 0; i < bcfg->target_n; i++) {
        s = synth_z_up(i);
        CHECK_EQ(imu_calib_feed(&c, &s, NULL), 0, "feed ok (raw=NULL 用 s 判轴)");
        if (i == bcfg->target_n / 2) {
            CHECK_EQ(imu_calib_ready(&c), 0, "not ready mid-collection");
        }
    }
    CHECK_EQ(imu_calib_ready(&c), 1, "ready after N samples");
    CHECK_EQ(c.state, IMU_CALIB_IDLE, "state IDLE after complete");
    CHECK_EQ(c.grav_axis, 2, "gravity axis = Z");
    CHECK_EQ(c.grav_sign, 1, "gravity sign = +1");
    CHECK_EQ(imu_calib_result(&c, &bias1), 0, "result ok");
    CHECK_NEAR(bias1.gx, TRUTH.gx, 0.1, "a: bias gx");
    CHECK_NEAR(bias1.gy, TRUTH.gy, 0.1, "a: bias gy");
    CHECK_NEAR(bias1.gz, TRUTH.gz, 0.1, "a: bias gz");
    CHECK_NEAR(bias1.ax, TRUTH.ax, 0.1, "a: bias ax");
    CHECK_NEAR(bias1.ay, TRUTH.ay, 0.1, "a: bias ay");
    CHECK_NEAR(bias1.az, TRUTH.az, 0.1, "a: bias az");
    CHECK_EQ(imu_calib_feed(&c, &s, &s), -1, "feed rejected after complete");

    /* ============ b. 防回归：同实例第二轮（y 朝上）= 一致 ============
     * 旧式实现雷：start 不清 raw 三轴累加 → 第二轮重力轴判定被上轮 z 轴
     * 残留污染 → ay/az 零偏差约 1g。修后两轮都应回到同一真值。 */
    CHECK_EQ(imu_calib_start(&c), 0, "start round 2 (restart clears all sums)");
    for (int i = 0; i < bcfg->target_n; i++) {
        s = synth_y_up(i);
        CHECK_EQ(imu_calib_feed(&c, &s, &s), 0, "feed round 2");
        if (i == bcfg->target_n / 2) {
            /* 幂等路径：采集中重发 start 不重启、不清进度 */
            const unsigned long n_before = c.n;
            CHECK_EQ(imu_calib_start(&c), 0, "idempotent start re-send ok");
            CHECK_EQ(c.n, n_before, "idempotent start keeps progress");
        }
    }
    CHECK_EQ(imu_calib_ready(&c), 1, "round 2 ready");
    CHECK_EQ(c.grav_axis, 1, "round 2 gravity axis = Y");
    CHECK_EQ(c.grav_sign, 1, "round 2 gravity sign = +1");
    CHECK_EQ(imu_calib_result(&c, &bias2), 0, "round 2 result ok");
    /* 第二轮与第一轮一致（同一真值、不同姿态） */
    CHECK_NEAR(bias2.gx, bias1.gx, 0.1, "b: round2 gx == round1");
    CHECK_NEAR(bias2.gy, bias1.gy, 0.1, "b: round2 gy == round1");
    CHECK_NEAR(bias2.gz, bias1.gz, 0.1, "b: round2 gz == round1");
    CHECK_NEAR(bias2.ax, bias1.ax, 0.1, "b: round2 ax == round1");
    CHECK_NEAR(bias2.ay, bias1.ay, 0.1, "b: round2 ay == round1");
    CHECK_NEAR(bias2.az, bias1.az, 0.1, "b: round2 az == round1");
    /* 且都与真值一致（防"两轮一起漂"的假一致） */
    CHECK_NEAR(bias2.gx, TRUTH.gx, 0.1, "b: round2 gx == truth");
    CHECK_NEAR(bias2.gy, TRUTH.gy, 0.1, "b: round2 gy == truth");
    CHECK_NEAR(bias2.gz, TRUTH.gz, 0.1, "b: round2 gz == truth");
    CHECK_NEAR(bias2.ax, TRUTH.ax, 0.1, "b: round2 ax == truth");
    CHECK_NEAR(bias2.ay, TRUTH.ay, 0.1, "b: round2 ay == truth");
    CHECK_NEAR(bias2.az, TRUTH.az, 0.1, "b: round2 az == truth");

    /* ============ c. 去偏验证：带零偏样本 − bias → 残差 < 0.05 ============ */
    for (int i = 0; i < 10; i++) {
        s = synth_z_up(1000 + i);   /* 标定后的新鲜样本 */
        CHECK_NEAR(s.gx - bias1.gx, 0.0, 0.05, "c: residual gx");
        CHECK_NEAR(s.gy - bias1.gy, 0.0, 0.05, "c: residual gy");
        CHECK_NEAR(s.gz - bias1.gz, 0.0, 0.05, "c: residual gz");
        CHECK_NEAR(s.ax - bias1.ax, 0.0, 0.05, "c: residual ax");
        CHECK_NEAR(s.ay - bias1.ay, 0.0, 0.05, "c: residual ay");
        CHECK_NEAR(s.az - bias1.az - 1.0f, 0.0, 0.05, "c: residual az (gravity removed)");
    }

    /* ============ e. 残差累加（自动补偿法）：bias += delta，不是覆盖 ============
     * 从「偏差很大」的初值起标（gx 偏 +3 dps、重力轴 az 偏 −0.8 g）：
     * 若实现是「覆盖」，结果会是 delta（≈ −3）而非真值；累加才回到真值。 */
    imu_bias_t b_acc = TRUTH;
    b_acc.gx += 3.0f;
    b_acc.az -= 0.8f;
    imu_bias_t delta = { 0 };
    CHECK_EQ(imu_calib_init(&c, bcfg), 0, "accum: init");
    CHECK_EQ(imu_calib_start(&c), 0, "accum: start round 1");
    for (int i = 0; i < bcfg->target_n; i++) {
        s = synth_z_up(i);
        /* 自动补偿法：喂**当前零偏补偿后**的残差，raw 单独传（判重力轴） */
        imu_sample_t resid = s;
        resid.gx -= b_acc.gx;
        resid.gy -= b_acc.gy;
        resid.gz -= b_acc.gz;
        resid.ax -= b_acc.ax;
        resid.ay -= b_acc.ay;
        resid.az -= b_acc.az;
        CHECK_EQ(imu_calib_feed(&c, &resid, &s), 0, "accum: feed residual");
    }
    CHECK_EQ(imu_calib_ready(&c), 1, "accum: ready round 1");
    CHECK_EQ(c.grav_axis, 2, "accum: gravity axis from raw = Z");
    CHECK_EQ(imu_calib_result(&c, &delta), 0, "accum: result ok");
    /* 本轮结果是**残差**（≈ −3 dps），不是绝对零偏（≈ 2.5 dps） */
    CHECK_NEAR(delta.gx, -3.0, 0.05, "e: result is residual (-3), not absolute bias");
    CHECK_NEAR(delta.az, 0.8, 0.05, "e: gravity-axis residual cancels the step (+0.8)");
    CHECK_EQ(imu_calib_accumulate(&c, &b_acc), 0, "accumulate round 1 ok");
    CHECK_NEAR(b_acc.gx, TRUTH.gx, 0.05, "e: accumulate -> truth (gx)");
    CHECK_NEAR(b_acc.gy, TRUTH.gy, 0.05, "e: accumulate -> truth (gy)");
    CHECK_NEAR(b_acc.gz, TRUTH.gz, 0.05, "e: accumulate -> truth (gz)");
    CHECK_NEAR(b_acc.ax, TRUTH.ax, 0.05, "e: accumulate -> truth (ax)");
    CHECK_NEAR(b_acc.ay, TRUTH.ay, 0.05, "e: accumulate -> truth (ay)");
    CHECK_NEAR(b_acc.az, TRUTH.az, 0.05, "e: accumulate -> truth (az, gravity axis)");

    /* 连续两次标定：轮 1 后再引入 0.5 dps 偏差（等价 SET_BIAS 手改），
     * 轮 2 的残差只应修正这 0.5，累加后回到真值（覆盖式会留在 −0.5） */
    b_acc.gy += 0.5f;
    CHECK_EQ(imu_calib_start(&c), 0, "accum: start round 2");
    for (int i = 0; i < bcfg->target_n; i++) {
        s = synth_z_up(i);
        imu_sample_t resid = s;
        resid.gx -= b_acc.gx;
        resid.gy -= b_acc.gy;
        resid.gz -= b_acc.gz;
        resid.ax -= b_acc.ax;
        resid.ay -= b_acc.ay;
        resid.az -= b_acc.az;
        CHECK_EQ(imu_calib_feed(&c, &resid, &s), 0, "accum: feed residual round 2");
    }
    CHECK_EQ(imu_calib_result(&c, &delta), 0, "accum: result round 2");
    CHECK_NEAR(delta.gx, 0.0, 0.05, "e2: round2 gx residual ~ 0 (already calibrated)");
    CHECK_NEAR(delta.gy, -0.5, 0.05, "e2: round2 gy residual = step error only");
    CHECK_EQ(imu_calib_accumulate(&c, &b_acc), 0, "accumulate round 2 ok");
    CHECK_NEAR(b_acc.gx, TRUTH.gx, 0.05, "e2: two rounds accumulate -> truth (gx)");
    CHECK_NEAR(b_acc.gy, TRUTH.gy, 0.05, "e2: two rounds accumulate -> truth (gy)");
    CHECK_NEAR(b_acc.az, TRUTH.az, 0.05, "e2: two rounds accumulate -> truth (az)");
    CHECK_EQ(imu_calib_accumulate(NULL, &b_acc), -1, "accumulate NULL instance rejected");
    CHECK_EQ(imu_calib_accumulate(&c, NULL), -1, "accumulate NULL bias rejected");
    imu_calib_t c_idle;
    CHECK_EQ(imu_calib_init(&c_idle, bcfg), 0, "accum: idle init");
    CHECK_EQ(imu_calib_accumulate(&c_idle, &b_acc), -1, "accumulate rejected when not ready");

    /* ============ d. 温补：两温度点 → 温标 → 中间温度补偿 ============ */
    /* 真值模型：bias(T) = bias_ref + k·(T − 25)，z 朝上静止（az 含 +1g 常值，
     * 进拟合截距不影响斜率）。温标样本数改小（400）提速，其余参数取预设。 */
    temp_comp_cfg_t dcfg = *dcfg_p;
    dcfg.calib_target_n = 400;
    const float T_REF = 25.0f, T1 = 22.0f, T2 = 27.0f, T_MID = 24.5f;
    const float KG[3] = { 0.08f, -0.02f, 0.05f };        /* dps/°C */
    const float KA[3] = { 0.0004f, -0.0002f, 0.0003f };  /* g/°C */
    const imu_bias_t BREF = {                            /* T_REF 处真值零偏 */
        .gx = 1.2f,  .gy = -0.6f, .gz = 0.4f,
        .ax = 0.01f, .ay = -0.005f, .az = 0.008f,
    };

    temp_calib_t tcal;
    CHECK_EQ(temp_calib_init(&tcal, &dcfg), 0, "temp_calib init ok");
    CHECK_EQ(temp_calib_result(&tcal, kg, ka), -1, "temp_calib result rejected before fit");
    CHECK_EQ(temp_calib_start(&tcal), 0, "temp_calib start");
    for (int i = 0; i < dcfg.calib_target_n; i++) {
        /* 前 200 样本在 T1、后 200 在 T2（跨度 5°C ≥ 2°C） */
        const float t = (i < dcfg.calib_target_n / 2) ? T1 : T2;
        const float dt = t - T_REF;
        s.gx = BREF.gx + KG[0] * dt + noise(i);
        s.gy = BREF.gy + KG[1] * dt - noise(i);
        s.gz = BREF.gz + KG[2] * dt + noise(i);
        s.ax = BREF.ax + KA[0] * dt + noise(i);
        s.ay = BREF.ay + KA[1] * dt - noise(i);
        s.az = 1.0f + BREF.az + KA[2] * dt + noise(i);
        CHECK_EQ(temp_calib_feed(&tcal, t, &s), 0, "temp_calib feed ok");
    }
    CHECK_EQ(temp_calib_ready(&tcal), 1, "temp_calib ready");
    CHECK_EQ(temp_calib_valid(&tcal), 1, "temp_calib fit valid");
    CHECK_EQ(temp_calib_result(&tcal, kg, ka), 0, "temp_calib result ok");
    CHECK_NEAR(kg[0], KG[0], 0.001, "d: fitted gyro kx");
    CHECK_NEAR(kg[1], KG[1], 0.001, "d: fitted gyro ky");
    CHECK_NEAR(kg[2], KG[2], 0.001, "d: fitted gyro kz");
    CHECK_NEAR(ka[0], KA[0], 0.0001, "d: fitted accel kx");
    CHECK_NEAR(ka[1], KA[1], 0.0001, "d: fitted accel ky");
    CHECK_NEAR(ka[2], KA[2], 0.0001, "d: fitted accel kz");

    /* 应用：中间温度补偿误差 << 不补偿 */
    temp_comp_t tc;
    CHECK_EQ(temp_comp_init(&tc, &dcfg, T_REF, &BREF), 0, "temp_comp init ok");
    CHECK_EQ(temp_comp_set_drift(&tc, kg, ka), 0, "set_drift ok");
    CHECK_EQ(temp_comp_apply(&tc, T_MID, &b), 0, "apply at mid temp");
    /* T_MID 处真值零偏 = BREF + k·(T_MID − T_REF) */
    CHECK_NEAR(b.gx, BREF.gx + KG[0] * (T_MID - T_REF), 0.01, "d: comp gx ~= truth");
    CHECK_NEAR(b.gy, BREF.gy + KG[1] * (T_MID - T_REF), 0.01, "d: comp gy ~= truth");
    CHECK_NEAR(b.gz, BREF.gz + KG[2] * (T_MID - T_REF), 0.01, "d: comp gz ~= truth");
    CHECK_NEAR(b.ax, BREF.ax + KA[0] * (T_MID - T_REF), 0.0001, "d: comp ax ~= truth");
    CHECK_NEAR(b.ay, BREF.ay + KA[1] * (T_MID - T_REF), 0.0001, "d: comp ay ~= truth");
    CHECK_NEAR(b.az, BREF.az + KA[2] * (T_MID - T_REF), 0.0001, "d: comp az ~= truth");
    /* 显著性：补偿误差 < 不补偿误差 / 10（不补偿 = 沿用 T_REF 处零偏） */
    CHECK(fabs((double)(b.gx - (BREF.gx + KG[0] * (T_MID - T_REF)))) <
          fabs((double)(KG[0] * (T_MID - T_REF))) / 10.0, "d: comp err << uncomp (gx)");
    CHECK(fabs((double)(b.az - (BREF.az + KA[2] * (T_MID - T_REF)))) <
          fabs((double)(KA[2] * (T_MID - T_REF))) / 10.0, "d: comp err << uncomp (az)");

    /* 阶梯语义：档内（Δ0.1 < 0.5）保持；跨档（Δ0.8 ≥ 0.5）重算 */
    imu_bias_t b_hold;
    CHECK_EQ(temp_comp_apply(&tc, T_MID + 0.1f, &b_hold), 0, "apply within step");
    CHECK(feq(b_hold.gx, b.gx), "ladder holds bias within step");
    CHECK(feq(b_hold.az, b.az), "ladder holds bias within step (az)");
    CHECK_EQ(temp_comp_apply(&tc, T_MID + 0.8f, &b_hold), 0, "apply across step");
    CHECK_NEAR(b_hold.gx, BREF.gx + KG[0] * 0.3f, 0.001, "recompute across step (gx)");
    CHECK_NEAR(b_hold.az, BREF.az + KA[2] * 0.3f, 0.0001, "recompute across step (az)");

    /* disable：回退参考值，且再 apply 立即重算（不被阶梯挡） */
    CHECK_EQ(temp_comp_disable(&tc, &b), 0, "disable ok");
    CHECK(feq(b.gx, BREF.gx), "disable restores bias_ref (gx)");
    CHECK(feq(b.az, BREF.az), "disable restores bias_ref (az)");
    CHECK_EQ(temp_comp_apply(&tc, T_MID, &b), 0, "apply after disable (immediate)");
    CHECK_NEAR(b.gx, BREF.gx + KG[0] * (T_MID - T_REF), 0.001, "post-disable apply recomputes");

    /* 双判（开关位 && 已标定）：两项缺一都不补偿（原型 poll_temp_tick 同式） */
    CHECK_EQ(temp_comp_should_apply(1, 1), 1, "gate: enabled && valid -> apply");
    CHECK_EQ(temp_comp_should_apply(1, 0), 0, "gate: enabled but uncalibrated -> no apply");
    CHECK_EQ(temp_comp_should_apply(0, 1), 0, "gate: calibrated but disabled -> no apply");
    CHECK_EQ(temp_comp_should_apply(0, 0), 0, "gate: both off -> no apply");

    /* ============ 附加：钳制 / 无效拟合 / 温标段参数校验 ============ */
    float bad_k[3] = { 14.0f, 0.03f, -0.04f };   /* 14 dps/°C 超上限，其余在界内 */
    float ok_a[3]  = { 0.0001f, 0.0f, 0.0f };
    CHECK_EQ(temp_comp_set_drift(&tc, bad_k, ok_a), 0, "set_drift clamps ok");
    CHECK(feq(tc.gyro_drift[0], 0.0f), "out-of-range axis zeroed");
    CHECK(feq(tc.gyro_drift[1], 0.03f), "in-range axis kept (gy)");
    CHECK(feq(tc.gyro_drift[2], -0.04f), "in-range axis kept (gz)");

    temp_comp_cfg_t bad_dcfg = dcfg;
    bad_dcfg.step_c = -1.0f;
    CHECK_EQ(temp_comp_init(&tc, &bad_dcfg, T_REF, &BREF), -1, "temp_comp init step<0 rejected");
    bad_dcfg = dcfg;
    bad_dcfg.calib_target_n = 1;
    CHECK_EQ(temp_calib_init(&tcal, &bad_dcfg), -1, "temp_calib init n<2 rejected");
    CHECK_EQ(temp_calib_init(&tcal, NULL), -1, "temp_calib init NULL cfg rejected");
    CHECK_EQ(temp_calib_start(NULL), -1, "temp_calib start NULL rejected");
    CHECK_EQ(temp_calib_feed(NULL, 25.0f, &s), -1, "temp_calib feed NULL rejected");
    CHECK_EQ(temp_calib_result(NULL, kg, ka), -1, "temp_calib result NULL rejected");
    CHECK_EQ(temp_comp_apply(NULL, 25.0f, &b), -1, "apply NULL instance rejected");
    CHECK_EQ(temp_comp_apply(&tc, 25.0f, NULL), -1, "apply NULL bias rejected");
    CHECK_EQ(temp_comp_disable(&tc, NULL), -1, "disable NULL bias rejected");

    /* 无效拟合路径 1：温度恒定（跨度 0）→ 系数全 0、valid=0 */
    CHECK_EQ(temp_calib_init(&tcal, &dcfg), 0, "temp_calib re-init ok");
    CHECK_EQ(temp_calib_start(&tcal), 0, "temp_calib re-start");
    for (int i = 0; i < dcfg.calib_target_n; i++) {
        s = synth_z_up(i);
        CHECK_EQ(temp_calib_feed(&tcal, T_REF, &s), 0, "feed constant temp ok");
    }
    CHECK_EQ(temp_calib_ready(&tcal), 1, "constant temp: ready");
    CHECK_EQ(temp_calib_valid(&tcal), 0, "constant temp: invalid");
    CHECK_EQ(temp_calib_result(&tcal, kg, ka), 0, "constant temp: result ok");
    CHECK(feq(kg[0], 0.0f) && feq(kg[1], 0.0f) && feq(kg[2], 0.0f), "constant temp: k all zero");

    /* 无效拟合路径 2：斜率超物理上限（5 dps/°C > 0.5）→ 整组判废 */
    CHECK_EQ(temp_calib_start(&tcal), 0, "temp_calib re-start (slope case)");
    for (int i = 0; i < dcfg.calib_target_n; i++) {
        const float t = (i < dcfg.calib_target_n / 2) ? T1 : T2;
        s.gx = 100.0f + 5.0f * (t - T_REF);   /* 斜率 5 dps/°C */
        s.gy = 1.0f; s.gz = -2.0f;            /* 零斜率（在界内） */
        s.ax = 0.01f; s.ay = -0.02f; s.az = 1.0f;
        CHECK_EQ(temp_calib_feed(&tcal, t, &s), 0, "feed steep slope ok");
    }
    CHECK_EQ(temp_calib_ready(&tcal), 1, "steep slope: ready");
    CHECK_EQ(temp_calib_valid(&tcal), 0, "steep slope: invalid");
    CHECK_EQ(temp_calib_result(&tcal, kg, ka), 0, "steep slope: result ok");
    CHECK(feq(kg[0], 0.0f) && feq(kg[1], 0.0f) && feq(kg[2], 0.0f), "steep slope: k all zero");
    CHECK(feq(ka[0], 0.0f) && feq(ka[2], 0.0f), "steep slope: accel k all zero");

    printf("== test_calibration: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
