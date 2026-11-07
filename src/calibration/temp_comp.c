/* SPDX-License-Identifier: MIT */
/**
 * @file    temp_comp.c
 * @brief   IMU 温漂两段实现：最小二乘拟合（温标）+ 阶梯式补偿（应用）
 *
 * @note    提炼自预研仓自有实现 `MagicRobot@d4eda08`
 *          `embedded/src/core/calibration/temp_calib.c` 与 `temp_comp.c`
 *          （重组注释；原两文件的 doc 错位已修，见 temp_comp_apply）。
 * @note    默认系数只是量级参考（预研仓取的 datasheet 典型值）：
 *          真实温漂因芯片而异，量产路径 = 温标拟合后 set_drift 覆盖。
 */
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "temp_comp.h"

/* ---------------------------------------------------------------- 预设表 */

const temp_comp_cfg_preset_t temp_comp_cfg_presets[] = {
    /* exp2 温标+温补：取值全部锚预研仓（MagicRobot@d4eda08）——
     * 阶梯 0.5 °C（temp_comp.h:40）、最小跨度 2 °C（temp_calib.h:51）、
     * 幅值上限 0.5 dps/°C 与 0.01 g/°C（temp_calib.h:54-57）、
     * 30 s × 100 Hz = 3000 样本（app_imu.c:70-72）、
     * 默认系数 0.05 dps/°C 与 0.0005 g/°C（temp_comp.c:16,23） */
    { "cal.exp2.drift", {
        0.5f, 2.0f, 0.5f, 0.01f, 3000,
        { 0.05f, 0.05f, 0.05f },
        { 0.0005f, 0.0005f, 0.0005f },
    } },
};
const int temp_comp_cfg_preset_count =
    (int)(sizeof(temp_comp_cfg_presets) / sizeof(temp_comp_cfg_presets[0]));

const temp_comp_cfg_t *temp_comp_cfg_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < temp_comp_cfg_preset_count; i++) {
        if (strcmp(name, temp_comp_cfg_presets[i].name) == 0) {
            return &temp_comp_cfg_presets[i].cfg;
        }
    }
    return NULL;
}

/* ================================================================ 温标段 */

int temp_calib_init(temp_calib_t *tc, const temp_comp_cfg_t *cfg)
{
    if (tc == NULL || cfg == NULL) return -1;
    if (cfg->calib_target_n < 2) return -1;              /* 两点回归至少 2 样本 */
    if (!(cfg->calib_min_span_c >= 0.0f)) return -1;     /* 兼拒 NaN */
    if (!(cfg->gyro_k_max > 0.0f) || !(cfg->accel_k_max > 0.0f)) return -1;

    tc->cfg = *cfg;
    tc->state = TEMP_CALIB_IDLE;
    tc->n = 0;
    tc->sum_t = tc->sum_t2 = 0.0;
    tc->t_min = HUGE_VALF;
    tc->t_max = -HUGE_VALF;
    for (int i = 0; i < 3; i++) {
        tc->sum_g[i] = tc->sum_tg[i] = 0.0;
        tc->sum_a[i] = tc->sum_ta[i] = 0.0;
        tc->gyro_drift[i] = tc->accel_drift[i] = 0.0f;
    }
    tc->fitted = 0;
    tc->valid = 0;
    return 0;
}

int temp_calib_start(temp_calib_t *tc)
{
    if (tc == NULL) return -1;
    if (tc->state == TEMP_CALIB_COLLECTING) {
        return 0;   /* 幂等：采集中重发命令不重启、不清进度 */
    }

    /* 清全部状态：累加 / 计数 / 温度极值 / 拟合完成与有效标志。
     * （fitted/valid 清零后，上一轮拟合结果不可再被 result 读出——
     * 与 imu_calib_start 的 raw 累加雷同型的预防。） */
    tc->n = 0;
    tc->sum_t = tc->sum_t2 = 0.0;
    tc->t_min = HUGE_VALF;
    tc->t_max = -HUGE_VALF;
    for (int i = 0; i < 3; i++) {
        tc->sum_g[i] = tc->sum_tg[i] = 0.0;
        tc->sum_a[i] = tc->sum_ta[i] = 0.0;
    }
    tc->fitted = 0;
    tc->valid = 0;
    tc->state = TEMP_CALIB_COLLECTING;
    return 0;
}

/**
 * @brief   最小二乘拟合（凑满后内部调用；置 fitted/valid 与六轴系数）
 *
 * @note    无效判定（任命中 → 六轴全 0、valid=0）：
 *          ① 跨度 < calib_min_span_c 或 |denom| < TEMP_COMP_DENOM_EPS；
 *          ② 任一轴 |k| 超幅值上限 → 整组判废（不混用半好系数，
 *             沿预研仓策略，原 temp_calib.c:114-121）。
 */
static void temp_calib_fit(temp_calib_t *tc)
{
    tc->fitted = 1;

    const double N = (double)tc->n;
    const double denom = N * tc->sum_t2 - tc->sum_t * tc->sum_t;
    const float span = tc->t_max - tc->t_min;

    if (span < tc->cfg.calib_min_span_c || fabs(denom) < TEMP_COMP_DENOM_EPS) {
        for (int i = 0; i < 3; i++) {
            tc->gyro_drift[i] = 0.0f;
            tc->accel_drift[i] = 0.0f;
        }
        tc->valid = 0;
        return;
    }

    int bad = 0;
    for (int i = 0; i < 3; i++) {
        const double kg = (N * tc->sum_tg[i] - tc->sum_t * tc->sum_g[i]) / denom;
        const double ka = (N * tc->sum_ta[i] - tc->sum_t * tc->sum_a[i]) / denom;
        if (fabs(kg) > (double)tc->cfg.gyro_k_max ||
            fabs(ka) > (double)tc->cfg.accel_k_max) {
            bad = 1;
        }
        tc->gyro_drift[i] = (float)kg;
        tc->accel_drift[i] = (float)ka;
    }
    if (bad) {
        for (int i = 0; i < 3; i++) {
            tc->gyro_drift[i] = 0.0f;
            tc->accel_drift[i] = 0.0f;
        }
        tc->valid = 0;
        return;
    }
    tc->valid = 1;
}

int temp_calib_feed(temp_calib_t *tc, float temp, const imu_sample_t *s)
{
    if (tc == NULL || s == NULL) return -1;
    if (!isfinite((double)temp)) return -1;
    if (tc->state != TEMP_CALIB_COLLECTING) return -1;   /* 未启动即喂样 = 调用序错误 */

    const double t = (double)temp;
    const double g[3] = { s->gx, s->gy, s->gz };
    const double a[3] = { s->ax, s->ay, s->az };

    tc->sum_t += t;
    tc->sum_t2 += t * t;
    if (temp < tc->t_min) tc->t_min = temp;
    if (temp > tc->t_max) tc->t_max = temp;
    for (int i = 0; i < 3; i++) {
        tc->sum_g[i]  += g[i];
        tc->sum_tg[i] += t * g[i];
        tc->sum_a[i]  += a[i];
        tc->sum_ta[i] += t * a[i];
    }

    if (++tc->n < (uint32_t)tc->cfg.calib_target_n) return 0;

    /* 凑满：转 IDLE 并拟合 */
    tc->state = TEMP_CALIB_IDLE;
    temp_calib_fit(tc);
    return 0;
}

int temp_calib_ready(const temp_calib_t *tc)
{
    if (tc == NULL) return -1;
    return tc->fitted ? 1 : 0;
}

int temp_calib_valid(const temp_calib_t *tc)
{
    if (tc == NULL) return -1;
    return tc->valid ? 1 : 0;
}

int temp_calib_result(const temp_calib_t *tc, float gyro_drift[3], float accel_drift[3])
{
    if (tc == NULL || gyro_drift == NULL || accel_drift == NULL) return -1;
    if (tc->fitted != 1) return -1;   /* 未完成拒绝：不出旧一轮结果 */

    for (int i = 0; i < 3; i++) {
        gyro_drift[i] = tc->gyro_drift[i];
        accel_drift[i] = tc->accel_drift[i];
    }
    return 0;
}

/* ================================================================ 应用段 */

int temp_comp_init(temp_comp_t *tc, const temp_comp_cfg_t *cfg,
                   float t_ref, const imu_bias_t *bias_ref)
{
    if (tc == NULL || cfg == NULL || bias_ref == NULL) return -1;
    if (!isfinite((double)t_ref)) return -1;
    /* 应用段参数校验（NaN 经取反倒置命中） */
    if (!(cfg->step_c >= 0.0f)) return -1;
    if (!(cfg->gyro_k_max > 0.0f) || !(cfg->accel_k_max > 0.0f)) return -1;

    tc->cfg = *cfg;
    tc->t_ref = t_ref;
    tc->bias_ref = *bias_ref;
    for (int i = 0; i < 3; i++) {
        tc->gyro_drift[i] = cfg->default_gyro_drift[i];
        tc->accel_drift[i] = cfg->default_accel_drift[i];
    }
    tc->applied = 0;            /* 首次 apply 无条件重算（初始化生效零偏） */
    tc->last_applied_temp = 0.0f;
    tc->bias_last = *bias_ref;
    tc->apply_count = 0;
    return 0;
}

int temp_comp_set_drift(temp_comp_t *tc,
                        const float gyro_drift[3], const float accel_drift[3])
{
    if (tc == NULL || gyro_drift == NULL || accel_drift == NULL) return -1;

    /* 逐轴钳制：超幅值上限（含 NaN，比较式对 NaN 恒假 → 取反命中）置 0。
     * 预研仓实测教训：14 dps/°C 的坏系数每拍改写零偏 → "读零偏每次都不同" */
    for (int i = 0; i < 3; i++) {
        float kg = gyro_drift[i];
        float ka = accel_drift[i];
        if (!(kg > -tc->cfg.gyro_k_max && kg < tc->cfg.gyro_k_max)) kg = 0.0f;
        if (!(ka > -tc->cfg.accel_k_max && ka < tc->cfg.accel_k_max)) ka = 0.0f;
        tc->gyro_drift[i] = kg;
        tc->accel_drift[i] = ka;
    }
    tc->applied = 0;   /* 「换系数」语义：下次 apply 立即生效，不被阶梯挡住 */
    return 0;
}

int temp_comp_apply(temp_comp_t *tc, float temp, imu_bias_t *bias)
{
    if (tc == NULL || bias == NULL) return -1;
    if (!isfinite((double)temp)) return -1;

    /* 阶梯判据：温度相对**上次应用点**变化不足阈值 → 不重算，输出上次值。
     * 理由：板载温度传感器有效精度有限，0.0X °C 抖动是噪声；
     * 每拍重算会把噪声乘 k 放大成零偏抖动。首次（applied=0）必算。 */
    if (tc->applied) {
        const float step = temp - tc->last_applied_temp;
        if (!(step >= tc->cfg.step_c || step <= -tc->cfg.step_c)) {
            *bias = tc->bias_last;   /* 同档内：保持（噪声被吸收） */
            return 0;
        }
    }

    /* 跨档/首次：按**相对 t_ref 的绝对温差**重算（不逐档累加，防漂移积累） */
    const float dt = temp - tc->t_ref;
    tc->bias_last.gx = tc->bias_ref.gx + tc->gyro_drift[0] * dt;
    tc->bias_last.gy = tc->bias_ref.gy + tc->gyro_drift[1] * dt;
    tc->bias_last.gz = tc->bias_ref.gz + tc->gyro_drift[2] * dt;
    tc->bias_last.ax = tc->bias_ref.ax + tc->accel_drift[0] * dt;
    tc->bias_last.ay = tc->bias_ref.ay + tc->accel_drift[1] * dt;
    tc->bias_last.az = tc->bias_ref.az + tc->accel_drift[2] * dt;

    tc->applied = 1;
    tc->last_applied_temp = temp;
    if (tc->apply_count < UINT16_MAX) tc->apply_count++;   /* 诊断计数（饱和不回绕） */
    *bias = tc->bias_last;
    return 0;
}

int temp_comp_disable(temp_comp_t *tc, imu_bias_t *bias)
{
    if (tc == NULL || bias == NULL) return -1;

    /* 显式回退到参考值：不恢复的话零偏停在最后一次补偿值，
     * 用户会以为开关不生效（预研仓实测确认的真 bug） */
    *bias = tc->bias_ref;
    tc->bias_last = tc->bias_ref;
    tc->applied = 0;        /* 再次启用时立即重算一次 */
    tc->apply_count = 0;
    return 0;
}

int temp_comp_should_apply(int tempcomp_en, int tc_valid)
{
    /* 双判单源：开关位与标定标志分离（标定=测量，开关=应用） */
    return (tempcomp_en != 0 && tc_valid != 0) ? 1 : 0;
}
