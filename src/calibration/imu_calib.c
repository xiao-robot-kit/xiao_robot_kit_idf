/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_calib.c
 * @brief   IMU 静止零偏标定实现（均值法 + 重力轴检测）
 *
 * @note    提炼自预研仓自有实现 `MagicRobot@d4eda08`
 *          `embedded/src/core/calibration/imu_calib.c`（重组注释，修正其失实处）。
 * @note    全部计算 double 累积、输出转 float（沿预研仓选型）。
 */
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "imu_calib.h"

/* ---------------------------------------------------------------- 预设表 */

const imu_calib_cfg_preset_t imu_calib_cfg_presets[] = {
    /* exp2 调零：500 样本 = 预研仓 CALIB_SAMPLES（app_ctx.h:86），
     * 100Hz 采样下约 5 秒采集窗 */
    { "cal.exp2.bias", { 500 } },
};
const int imu_calib_cfg_preset_count =
    (int)(sizeof(imu_calib_cfg_presets) / sizeof(imu_calib_cfg_presets[0]));

const imu_calib_cfg_t *imu_calib_cfg_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < imu_calib_cfg_preset_count; i++) {
        if (strcmp(name, imu_calib_cfg_presets[i].name) == 0) {
            return &imu_calib_cfg_presets[i].cfg;
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------- 实现 */

int imu_calib_init(imu_calib_t *c, const imu_calib_cfg_t *cfg)
{
    if (c == NULL || cfg == NULL) return -1;
    if (cfg->target_n < 1) return -1;

    c->cfg = *cfg;
    c->state = IMU_CALIB_IDLE;
    c->n = 0;
    c->gx_sum = c->gy_sum = c->gz_sum = 0.0;
    c->ax_sum = c->ay_sum = c->az_sum = 0.0;
    c->ax_raw_sum = c->ay_raw_sum = c->az_raw_sum = 0.0;
    c->grav_axis = -1;
    c->grav_sign = 1;
    return 0;
}

int imu_calib_start(imu_calib_t *c)
{
    if (c == NULL) return -1;
    if (c->state == IMU_CALIB_COLLECTING) {
        return 0;   /* 幂等：采集中重发命令不重启、不清进度 */
    }

    /* 预研仓雷已修【high】：旧实现只清 6 个求均值累加与 n，漏清 raw 三轴
     * （MagicRobot@d4eda08 imu_calib.c:36-42）→ 同实例第二轮标定时，
     * 重力轴判定用的是两轮叠加的 raw 均值 → 轴/符号判错 → 零偏差 ~1g。
     * 此处清全部 9 个累加 + n + 重力轴检测字段，彻底隔离两轮。 */
    c->gx_sum = c->gy_sum = c->gz_sum = 0.0;
    c->ax_sum = c->ay_sum = c->az_sum = 0.0;
    c->ax_raw_sum = c->ay_raw_sum = c->az_raw_sum = 0.0;
    c->n = 0;
    c->grav_axis = -1;
    c->grav_sign = 1;
    c->state = IMU_CALIB_COLLECTING;
    return 0;
}

int imu_calib_feed(imu_calib_t *c, const imu_sample_t *s, const imu_sample_t *raw)
{
    if (c == NULL || s == NULL) return -1;
    if (c->state != IMU_CALIB_COLLECTING) return -1;   /* 未启动即喂样 = 调用序错误 */

    c->gx_sum += s->gx;
    c->gy_sum += s->gy;
    c->gz_sum += s->gz;
    c->ax_sum += s->ax;
    c->ay_sum += s->ay;
    c->az_sum += s->az;

    /* 原始值单独累加：只服务重力轴判定（物理事实，与补偿量无关，见头文件） */
    const imu_sample_t *r = (raw != NULL) ? raw : s;
    c->ax_raw_sum += r->ax;
    c->ay_raw_sum += r->ay;
    c->az_raw_sum += r->az;

    if (++c->n < (uint32_t)c->cfg.target_n) return 0;

    /* 凑满：用原始均值判重力轴（三轴 |均值| 取最大，先比 X/Y 再比 Z） */
    c->state = IMU_CALIB_IDLE;
    const double inv = 1.0 / (double)c->cfg.target_n;
    const double ravg[3] = {
        c->ax_raw_sum * inv,
        c->ay_raw_sum * inv,
        c->az_raw_sum * inv,
    };
    int axis = 0;
    if (fabs(ravg[1]) > fabs(ravg[0])) axis = 1;
    if (fabs(ravg[2]) > fabs(ravg[axis])) axis = 2;
    c->grav_axis = axis;
    c->grav_sign = (ravg[axis] > 0.0) ? 1 : -1;
    return 0;
}

int imu_calib_ready(const imu_calib_t *c)
{
    if (c == NULL) return -1;
    const int done = (c->state == IMU_CALIB_IDLE &&
                      c->grav_axis >= 0 &&
                      c->n >= (uint32_t)c->cfg.target_n);
    return done ? 1 : 0;
}

int imu_calib_result(const imu_calib_t *c, imu_bias_t *bias)
{
    if (c == NULL || bias == NULL) return -1;
    if (imu_calib_ready(c) != 1) return -1;   /* 未就绪拒绝：不出部分均值 */

    const double inv = 1.0 / (double)c->n;
    bias->gx = (float)(c->gx_sum * inv);
    bias->gy = (float)(c->gy_sum * inv);
    bias->gz = (float)(c->gz_sum * inv);
    bias->ax = (float)(c->ax_sum * inv);
    bias->ay = (float)(c->ay_sum * inv);
    bias->az = (float)(c->az_sum * inv);

    /* 减重力：重力轴减 sign × 1g（bias = measured − expected），其余轴不减 */
    if (c->grav_axis == 0) {
        bias->ax -= c->grav_sign * IMU_CALIB_GRAVITY_G;
    } else if (c->grav_axis == 1) {
        bias->ay -= c->grav_sign * IMU_CALIB_GRAVITY_G;
    } else if (c->grav_axis == 2) {
        bias->az -= c->grav_sign * IMU_CALIB_GRAVITY_G;
    }
    return 0;
}

int imu_calib_accumulate(const imu_calib_t *c, imu_bias_t *bias_inout)
{
    if (c == NULL || bias_inout == NULL) return -1;

    imu_bias_t delta;
    if (imu_calib_result(c, &delta) != 0) return -1;   /* 未就绪拒绝：不累加部分均值 */

    /* 自动补偿法（原型 app_stream.c:106-108 同式）：bias += 残差，不是覆盖 */
    bias_inout->gx += delta.gx;
    bias_inout->gy += delta.gy;
    bias_inout->gz += delta.gz;
    bias_inout->ax += delta.ax;
    bias_inout->ay += delta.ay;
    bias_inout->az += delta.az;
    return 0;
}
