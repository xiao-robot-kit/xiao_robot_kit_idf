/* SPDX-License-Identifier: MIT */
/**
 * @file    zero_cal.c
 * @brief   零点校准口径实现（同源 exp2 calibrate_bias：N 帧均值 → 零偏 → 去偏）
 *
 * @note    算法全部委托 cal/imu_calib（xiao 组件库 v0.2.0 发布面，vendor 只读）；
 *          本文件只做会话封装 + 0x02 帧打包 + TEXT 行格式单源，无自有数学。
 * @note    与 exp3/exp4/exp5 三工程逐字同源（见 zero_cal.h 文件头）。
 */

#include <stdio.h>
#include <string.h>
#include "zero_cal.h"

int zero_cal_begin(zero_cal_t *z, const imu_calib_cfg_t *cfg)
{
    if (z == NULL || cfg == NULL) return -1;
    if (imu_calib_init(&z->cal, cfg) != 0) return -1;
    z->target_n = (uint32_t)cfg->target_n;
    return imu_calib_start(&z->cal);
}

int zero_cal_feed(zero_cal_t *z, const imu_sample_t *raw)
{
    if (z == NULL) return -1;
    /* raw 传 NULL：首轮与重标定都无「已知零偏」可作比较基准，重力轴用样本
       自身判定——与 exp2 calibrate_bias() 同口径 */
    return imu_calib_feed(&z->cal, raw, NULL);
}

int zero_cal_done(const zero_cal_t *z)
{
    return z ? imu_calib_ready(&z->cal) : -1;
}

void zero_cal_progress(const zero_cal_t *z, uint32_t *n, uint32_t *target)
{
    if (n != NULL) *n = z ? z->cal.n : 0u;
    if (target != NULL) *target = z ? z->target_n : 0u;
}

int zero_cal_result(const zero_cal_t *z, imu_bias_t *bias)
{
    return z ? imu_calib_result(&z->cal, bias) : -1;
}

void zero_cal_apply(const imu_bias_t *bias, const imu_sample_t *raw,
                    imu_sample_t *deb)
{
    deb->ax = raw->ax - bias->ax;
    deb->ay = raw->ay - bias->ay;
    deb->az = raw->az - bias->az;
    deb->gx = raw->gx - bias->gx;
    deb->gy = raw->gy - bias->gy;
    deb->gz = raw->gz - bias->gz;
}

void zero_cal_pack_frame(const imu_sample_t *raw, const imu_sample_t *deb,
                         const imu_bias_t *bias, float temp, uint8_t *out)
{
    /* 块序按协议（采样序）：raw ax..gz | deb ax..gz | bias ax..gz | temp */
    const float v[ZERO_CAL_FRAME_FLOATS] = {
        raw->ax, raw->ay, raw->az, raw->gx, raw->gy, raw->gz,
        deb->ax, deb->ay, deb->az, deb->gx, deb->gy, deb->gz,
        bias->ax, bias->ay, bias->az, bias->gx, bias->gy, bias->gz,
        temp,
    };
    memcpy(out, v, ZERO_CAL_FRAME_BYTES);   /* Xtensa 小端，与协议字节序一致（同 exp2） */
}

/** snprintf 结果 → 契约返回（写得下才算成功） */
static int text_ok(int rc, size_t cap)
{
    return (rc > 0 && (size_t)rc < cap) ? rc : 0;
}

int zero_cal_text_start(char *buf, size_t cap, uint32_t n)
{
    if (buf == NULL || cap == 0) return 0;
    return text_ok(snprintf(buf, cap, "OK calib start N=%u", (unsigned)n), cap);
}

int zero_cal_text_progress(char *buf, size_t cap, uint32_t n, uint32_t target)
{
    if (buf == NULL || cap == 0) return 0;
    return text_ok(snprintf(buf, cap, "OK calib progress %u/%u",
                            (unsigned)n, (unsigned)target), cap);
}

int zero_cal_text_done(char *buf, size_t cap, const imu_bias_t *bias, float temp)
{
    if (buf == NULL || cap == 0 || bias == NULL) return 0;
    return text_ok(snprintf(buf, cap,
                            "OK calib done bias ax=%+.4f ay=%+.4f az=%+.4f "
                            "gx=%+.4f gy=%+.4f gz=%+.4f temp=%.1f",
                            (double)bias->ax, (double)bias->ay, (double)bias->az,
                            (double)bias->gx, (double)bias->gy, (double)bias->gz,
                            (double)temp), cap);
}
