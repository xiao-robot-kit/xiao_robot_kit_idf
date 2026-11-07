/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_hal_sim.c
 * @brief   电机 QEMU/仿真 port 层实现（契约见 motor_hal_sim.h；纯 C99 零平台依赖）
 *
 * @note    原创律：自研合成模型，零第三方。固件与 host gcc 自检共用本文件
 *          ——两侧行为逐字节一致（QEMU 实跑波形 = host 自检波形的重放）。
 */

#include <stddef.h>
#include "motor_hal_sim.h"

/* ── 容量（port 层资源表，非业务参数） ── */
#define SIM_PWM_TIMERS_MAX    4
#define SIM_PWM_CHANNELS_MAX  8
#define SIM_PLANTS_MAX        4

/** PWM 合成定时器记录（plat 句柄即指向本记录） */
typedef struct {
    uint32_t freq_hz;                         /**< 载波频率（duty 存储键） */
    uint8_t  channels;                        /**< 通道数 */
    float    duty[SIM_PWM_CHANNELS_MAX];      /**< 各通道当前占空 */
    uint8_t  used;                            /**< 占用标记 */
} sim_pwm_rec_t;

/** 合成编码器 plant（一阶滞后 + 计数累积） */
typedef struct {
    sim_motor_plant_cfg_t cfg;                /**< 注册配置（深拷） */
    double   rpm;                             /**< plant 当前转速（带符号 rpm） */
    double   count_f;                         /**< 累计计数（浮点累积，读出取整） */
    uint8_t  used;
} sim_plant_t;

static sim_pwm_rec_t s_pwm[SIM_PWM_TIMERS_MAX];
static sim_plant_t   s_plants[SIM_PLANTS_MAX];

/* ── PWM 合成存储检索 ── */

static sim_pwm_rec_t *pwm_rec_by_plat(void *plat)
{
    sim_pwm_rec_t *r = (sim_pwm_rec_t *)plat;
    if (r == NULL || r < &s_pwm[0] || r >= &s_pwm[SIM_PWM_TIMERS_MAX] || !r->used)
        return NULL;   /* 句柄不在登记区或未占用（release 后的句柄拒收） */
    return r;
}

static sim_pwm_rec_t *pwm_rec_by_freq(uint32_t freq_hz)
{
    for (int i = 0; i < SIM_PWM_TIMERS_MAX; i++)
        if (s_pwm[i].used && s_pwm[i].freq_hz == freq_hz)
            return &s_pwm[i];
    return NULL;
}

/* ── PWM 平台操作表实现（pwm_hal_t 契约：0=成功 / -1=失败） ── */

static int sim_timer_config(uint32_t freq_hz, uint8_t channels,
                            void **out_plat, void *ctx)
{
    (void)ctx;
    if (freq_hz == 0u || channels == 0u || channels > SIM_PWM_CHANNELS_MAX ||
        out_plat == NULL)
        return -1;
    if (pwm_rec_by_freq(freq_hz) != NULL)
        return -1;   /* 同频唯一（频率是 duty 存储键，见头注约束） */
    for (int i = 0; i < SIM_PWM_TIMERS_MAX; i++) {
        if (s_pwm[i].used) continue;
        s_pwm[i].freq_hz = freq_hz;
        s_pwm[i].channels = channels;
        for (int c = 0; c < SIM_PWM_CHANNELS_MAX; c++)
            s_pwm[i].duty[c] = 0.0f;   /* 上电安全态：占空全 0 */
        s_pwm[i].used = 1;
        *out_plat = &s_pwm[i];
        return 0;
    }
    return -1;   /* 容量不足 */
}

static int sim_timer_release(void *plat, void *ctx)
{
    (void)ctx;
    sim_pwm_rec_t *r = pwm_rec_by_plat(plat);
    if (r == NULL) return -1;
    r->used = 0;   /* 真实释放（pwm_hal.h 平台契约：此后句柄不得再被接受） */
    return 0;
}

static int sim_channel_set_duty(void *plat, uint8_t channel, float duty, void *ctx)
{
    (void)ctx;
    sim_pwm_rec_t *r = pwm_rec_by_plat(plat);
    if (r == NULL || channel >= r->channels) return -1;
    r->duty[channel] = duty;
    return 0;
}

static int sim_channel_enable(void *plat, uint8_t channel, int enable, void *ctx)
{
    (void)ctx; (void)enable;   /* 合成输出无使能语义：记录于 duty 即生效 */
    sim_pwm_rec_t *r = pwm_rec_by_plat(plat);
    if (r == NULL || channel >= r->channels) return -1;
    return 0;
}

const pwm_hal_t sim_pwm_hal = {
    sim_timer_config,
    sim_timer_release,
    sim_channel_set_duty,
    sim_channel_enable,
    NULL,   /* ctx */
};

/* ── 合成编码器 HAL 实现（motor_hal.h 原子快照契约） ── */

/**
 * @brief   单调用原子快照：推进 plant 一个 cfg.dt_s 并返回累计计数
 * @note    从 PWM 合成存储读桥双通道（上一拍写入的 duty）→ 一阶滞后推
 *          转速 → 计数累积。永不复位计数器（差分语义由电机层做）。
 */
static int sim_encoder_read(int pin_a, int pin_b, int32_t *count, void *ctx)
{
    (void)ctx;
    if (count == NULL) return -1;

    sim_plant_t *p = NULL;
    for (int i = 0; i < SIM_PLANTS_MAX; i++) {
        if (!s_plants[i].used) continue;
        if (s_plants[i].cfg.enc_a == pin_a && s_plants[i].cfg.enc_b == pin_b) {
            p = &s_plants[i];
            break;
        }
    }
    if (p == NULL) return -1;   /* 引脚对未注册：如实报读失败 */

    sim_pwm_rec_t *r = pwm_rec_by_freq(pwm_timer_freq(p->cfg.timer));
    if (r == NULL) return -1;
    if (p->cfg.ch_fwd >= r->channels || p->cfg.ch_rev >= r->channels)
        return -1;

    /* 桥双通道差（正=正转，负=反转）→ plant 目标转速 */
    const float u = r->duty[p->cfg.ch_fwd] - r->duty[p->cfg.ch_rev];
    const float rpm_target = u * p->cfg.rpm_per_duty;

    /* 一阶滞后：rpm += (target - rpm)·dt/τ（α 截 1 防大步发散） */
    float alpha = p->cfg.dt_s / p->cfg.tau_s;
    if (alpha > 1.0f) alpha = 1.0f;
    p->rpm += (rpm_target - p->rpm) * (double)alpha;

    /* 计数累积：counts = rpm/60·cpr·dt（浮点累积，读出取整） */
    p->count_f += p->rpm / 60.0 * (double)p->cfg.cpr * (double)p->cfg.dt_s;
    *count = (int32_t)p->count_f;
    return 0;
}

const motor_hal_t sim_encoder_hal = {
    sim_encoder_read,
    NULL,   /* ctx */
};

/* ── plant 注册 ── */

int sim_motor_plant_register(const sim_motor_plant_cfg_t *cfg)
{
    if (cfg == NULL || cfg->timer == NULL) return -1;
    if (cfg->enc_a < 0 || cfg->enc_b < 0) return -1;
    if (!(cfg->rpm_per_duty > 0.0f) || !(cfg->tau_s > 0.0f) ||
        !(cfg->dt_s > 0.0f))
        return -1;
    if (cfg->cpr <= 0) return -1;

    for (int i = 0; i < SIM_PLANTS_MAX; i++) {
        if (!s_plants[i].used) continue;
        if (s_plants[i].cfg.enc_a == cfg->enc_a &&
            s_plants[i].cfg.enc_b == cfg->enc_b)
            return -1;   /* 引脚对重复注册 */
    }
    for (int i = 0; i < SIM_PLANTS_MAX; i++) {
        if (s_plants[i].used) continue;
        s_plants[i].cfg = *cfg;   /* 深拷 */
        s_plants[i].rpm = 0.0;
        s_plants[i].count_f = 0.0;
        s_plants[i].used = 1;
        return 0;
    }
    return -1;   /* 容量不足 */
}

int sim_motor_plant_unregister(int enc_a, int enc_b)
{
    for (int i = 0; i < SIM_PLANTS_MAX; i++) {
        if (!s_plants[i].used) continue;
        if (s_plants[i].cfg.enc_a == enc_a && s_plants[i].cfg.enc_b == enc_b) {
            s_plants[i].used = 0;   /* 释放（cfg 留旧值无害：used=0 不再被检索） */
            return 0;
        }
    }
    return -1;   /* 引脚对未注册 */
}

int sim_pwm_channel_duty(const pwm_timer_t *timer, uint8_t channel, float *out_duty)
{
    if (timer == NULL || out_duty == NULL) return -1;
    sim_pwm_rec_t *r = pwm_rec_by_freq(pwm_timer_freq(timer));
    if (r == NULL || channel >= r->channels) return -1;
    *out_duty = r->duty[channel];
    return 0;
}

int sim_pwm_bridge_duty(const pwm_timer_t *timer, uint8_t ch_fwd, uint8_t ch_rev,
                        sim_bridge_duty_t *out)
{
    if (timer == NULL || out == NULL) return -1;
    sim_pwm_rec_t *r = pwm_rec_by_freq(pwm_timer_freq(timer));
    if (r == NULL || ch_fwd >= r->channels || ch_rev >= r->channels) return -1;
    out->duty_fwd = r->duty[ch_fwd];
    out->duty_rev = r->duty[ch_rev];
    out->signed_duty = r->duty[ch_fwd] - r->duty[ch_rev];
    return 0;
}
