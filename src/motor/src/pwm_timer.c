/* SPDX-License-Identifier: MIT */
/**
 * @file    pwm_timer.c
 * @brief   PWM 定时器实例逻辑 + 预设注册表（平台无关；平台面经 pwm_hal_t 抽象）
 *
 * @note    实例化模型：create 一次 = 一个定时器实例（载波频率 + 通道组）；
 *          通道占用登记在实例内部（一通道一电机，flag 而非引用计数——不过度设计）。
 *          嵌套语义天然成立：电机配置持有本实例句柄 + 各自 channel。
 * @note    destroy 语义（对契约头「-1 = 仍有电机关联」的完整实现面）：
 *          -1 返回于 ① 句柄无效/NULL；② 仍有通道被占用（先销毁电机——
 *          上层保证的协议，本层以占用登记强制执法）；③ 平台 timer_release
 *          失败（实例保留、句柄仍可用、可重试 destroy）。
 * @note    防雷（对照旧式实现 pwm.c，按审查清单重组）：
 *          - 旧式实现 LEDC timer 只增不减 → destroy 真调 timer_release；
 *            create 中 malloc 失败也回滚平台资源，不留孤儿定时器。
 *          - 哨兵与值域重叠 → freq 的 0 保留为「句柄无效」回值，
 *            create 一律拒绝 freq=0，合法频域与哨兵无重叠。
 *          - 换频不贯通 → 频率只存于本实例，唯一取频入口 pwm_timer_freq()。
 * @note    句柄护栏（尽力防御，非契约保证）：destroy 成功即 free 实例；
 *          正式契约与 free() 同——销毁后不得再使用句柄。护栏魔数用于
 *          主流分配器下的确定性拒绝（销毁前置 0 再 free），不作保证。
 */

#include <stdlib.h>
#include <string.h>
#include "mtr/pwm_timer.h"
#include "mtr/pwm_hal.h"

/* 句柄护栏魔数（活实例标识；销毁前置 0——见文件头注） */
#define PWM_TIMER_MAGIC 0x50574D31u

/** @brief 定时器实例（不透明结构，对外只暴露 pwm_timer_t*） */
struct pwm_timer {
    uint32_t          guard;       /**< 护栏魔数（活实例 = PWM_TIMER_MAGIC） */
    const pwm_hal_t  *hal;         /**< create 时快照的平台操作表 */
    void             *plat;        /**< 平台定时器句柄（timer_config 出参，原样归还） */
    uint32_t          freq_hz;     /**< 载波频率，Hz（唯一频率事实源） */
    uint8_t           channels;    /**< 通道数（chan_used 长度） */
    uint8_t           chan_used[]; /**< 通道占用登记：0 = 空闲，1 = 被电机占用 */
};

/* 平台注册表（启动期注册一次；实例 create 时快照，运行期不换） */
static const pwm_hal_t *s_hal = 0;

/* ---------- 句柄护栏 ---------- */

/** @brief 活实例校验（NULL / 已销毁句柄拒绝） */
static int timer_alive(const pwm_timer_t *t)
{
    return t != 0 && t->guard == PWM_TIMER_MAGIC;
}

/* ---------- 平台注册 ---------- */

int pwm_hal_register(const pwm_hal_t *hal)
{
    if (hal == 0)                                    return -1; /* 空表拒绝 */
    if (hal->timer_config == 0 || hal->timer_release == 0 ||
        hal->channel_set_duty == 0 || hal->channel_enable == 0)
        return -1;                                   /* 表缺位拒绝 */
    if (s_hal != 0)                                  return -1; /* 启动期一次，重复拒绝 */
    s_hal = hal;
    return 0;
}

/* ---------- 生命周期 ---------- */

pwm_timer_t *pwm_timer_create(uint32_t freq_hz, uint8_t channels)
{
    if (s_hal == 0)        return 0;   /* 平台未注册 */
    if (freq_hz == 0u)     return 0;   /* 载波频率非法（0 保留给无效句柄回值） */
    if (channels == 0u)    return 0;   /* 通道数非法（上限由平台 timer_config 裁） */

    void *plat = 0;
    if (s_hal->timer_config(freq_hz, channels, &plat, s_hal->ctx) != 0)
        return 0;   /* 超平台能力或资源不足（平台拒绝也走此分支） */

    pwm_timer_t *t = (pwm_timer_t *)malloc(sizeof *t + channels);
    if (t == 0) {
        /* 回滚：平台资源已配，不留孤儿定时器（防「只增不减」雷） */
        (void)s_hal->timer_release(plat, s_hal->ctx);
        return 0;
    }
    t->guard    = PWM_TIMER_MAGIC;
    t->hal      = s_hal;
    t->plat     = plat;
    t->freq_hz  = freq_hz;
    t->channels = channels;
    memset(t->chan_used, 0, (size_t)channels);
    return t;
}

int pwm_timer_destroy(pwm_timer_t *t)
{
    if (!timer_alive(t)) return -1;   /* NULL / 已销毁句柄 */

    for (uint8_t c = 0; c < t->channels; c++)
        if (t->chan_used[c] != 0u) return -1;   /* 仍有电机关联（先销毁电机） */

    if (t->hal->timer_release(t->plat, t->hal->ctx) != 0)
        return -1;   /* 平台释放失败：实例保留，句柄仍可用，可重试 */

    t->guard = 0u;   /* 护栏失效标记（尽力防御，见文件头注） */
    free(t);
    return 0;
}

uint32_t pwm_timer_freq(const pwm_timer_t *t)
{
    if (!timer_alive(t)) return 0u;   /* 0 = 句柄无效（create 拒绝 0 频率，无值域重叠） */
    return t->freq_hz;
}

/* ---------- 通道面（占用登记 + HAL 路由；嵌套关联的落地） ---------- */

int pwm_timer_channel_acquire(pwm_timer_t *t, uint8_t channel)
{
    if (!timer_alive(t))  return -1;
    if (channel >= t->channels) return -1;   /* 越界（电机 cfg 的 channel 须在范围内） */
    if (t->chan_used[channel] != 0u) return -1;   /* 一通道一电机，重复占用拒绝 */
    t->chan_used[channel] = 1u;
    return 0;
}

int pwm_timer_channel_release(pwm_timer_t *t, uint8_t channel)
{
    if (!timer_alive(t))  return -1;
    if (channel >= t->channels) return -1;
    if (t->chan_used[channel] == 0u) return -1;   /* 本就未占用 */
    t->chan_used[channel] = 0u;
    return 0;
}

int pwm_timer_channel_set_duty(pwm_timer_t *t, uint8_t channel, float duty)
{
    if (!timer_alive(t))  return -1;
    if (channel >= t->channels) return -1;
    if (t->chan_used[channel] == 0u) return -1;   /* 只许操作本电机占用的通道 */
    if (!(duty >= 0.0f && duty <= 1.0f)) return -1;   /* 越界/NaN 拒绝（不夹紧） */
    return t->hal->channel_set_duty(t->plat, channel, duty, t->hal->ctx);
}

int pwm_timer_channel_enable(pwm_timer_t *t, uint8_t channel, int enable)
{
    if (!timer_alive(t))  return -1;
    if (channel >= t->channels) return -1;
    if (t->chan_used[channel] == 0u) return -1;
    return t->hal->channel_enable(t->plat, channel, enable, t->hal->ctx);
}

/* ---------- 预设注册表（参数零写死：档位值只住本表） ---------- */

/**
 * @note 档位来源：旧式实现 PLAN-005（20kHz 直流载波 / 50Hz 舵机载波），
 *       实机验证过的组合；分辨率由平台 timer_config 自选（本层零平台知识）。
 * @note channels 是常用默认（业务层可自建任意通道数实例），不是上限。
 */
const pwm_timer_preset_t pwm_timer_presets[] = {
    { "pwm.motor.20k", 20000u, 4u },   /* 直流电机载波：四轮底盘常用 4 通道 */
    { "pwm.servo.50",     50u, 4u },   /* 舵机载波：脉宽 0.5~2.5ms 域 */
};

const int pwm_timer_preset_count =
    (int)(sizeof(pwm_timer_presets) / sizeof(pwm_timer_presets[0]));

pwm_timer_t *pwm_timer_create_preset(const char *name)
{
    if (name == 0) return 0;
    for (int i = 0; i < pwm_timer_preset_count; i++) {
        if (strcmp(pwm_timer_presets[i].name, name) == 0)
            return pwm_timer_create(pwm_timer_presets[i].freq_hz,
                                    pwm_timer_presets[i].channels);
    }
    return 0;   /* 无此预设 */
}
