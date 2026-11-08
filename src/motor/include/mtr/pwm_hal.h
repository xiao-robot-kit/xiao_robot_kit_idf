/* SPDX-License-Identifier: MIT */
#ifndef MTR_PWM_HAL_H
#define MTR_PWM_HAL_H

#include <stdint.h>
#include "mtr/pwm_timer.h"

/**
 * @file    pwm_hal.h
 * @brief   PWM 平台抽象层（HAL 函数指针表 + 通道面组件 API）
 *
 * @note    分层（对齐 imu_hal_i2c.h 模式，依赖单向）：
 *          port 层（如 ESP-IDF LEDC）填一份 pwm_hal_t 并在启动期注册；
 *          本组件（pwm_timer.c）与上层电机驱动只认本头，不 include 任何平台头。
 * @note    返回值语义（全平台统一）：0 = 成功；-1 = 失败。无部分成功。
 * @note    平台实现方契约（不遵守 = port 层 bug）：
 *          - timer_config 成功后载波立即输出，全部通道占空 0（上电安全态）；
 *          - timer_release 必须真实释放平台定时器与其全部通道资源
 *            （旧式实现雷：LEDC timer 只配置永不释放——禁止照搬，mock 单测守此条）；
 *          - plat 句柄经 timer_release 归还后，一切 channel_* 不得再接受它。
 * @note    频率单源：载波频率只属于 pwm_timer_t 实例（创建参数）；
 *          电机层换算一律经 pwm_timer_freq(句柄) 取频，cfg 内不复制频率值
 *          （防「换频不贯通」雷：单源无分叉）。
 */

/** @brief PWM 平台操作表（port 层注册；实例在 create 时快照本表指针） */
typedef struct {
    /**
     * @brief   配置一个平台定时器（一组同频通道）
     * @param   freq_hz   载波频率，Hz（>0；上限由平台能力决定，超限拒绝）
     * @param   channels  该定时器名下的通道数（1~平台上限）
     * @param   out_plat  出参：平台定时器句柄（组件不透明转存，
     *                    release 时原样归还；非 NULL）
     * @return  0 = 成功（载波已输出，占空全 0）；-1 = 参数超平台能力或资源不足
     */
    int  (*timer_config)(uint32_t freq_hz, uint8_t channels,
                         void **out_plat, void *ctx);

    /**
     * @brief   真实释放平台定时器（含其全部通道资源）
     * @param   plat  timer_config 的出参句柄
     * @return  0 = 成功（句柄自此无效）；-1 = 释放失败（句柄仍有效，可重试）
     */
    int  (*timer_release)(void *plat, void *ctx);

    /**
     * @brief   设通道占空比
     * @param   duty   0.0~1.0（0 = 恒低，1 = 恒高；桥向由上层双通道表达）
     * @return  0 = 成功；-1 = 句柄无效 / 通道越界 / 平台失败
     */
    int  (*channel_set_duty)(void *plat, uint8_t channel, float duty, void *ctx);

    /**
     * @brief   通道输出使能
     * @param   enable  1 = 输出载波（按当前占空）；0 = 停止输出（安全低电平）
     * @return  0 = 成功；-1 = 句柄无效 / 通道越界 / 平台失败
     */
    int  (*channel_enable)(void *plat, uint8_t channel, int enable, void *ctx);

    /** @brief 平台私有上下文（时钟源 / 速度模式等），由平台实现填充 */
    void *ctx;
} pwm_hal_t;

/**
 * @brief   注册平台 HAL（port 层启动期调用一次，之后不改）
 * @return  0 = 成功；-1 = 参数非法（NULL / 表内函数缺位）或重复注册
 * @note    实例在 create 时快照注册表指针：运行期注册表不换、不影响存量实例。
 */
int pwm_hal_register(const pwm_hal_t *hal);

/* ---- 通道面组件 API（电机层经定时器句柄操作通道——嵌套关联的落地） ---- */
/* 嵌套用法：多台电机传同一定时器句柄 + 各自 channel；一通道同时只属一台电机。 */

/**
 * @brief   占用一个通道（电机 create 时调用；占用登记在实例内部）
 * @return  0 = 成功；-1 = 句柄无效 / 通道越界 / 该通道已被占用
 */
int pwm_timer_channel_acquire(pwm_timer_t *t, uint8_t channel);

/**
 * @brief   释放通道占用（电机 destroy 时调用；全部通道释放后定时器才可 destroy）
 * @return  0 = 成功；-1 = 句柄无效 / 通道越界 / 该通道本就未占用
 */
int pwm_timer_channel_release(pwm_timer_t *t, uint8_t channel);

/**
 * @brief   设该通道占空比（仅对已占用通道生效）
 * @param   duty  0.0~1.0，越界拒绝（不静默夹紧；NaN 一并拒绝）
 * @return  0 = 成功；-1 = 句柄无效 / 通道未占用 / 占空越界 / 平台失败
 */
int pwm_timer_channel_set_duty(pwm_timer_t *t, uint8_t channel, float duty);

/**
 * @brief   该通道输出使能 / 停止
 * @param   enable  1 = 输出载波；0 = 停止（安全低电平）
 * @return  0 = 成功；-1 = 句柄无效 / 通道未占用 / 平台失败
 */
int pwm_timer_channel_enable(pwm_timer_t *t, uint8_t channel, int enable);

#endif /* MTR_PWM_HAL_H */
