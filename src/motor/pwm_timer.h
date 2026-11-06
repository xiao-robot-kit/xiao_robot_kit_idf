/* SPDX-License-Identifier: MIT */
#ifndef MTR_PWM_TIMER_H
#define MTR_PWM_TIMER_H

#include <stdint.h>

/**
 * @file    pwm_timer.h
 * @brief   PWM 定时器实例（电机嵌套关联的锚点）
 *
 * @note    嵌套语义：电机配置（motor_cfg_t.timer）持有本实例句柄——
 *          多台电机传同一定时器句柄 + 各自 channel = 共定时器分通道；
 *          传不同句柄 = 各自定时器。定时器数量与频率由业务层决定。
 * @note    平台面（LEDC 等）由平台层 HAL 实现，本头零平台依赖。
 */

/** @brief PWM 定时器实例（不透明句柄，实现定义于 src） */
typedef struct pwm_timer pwm_timer_t;

/**
 * @brief   创建定时器实例
 * @param   freq_hz  PWM 载波频率，Hz（如电机 20000、舵机 50）
 * @param   channels 该定时器要用的通道数（1~N，按业务层电机数决定）
 * @return  实例句柄；NULL = 参数非法或资源不足
 * @note    创建即生效（平台层已输出载波）；销毁须先销毁全部关联电机。
 */
pwm_timer_t *pwm_timer_create(uint32_t freq_hz, uint8_t channels);

/**
 * @brief   销毁定时器实例
 * @return  0 = 成功；-1 = 仍有电机关联（先销毁电机）
 */
int pwm_timer_destroy(pwm_timer_t *t);

/** @brief 查询载波频率（Hz） */
uint32_t pwm_timer_freq(const pwm_timer_t *t);

/* ---- 定时器预设注册表（与 IMU 配置预设同构：命名挂载，零写死） ---- */

/** @brief 定时器命名预设项 */
typedef struct {
    const char *name;       /**< 预设名（如 "pwm.motor.20k" / "pwm.servo.50"） */
    uint32_t    freq_hz;    /**< 载波频率 */
    uint8_t     channels;   /**< 预设通道数 */
} pwm_timer_preset_t;

/** @brief 预设表（定义在 pwm_timer.c，追加预设只改那一个文件） */
extern const pwm_timer_preset_t pwm_timer_presets[];
extern const int                 pwm_timer_preset_count;

/**
 * @brief   按名查预设创建定时器
 * @return  实例句柄；NULL = 无此预设或创建失败
 */
pwm_timer_t *pwm_timer_create_preset(const char *name);

#endif /* MTR_PWM_TIMER_H */
