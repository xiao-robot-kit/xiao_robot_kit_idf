/* SPDX-License-Identifier: MIT */
#ifndef MOTOR_HAL_SIM_H
#define MOTOR_HAL_SIM_H

/**
 * @file    motor_hal_sim.h
 * @brief   电机仿真/SIM port 层：PWM RAM 合成 HAL + 合成编码器 HAL（零平台依赖）
 *
 * @note    与 imu_hal_i2c.h 分层同构（依赖单向：本头只 include 电机库契约头，
 *          不 include 任何平台头）——固件与 host gcc 自检
 *          （testing/exp_motor_sim_main.c）编译同一份实现，行为完全一致。
 * @note    PWM 合成：timer_config 按「载波频率」登记一个 RAM 定时器记录，
 *          channel_set_duty 只写记录内的 duty 数组——无任何硬件访问
 *          （esp32s3 无 LEDC 模型，演示固件的输出面就是本合成存储）。
 * @note    编码器合成（合成计数回灌）：plant = 一阶滞后模型
 *          rpm → (duty_fwd − duty_rev)·rpm_per_duty，τ 秒滞后；
 *          count 按 rpm/60·cpr·dt 累积（每次 encoder_read 推进一个 dt）。
 *          plant 的 duty 从 PWM 合成存储读出——控制环（PID→桥双通道）与
 *          反馈环（编码器差分）在纯 RAM 里闭环，形成可验收的模拟示波器波形。
 * @note    约束（如实）：sim PWM 存储以载波频率为键 → 同频只允许一个定时器
 *          实例（create 第二个同频定时器被拒）。演示表（20k+50Hz 两个预设）
 *          天然满足；多电机共享同一定时器正是 pwm_timer.h 的嵌套语义。
 */

#include <stdint.h>
#include "mtr/pwm_hal.h"
#include "mtr/motor_hal.h"
#include "mtr/pwm_timer.h"

/**
 * @brief   合成编码器 plant 配置（注册进 sim 编码器 HAL 的「表」）
 * @note    timer/ch_fwd/ch_rev 定位桥双通道（合成反馈的输入）；
 *          enc_a/enc_b 是 encoder_read 的分派键（与电机 cfg 的 enc 引脚一致）；
 *          dt_s = 每次 encoder_read 推进的步长（= 业务层 poll 周期）。
 */
typedef struct {
    const pwm_timer_t *timer;      /**< 电机所在定时器（频率经 pwm_timer_freq 现取） */
    uint8_t            ch_fwd;     /**< 桥正向通道（cfg.channel） */
    uint8_t            ch_rev;     /**< 桥反向通道（cfg.channel+1） */
    int                enc_a;      /**< 编码器 A 相引脚（分派键） */
    int                enc_b;      /**< 编码器 B 相引脚（分派键） */
    float              rpm_per_duty; /**< plant 增益：duty=1 → rpm */
    float              tau_s;      /**< 一阶滞后时间常数，s */
    int32_t            cpr;        /**< 每转计数（与电机 cfg.cpr 同源） */
    float              dt_s;       /**< 步长，s（= poll 周期） */
} sim_motor_plant_cfg_t;

/**
 * @brief   注册一个合成 plant（ENC_CLOSED 电机 create 之后、poll 之前调用）
 * @return  0 = 成功；-1 = 参数非法 / 超容量 / enc 引脚对已注册
 */
int sim_motor_plant_register(const sim_motor_plant_cfg_t *cfg);

/**
 * @brief   方向桥输出快照（sim_pwm_bridge_duty 出参：DC/ENC 桥对两臂 + 带符号差）
 * @note    duty_fwd/duty_rev = 两臂存储回读（物理输出面）；signed_duty = 差
 *          （符号 = 方向位）。见 sim_pwm_bridge_duty 契约注。
 */
typedef struct {
    float duty_fwd;      /**< 正向臂占空（cfg.channel 通道存储值） */
    float duty_rev;      /**< 反向臂占空（cfg.channel+1 通道存储值） */
    float signed_duty;   /**< duty_fwd − duty_rev（+正转 / −反转，幅值=占空） */
} sim_bridge_duty_t;

/**
 * @brief   注销一个合成 plant（按 enc 引脚对定位；场景切换/反装配用）
 * @return  0 = 成功（记录释放，可再注册）；-1 = 引脚对未注册
 * @note    与 register 对称的生命周期面：同引脚对再注册在未注销时被拒——
 *          场景反复 start/stop（SCEN 命令重发即重入）须先注销再重建。
 */
int sim_motor_plant_unregister(int enc_a, int enc_b);

/**
 * @brief   读回 PWM 合成存储的通道占空（遥测/调试面：直读 RAM 存储）
 * @return  0 = 成功（*out_duty = 该通道当前存储值）；-1 = 句柄无效/记录缺失/通道越界
 * @note    检索同 plant：频率经 pwm_timer_freq 现取 → 按频键查记录（同频唯一
 *          由 timer_config 执法）。用途：0x06 场景帧 duty 字段 = 物理输出面回读
 *          ——同定时器他电机写串扰（通道隔离破坏）在此可观测，不采信电机自身状态。
 */
int sim_pwm_channel_duty(const pwm_timer_t *timer, uint8_t channel, float *out_duty);

/**
 * @brief   方向桥模拟输出回读（DC/ENC 桥对：channel 与 channel+1 两臂）
 * @return  0 = 成功（*out 三字段已写入）；-1 = 句柄无效/记录缺失/通道越界
 * @note    DC 语义（模拟 HAL 的方向桥输出面，与 pwm_hal.h「桥向由双通道表达」
 *          同构）：正转 = fwd 臂幅值 / rev 臂 0，反转反对称；
 *          signed_duty = duty_fwd − duty_rev——**符号 = 方向位**（+ 正转 /
 *          − 反转），幅值 = 输出占空。0x05/0x06 帧的 DC 槽 duty 字段即本值
 *          （带符号，域 −1..+1，区别于单通道槽的 0..1）。
 * @note    HAL 只如实回读，不做直通判定：两臂同时非零（H 桥直通态）由调用方
 *          断言（duty_fwd·duty_rev == 0）——合成存储是物理输出面，直通必然
 *          来自电机层写错，断言放消费侧可见性更好（host 自检即此姿势）。
 */
int sim_pwm_bridge_duty(const pwm_timer_t *timer, uint8_t ch_fwd, uint8_t ch_rev,
                        sim_bridge_duty_t *out);

/** @brief PWM 合成 HAL（启动期 pwm_hal_register(&sim_pwm_hal) 注册一次） */
extern const pwm_hal_t sim_pwm_hal;

/** @brief 合成编码器 HAL（ENC_CLOSED 实例 motor_hal_attach 挂载） */
extern const motor_hal_t sim_encoder_hal;

#endif /* MOTOR_HAL_SIM_H */
