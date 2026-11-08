/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_PWM_SCEN_H
#define EXP_MOTOR_PWM_SCEN_H

/**
 * @file    exp_motor_pwm_scen.h
 * @brief   PWM 占空阶梯扫描实验运行时（单电机实例，表驱动；固化自 exp_motor_qemu 场景 1）
 *
 * @note    单实例工程化（用户定案：电机五实例展开）：无场景选择器——boot 即本
 *          实验；综合演示（SCEN 命令 + 五场景切换）仍看 exp_motor_qemu。
 * @note    装配序（同 motor.h 组件 README 三行样板）：pwm_hal_register →
 *          按预设名建定时器 → 预设底版填 wiring → motor_create →（ENC 才有
 *          plant/HAL 挂载，本实验无）。一拍推进 = 步进发生器（hold_ticks 到期
 *          换步，循环）→ set_target → motor_poll → 遥测。固件主循环做
 *          「tick + 打 0x05 帧」；host gcc 自检（testing/exp_motor_pwm_sim_main.c）
 *          做「tick × N + 断言」——同一状态机，同一行为。
 * @note    遥测 duty = sim PWM 合成存储回读（物理输出面）：单通道槽 = 通道占空
 *          （0~1）；采信存储而非电机自身状态，回读失败留域外哨兵 -1（断言必抓）。
 * @note    参数零写死：步表/保持拍数/wiring 全在 exp_motor_pwm_scen.c 的表里；
 *          电机/定时器底版 = 库预设（motor_cfg_find 键）。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"

/** 本实验电机实例数（单实例工程） */
#define EXP_MOTOR_PWM_SLOTS  1

/** 控制周期，us（拍时基单源：hold_ticks 以此计 = 固件 vTaskDelayUntil 周期） */
#define EXP_MOTOR_PWM_POLL_US 2000

/**
 * @brief   单实例遥测（一拍产出；0x05 帧 16B 记录的 C 侧载体，字段序 =
 *          proto/frame.h PROTO_TYPE_MOTOR 注释——布局单源在彼）
 */
typedef struct {
    float    setpoint;   /**< 当前步进目标（PWM_OPEN 域 = duty 0~1） */
    float    duty;       /**< 输出占空 = sim PWM 存储回读（物理输出面；回读失败 -1 哨兵） */
    float    actual;     /**< 实际值（开环 = 施加占空） */
    int32_t  enc_count;  /**< 编码器累计（本实验无编码器，恒 0） */
    uint32_t fault;      /**< 活状态故障位图（位义见 mtr/motor_types.h） */
    int      phase;      /**< 步进下标（0x05 记录 phase 字段） */
    int      kind;       /**< motor_kind_t（帧 payload 用） */
} exp_motor_tele_t;

/**
 * @brief   装配实验（注册 sim PWM HAL + 建定时器/电机，全表驱动；init 一次）
 * @retval  0    装配成功（tick/pack 即可用）
 * @retval  -1   装配失败（细节已 printf 到 stderr；失败原子回滚不留半挂）
 */
int exp_motor_pwm_scen_init(void);

/**
 * @brief   推进一拍（步进发生器 + set/poll/get_state + 存储回读）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：EXP_MOTOR_PWM_SLOTS 个槽的遥测（非 NULL）
 */
void exp_motor_pwm_scen_tick(uint32_t now_us,
                             exp_motor_tele_t out[EXP_MOTOR_PWM_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（1×16B，布局单源 = proto/frame.h 注释）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约。
 */
void exp_motor_pwm_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_PWM_SLOTS],
                             uint8_t payload[EXP_MOTOR_PWM_SLOTS * 16]);

#endif /* EXP_MOTOR_PWM_SCEN_H */
