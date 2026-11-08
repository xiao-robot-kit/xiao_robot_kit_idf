/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_NESTED_SCEN_H
#define EXP_MOTOR_NESTED_SCEN_H

/**
 * @file    exp_motor_nested_scen.h
 * @brief   嵌套定时器通道隔离实验运行时（三电机实例：双电机共定时器 +
 *          独立定时器一电机；固化自 exp_motor_qemu 场景 4）
 *
 * @note    单实例工程化（用户定案：电机五实例展开）：无场景选择器——boot 即本
 *          实验；综合演示（SCEN 命令 + 五场景切换）仍看 exp_motor_qemu。
 * @note    装配序（嵌套语义锚 = pwm_timer.h）：pwm_hal_register → 按预设名表
 *          建**两个**定时器（20kHz + 50Hz）→ 每槽预设底版填 wiring →
 *          motor_create（同定时器句柄 + 不同 channel = 共定时器分通道；
 *          不同句柄 = 独立定时器）。一拍推进 = 各槽步进发生器 → set_target
 *          → motor_poll → 遥测。固件主循环做「tick + 打 0x05 帧（3×16B）」；
 *          host gcc 自检（testing/exp_motor_nested_sim_main.c）做
 *          「tick × N + 断言」。
 * @note    隔离断言的可观测面：遥测 duty = sim PWM 合成存储回读（物理输出
 *          面）——A（动方，阶梯步进）的写若串扰到 B 的通道（静方，恒 0.5），
 *          B 的回读立即可见；C（独立 50Hz 定时器舵机）= 频率并存的可观测面。
 * @note    参数零写死：步表/保持拍数/wiring 全在 exp_motor_nested_scen.c 的表
 *          里；电机/定时器底版 = 库预设（motor_cfg_find 键）。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"

/** 本实验电机实例数（A/B 共 20kHz 定时器 + C 独立 50Hz 定时器） */
#define EXP_MOTOR_NESTED_SLOTS  3

/** 控制周期，us（拍时基单源：hold_ticks 以此计 = 固件 vTaskDelayUntil 周期） */
#define EXP_MOTOR_NESTED_POLL_US 2000

/**
 * @brief   多实例遥测（一拍产出；0x05 帧 3×16B 记录的 C 侧载体，字段序 =
 *          proto/frame.h PROTO_TYPE_MOTOR 注释——布局单源在彼）
 */
typedef struct {
    float    setpoint;   /**< 当前步进目标（PWM 槽 = duty 0~1；SERVO 槽 = deg） */
    float    duty;       /**< 输出占空 = sim PWM 存储回读（回读失败 -1 哨兵） */
    float    actual;     /**< 实际值（开环 PWM = 施加占空；SERVO = 施加角度 deg） */
    int32_t  enc_count;  /**< 编码器累计（本实验无编码器，恒 0） */
    uint32_t fault;      /**< 活状态故障位图（位义见 mtr/motor_types.h） */
    int      phase;      /**< 步进下标（0x05 记录 phase 字段） */
    int      kind;       /**< motor_kind_t（帧 payload 用） */
} exp_motor_tele_t;

/**
 * @brief   装配实验（注册 sim PWM HAL + 建 2 定时器/3 电机，全表驱动；init 一次）
 * @retval  0    装配成功（tick/pack 即可用）
 * @retval  -1   装配失败（细节已 printf 到 stderr；失败原子回滚不留半挂）
 */
int exp_motor_nested_scen_init(void);

/**
 * @brief   推进一拍（各槽步进发生器 + set/poll/get_state + 存储回读）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：EXP_MOTOR_NESTED_SLOTS 个槽的遥测（非 NULL）
 */
void exp_motor_nested_scen_tick(uint32_t now_us,
                                exp_motor_tele_t out[EXP_MOTOR_NESTED_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（3×16B = 48B ≤ PAYLOAD_MAX，布局单源 =
 *          proto/frame.h 注释——解码器按 payload 长度自适配）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约。
 */
void exp_motor_nested_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_NESTED_SLOTS],
                                uint8_t payload[EXP_MOTOR_NESTED_SLOTS * 16]);

#endif /* EXP_MOTOR_NESTED_SCEN_H */
