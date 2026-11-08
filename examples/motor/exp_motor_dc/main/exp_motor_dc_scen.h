/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_DC_SCEN_H
#define EXP_MOTOR_DC_SCEN_H

/**
 * @file    exp_motor_dc_scen.h
 * @brief   DC 电机 H 桥正反转切换实验运行时（单电机实例，表驱动；固化自 exp_motor_qemu 场景 5）
 *
 * @note    单实例工程化（用户定案：电机五实例展开）：无场景选择器——boot 即本
 *          实验；综合演示（SCEN 命令 + 五场景切换）仍看 exp_motor_qemu。
 * @note    装配序：pwm_hal_register → 按预设名建定时器 → 预设底版填 wiring →
 *          motor_create（DC 桥对自动占 channel/channel+1 两通道）。一拍推进 =
 *          步进发生器 → set_target（带符号 duty −1~+1，符号 = 方向位）→
 *          motor_poll → 遥测。固件主循环做「tick + 打 0x05 帧」；host gcc 自检
 *          （testing/exp_motor_dc_sim_main.c）做「tick × N + 断言」——同一状态机。
 * @note    遥测 duty = sim 方向桥带符号回读（物理输出面，符号 = 方向位，域
 *          −1~+1）；桥臂快照另有访问器（exp_motor_dc_scen_bridge，host 自检
 *          断言两臂互补/直通零容忍用）。回读失败留域外哨兵 -2（断言必抓）。
 * @note    参数零写死：步表/保持拍数/wiring 全在 exp_motor_dc_scen.c 的表里；
 *          电机/定时器底版 = 库预设（motor_cfg_find 键）。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"
#include "motor_hal_sim.h"

/** 本实验电机实例数（单实例工程） */
#define EXP_MOTOR_DC_SLOTS  1

/** 控制周期，us（拍时基单源：hold_ticks 以此计 = 固件 vTaskDelayUntil 周期） */
#define EXP_MOTOR_DC_POLL_US 2000

/**
 * @brief   单实例遥测（一拍产出；0x05 帧 16B 记录的 C 侧载体，字段序 =
 *          proto/frame.h PROTO_TYPE_MOTOR 注释——布局单源在彼）
 */
typedef struct {
    float    setpoint;   /**< 当前步进目标（MOTOR_DC 域 = 带符号 duty −1~+1） */
    float    duty;       /**< 方向桥带符号回读（符号 = 方向位；回读失败 -2 哨兵） */
    float    actual;     /**< 实际值（开环 = 带符号施加占空） */
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
int exp_motor_dc_scen_init(void);

/**
 * @brief   推进一拍（步进发生器 + set/poll/get_state + 桥带符号回读）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：EXP_MOTOR_DC_SLOTS 个槽的遥测（非 NULL）
 */
void exp_motor_dc_scen_tick(uint32_t now_us,
                            exp_motor_tele_t out[EXP_MOTOR_DC_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（1×16B，布局单源 = proto/frame.h 注释）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约。
 */
void exp_motor_dc_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_DC_SLOTS],
                            uint8_t payload[EXP_MOTOR_DC_SLOTS * 16]);

/**
 * @brief   方向桥两臂快照（ch/ch+1 存储回读；host 自检直通零容忍断言用）
 * @retval  0    成功（*out 三字段已写入：duty_fwd/duty_rev/signed_duty）
 * @retval  -1   未装配/回读失败
 * @note    直通判据由调用方断言（duty_fwd·duty_rev == 0）；HAL 只如实回读。
 */
int exp_motor_dc_scen_bridge(sim_bridge_duty_t *out);

#endif /* EXP_MOTOR_DC_SCEN_H */
