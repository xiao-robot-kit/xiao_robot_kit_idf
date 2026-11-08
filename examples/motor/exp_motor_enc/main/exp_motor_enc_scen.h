/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_ENC_SCEN_H
#define EXP_MOTOR_ENC_SCEN_H

/**
 * @file    exp_motor_enc_scen.h
 * @brief   编码器闭环阶跃响应+调参记录实验运行时（单电机实例，表驱动；
 *          固化自 exp_motor_qemu 场景 3）
 *
 * @note    单实例工程化（用户定案：电机五实例展开）：无场景选择器——boot 即本
 *          实验；综合演示（SCEN 命令 + 五场景切换）仍看 exp_motor_qemu。
 * @note    装配序（ENC 专属步）：pwm_hal_register → 按预设名建定时器 →
 *          预设底版填 wiring → motor_create → 编码器 HAL 挂载
 *          （motor_hal_attach）→ 合成 plant 注册（sim_motor_plant_register：
 *          控制环 PID→桥双通道 与 反馈环 编码器差分 在纯 RAM 闭环）。
 *          一拍推进 = 步进发生器 → set_target（域 = rpm）→ motor_poll →
 *          遥测。固件主循环做「tick + 打 0x05 帧」；host gcc 自检
 *          （testing/exp_motor_enc_sim_main.c）做「tick × N + 断言」。
 * @note    遥测 actual = 编码器测量 rpm（plant 一阶滞后模型）；duty = 速度环
 *          PID 输出占空的 sim 存储回读。参数零写死：步表/保持拍数/wiring/
 *          plant 增益全在 exp_motor_enc_scen.c 的表里；电机/PID/定时器底版 =
 *          库预设（motor.enc.n20：cpr 单源在预设）。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"

/** 本实验电机实例数（单实例工程） */
#define EXP_MOTOR_ENC_SLOTS  1

/** 控制周期，us（拍时基单源：hold_ticks 与 plant dt_s 以此计 = 固件周期） */
#define EXP_MOTOR_ENC_POLL_US 2000

/**
 * @brief   单实例遥测（一拍产出；0x05 帧 16B 记录的 C 侧载体，字段序 =
 *          proto/frame.h PROTO_TYPE_MOTOR 注释——布局单源在彼）
 */
typedef struct {
    float    setpoint;   /**< 当前步进目标（ENC_CLOSED 域 = rpm） */
    float    duty;       /**< 速度环输出占空 = sim 存储回读（回读失败 -1 哨兵） */
    float    actual;     /**< 实际值 = 编码器测量 rpm（合成 plant） */
    int32_t  enc_count;  /**< 编码器累计计数（plant 累积） */
    uint32_t fault;      /**< 活状态故障位图（位义见 mtr/motor_types.h） */
    int      phase;      /**< 步进下标（0x05 记录 phase 字段） */
    int      kind;       /**< motor_kind_t（帧 payload 用） */
} exp_motor_tele_t;

/**
 * @brief   装配实验（注册 sim HAL + 建定时器/电机/plant，全表驱动；init 一次）
 * @retval  0    装配成功（tick/pack 即可用）
 * @retval  -1   装配失败（细节已 printf 到 stderr；失败原子回滚不留半挂）
 */
int exp_motor_enc_scen_init(void);

/**
 * @brief   推进一拍（步进发生器 + set/poll/get_state + 存储回读）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：EXP_MOTOR_ENC_SLOTS 个槽的遥测（非 NULL）
 */
void exp_motor_enc_scen_tick(uint32_t now_us,
                             exp_motor_tele_t out[EXP_MOTOR_ENC_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（1×16B，布局单源 = proto/frame.h 注释）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约。
 */
void exp_motor_enc_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_ENC_SLOTS],
                             uint8_t payload[EXP_MOTOR_ENC_SLOTS * 16]);

#endif /* EXP_MOTOR_ENC_SCEN_H */
