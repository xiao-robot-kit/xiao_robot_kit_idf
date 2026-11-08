/* SPDX-License-Identifier: MIT */
#ifndef MOTOR_SCEN_H
#define MOTOR_SCEN_H

/**
 * @file    motor_scen.h
 * @brief   电机实验场景运行时（表驱动装配 + 步进轨迹 + 0x06 场景帧打包）
 *
 * @note    装配序（与 motor_demo 同构；HAL 注册归调用方——固件 boot 时
 *          motor_demo_init 已消费一次，进程级不可重复）：
 *          场景表 timer_presets 建定时器 → 预设底版填 wiring → motor_create
 *          →（ENC：HAL 挂载 + plant 注册）→ 步进状态清零。
 * @note    一拍推进 = 步进发生器（hold_ticks 到期换步，循环）→ set_target
 *          → motor_poll → 遥测（duty 走 sim PWM 存储回读：单通道槽 = 通道
 *          占空，DC 桥对槽 = 带符号桥回读，符号 = 方向位）。
 *          固件主循环做「tick + 打 0x06 帧」；host 自检做「tick × N + 断言」
 *          ——同一状态机，同一行为。
 * @note    start 全量重置语义（销毁旧场景电机/plant/定时器再重建）：
 *          「SCEN n」命令重发即场景重启，无残留状态；失败原子回滚到「已停」。
 */

#include <stdint.h>
#include <stddef.h>
#include "motor_scen_table.h"
#include "motor_demo.h"   /* motor_demo_tele_t：遥测结构单源（0x06 记录同 16B 布局） */

/**
 * @brief   按场景号装配并激活（旧场景全量销毁；n 为 1..motor_scen_def_count）
 * @retval  0    装配成功（tick/pack 即可用）
 * @retval  -1   场景号不存在或装配失败（细节已 printf 到 stderr；失败原子回滚）
 */
int motor_scen_start(int id);

/**
 * @brief   停止并反装配（销毁电机/plant/定时器；active 回 -1）
 * @retval  0    成功
 */
int motor_scen_stop(void);

/** @brief 当前活动场景号；-1 = 无活动场景 */
int motor_scen_active(void);

/** @brief 按场景号取定义（id 1..count）；NULL = 无此场景（名字/槽数等只读面） */
const motor_scen_def_t *motor_scen_def_get(int id);

/**
 * @brief   推进一拍（步进发生器 + 各槽 set/poll/get_state + 存储回读）
 * @param   now_us  单调时基，us（调用方提供；首拍建时基基线）
 * @param   out     出参：活动场景 n_slots 个槽的遥测
 * @param   n_out   出参：写入的槽数（无活动场景 = 0）
 */
void motor_scen_tick(uint32_t now_us,
                     motor_demo_tele_t out[MOTOR_SCEN_MAX_SLOTS], int *n_out);

/**
 * @brief   遥测打包成 0x06 场景帧 payload（布局单源 = proto/frame.h 注释）
 * @param   n  tick 出参的槽数（须等于活动场景 n_slots）
 * @return  payload 字节数；0 = 参数/状态非法（无活动场景等）
 * @note    定时器快照的 freq 经 pwm_timer_freq 活实例现取；channels 取自
 *          库预设表（pwm_timer_presets 查名）。本函数 = 0x06 布局的 C 侧锚。
 */
size_t motor_scen_pack(const motor_demo_tele_t tele[], int n,
                       uint8_t *payload, size_t cap);

#endif /* MOTOR_SCEN_H */
