/* SPDX-License-Identifier: MIT */
#ifndef MOTOR_DEMO_H
#define MOTOR_DEMO_H

/**
 * @file    motor_demo.h
 * @brief   电机演示运行时（表驱动装配 + 单拍推进；固件与 host 自检共用）
 *
 * @note    装配序（与 motor.h 组件 README 三行样板一致）：
 *          pwm_hal_register → 按预设名建定时器 → 预设底版填 wiring →
 *          motor_create →（ENC：plant 注册 + HAL 挂载）→ scurve_init。
 * @note    一拍推进 = dwell/retarget 状态机 + scurve_step 喂 set_target +
 *          motor_poll + motor_get_state。固件主循环只做「tick + 打帧」，
 *          host 自检只做「tick × N + 断言」——同一状态机，同一行为。
 */

#include <stdint.h>
#include "motor_demo_table.h"

/** 单槽遥测（一拍产出；单位契约见 motor_types.h） */
typedef struct {
    float    setpoint;   /**< S 梯度当前设定值（duty / deg / rpm / 带符号 duty 按 kind 域） */
    float    duty;       /**< 输出占空：单通道槽幅值 0~1；DC 槽 = 方向桥带符号
                              回读（−1~+1，符号 = 方向位；见 motor_demo.c 分支注） */
    float    actual;     /**< 实际值（开环=施加值；ENC=编码器测量 rpm；DC=带符号施加值） */
    int32_t  enc_count;  /**< 编码器累计计数（仅 ENC 槽有效） */
    uint32_t fault;      /**< 活状态故障位图（motor_types.h 位义） */
    int      phase;      /**< S 梯度相位（scurve_phase_t；遥测用） */
    int      kind;       /**< motor_kind_t（帧 payload 用） */
} motor_demo_tele_t;

/**
 * @brief   装配演示（注册 sim PWM HAL + 建定时器/电机/S 梯度，全表驱动）
 * @retval  0    全部槽装配成功
 * @retval  -1   装配失败（细节已 printf 到 stderr；任一槽失败即停）
 */
int motor_demo_init(void);

/**
 * @brief   反装配演示（销毁电机/合成 plant/定时器；给场景运行时让位，T-112）
 * @note    HAL 注册留驻（pwm_hal_register 进程级一次，不重复消费）；
 *          ENC 槽的合成 plant 一并注销（同引脚对再注册才不被拒）。
 *          motor_demo_init **不可重入**：其内部含 pwm_hal_register——deinit 后
 *          同进程重建必失败；回 demo 面的正确姿势 = 重启（QEMU）。
 *          本接口由固件主循环在「SCEN <n>」切换时调用（host 自检不消费）。
 * @retval  0    全部销毁成功
 * @retval  -1   有销毁失败（状态已尽量清理，不会半挂占用）
 */
int motor_demo_deinit(void);

/**
 * @brief   推进一拍（dwell/retarget 状态机 + 四槽 set/poll/get_state + DC 桥回读）
 * @param   now_us  单调时基，us（调用方提供；首拍建立时基基线）
 * @param   out     出参：MOTOR_DEMO_SLOTS 个槽的遥测（非 NULL）
 */
void motor_demo_tick(uint32_t now_us, motor_demo_tele_t out[MOTOR_DEMO_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（4×16B，布局见 proto/frame.h 注释）
 * @note    单一布局源：固件帧输出与 host 自检/上位机解码共用同一布局契约
 *          （本函数 = 布局的 C 侧锚；Python 侧 scope_motor.py 按同一注释解码）。
 */
void motor_demo_pack(const motor_demo_tele_t tele[MOTOR_DEMO_SLOTS],
                     uint8_t payload[MOTOR_DEMO_SLOTS * 16]);

#endif /* MOTOR_DEMO_H */
