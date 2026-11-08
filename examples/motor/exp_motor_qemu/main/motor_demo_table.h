/* SPDX-License-Identifier: MIT */
#ifndef MOTOR_DEMO_TABLE_H
#define MOTOR_DEMO_TABLE_H

/**
 * @file    motor_demo_table.h
 * @brief   电机演示槽表契约（T-102e：业务层注册表——参数零写死，全走本表）
 *
 * @note    与 imu/att 同构：底版来自库预设（motor_cfg_find / 定时器预设名），
 *          wiring（通道/引脚/定时器 override）与演示轨迹（S 梯度 cfg + 两腿
 *          交替目标）由本表填齐。固件（exp_motor_main.c）与 host 自检
 *          （testing/exp_motor_sim_main.c）消费同一张表——两侧行为逐拍一致。
 * @note    域约定（scurve u 域 = motor_set_target 域）：
 *          槽0 PWM_OPEN: u = duty（0~1）；槽1 SERVO: u = 角度 deg；
 *          槽2 ENC_CLOSED: u = rpm；槽3 MOTOR_DC: u = 带符号 duty（−1~+1，
 *          符号 = H 桥方向位；legs 交替 ±0.5 = 正反转切换演示）。
 *          每槽 scurve cfg.dt 必须等于 MOTOR_DEMO_POLL_US 对应秒值（时基单源）。
 */

#include <stdint.h>
#include "mtr/motor.h"
#include "mtr/scurve.h"

/** 演示槽数（1×PWM_OPEN + 1×SERVO + 1×ENC_CLOSED + 1×MOTOR_DC——四类齐） */
#define MOTOR_DEMO_SLOTS        4

/** 控制周期，us（= scurve cfg.dt × 1e6 = plant cfg.dt_s × 1e6，时基单源） */
#define MOTOR_DEMO_POLL_US      2000

/** 一腿走完后保持（dwell）拍数：匀速段/停驻段，供示波器断言「匀速段稳定」 */
#define MOTOR_DEMO_DWELL_TICKS  500

/** 演示槽定义（见文件头域约定） */
typedef struct {
    /* —— 电机身份与 wiring（底版 = 库预设，wiring 在此填齐） —— */
    const char   *motor_preset;   /**< motor_cfg_find 键（如 "motor.pwm.generic"） */
    uint8_t       channel;        /**< 定时器上的通道（桥类自动占 channel+1） */
    int           pin_a;          /**< PWM 正向引脚（非桥类唯一输出） */
    int           pin_b;          /**< PWM 反向引脚（非桥类填 -1） */
    int           enc_a;          /**< 编码器 A 相（非 ENC 类填 -1） */
    int           enc_b;          /**< 编码器 B 相（非 ENC 类填 -1） */
    /* —— 定时器选择（wiring 的一部分；仅演示层） —— */
    const char   *timer_override; /**< NULL = 底版建议定时器；非 NULL = 显式指定
                                       motor_demo_timer_presets[] 内的定时器名
                                       （唯一用途：槽3 DC——20k 四通道已被槽0/槽2
                                       占满，见表数据注释） */
    /* —— 演示轨迹（S 梯度，域 = set_target 域；dt 须 = MOTOR_DEMO_POLL_US） —— */
    scurve_cfg_t  scurve;         /**< 轨迹动力学（vmax/amax/jerk/dt） */
    float         target[2];      /**< 两腿交替目标（首腿 target[0]） */
    /* —— ENC 合成 plant（仅 ENC_CLOSED 槽使用；cpr 单源自电机预设） —— */
    float         plant_rpm_per_duty; /**< plant 增益：duty=1 → rpm */
    float         plant_tau_s;        /**< 一阶滞后时间常数，s */
} motor_demo_slot_t;

/** 演示槽表（定义在 motor_demo_table.c；追加槽只改那一个文件） */
extern const motor_demo_slot_t motor_demo_slots[MOTOR_DEMO_SLOTS];

/** 定时器预设名表（去重；槽表内 channel 按此表顺序对应定时器实例） */
extern const char *const motor_demo_timer_presets[];
extern const int        motor_demo_timer_preset_count;

#endif /* MOTOR_DEMO_TABLE_H */
