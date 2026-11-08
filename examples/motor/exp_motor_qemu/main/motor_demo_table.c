/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_demo_table.c
 * @brief   电机演示槽表数据（契约见 motor_demo_table.h；追加槽/改演示轨迹只改本文件）
 *
 * @note    槽 0..3 = PWM_OPEN / SERVO / ENC_CLOSED / MOTOR_DC（四类齐），各挂
 *          库预设底版：motor.pwm.generic / motor.servo.sg90 / motor.enc.n20 /
 *          motor.dc.generic（见 the `xiao_motor` component src/motor_cfg_presets.c）。
 * @note    轨迹域（scurve u）：duty / deg / rpm / 带符号 duty；动力学取值按演示
 *          节奏定档（首腿 ≤0.8s 内走完 + 1s dwell，500 拍自检窗口可见完整 S 形）。
 * @note    ENC plant（合成回灌模型）：增益 160 rpm/duty（120 rpm 目标 → 稳态
 *          duty 0.75，界内不饱和）；τ=20ms 与速度环（kp=0.015, ki=1，预设
 *          motor.enc.n20 原样）合成约 70° 相位裕度的闭环——单调收敛无振荡。
 * @note    槽3 DC 的 timer_override（唯一用途）：sim PWM 合成存储以载波频率为
 *          键（同频定时器唯一，见 motor_hal_sim.h 约束注），20k 四通道已被
 *          槽0（ch0）+槽2 桥对（ch1/2）占满——DC 演示槽挂 50Hz 定时器的空闲
 *          桥对 ch1/2（duty 存储与载波频率无关，0x05 帧也不含定时器快照，
 *          演示输出不受影响；库建议值 20k 由 SCEN 5 专项实验独立运行时采用）。
 * @note    槽3 legs：0 → +0.5（首腿，正转 S 形爬升）→ dwell → +0.5 → −0.5
 *          （反向穿越，方向位翻转）→ dwell → 循环——即「+50% 持续 → −50%」
 *          的 demo 版形态；遥测 duty = 方向桥带符号回读（motor_demo.c 分支）。
 */

#include "motor_demo_table.h"

const motor_demo_slot_t motor_demo_slots[MOTOR_DEMO_SLOTS] = {
    /* 槽0：开环 PWM（u = duty）——S 梯度直接整形占空，示波器主断言对象 */
    {
        .motor_preset = "motor.pwm.generic",
        .channel = 0, .pin_a = 4, .pin_b = -1, .enc_a = -1, .enc_b = -1,
        .timer_override = NULL,
        .scurve = { .vmax = 1.2f, .amax = 6.0f, .jerk = 40.0f, .dt = 0.002f },
        .target = { 0.8f, 0.2f },
        .plant_rpm_per_duty = 0.0f, .plant_tau_s = 0.0f,
    },
    /* 槽1：SG90 舵机（u = 角度 deg）——脉宽换算后 duty 同样呈 S 形 */
    {
        .motor_preset = "motor.servo.sg90",
        .channel = 0, .pin_a = 5, .pin_b = -1, .enc_a = -1, .enc_b = -1,
        .timer_override = NULL,
        .scurve = { .vmax = 360.0f, .amax = 1600.0f, .jerk = 16000.0f, .dt = 0.002f },
        .target = { 135.0f, 45.0f },
        .plant_rpm_per_duty = 0.0f, .plant_tau_s = 0.0f,
    },
    /* 槽2：N20 编码器闭环（u = rpm）——S 梯度整形速度目标，PID+合成编码器闭环 */
    {
        .motor_preset = "motor.enc.n20",
        .channel = 1, .pin_a = 6, .pin_b = 7, .enc_a = 8, .enc_b = 9,
        .timer_override = NULL,
        .scurve = { .vmax = 300.0f, .amax = 1500.0f, .jerk = 15000.0f, .dt = 0.002f },
        .target = { 120.0f, 0.0f },
        .plant_rpm_per_duty = 160.0f, .plant_tau_s = 0.02f,
    },
    /* 槽3：直流 H 桥（u = 带符号 duty）——方向位随 leg 翻转，桥臂反对称输出 */
    {
        .motor_preset = "motor.dc.generic",
        .channel = 1, .pin_a = 11, .pin_b = 12, .enc_a = -1, .enc_b = -1,
        .timer_override = "pwm.servo.50",   /* 20k 满员 → 挂 50Hz 空闲桥对（见表头注） */
        .scurve = { .vmax = 1.2f, .amax = 6.0f, .jerk = 40.0f, .dt = 0.002f },
        .target = { 0.5f, -0.5f },          /* +50% 持续 → −50%（正反转交替） */
        .plant_rpm_per_duty = 0.0f, .plant_tau_s = 0.0f,
    },
};

const char *const motor_demo_timer_presets[] = {
    "pwm.motor.20k",   /* 槽0/槽2 共享（20kHz，通道0=槽0、1+2=槽2 桥对） */
    "pwm.servo.50",    /* 槽1/槽3 共享（50Hz：通道0=槽1 舵机、1+2=槽3 DC 桥对） */
};

const int motor_demo_timer_preset_count =
    (int)(sizeof(motor_demo_timer_presets) / sizeof(motor_demo_timer_presets[0]));
