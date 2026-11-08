/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_cfg_presets.c
 * @brief   电机配置预设注册表（追加预设只改本文件，不动驱动）
 *
 * @note    底版约定（与 motor.h 预设注释一致）：timer=NULL、引脚=-1
 *          由业务层填（三行样板：create 定时器→填句柄/引脚/通道→create 电机）；
 *          create 对未填 wiring 的底版直接拒（fail-fast，不静默落到错引脚）。
 * @note    定时器预设名对齐 pwm_timer.c 交付的注册表：
 *          "pwm.motor.20k"（20kHz/4 通道）/ "pwm.servo.50"（50Hz/4 通道）。
 * @note    桥类（DC/ENC）通道语义：cfg.channel=正向通道（↔pin_a），
 *          channel+1=反向通道（↔pin_b）——成对占用（见 motor.c 输出面注）。
 */

#include <string.h>
#include "mtr/motor.h"

const motor_cfg_preset_t motor_cfg_presets[] = {
    /* 开环基础 PWM：target 即归一化占空（0~duty_max 直控，无反馈） */
    { "motor.pwm.generic", {
        .kind = MOTOR_PWM_OPEN, .timer = NULL, .channel = 0,
        .pin_a = -1, .pin_b = -1, .enc_a = -1, .enc_b = -1, .cpr = 0,
        .pulse_min_us = 0, .pulse_max_us = 0,
        .angle_min = 0.0f, .angle_max = 0.0f,
        .vel_pid = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
        .duty_max = 1.0f,
    }, "pwm.motor.20k" },

    /* 通用直流桥（双通道方向桥；target 为带符号归一化占窗） */
    { "motor.dc.generic", {
        .kind = MOTOR_DC, .timer = NULL, .channel = 0,
        .pin_a = -1, .pin_b = -1, .enc_a = -1, .enc_b = -1, .cpr = 0,
        .pulse_min_us = 0, .pulse_max_us = 0,
        .angle_min = 0.0f, .angle_max = 0.0f,
        .vel_pid = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
        .duty_max = 1.0f,
    }, "pwm.motor.20k" },

    /* SG90 舵机：500~2500us / 0~180°（厂商规格；50Hz 载波周期 20ms） */
    { "motor.servo.sg90", {
        .kind = MOTOR_SERVO, .timer = NULL, .channel = 0,
        .pin_a = -1, .pin_b = -1, .enc_a = -1, .enc_b = -1, .cpr = 0,
        .pulse_min_us = 500, .pulse_max_us = 2500,
        .angle_min = 0.0f, .angle_max = 180.0f,
        .vel_pid = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
        .duty_max = 1.0f,
    }, "pwm.servo.50" },

    /* N20 带编码器闭环：cpr=输出轴每转计数（磁编 12PPR×4 倍频×减速比 150
     * =7200；换减速比须改）；vel_pid 为起始整定（须按实际负载重整）；
     * target 单位 rpm，闭环输出走双通道方向桥同 DC */
    { "motor.enc.n20", {
        .kind = MOTOR_ENC_CLOSED, .timer = NULL, .channel = 0,
        .pin_a = -1, .pin_b = -1, .enc_a = -1, .enc_b = -1, .cpr = 7200,
        .pulse_min_us = 0, .pulse_max_us = 0,
        .angle_min = 0.0f, .angle_max = 0.0f,
        .vel_pid = { 0.015f, 1.0f, 0.0f, -1.0f, 1.0f },
        .duty_max = 1.0f,
    }, "pwm.motor.20k" },
};

const int motor_cfg_preset_count =
    (int)(sizeof(motor_cfg_presets) / sizeof(motor_cfg_presets[0]));

const motor_cfg_t *motor_cfg_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < motor_cfg_preset_count; i++)
        if (strcmp(motor_cfg_presets[i].name, name) == 0)
            return &motor_cfg_presets[i].cfg;
    return NULL;   /* 无此预设 */
}
