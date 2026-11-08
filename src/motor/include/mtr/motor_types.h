/* SPDX-License-Identifier: MIT */
#ifndef MTR_MOTOR_TYPES_H
#define MTR_MOTOR_TYPES_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file    motor_types.h
 * @brief   电机驱动库公共类型（单位契约——全库统一，换算只发生在驱动边界）
 *
 * @note    单位：速度 rpm；角度 deg（舵机）；占空比 0.0~1.0；时间 us。
 * @note    定时器/通道语义：一台电机占用其关联定时器上的一个通道；
 *          同一定时器的不同通道可被多台电机共享。
 */

/** @brief 电机种类（进配置，同一 create 接口覆盖） */
typedef enum {
    MOTOR_PWM_OPEN = 0,   /**< 基础 PWM 开环（无反馈，duty 直控） */
    MOTOR_DC,             /**< 直流电机（PWM+方向桥，开环或后配速度环） */
    MOTOR_SERVO,          /**< 舵机（脉宽对应角度，需低频定时器如 50Hz） */
    MOTOR_ENC_CLOSED,     /**< 带编码器电机（闭环：速度环/位置环） */
} motor_kind_t;

/** @brief 电机运行状态（motor_get_state 输出） */
typedef struct {
    float      target;     /**< 当前目标（rpm 或 deg，按 kind 语义） */
    float      actual;     /**< 当前实际（编码器测量或上次设定值） */
    float      duty;       /**< 当前输出占空比 0.0~1.0 */
    int32_t    enc_count;  /**< 编码器累计计数（ENC_CLOSED 有效） */
    uint32_t   fault;      /**< 故障位图（0=正常；位义见下——禁止值域内哨兵） */
} motor_state_t;

/*
 * 故障位图位义（motor_state_t.fault 的实现期定义——补齐上方注释的欠账）。
 *
 * 语义：非锁存「活状态」——每次 motor_poll 重估：被拒绝的那一拍置位并
 * 保持到下一次成功推进，可恢复故障（如瞬时编码器读失败）消除后自清；
 * 正常运行 steady 态恒为 0。真值走 float 字段，位图只表达「发生了什么」，
 * 不用任何值域内哨兵伪装状态（防雷：旧式实现 0xFF 撞 speed 值域）。
 */
#define MOTOR_FAULT_ENC_READ     (1u << 0)  /**< 编码器读失败（HAL 返回 -1） */
#define MOTOR_FAULT_ENC_HAL      (1u << 1)  /**< 编码器 HAL 未挂载或缺 encoder_read */
#define MOTOR_FAULT_PWM_WRITE    (1u << 2)  /**< PWM 通道写失败（含桥双通道） */
#define MOTOR_FAULT_TIME_REGRESS (1u << 3)  /**< 时基回退（该次 poll 被拒绝） */

#endif /* MTR_MOTOR_TYPES_H */
