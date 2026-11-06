/* SPDX-License-Identifier: MIT */
#ifndef MTR_MOTOR_HAL_H
#define MTR_MOTOR_HAL_H

#include <stdint.h>
#include <stddef.h>
#include "motor.h"

/**
 * @file    motor_hal.h
 * @brief   电机编码器 HAL 契约（只定义函数指针表，不实现——平台件后置）
 *
 * @note    分层对齐 imu 库总线抽象模式（依赖单向）：平台层（如 ESP-IDF
 *          PCNT/中断式正交解码）填一份 motor_hal_t 并 attach 给电机实例；
 *          驱动只认本结构体，不 include 任何平台头。PWM 输出面不在此处——
 *          走 pwm_timer 通道面（见 pwm_hal.h），本表只管编码器输入。
 * @note    返回值语义：0 = 成功；-1 = 失败。无部分成功。
 *
 * @note    原子性契约（编码器「复合读/复位」两步操作与 ISR 有竞态——
 *          本驱动不采用，改为下述单源差分设计）：
 *          - encoder_read 必须在**单次调用**内返回一致的累计计数快照
 *            （平台实现用临界区 / 硬件原子寄存器 / seqlock 式重读保证）；
 *          - 驱动**永不复位**计数器（不存在「读+清零」复合操作）——
 *            增量一律由相邻两次快照差分得出，读与清零不在同一窗内即无竞态；
 *          - int32 回绕由无符号差分吸收（模 2^32 语义，见 motor.c）；
 *          - 平台计数器须单调累计（上电以来），多电机各挂各的引脚对，
 *            经 pin_a/pin_b 入参区分，一张表可服务全部实例。
 */

/** @brief 编码器输入操作表（平台层提供；实例 attach 时快照指针） */
typedef struct {
    /**
     * @brief   原子读编码器累计计数（上电以来单调累计，单位 count）
     * @param   pin_a  编码器 A 相引脚（来自 motor_cfg_t.enc_a）
     * @param   pin_b  编码器 B 相引脚（来自 motor_cfg_t.enc_b）
     * @param   count  出参：累计计数快照（非 NULL）
     * @return  0 = 成功；-1 = 平台读失败（驱动置 MOTOR_FAULT_ENC_READ）
     * @note    单调用原子快照契约见文件头；不得在本次调用内复位计数器。
     */
    int  (*encoder_read)(int pin_a, int pin_b, int32_t *count, void *ctx);

    /** @brief 平台私有上下文（解码器句柄/单元号等），由平台实现填充 */
    void *ctx;
} motor_hal_t;

/**
 * @brief   挂载平台 HAL（create 后调用；const 表，调用方保活——同 imu 总线模式）
 * @param   m    电机实例（ENC_CLOSED 需要 encoder_read 才能闭环）
 * @param   hal  编码器操作表（非 NULL）
 * @return  0 = 成功；-1 = 参数 NULL，或 ENC_CLOSED 实例缺 encoder_read 成员
 * @note    未挂载即 poll ENC_CLOSED → MOTOR_FAULT_ENC_HAL + 返回 -1。
 *          非 ENC 类实例可挂可不挂（挂了也不使用）。
 */
int motor_hal_attach(motor_t *m, const motor_hal_t *hal);

#endif /* MTR_MOTOR_HAL_H */
