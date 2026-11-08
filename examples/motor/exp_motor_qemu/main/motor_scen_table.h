/* SPDX-License-Identifier: MIT */
#ifndef MOTOR_SCEN_TABLE_H
#define MOTOR_SCEN_TABLE_H

/**
 * @file    motor_scen_table.h
 * @brief   电机实验场景表契约（T-112 四专项 + DC 收满：pwm/servo/enc/nested/dc）
 *
 * @note    与 motor_demo_table 同构的「业务层注册表」：电机底版 = 库预设
 *          （motor_cfg_find 键），wiring（通道/引脚）与实验轨迹（步进目标
 *          序列 + 每步保持拍数）由本表填齐。参数零写死：改实验只改本文件。
 * @note    场景选择运行时切换（固件主循环「SCEN <n>」串口命令，UART0）——
 *          五场景共用一份固件/镜像；0x06 场景帧的 scen_id/n_tim/n_rec 单源
 *          即本表（帧布局见 proto/frame.h）。
 * @note    步进域（steps 值域）= motor_set_target 域，同 0x05 契约：
 *          PWM_OPEN = duty 0~1；SERVO = 角度 deg；ENC_CLOSED = rpm；
 *          MOTOR_DC = 带符号 duty −1~+1（符号 = H 桥方向位）。
 *          拍周期沿用 MOTOR_DEMO_POLL_US（motor_demo_table.h，时基单源）。
 * @note    遥测 duty 字段 = sim PWM 合成存储回读（物理输出面）——单通道槽走
 *          sim_pwm_channel_duty（nested 场景的通道隔离断言锚）；DC 桥对槽走
 *          sim_pwm_bridge_duty 带符号回读（方向位可观测）；帧内 phase = 步进下标。
 */

#include <stdint.h>

/** 场景最大电机数（表容量；nested 用满 3） */
#define MOTOR_SCEN_MAX_SLOTS   3

/** 场景最大定时器数（表容量；nested 用满 2 = 20kHz + 50Hz） */
#define MOTOR_SCEN_MAX_TIMERS  2

/** 单场景单电机定义（wiring + 实验轨迹；底版 = 库预设） */
typedef struct {
    /* —— 电机身份与 wiring（与 motor_demo_slot_t 同语义） —— */
    const char   *motor_preset;   /**< motor_cfg_find 键（库预设底版） */
    uint8_t       channel;        /**< 定时器上的通道（桥类自动占 channel+1） */
    int           pin_a;          /**< PWM 正向引脚 */
    int           pin_b;          /**< PWM 反向引脚（非桥类 -1） */
    int           enc_a;          /**< 编码器 A 相（非 ENC 类 -1） */
    int           enc_b;          /**< 编码器 B 相（非 ENC 类 -1） */
    /* —— ENC 合成 plant（仅 ENC_CLOSED 槽；cpr 单源自电机预设） —— */
    float         plant_rpm_per_duty; /**< plant 增益：duty=1 → rpm */
    float         plant_tau_s;        /**< 一阶滞后时间常数，s */
    /* —— 实验轨迹（步进发生器；域 = set_target 域） —— */
    const float  *steps;          /**< 步进目标序列（循环推进） */
    int           n_steps;        /**< 序列长度 */
    uint32_t      hold_ticks;     /**< 每步保持拍数（MOTOR_DEMO_POLL_US 计） */
} motor_scen_slot_t;

/** 单场景定义（定时器预设名表 + 电机槽表；一场景一表） */
typedef struct {
    int                       id;      /**< 场景号 1..5（0 保留 = demo） */
    const char               *name;    /**< 短名（pwm/servo/enc/nested） */
    const char               *title;   /**< 一句话描述（TEXT 横幅用） */
    const char *const        *timer_presets;  /**< 定时器预设名表（去重） */
    int                       n_timer_presets;
    const motor_scen_slot_t  *slots;   /**< 电机槽表 */
    int                       n_slots;
} motor_scen_def_t;

/** 场景定义表（定义在 motor_scen_table.c；追加场景只改那一个文件） */
extern const motor_scen_def_t motor_scen_defs[];
extern const int              motor_scen_def_count;

#endif /* MOTOR_SCEN_TABLE_H */
