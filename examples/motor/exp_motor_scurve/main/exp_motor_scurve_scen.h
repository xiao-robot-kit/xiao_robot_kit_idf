/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_SCURVE_SCEN_H
#define EXP_MOTOR_SCURVE_SCEN_H

/**
 * @file    exp_motor_scurve_scen.h
 * @brief   S 梯度轨迹实验运行时（双电机槽：PWM_OPEN + ENC_CLOSED，表驱动）
 *
 * @note    单实例工程化（用户定案：电机实例展开，阶段维补齐 S 曲线）：无场景
 *          选择器——boot 即本实验；综合演示看 exp_motor_qemu。
 * @note    **域映射（本实验核心设计）**：scurve 生成「目标位置」的点对点轨迹，
 *          其速度输出前馈为电机设定（位置环由曲线闭式保证，速度面由电机跟踪）：
 *          - 槽0 PWM_OPEN：u = duty·s（占空·秒，虚拟里程）。v [u/s] 数值即占空
 *            （d/dt(duty·s) = duty），motor_set_target(v) 直控；实际位置 =
 *            实际 duty 的逐拍积分（开环无编码器的可见位置面）。
 *          - 槽1 ENC_CLOSED：u = rev（转）。v [rev/s] × 60 = rpm 喂速度环；
 *            实际位置 = (enc_count − 腿基线)/cpr——编码器可测的真位置。
 * @note    腿状态机：scurve_init(0→target) 一腿 → scurve_done 后 DWELL 拍停驻
 *          （速度 0，闭环收敛到目标）→ scurve_init 复位重走（rewind 语义，
 *          scurve.h「重调 init = 复位重走」）。每腿速度面恰好经历
 *          加速(相位1~3)/匀速(相位4)/减速(相位5~7) 三段——形状断言的分段依据。
 * @note    装配序（同 motor.h 组件 README 三行样板）：pwm_hal_register → 按预设
 *          名建定时器 → 预设底版填 wiring → motor_create →（ENC 槽挂编码器 HAL
 *          + 注册合成 plant）→ scurve_init。一拍推进 = scurve_step → get →
 *          set_target(v·scale) → motor_poll → 遥测。固件主循环做
 *          「tick + 打 0x05 帧（2×16B）」；host gcc 自检
 *          （testing/exp_motor_scurve_sim_main.c）做「tick × N + 形状断言」。
 * @note    遥测 duty = sim PWM 合成存储回读（物理输出面）：PWM 槽 = 单通道
 *          占空 [0,1]（回读失败 -1 哨兵）；ENC 槽 = 方向桥带符号回读
 *          [−1,1]（回读失败 -2 哨兵，符号 = 桥向位——同 motor_demo DC 槽分支）。
 * @note    参数零写死：scurve 动力学/目标位置/wiring/dwell 全在
 *          exp_motor_scurve_scen.c 的表里；电机/定时器底版 = 库预设
 *          （motor_cfg_find 键）。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"
#include "mtr/scurve.h"

/** 本实验电机实例数（1×PWM_OPEN + 1×ENC_CLOSED——用户矩阵「PWM/ENC 各跑」） */
#define EXP_MOTOR_SCURVE_SLOTS  2

/** 控制周期，us（= scurve cfg.dt × 1e6 = plant cfg.dt_s × 1e6，时基单源） */
#define EXP_MOTOR_SCURVE_POLL_US 2000

/** 一腿走完后的停驻拍数（速度 0 保持：闭环收敛 + 断言采样窗） */
#define EXP_MOTOR_SCURVE_DWELL_TICKS 500

/**
 * @brief   双槽遥测（一拍产出；0x05 帧 2×16B 记录的 C 侧载体，帧内字段序 =
 *          proto/frame.h PROTO_TYPE_MOTOR 注释——布局单源在彼）
 * @note    pos_target/pos_actual 不进帧（16B 布局冻结）——host 自检经同一
 *          状态机直接断言（固件与自检共用本结构与 tick 源码）。
 */
typedef struct {
    float    setpoint;    /**< 曲线速度目标（电机域：PWM=duty 0~1；ENC=rpm） */
    float    duty;        /**< 输出占空回读（PWM [0,1] 哨兵-1；ENC 桥带符号 [−1,1] 哨兵-2） */
    float    actual;      /**< 实际值（PWM=施加占空；ENC=编码器测量 rpm） */
    float    pos_target;  /**< 曲线位置目标 [u]（duty·s / rev——到目标收敛锚） */
    float    pos_actual;  /**< 实际位置 [u]（PWM=duty 积分；ENC=编码器位移） */
    int32_t  enc_count;   /**< 编码器累计（PWM 槽恒 0） */
    uint32_t fault;       /**< 活状态故障位图（位义见 mtr/motor_types.h） */
    int      phase;       /**< scurve 相位（0..6 运动，7=DONE；0x05 记录 phase 字段） */
    int      kind;        /**< motor_kind_t（帧 payload 用） */
} exp_motor_tele_t;

/**
 * @brief   装配实验（注册 sim PWM HAL + 建定时器/电机/双 scurve，全表驱动）
 * @retval  0    装配成功（tick/pack 即可用）
 * @retval  -1   装配失败（细节已 printf 到 stderr；失败原子回滚不留半挂）
 */
int exp_motor_scurve_scen_init(void);

/**
 * @brief   推进一拍（腿状态机 + scurve_step/get + set/poll + 存储回读）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：EXP_MOTOR_SCURVE_SLOTS 个槽的遥测（非 NULL）
 */
void exp_motor_scurve_scen_tick(uint32_t now_us,
                                exp_motor_tele_t out[EXP_MOTOR_SCURVE_SLOTS]);

/**
 * @brief   遥测打包成 0x05 帧 payload（2×16B = 32B ≤ PAYLOAD_MAX，布局单源 =
 *          proto/frame.h 注释——解码器按 payload 长度自适配）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约。
 */
void exp_motor_scurve_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_SCURVE_SLOTS],
                                uint8_t payload[EXP_MOTOR_SCURVE_SLOTS * 16]);

#endif /* EXP_MOTOR_SCURVE_SCEN_H */
