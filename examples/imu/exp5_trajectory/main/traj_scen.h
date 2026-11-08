/* SPDX-License-Identifier: MIT */
#ifndef EXP5_TRAJ_SCEN_H
#define EXP5_TRAJ_SCEN_H

#include <stddef.h>
#include <stdint.h>
#include "imu/imu_types.h"

/**
 * @file    traj_scen.h
 * @brief   实验五轨迹标记场景：三段一循环合成运动源（纯算法，零平台依赖）
 *
 * @note    职责边界：轨迹重建算法在 Host（backend/trajectory.py）——本模块只
 *          产出「轨迹实验数据流」的合成源（比力 + 角速度）与段边界标记行；
 *          固件侧把样本流送 mahony 出 QUAT 帧、把原始样本出 0x01 帧。
 * @note    运动模型（世界系 X 前 Z 上，姿态恒水平——教学直线段标记场景）：
 *          三段等长（各占循环 1/3 时长，拍数对齐由预设保证）：
 *            段 0 ACCEL ：线加速度 +a（m/s²）
 *            段 1 CRUISE：匀速（线加速度 0）
 *            段 2 DECEL ：线加速度 −a（段末速度归零）
 *          速度连续（段边界不跳变；循环首尾 v=0），位移每循环单调前进
 *          s = 2·a·T²/9（T = 循环周期 = 3·seg_len_s）——**真运动不回卷**，
 *          段标记语义是「周期性运动相位」，非往返摆。
 * @note    比力契约（与 imu_sample_t 单位一致，g/dps）：水平姿态下机体
 *          比力 = (a_world/g, 0, +1)——加速度计静止读 +1g 反作用，与 Host
 *          重建器去重力口径（减 [0,0,+g]）一致；角速度全零（无旋转）。
 * @note    段标记行（固件侧以 TEXT 0x7F 帧外发，轨迹页按行对齐段边界）：
 *          "SEG <cycle> <seg> <NAME>"（NAME = ACCEL/CRUISE/DECEL）。
 *          **不走 0x06 SCEN**：该帧型已由 motor 实验（exp_motor_qemu，
 *          vendor/xiao 组件库/契约/protocol-spec.md 0x06 行）占用且
 *          payload 为电机专用布局——复用即契约冲突，故走 TEXT 行约定。
 */

/** @brief 重力换算常数（m/s²，单位换算非可调参数；口径同 imu_types.h 注） */
#define TRAJ_SCEN_GRAVITY_MPS2 9.80665f

/** @brief 轨迹段标识（枚举值 = 标记行里的段号；三段一循环） */
typedef enum {
    TRAJ_SEG_ACCEL  = 0,   /**< 段 0：直线加速（线加速度 +a） */
    TRAJ_SEG_CRUISE = 1,   /**< 段 1：匀速（线加速度 0） */
    TRAJ_SEG_DECEL  = 2,   /**< 段 2：直线减速（段末速度归零） */
    TRAJ_SEG_COUNT  = 3,   /**< 段数（内部用） */
} traj_seg_t;

/**
 * @brief   三段场景配置（参数零写死：可调项全部经此注册）
 *
 * @note    约束：sample_rate_hz · seg_len_s 四舍五入 ≥ 1（段至少 1 拍）；
 *          预设取整数对齐（200Hz × 1.0s = 200 拍/段），使「三段各占
 *          循环 1/3 拍数」为精确断言（testing/exp5_sim_main.c）。
 */
typedef struct {
    float sample_rate_hz;   /**< 采样率，Hz（时间基：每读一拍 = 1/rate 秒） */
    float seg_len_s;        /**< 单段时长，秒（> 0；三段等长 → 各占循环 1/3） */
    float accel_mps2;       /**< 加/减段线加速度幅值，m/s²（> 0） */
} traj_scen_cfg_t;

/** @brief 命名预设（注册表项；表定义在 traj_scen.c，追加预设只改那一个文件） */
typedef struct {
    const char    *name;    /**< 预设名（点分命名空间如 "traj.exp5.seg3"） */
    traj_scen_cfg_t cfg;    /**< 参数 */
} traj_scen_cfg_preset_t;

/** @brief 预设表与项数（定义在 traj_scen.c） */
extern const traj_scen_cfg_preset_t traj_scen_cfg_presets[];
extern const int                    traj_scen_cfg_preset_count;

/**
 * @brief   按名查预设
 * @return  匹配项的 cfg 指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const traj_scen_cfg_t *traj_scen_cfg_find(const char *name);

/** @brief 合成场景实例（状态全部实例内，无全局可变态） */
typedef struct {
    traj_scen_cfg_t cfg;         /**< init 时拷贝的配置 */
    uint8_t         inited;      /**< 1 = 已 init（防未初始化即读） */
    uint32_t        ticks_per_seg;  /**< 每段拍数（rate·seg_len 四舍五入，≥1） */
    uint32_t        tick;       /**< 已产出采样拍号（uint32 饱和不回绕） */
} traj_scen_t;

/**
 * @brief   初始化实例（绑定配置并复位拍号）
 * @retval  0  成功
 * @retval -1  参数非法（NULL / 非有限 / rate·seg_len 四舍五入 < 1）
 */
int traj_scen_init(traj_scen_t *sc, const traj_scen_cfg_t *cfg);

/**
 * @brief   读一拍样本并推进拍号（段号/循环号按该拍计算，读后推进）
 * @param[out] seg_out    该拍所属段（可 NULL = 不需要）
 * @param[out] cycle_out  该拍所属循环号（0 起；可 NULL）
 * @retval  0  已写入并推进一拍
 * @retval -1  sc/out 为 NULL，或实例未 init
 */
int traj_scen_read(traj_scen_t *sc, imu_sample_t *out,
                   traj_seg_t *seg_out, uint32_t *cycle_out);

/**
 * @brief   段标记行（单源：固件 TEXT 帧与 host 自检共用同一格式）
 * @return  写入 buf 的字符数（不含 NUL）；0 = cap 不足
 * @note    格式 "SEG <cycle> <seg> <NAME>"，NAME ∈ {ACCEL,CRUISE,DECEL}
 */
int traj_scen_marker_line(char *buf, size_t cap, uint32_t cycle, unsigned seg);

#endif /* EXP5_TRAJ_SCEN_H */
