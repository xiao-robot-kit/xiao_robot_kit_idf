/* SPDX-License-Identifier: MIT */
#ifndef EXP4_ATT_CFG_H
#define EXP4_ATT_CFG_H

/**
 * @file    att_cfg.h
 * @brief   Mahony 滤波参数预设表（模式仿 xiao 组件库 imu_cfg.h：参数零写死）
 *
 * @note    语义参数（人读的物理含义），不是算法内部系数——算法只消费
 *          mahony_init(kp, ki, dt) 契约面，换滤波器契约面不动。
 * @note    预设名命名空间 att.<实验>.<场景>；代码里禁止出现增益 magic number。
 */

/**
 * @brief Mahony 滤波语义配置
 *
 * @note dt 单位秒（0.005f ↔ 5ms），须与所配 imu 预设的 sample_period_ms/1000
 *       一致；不一致时固件启动会 WARN 并以 imu 节拍校正（不静默漂移）。
 */
typedef struct {
    float kp;   /**< 比例增益（收敛速度：大=快，过大放大加速度噪声；典型 2.0） */
    float ki;   /**< 积分增益（陀螺零偏补偿通道，典型 0.0~0.5；0=关闭） */
    float dt;   /**< 采样周期，秒（初始值；运行时以实测节拍为准） */
} att_cfg_t;

/** @brief 命名配置预设（注册表项） */
typedef struct {
    const char *name;   /**< 预设名（协议可寻址，点分命名空间如 "att.exp4.mahony"） */
    att_cfg_t   cfg;    /**< 参数 */
} att_cfg_preset_t;

/**
 * @brief   按名查预设
 * @return  匹配项指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const att_cfg_t *att_cfg_find(const char *name);

/** @brief 预设表（定义在 att_cfg_presets.c，追加预设只改那一个文件） */
extern const att_cfg_preset_t att_cfg_presets[];
extern const int              att_cfg_preset_count;

#endif /* EXP4_ATT_CFG_H */
