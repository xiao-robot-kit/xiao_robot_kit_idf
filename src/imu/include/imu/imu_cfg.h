/* SPDX-License-Identifier: MIT */
#ifndef IMU_CFG_H
#define IMU_CFG_H

#include <stdint.h>

/**
 * @file    imu_cfg.h
 * @brief   IMU 配置表机制（项目铁律：参数零写死，一切经表注册）
 *
 * @note    语义参数（人读的物理量：Hz / g / dps），不是芯片寄存器编码——
 *          寄存器编码与灵敏度换算藏在驱动查表里，换器件契约面不动。
 * @note    用法：实验固件从命名预设表取配置（imu_cfg_find），或整结构体
 *          经协议命令动态改写；代码里禁止出现量程/速率 magic number。
 */

/**
 * @brief IMU 语义配置（芯片无关）
 *
 * @note odr/fs 取常用档位值；驱动查表转寄存器编码，不支持的档位返回错误。
 */
typedef struct {
    uint16_t accel_odr_hz;      /**< 加速度计输出速率，Hz（如 104/208/416） */
    uint16_t accel_fs_g;        /**< 加速度计量程，±g（2/4/8/16） */
    uint16_t gyro_odr_hz;       /**< 陀螺仪输出速率，Hz */
    uint16_t gyro_fs_dps;       /**< 陀螺仪量程，±dps（125/250/500/1000/2000） */
    uint32_t i2c_speed_hz;      /**< I2C 总线速率，Hz（100000/400000） */
    uint16_t sample_period_ms;  /**< 采样周期，ms（推流节拍，由上层定时器用） */
} imu_cfg_t;

/** @brief 命名配置预设（注册表项） */
typedef struct {
    const char *name;           /**< 预设名（协议可寻址，点分命名空间如 "imu.raw.104"） */
    imu_cfg_t   cfg;            /**< 参数 */
} imu_cfg_preset_t;

/**
 * @brief   按名查预设
 * @return  匹配项指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const imu_cfg_t *imu_cfg_find(const char *name);

/** @brief 预设表（定义在 src/imu_cfg_presets.c，追加预设只改那一个文件） */
extern const imu_cfg_preset_t imu_cfg_presets[];
extern const int              imu_cfg_preset_count;

#endif /* IMU_CFG_H */
