/* SPDX-License-Identifier: MIT */
#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "imu_types.h"
#include "imu_cfg.h"
#include "imu_hal_i2c.h"

/**
 * @file    lsm6ds3.h
 * @brief   LSM6DS3TR-C 六轴 IMU 驱动接口（当前唯一器件实现）
 *
 * @note    生命周期（显式函数式，无状态机框架——等第二个器件出现再提炼框架）：
 *          init（绑总线）→ probe（WHO_AM_I 验证）→ configure（写配置表）
 *          → read / read_temp 循环。
 * @note    器件事实（数据手册，换算表在 lsm6ds3.c）：
 *          - I2C 7 位地址 0x6A（SA0 接地）；WHO_AM_I(0x0F) = 0x6A
 *          - 数据小端 int16；加速度 OUTX_L_XL=0x28 起 6 字节；
 *            陀螺 OUTX_L_G=0x22 起 6 字节；温度 OUT_TEMP_L=0x20 起 2 字节
 *          - BDU + 地址自增：CTRL3_C(0x12) = 0x44
 */

/** @brief 器件 I2C 7 位地址（SA0 接地） */
#define LSM6DS3_I2C_ADDR       0x6A

/** @brief 驱动生命周期状态 */
typedef enum {
    LSM6DS3_STATE_UNINIT = 0,   /**< 未初始化（init 前） */
    LSM6DS3_STATE_PROBED,       /**< 已验证 WHO_AM_I，未配置 */
    LSM6DS3_STATE_READY,        /**< 已配置，可采样 */
    LSM6DS3_STATE_FAULT,        /**< 总线连续失败 */
} lsm6ds3_state_t;

/** @brief 驱动实例（调用方分配） */
typedef struct {
    const imu_i2c_bus_t *bus;       /**< I2C 总线操作表（init 绑定） */
    imu_cfg_t            cfg;       /**< 当前生效配置（configure 成功后同步） */
    float                accel_sens;/**< 加速度灵敏度 g/LSB（查表） */
    float                gyro_sens; /**< 角速度灵敏度 dps/LSB（查表） */
    lsm6ds3_state_t      state;     /**< 生命周期状态 */
    uint8_t              fault_cnt; /**< 连续读失败计数（清零阈值后转 FAULT） */
} lsm6ds3_t;

/** @brief 六轴原始计数（未换算，供示波/诊断） */
typedef struct {
    int16_t ax, ay, az;   /**< 加速度原始计数（LSB） */
    int16_t gx, gy, gz;   /**< 角速度原始计数（LSB） */
} lsm6ds3_raw_t;

/**
 * @brief   初始化：绑定总线，状态置 UNINIT（不写硬件）
 * @note    cfg 清零、灵敏度置 0；须先 probe 再 configure。
 */
void lsm6ds3_init(lsm6ds3_t *imu, const imu_i2c_bus_t *bus);

/**
 * @brief   链路探测：读 WHO_AM_I 并比对期望值 0x6A
 * @return  0 = 器件应答正确；-1 = 总线失败；-2 = 器件 ID 不符
 * @note    总线连通性验证的固件入口即此函数。
 */
int  lsm6ds3_probe(lsm6ds3_t *imu);

/**
 * @brief   按配置表写寄存器并同步灵敏度查表结果
 * @return  0 = 成功；-1 = 总线失败；-2 = 配置含不支持的档位
 * @note    写 CTRL1_XL / CTRL2_G / CTRL3_C 三寄存器；成功后 imu->cfg == *cfg。
 */
int  lsm6ds3_configure(lsm6ds3_t *imu, const imu_cfg_t *cfg);

/**
 * @brief   读一次采样（原始计数 × 灵敏度 → 物理量）
 * @return  0 = 成功；-1 = 总线失败
 * @note    未 configure 时灵敏度为 0，输出全 0（不报错——probe 后允许裸读）。
 */
int  lsm6ds3_read(lsm6ds3_t *imu, imu_sample_t *out);

/** @brief 读六轴原始计数（LSB，不换算） @return 0=成功；-1=总线失败 */
int  lsm6ds3_read_raw(lsm6ds3_t *imu, lsm6ds3_raw_t *out);

/**
 * @brief   读片上温度
 * @return  0 = 成功；-1 = 总线失败
 * @note    换算 T = 25 + raw/256（25°C 时输出 0，256 LSB/°C）。
 */
int  lsm6ds3_read_temp(lsm6ds3_t *imu, float *celsius);

/**
 * @brief   回读两个控制寄存器（配置↔寄存器一致性校验用）
 * @return  0 = 成功；-1 = 总线失败
 */
int  lsm6ds3_get_ctrl(lsm6ds3_t *imu, uint8_t *ctrl1_xl, uint8_t *ctrl2_g);

#endif /* LSM6DS3_H */
