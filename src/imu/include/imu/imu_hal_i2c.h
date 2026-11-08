/* SPDX-License-Identifier: MIT */
#ifndef IMU_HAL_I2C_H
#define IMU_HAL_I2C_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file    imu_hal_i2c.h
 * @brief   I2C 总线抽象（只定义函数指针表，不实现）
 *
 * @note    平台在移植层填一份实现并注册给驱动；驱动只认本结构体，
 *          不 include 任何平台头（依赖单向铁律）。
 *
 * @note    返回值语义（全平台统一，消掉旧式实现的 TODO 债）：
 *          0 = 成功；负值 = 失败。无部分成功。
 */

/** @brief I2C 总线操作表（平台注册） */
typedef struct {
    /**
     * @brief   写寄存器（寄存器地址 + 数据的典型 sensor 访问模式）
     * @return  0 = 成功；负值 = 失败
     */
    int  (*write)(uint8_t addr, uint8_t reg, const uint8_t *tx, size_t n, void *ctx);

    /**
     * @brief   读寄存器（写寄存器地址后读 n 字节）
     * @return  0 = 成功；负值 = 失败
     */
    int  (*read) (uint8_t addr, uint8_t reg, uint8_t *rx, size_t n, void *ctx);

    /** @brief 平台私有句柄（如总线端口号），由平台实现填充 */
    void *ctx;
} imu_i2c_bus_t;

#endif /* IMU_HAL_I2C_H */
