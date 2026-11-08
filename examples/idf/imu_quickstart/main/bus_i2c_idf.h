/* SPDX-License-Identifier: MIT */
#ifndef BUS_I2C_IDF_H
#define BUS_I2C_IDF_H

/*
 * bus_i2c_idf — example-local ESP-IDF I2C bus glue
 *
 * Implements the library's platform-neutral bus contract (imu/imu_hal_i2c.h)
 * on top of the ESP-IDF v6.x i2c_master driver. The library core never
 * includes any platform header; this file is the only place where ESP-IDF
 * types appear.
 *
 * Default wiring: XIAO ESP32-S3 (SDA = GPIO5, SCL = GPIO6, I2C0).
 */

#include "imu/imu_hal_i2c.h"

/**
 * @brief  Install the I2C master bus and attach the IMU device
 * @param  speed_hz  bus speed in Hz (take it from the preset table)
 * @return 0 = ok, -1 = failure
 */
int bus_i2c_init(uint32_t speed_hz);

/** @brief bus operation table passed to lsm6ds3_init() */
extern const imu_i2c_bus_t bus_i2c;

#endif /* BUS_I2C_IDF_H */
