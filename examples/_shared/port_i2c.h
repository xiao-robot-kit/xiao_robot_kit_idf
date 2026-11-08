/* SPDX-License-Identifier: MIT */
#ifndef ESP_PORT_I2C_H
#define ESP_PORT_I2C_H

#include "imu/imu_hal_i2c.h"

/**
 * @file    port_i2c.h（port/esp_idf）
 * @brief   ESP-IDF v6.x 新 I2C 主机 API 的 HAL 实现
 *
 * @note    本目录是固件侧唯一允许出现 esp_xxx / i2c_master_xxx 平台头的地方
 *          （依赖单向铁律的平台端点）。
 * @note    板级固定（XIAO Robot Board 实机验证引脚）：SDA=GPIO5(D4)、SCL=GPIO6(D5)，
 *          总线 I2C_NUM_0。
 * @note    v6.x 已移除 legacy 旧 I2C 驱动头，本实现用 i2c_master 新 API。
 */

/**
 * @brief   安装 I2C 主机总线并添加 IMU 器件
 * @param   speed_hz  总线速率（来自配置表 preset 的 i2c_speed_hz，禁止写死）
 * @return  0 = 成功；-1 = 失败
 * @note    单器件场景：exp 系列只挂 0x6A；read/write 的 addr 参数与本器件
 *          不符时返回错误（契约 addr 语义不被忽略）。
 */
int esp_port_i2c_init(uint32_t speed_hz);

/**
 * @brief   I2C 总线地址探测（P-08 总线扫描端点）
 * @param   addr         7-bit 器件地址（0x03–0x77）
 * @param   timeout_ms   单次探测传输超时（传 <=0 用默认）
 * @return  0 = 有器件响应；-1 = 无响应 / 总线未初始化 / 参数非法
 * @note    扫描专用端点，不改动单器件 read/write 的 addr=0x6A 契约；
 *          基于 i2c_master_probe（v6.x），不改变总线现有器件句柄。
 */
int esp_port_i2c_probe(uint8_t addr, int timeout_ms);

/** @brief 导出给驱动的总线操作表实例（esp_port_i2c_init 成功后可用） */
extern const imu_i2c_bus_t esp_port_i2c_bus;

#endif /* ESP_PORT_I2C_H */
