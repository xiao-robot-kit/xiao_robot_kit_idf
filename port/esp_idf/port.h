/* SPDX-License-Identifier: MIT */
#ifndef PORT_ESP_IDF_PORT_H
#define PORT_ESP_IDF_PORT_H

/**
 * @file    port.h（port/esp_idf 统一入口）
 * @brief   port 层对外的唯一契约：板级装配隐函数 board_init
 *
 * @note    本头只声明 board_init，不传导任何平台/驱动端点头——
 *          各例程按本实例需要自行 include port_i2c.h / motor_hal_sim.h /
 *          pstore_flash.h 等端点，避免「不用 I2C 的例程也被迫依赖 IMU 头」。
 * @note    依赖单向铁律：本层是固件侧唯一允许出现 esp_xxx / i2c_master_xxx
 *          平台头的地方（具体平台头只落在各 .c 内，头文件不泄漏平台类型）。
 */

/**
 * @brief   板级装配隐函数（库只声明，不实现）
 *
 * 由每个例程的 main.c 按本实例需要实现：内部调用各硬件独立 init
 * （esp_port_i2c_init / motor / pstore ...）。引脚、速率、分区 label 等
 * board 属性全部收敛在各 main.c 的 board_init() 内，库层零板子感知。
 *
 * 例：
 *   void board_init(void) {
 *       esp_port_i2c_init(400000, 5, 6, 0x6A);   // 只装配本实例用到的硬件
 *   }
 */
void board_init(void);

#endif /* PORT_ESP_IDF_PORT_H */