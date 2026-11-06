/* SPDX-License-Identifier: MIT */
/*
 * bus_i2c_idf — example-local ESP-IDF I2C bus glue
 * See bus_i2c_idf.h for the contract and default wiring.
 */

#include <stddef.h>
#include "driver/i2c_master.h"
#include "bus_i2c_idf.h"

#define BUS_SDA_GPIO   5    /* XIAO ESP32-S3 D4 */
#define BUS_SCL_GPIO   6    /* XIAO ESP32-S3 D5 */
#define IMU_I2C_ADDR   0x6A /* LSM6DS3TR-C, SA0 tied to GND */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

int bus_i2c_init(uint32_t speed_hz)
{
    if (speed_hz == 0) return -1;                     /* validate: no silent default */

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BUS_SDA_GPIO,
        .scl_io_num = BUS_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = true },
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) return -1;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = speed_hz,
        .scl_wait_us = 0,                              /* driver default timeout */
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) return -1;

    return 0;
}

/* write: START → addr(W) → reg → data → STOP */
static int bus_write(uint8_t addr, uint8_t reg, const uint8_t *tx, size_t n, void *ctx)
{
    (void)ctx;
    if (addr != IMU_I2C_ADDR) return -1;              /* single-device bus, still check */
    if (n > 8) return -1;                             /* driver max burst: 1 register value */
    uint8_t buf[1 + 8];
    buf[0] = reg;
    for (size_t i = 0; i < n; i++) buf[1 + i] = tx[i];
    return i2c_master_transmit(s_dev, buf, n + 1, -1) == ESP_OK ? 0 : -1;
}

/* read: START → addr(W) → reg → RESTART → addr(R) → n bytes → STOP */
static int bus_read(uint8_t addr, uint8_t reg, uint8_t *rx, size_t n, void *ctx)
{
    (void)ctx;
    if (addr != IMU_I2C_ADDR) return -1;
    return i2c_master_transmit_receive(s_dev, &reg, 1, rx, n, -1) == ESP_OK ? 0 : -1;
}

const imu_i2c_bus_t bus_i2c = {
    .write = bus_write,
    .read  = bus_read,
    .ctx   = 0,
};
