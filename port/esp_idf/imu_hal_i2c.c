/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_hal_i2c.c
 * @brief   ESP-IDF v6.x I2C 主机 HAL（唯一 esp 平台头出现处，见同名 .h 说明）
 */

#include <stddef.h>
#include "driver/i2c_master.h"
#include "port_i2c.h"       /* 本目录；类型契约来自 vendor 库包 imu/imu_hal_i2c.h */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t  s_dev_addr;          /* init 注册的器件地址（read/write 契约校验用） */
static uint32_t s_speed_hz;

int esp_port_i2c_init(uint32_t speed_hz, int sda, int scl, uint8_t addr)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = true },
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) return -1;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = speed_hz,
        .scl_wait_us = 0,           /* 默认超时 */
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) return -1;

    s_dev_addr = addr;
    s_speed_hz = speed_hz;
    return 0;
}

/* 写寄存器：START → addr(W) → reg → data → STOP */
static int port_write(uint8_t addr, uint8_t reg, const uint8_t *tx, size_t n, void *ctx)
{
    (void)ctx;
    if (addr != s_dev_addr) return -1;   /* 单器件总线，addr 契约仍校验 */
    uint8_t buf[1 + 8];
    if (n > 8) return -1;                   /* 驱动最大单笔：1 寄存器值 */
    buf[0] = reg;
    for (size_t i = 0; i < n; i++) buf[1 + i] = tx[i];
    return i2c_master_transmit(s_dev, buf, n + 1, -1) == ESP_OK ? 0 : -1;
}

/* 读寄存器：START → addr(W) → reg → RESTART → addr(R) → n 字节 → STOP */
static int port_read(uint8_t addr, uint8_t reg, uint8_t *rx, size_t n, void *ctx)
{
    (void)ctx;
    if (addr != s_dev_addr) return -1;
    return i2c_master_transmit_receive(s_dev, &reg, 1, rx, n, -1) == ESP_OK ? 0 : -1;
}

/* P-08 总线扫描端点：任意地址探测（不动单器件 read/write 语义） */
int esp_port_i2c_probe(uint8_t addr, int timeout_ms)
{
    if (s_bus == NULL) return -1;
    if (addr < 0x03 || addr > 0x77) return -1;
    int tmo = (timeout_ms > 0) ? timeout_ms : 20;
    return i2c_master_probe(s_bus, addr, tmo) == ESP_OK ? 0 : -1;
}

const imu_i2c_bus_t esp_port_i2c_bus = {
    .write = port_write,
    .read  = port_read,
    .ctx   = 0,
};
