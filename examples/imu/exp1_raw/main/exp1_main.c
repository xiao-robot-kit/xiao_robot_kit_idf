/* SPDX-License-Identifier: MIT */
/**
 * @file    exp1_main.c
 * @brief   实验一：IMU 裸数据直出（init→probe→configure→loop read→串口帧直出）
 *
 * @note    链路：LSM6DS3(0x6A, I2C@GPIO5/6) → 驱动换算(g/dps) → 最小帧 → USB-Serial-JTAG
 * @note    参数零写死：全部来自预设表 imu.raw.104（xiao 组件库 驱动库包）。
 * @note    验收（上板）：上位机收 0x01 帧，波形滚动，az ≈ ±1g（静止）。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "port_i2c.h"         /* port/esp_idf */
#include "proto/frame.h"

static uint8_t s_seq = 0;

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n)
        usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(100));
}

static void send_text(const char *msg)
{
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)strlen(msg));
}

/* float → payload 小端 24B */
static void pack_sample(const imu_sample_t *s, uint8_t *p)
{
    const float v[6] = { s->ax, s->ay, s->az, s->gx, s->gy, s->gz };
    memcpy(p, v, 24);   /* Xtensa 小端，与协议字节序一致 */
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&uc);

    const imu_cfg_t *cfg = imu_cfg_find("imu.raw.104");   /* 表注册，零写死 */
    if (!cfg) { send_text("FATAL preset imu.raw.104 not found"); return; }

    if (esp_port_i2c_init(cfg->i2c_speed_hz) != 0) {
        send_text("FATAL i2c bus init failed"); return;
    }

    lsm6ds3_t imu;
    lsm6ds3_init(&imu, &esp_port_i2c_bus);

    for (;;) {
        int r = lsm6ds3_probe(&imu);
        if (r == 0) break;
        send_text(r == -2 ? "ERR who_am_i mismatch" : "ERR i2c no ack, retry 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    send_text("OK probe: LSM6DS3TR-C @0x6A");

    if (lsm6ds3_configure(&imu, cfg) != 0) {
        send_text("FATAL configure failed"); return;
    }

    char hi[64];
    snprintf(hi, sizeof(hi), "OK stream: %uHz +-%ug +-%udps period %ums",
             (unsigned)cfg->accel_odr_hz, (unsigned)cfg->accel_fs_g,
             (unsigned)cfg->gyro_fs_dps, (unsigned)cfg->sample_period_ms);
    send_text(hi);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(cfg->sample_period_ms));
        imu_sample_t s;
        if (lsm6ds3_read(&imu, &s) == 0) {
            uint8_t p[24];
            pack_sample(&s, p);
            send_frame(PROTO_TYPE_IMU_SAMPLE, p, sizeof(p));
        } else {
            static uint8_t err = 0;
            if (++err % 50 == 1)   /* 限流报错，不淹总线 */
                send_text("WARN read fail (counter running)");
        }
    }
}
