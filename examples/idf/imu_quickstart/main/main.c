/* SPDX-License-Identifier: MIT */
/**
 * @file    main.c
 * @brief   IMU quickstart: probe → configure from a named preset → stream samples.
 *
 * Wiring (XIAO ESP32-S3): SDA = GPIO5, SCL = GPIO6, IMU @0x6A.
 * Output: console UART, one line per sample (units: g / dps).
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "bus_i2c_idf.h"

void app_main(void)
{
    const imu_cfg_t *cfg = imu_cfg_find("imu.raw.104");  /* preset table, zero hardcoded params */
    if (!cfg || bus_i2c_init(cfg->i2c_speed_hz) != 0) { puts("FATAL bus init"); return; }

    lsm6ds3_t imu;
    lsm6ds3_init(&imu, &bus_i2c);
    while (lsm6ds3_probe(&imu) != 0) {
        puts("IMU not found @0x6A, retrying in 1s ...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (lsm6ds3_configure(&imu, cfg) != 0) { puts("FATAL configure"); return; }
    printf("streaming: %u Hz, +-%u g, +-%u dps\n",
           (unsigned)cfg->accel_odr_hz, (unsigned)cfg->accel_fs_g, (unsigned)cfg->gyro_fs_dps);

    for (;;) {
        imu_sample_t s;                       /* ax/ay/az in g, gx/gy/gz in dps */
        if (lsm6ds3_read(&imu, &s) == 0)
            printf("ax=%+.3f ay=%+.3f az=%+.3f g | gx=%+.1f gy=%+.1f gz=%+.1f dps\n",
                   s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
        vTaskDelay(pdMS_TO_TICKS(cfg->sample_period_ms));
    }
}
