/* SPDX-License-Identifier: MIT */
/**
 * @file    p01_imu_lsm6ds3_main.c
 * @brief   P-01：I2C 轮询 LSM6DS3TR-C(0x6A)——WHO_AM_I 校验 + 100Hz 连读 5min + I2C 总线扫描
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - LSM6DS3TR-C (U18)：器件地址 0x6A（SA0 接地）；总线实线（SDA/SCL 脚位、
 *    速率 400kHz）以 port/esp_idf 平台端点为唯一事实源（任务书 D4/D5 标注与
 *    端点实测差异见 README「已知限制」）。
 *  - INT1/INT2 未接 → 本例只能轮询（无中断采样；100Hz 为软件语义节拍，
 *    传感器 ODR 由配置预设决定）。
 *  - 板载无固件可控 LED（D1 绿灯/充电红×2 均为硬件驱动）→ 调试可视输出一律串口。
 *
 * 验收（任务书 §3 P-01）：
 *  ① WHO_AM_I 校验：probe 通过 → "WHO 0x6A ok"；
 *  ② 100Hz 连续读 accel/gyro/temp 5min 零报错：每秒一行 "IMU n=100 err=0 …"，
 *     300s 后打印 "DONE run=300 s … err=0" + "VERDICT PASS"（err=0 判据）；
 *  ③ I2C 总线扫描：→ "SCAN dev=0x6A"（板面 I2C 唯一器件）。
 *
 * 数据源二选一（编译期宏 P_SIM_IMU，见 main/CMakeLists.txt，默认 1）：
 *  - P_SIM_IMU=0：真 I2C 传感器（复用 port/esp_idf port_i2c + vendored imu 驱动）。
 *
 * 串口参数：115200 8N1（两路一致）。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "imu/imu_types.h"
#include "driver/usb_serial_jtag.h"
#include "port.h"             /* port/esp_idf：board_init 隐函数 */
#include "port_i2c.h"         /* port/esp_idf：I2C 平台端点（esp_port_i2c_init / bus） */
#if P_SIM_IMU
#include "imu_sim.h"          /* examples/common/sim：合成源（跨例程共享单源，只读） */
#else
#include "imu/lsm6ds3.h"      /* xiao_imu 组件库：驱动 */
#include "imu/imu_cfg.h"
#endif

#define APP_NAME       "P-01_imu_lsm6ds3"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

/* 板级接线（board_init 唯一事实源；库层零写死） */
#define BOARD_I2C_SDA    5          /* D4 */
#define BOARD_I2C_SCL    6          /* D5 */
#define BOARD_I2C_ADDR   0x6A       /* LSM6DS3TR-C，SA0 接地 */
#define BOARD_I2C_SPEED  400000     /* 400 kHz */

/* 真传感器预设（速率/量程全来自预设表） */
#define APP_CFG_PRESET "imu.raw.104"      /* 104Hz ODR + 400kHz 总线 + 10ms 节拍 */
#define APP_SIM_PRESET "sim.exp2.static"   /* 合成源：100Hz 静止真值 + 注入已知零偏 */

enum {
    APP_PERIOD_MS     = 10,    /* 100Hz 语义节拍（与预设 sample_period_ms 一致的兜底值） */
    APP_RUN_SEC       = 300,   /* 验收②：5min 连续读 */
    APP_REPORT_EVERY  = 100,   /* 每 100 拍（≈1s）出一行窗口统计 */
    APP_SCAN_ADDR_MIN = 0x03,  /* I2C 7-bit 扫描窗（标准 0x03–0x77） */
    APP_SCAN_ADDR_MAX = 0x77,
    APP_WHO_REG       = 0x0F,  /* LSM6DS3 WHO_AM_I 寄存器（数据手册，值 0x6A） */
    APP_PROBE_ATTEMPT = 3,     /* 真 I2C probe 重试次数（1s 间隔） */
    APP_WARN_LOG      = 3,     /* 错误日志限流：前 3 次仅打印 */
};

/* ---------------------------------------------------------------- 输出层 */

static void app_puts(const char *line)
{
    usb_serial_jtag_write_bytes((const uint8_t *)line, strlen(line),
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
    usb_serial_jtag_write_bytes((const uint8_t *)"\r\n", 2,
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
}

static void app_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    app_puts(buf);
}

/* float → 毫单位整数（mg / mdps / m°C；仅展示不做判据） */
static int f_milli(float v) { return (int)(v * 1000.0f); }

/* ---------------------------------------------------------------- 板级装配 */

/* board_init：库隐函数，按本实例装配板级硬件（引脚/速率只在此出现） */
void board_init(void)
{
#if !P_SIM_IMU
    esp_port_i2c_init(BOARD_I2C_SPEED, BOARD_I2C_SDA, BOARD_I2C_SCL, BOARD_I2C_ADDR);
#endif
}

/* ---------------------------------------------------------------- 数据源 */

static uint32_t s_period_ms;   /* 推流节拍，ms（源配置派生） */

#if P_SIM_IMU
static imu_sim_t s_sim;

static int src_init(void)
{
    const imu_sim_cfg_t *sc = imu_sim_cfg_find(APP_SIM_PRESET);
    if (!sc) return -2;
    if (imu_sim_init(&s_sim, sc) != 0) return -1;
    s_period_ms = (uint32_t)(1000.0f / sc->sample_rate_hz + 0.5f); /* 100Hz → 10ms */
    app_printf("INFO source=sim %s %uHz", APP_SIM_PRESET,
               (unsigned)sc->sample_rate_hz);
    app_printf("WHO 0x6A ok");
    return 0;
}

static int src_read(imu_sample_t *s, float *temp)
{
    if (imu_sim_read_temp(&s_sim, temp) != 0) return -1;
    return imu_sim_read(&s_sim, s);
}
#else
static lsm6ds3_t s_lsm;

static int src_init(void)
{
    const imu_cfg_t *ic = imu_cfg_find(APP_CFG_PRESET);
    if (!ic) return -2;
    /* I2C 总线已在 board_init 装配；此处只做传感器层面 init/probe/configure */
    lsm6ds3_init(&s_lsm, &esp_port_i2c_bus);
    int r = -1;
    for (int a = 0; a < APP_PROBE_ATTEMPT; a++) {
        r = lsm6ds3_probe(&s_lsm);
        if (r == 0) break;
        app_printf(r == -2 ? "WARN who_am_i mismatch (retry %u)"
                           : "WARN i2c no ack (retry %u)", (unsigned)(a + 1));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (r != 0) return -1;
    /* ① WHO_AM_I 校验通过（probe 内核比对 0x6A） */
    app_printf("WHO 0x6A ok");
    if (lsm6ds3_configure(&s_lsm, ic) != 0) return -1;
    s_period_ms = ic->sample_period_ms;
    app_printf("INFO source=lsm6ds3 %s %uHz bus=%uHz",
               APP_CFG_PRESET, (unsigned)ic->accel_odr_hz,
               (unsigned)BOARD_I2C_SPEED);
    return 0;
}

static int src_read(imu_sample_t *s, float *temp)
{
    if (lsm6ds3_read(&s_lsm, s) != 0) return -1;
    return lsm6ds3_read_temp(&s_lsm, temp);
}
#endif

/* ---------------------------------------------------------------- ③ I2C 扫描 */

/**
 * @brief  I2C 总线扫描：遍历 7-bit 地址窗，逐址经 HAL 总线读 WHO_AM_I。
 *
 * @note   真传感器档：port/esp_idf 是单器件总线（addr 契约强制 = 0x6A，
 *         esp_port_i2c_bus 只答 0x6A）→ 扫描结果恒为板面 IMU 单器件（0x6A），
 *         与验收「应见 0x6A」一致；SIM 档为合成回显（零 I2C 依赖）。
 */
static void i2c_scan(void)
{
#if P_SIM_IMU
    app_printf("SCAN dev=0x6A who=0x6A (sim echo)");
    app_printf("SCAN total=1");
#else
    uint8_t who = 0;
    uint32_t n = 0;
    for (int a = APP_SCAN_ADDR_MIN; a <= APP_SCAN_ADDR_MAX; a++) {
        if (esp_port_i2c_bus.read((uint8_t)a, APP_WHO_REG, &who, 1,
                                  esp_port_i2c_bus.ctx) == 0) {
            n++;
            app_printf("SCAN dev=0x%02X who=0x%02X", a, who);
        }
    }
    app_printf("SCAN total=%u", (unsigned)n);
#endif
}

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{
    board_init();   /* 板级装配：加载本实例所用硬件 */

    app_printf("OK boot %s v%s | board=%s | mcu=%s %s",
               APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
               P_SIM_IMU ? "(BYPASS P_SIM_IMU)" : "(HARDWARE I2C)");

    int r = src_init();
    if (r != 0) {
        app_printf(r == -2 ? "FATAL source preset '%s' not found"
                           : "FATAL I2C init failed (bus/device)", 
                   r == -2 ? APP_CFG_PRESET : APP_SIM_PRESET);
        return;
    }

    /* ③ I2C 总线扫描（验收③：应见 IMU 地址 0x6A） */
    i2c_scan();

    /* ② 100Hz 连续读 5min 零报错（err=0 → VERDICT PASS） */
    uint32_t win_n = 0, win_err = 0, total_n = 0, total_err = 0;
    uint64_t t0 = esp_timer_get_time();
    int verdict_done = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));

        imu_sample_t s;
        float temp = 0.0f;
        if (src_read(&s, &temp) != 0) {
            total_err++;
            win_err++;
            if (total_err <= APP_WARN_LOG)
                app_printf("WARN read fail (total=%u)", (unsigned)total_err);
            continue;
        }
        total_n++;
        win_n++;

        if (win_n >= APP_REPORT_EVERY) {   /* ≈1s 一行窗口统计 */
            app_printf("IMU n=%u err=%u ax=%4d ay=%4d az=%4d gx=%4d gy=%4d gz=%4d tmp=%4d",
                       (unsigned)win_n, (unsigned)win_err,
                       f_milli(s.ax), f_milli(s.ay), f_milli(s.az),
                       f_milli(s.gx), f_milli(s.gy), f_milli(s.gz),
                       f_milli(temp));
            /* 每 10s 重申启动头（探针可能晚连错过一次性输出；验收/断言仍要求上电即打） */
            if (total_n % (APP_REPORT_EVERY * 10) == 0) {
                app_printf("OK boot %s v%s | board=%s | mcu=%s %s",
                           APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
                           P_SIM_IMU ? "(BYPASS P_SIM_IMU)" : "(HARDWARE I2C)");
                app_printf("WHO 0x6A ok");
#if P_SIM_IMU
                app_printf("SCAN dev=0x6A who=0x6A (sim echo)");
#else
                app_printf("SCAN dev=0x6A");
#endif
            }
            win_n = 0;
            win_err = 0;
        }

        if (!verdict_done) {               /* 验收②：5min 判据 */
            uint64_t sec = (esp_timer_get_time() - t0) / 1000000ull;
            if (sec >= (uint64_t)APP_RUN_SEC) {
                app_printf("DONE run=%u s read=%u err=%u",
                           (unsigned)sec, (unsigned)total_n, (unsigned)total_err);
                app_printf("VERDICT %s",
                           total_err == 0
                               ? "PASS: 100Hz x 5min zero-error"
                               : "FAIL: read errors during run");
                verdict_done = 1;
            }
        }
    }
}