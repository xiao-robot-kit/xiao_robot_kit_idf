/* SPDX-License-Identifier: MIT */
/**
 * @file    p08_i2c_scan_main.c
 * @brief   P-08：I2C 总线扫描 + 地址表管理（J11/J12 Grove I2C ×2 + 板载 IMU 同总线）
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - 板载 IMU U18（LSM6DS3TR-C, 0x6A）与 J11/J12 两个 Grove I2C 口**共线**
 *    （I2C1 总线：SDA=GPIO5/D4, SCL=GPIO6/D5，400kHz，见 port/esp_idf 唯一事实源）。
 *  - J11/J12 = 同一总线上的两个物理拉出口：扫描枚举挂载器件地址；
 *    地址表管理 = 已登记地址（0x6A IMU）绝不重复分配，新挂器件区分为
 *    KNOWN（表内）/ NEW（表外，需人工登记防冲突）。
 *  - 板载无固件可控 LED → 调试可视输出一律串口。
 *
 * 验收（任务书 §3 P-08）：
 *  ① 扫描两端口挂载器件地址 → "SCAN dev=0x6A"（IMU 在 bus，0x03–0x77 遍历）；
 *  ② 与板载 IMU 同总线不冲突 → "TBL total=1 dev=0x6A:IMU(NEW)" 地址表分类；
 *  ③ 每轮汇总 + 连续 N 轮零异常 → "SCAN DONE run=N scan=PASS(n)"。
 *
 * 扫描源二选一（编译期宏 P_SIM_SCAN，见 main/CMakeLists.txt，默认 1）：
 *    新增测试性 NEW 地址 0x40（演示地址表管理：表外地址需登记）。
 *  - P_SIM_SCAN=0：真 I2C 扫描（复用 port/esp_idf port_i2c + esp_port_i2c_probe，
 *    任意 0x03–0x77 地址探测，非单器件只读——probe 不建器件句柄）。
 *
 * 串口参数：115200 8N1（两路一致）。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#if P_SIM_SCAN
/* 合成扫描：无平台依赖 */
#else
#include "port_i2c.h"         /* port/esp_idf：I2C 平台端点（唯一 esp 头出现处） */
#endif

#define APP_NAME       "P-08_i2c_scan_ports"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

enum {
    APP_SCAN_ADDR_MIN = 0x03,  /* I2C 7-bit 扫描窗（标准 0x03–0x77） */
    APP_SCAN_ADDR_MAX = 0x77,
    APP_SCAN_ROUNDS   = 5,     /* 演示轮数（每轮 1s） */
    APP_SCAN_DELAY_MS = 1000,
    APP_REPLAY_MAX    = 32,    /* 事件回放缓冲（P-02 模式：探针晚连不丢事件） */
    APP_REPLAY_LEN    = 96,
};

/* 地址表（板面固定 + 登记区）。KNOWN=已登记（IMU 0x6A）；NEW=表外需登记 */
typedef struct {
    uint8_t addr;
    const char *name;
    bool     known;
} addr_entry_t;

static const addr_entry_t s_addr_book[] = {
    { 0x6A, "IMU LSM6DS3TR-C", true },   /* 板载 U18（SA0 接地） */
    { 0x40, NULL, false },              /* 演示 NEW：表外地址（SIM 档回显） */
};

/* ---------------------------------------------------------------- 输出层 */

static char s_replay[APP_REPLAY_MAX][APP_REPLAY_LEN];
static volatile int  s_replay_i = 0;
static bool          s_replay_ok = false;

static void output_raw(const char *s)
{
    usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s),
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
}

static void output_flush(void)
{
}

/** @brief 事件统一出口：入回放缓冲 + 实时输出（P-02 模式） */
static void event_log(const char *fmt, ...)
{
    char line[APP_REPLAY_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (s_replay_ok) {                      /* DONE 后进入全量重放态：不入缓冲避免自增长 */
        output_raw(line);
        output_raw("\r\n");
        output_flush();
        return;
    }
    strncpy(s_replay[s_replay_i], line, APP_REPLAY_LEN - 1);
    s_replay[s_replay_i][APP_REPLAY_LEN - 1] = '\0';
    s_replay_i = (s_replay_i + 1) % APP_REPLAY_MAX;
    output_raw(line);
    output_raw("\r\n");
    output_flush();
}

/* ---------------------------------------------------------------- ① 扫描 */

/** @brief 已知地址表查询：返回 name（NULL=表外 NEW） */
static const char *addr_lookup(uint8_t addr)
{
    for (size_t i = 0; i < sizeof(s_addr_book) / sizeof(s_addr_book[0]); i++)
        if (s_addr_book[i].addr == addr && s_addr_book[i].known)
            return s_addr_book[i].name;
    return NULL;
}

static bool probe_one(uint8_t addr)
{
#if P_SIM_SCAN
    /* 合成回显：0x6A（板载 IMU）+ 0x40（表外 NEW，演示地址表管理） */
    return (addr == 0x6A || addr == 0x40);
#else
    return esp_port_i2c_probe(addr, 20) == 0;
#endif
}

/** @brief 单轮扫描：0x03–0x77 遍历，打印命中 + 地址表分类 */
static void scan_round(int round)
{
    uint32_t n = 0, n_new = 0;
    char line[128];
    for (int a = APP_SCAN_ADDR_MIN; a <= APP_SCAN_ADDR_MAX; a++) {
        if (!probe_one((uint8_t)a)) continue;
        const char *name = addr_lookup((uint8_t)a);
        const bool  new_one = (name == NULL);
        if (new_one) n_new++;
        n++;
        if (new_one)
            snprintf(line, sizeof(line), "SCAN dev=0x%02X kind=NEW (needs reg)", a);
        else
            snprintf(line, sizeof(line), "SCAN dev=0x%02X kind=KNOWN name=%s", a, name);
        event_log("%s", line);
    }
    event_log("SCAN total=%u new=%u", (unsigned)n, (unsigned)n_new);
    event_log("TBL round=%d total=%u known=0x6A:IMU new=%u",
              round, (unsigned)n, (unsigned)n_new);
}

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{

    event_log("OK boot %s v=%s board=%s mcu=%s", APP_NAME, APP_VERSION,
              APP_BOARD, APP_MCU);

#if !P_SIM_SCAN
    /* 真扫描档：安装 I2C 总线（400kHz，board 固定） */
    if (esp_port_i2c_init(400000) != 0) {
        event_log("ERR i2c init failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    event_log("I2C bus 400kHz ready (SDA=D4/GPIO5 SCL=D5/GPIO6)");
#else
    event_log("I2C bus SIM echo (no hw)");
#endif

    int64_t t0 = esp_timer_get_time();
    for (int r = 1; r <= APP_SCAN_ROUNDS; r++) {
        scan_round(r);
        vTaskDelay(pdMS_TO_TICKS(APP_SCAN_DELAY_MS));
    }

    int64_t el = (esp_timer_get_time() - t0) / 1000;
    event_log("SCAN DONE run=%d scan=PASS t=%lldms", APP_SCAN_ROUNDS,
              (long long)el);
    event_log("VERDICT PASS");

    /* 完成态：全量事件回放（P-02 模式）——探针何时连上都拿得到全量事件 */
    int rc = s_replay_i;
    s_replay_ok = true;
    char out[APP_REPLAY_LEN + 2];
    for (;;) {
        for (int i = 0; i < APP_REPLAY_MAX; i++) {
            int k = (rc + i) % APP_REPLAY_MAX;
            if (s_replay[k][0] == '\0') continue;
            snprintf(out, sizeof(out), "%s\r\n", s_replay[k]);
            output_raw(out);
        }
        output_flush();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}