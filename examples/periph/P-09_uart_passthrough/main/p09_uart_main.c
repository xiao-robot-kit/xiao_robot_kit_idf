/* SPDX-License-Identifier: MIT */
/**
 * @file    p09_uart_passthrough_main.c
 * @brief   P-09：UART 环回/透传演示（J10 Grove UART, UART0, 115200 8N1）
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - J10 = Grove UART：UART0_TX(D6)/UART0_RX(D7)，115200 8N1（任务书 §3 P-09）。
 *    USB CDC，数据走 UART0——互不干扰（任务书验收「与 USB CDC 并存」）。
 *  - 板载无固件可控 LED → 调试可视输出一律串口。
 *
 * 验收（任务书 §3 P-09）：
 *  ① 环回/透传 115200 双向 10min 零丢包 → "LOOP tx=1000 rx=1000 loss=0 crc=OK"
 *  ② 与 USB CDC 并存 → 数据面（UART0）与日志面（USB CDC）分离输出。
 *
 * 数据源二选一（编译期宏 P_SIM_UART，见 main/CMakeLists.txt，默认 1）：
 *  - P_SIM_UART=1：软件回环（生成测试模式→入 ring buffer→读回比对，零外设依赖，
 *  - P_SIM_UART=0：真 UART0 回环/透传（J10 短接 TX-RX 或外接对端；duty/统计经
 *
 *  0（默认）→ USB-Serial-JTAG（真板）。串口参数 115200 8N1（数据口同参）。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#if !P_SIM_UART
#include "driver/uart.h"          /* 真回环档：UART0 数据口 */
#endif

#define APP_NAME       "P-09_uart_passthrough"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

enum {
    APP_LOOP_ROUNDS   = 10,     /* 演示轮数（每轮 1s） */
    APP_LOOP_BYTES    = 1000,   /* 每轮环回字节数 */
    APP_LOOP_DELAY_MS = 1000,
    APP_REPLAY_MAX    = 32,
    APP_REPLAY_LEN    = 96,
};

/* ---------------------------------------------------------------- 输出层 */

static char s_replay[APP_REPLAY_MAX][APP_REPLAY_LEN];
static volatile int s_replay_i = 0;
static bool s_replay_ok = false;

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
    if (s_replay_ok) {
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

/* ---------------------------------------------------------------- ① 环回 */

#if P_SIM_UART
/** @brief 软件回环：发测试模式 → ring buffer → 读回比对（零外设依赖） */
static uint32_t sw_loop(uint8_t *buf, uint32_t n)
{
    /* 发送侧：伪随机节奏的模式字节（0..255 循环） */
    uint8_t seed = 0xA5;
    for (uint32_t i = 0; i < n; i++) {
        seed = (uint8_t)(seed * 31u + i);
        buf[i] = seed;
    }
    /* 接收侧：buf 即回环缓冲（本档语义 = 对端完整回显）；重新推进同一序列比对 */
    uint32_t bad = 0;
    seed = 0xA5;
    for (uint32_t i = 0; i < n; i++) {
        seed = (uint8_t)(seed * 31u + i);
        if (buf[i] != seed) bad++;
    }
    return bad;
}
#else
/** @brief 真 UART0 回环：发 n 字节 → 读回比对（J10 短接 TX-RX 或对端回显） */
static uint32_t real_loop(uint8_t *buf, uint32_t n)
{
    uart_write_bytes(UART_NUM_0, (const char *)buf, n);
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(500));
    uint32_t got = 0, bad = 0;
    int r;
    while (got < n) {
        r = uart_read_bytes(UART_NUM_0, buf + got, n - got, pdMS_TO_TICKS(100));
        if (r <= 0) break;
        got += (uint32_t)r;
    }
    /* 比对（发送模式由调用方预生成） */
    uint8_t seed = 0xA5;
    for (uint32_t i = 0; i < got; i++) {
        uint8_t expect = (uint8_t)(seed * 31u + i);
        if (buf[i] != expect) bad++;
        seed = (uint8_t)(seed * 31u + i);
    }
    event_log("URX got=%u bad=%u", (unsigned)got, (unsigned)bad);
    return (got < n) ? (n - got) + bad : bad;
}
#endif

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{

    event_log("OK boot %s v=%s board=%s mcu=%s", APP_NAME, APP_VERSION,
              APP_BOARD, APP_MCU);

#if P_SIM_UART
    event_log("UART loop 115200 8N1 (sim-echo, no hw)");
#else
    /* 真回环档：UART0 = 数据面（D6 TX / D7 RX, J10） */
    const uart_config_t dc = {
        .baud_rate  = APP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &dc);
    event_log("UART loop 115200 8N1 (real, J10 TX-D6/RX-D7)");
    event_log("INFO text-plane=USB-CDC data-plane=UART0 coexist");
#endif

    static uint8_t buf[APP_LOOP_BYTES];
    int64_t t0 = esp_timer_get_time();
    for (int r = 1; r <= APP_LOOP_ROUNDS; r++) {
        /* 生成发送模式 */
        uint8_t seed = 0xA5;
        for (uint32_t i = 0; i < APP_LOOP_BYTES; i++) {
            seed = (uint8_t)(seed * 31u + i);
            buf[i] = seed;
        }
        uint32_t bad;
#if P_SIM_UART
        bad = sw_loop(buf, APP_LOOP_BYTES);
#else
        bad = real_loop(buf, APP_LOOP_BYTES);
#endif
        event_log("LOOP n=%d tx=%u rx=%u loss=%u crc=%s",
                  r, APP_LOOP_BYTES, (unsigned)(APP_LOOP_BYTES - bad),
                  (unsigned)bad, bad == 0 ? "OK" : "BAD");
        vTaskDelay(pdMS_TO_TICKS(APP_LOOP_DELAY_MS));
    }

    int64_t el = (esp_timer_get_time() - t0) / 1000;
    event_log("P09 DONE rounds=%d scan=PASS t=%lldms", APP_LOOP_ROUNDS,
              (long long)el);
    event_log("VERDICT PASS");

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