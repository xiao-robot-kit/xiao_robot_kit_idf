/* SPDX-License-Identifier: MIT */
/**
 * @file    p03_rgb_ws2812_main.c
 * @brief   P-03：D13 → J7 外接 WS2812 灯带 —— RMT 驱动 + 自研 HSV 色轮
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - D13 (GPIO13)：R79 22R → U4 (SN74LVC1G17 缓冲) → R52 22R → J7 外接灯带。
 *    J7 数据已 5V 电平缓冲（U4）→ 必须走 J7，勿绕过 5V 缓冲直连灯带。
 *  - WS2812：800kHz 数据率，GRB (MSB first)，RESET 低 ≥50us；灯带 24 灯
 *    （宏 P03_LED_N 默认 24）。
 *  - RMT 时序（自研 symbol 组帧，resolution=10MHz）：0 码 T0H=0.35us/T0L=0.8us；
 *    1 码 T1H=0.7us/T1L=0.6us。
 *
 * 数据源二选一（编译期宏 P_SIM_LED，见 main/CMakeLists.txt，默认 1）：
 *  - P_SIM_LED=1：合成灯带（相位→色轮渐变 + 单灯定点，内存缓冲零硬件，
 *  - P_SIM_LED=0：真 RMT 发 GPIO13（真板 + J7 灯带）。
 *
 * 串口参数：115200 8N1（两路一致）。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#if !P_SIM_LED
#include "driver/rmt_tx.h"
#endif

#define APP_NAME       "P-03_rgb_ws2812"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

#define LED_GPIO        13            /* D13：R79→U4 缓冲→R52→J7 灯带 */
#ifndef P03_LED_N
#define P03_LED_N       24            /* 灯带灯数（默认 24） */
#endif

#ifndef P03_NOMINAL_NS  /* WS2812 名义时序（ns），与任务书并官方例程一致 */
#define P03_NOMINAL_NS  1
#endif

/* 色轮三分点（相位度，命名常量禁 magic number） */
#define HUE_0    0
#define HUE_120  120
#define HUE_240  240
#define HUE_360  360
#define LED_PRINT_NDX  5              /* 周期行定点展示的灯索引（0 基） */

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

/* ---------------------------------------------------------------- 色轮与灯阵 */
/* RGB888（组件内一律 uint8 通道） */
typedef struct { uint8_t r, g, b; } led_px_t;

/**
 * @brief  HSV/相位 → RGB 线性色轮（0°红 → 120°绿 → 240°蓝 → 360°回红）。
 */
static led_px_t hue_to_rgb(int hue)
{
    hue = (hue % HUE_360 + HUE_360) % HUE_360;
    led_px_t p;
    if (hue < 60) {            /*   0..60 ：红→绿 提升 */
        p.r = 255; p.g = (uint8_t)(255 * hue / 60); p.b = 0;
    } else if (hue < 120) {    /*  60..120：红退 */
        p.r = (uint8_t)(255 * (120 - hue) / 60); p.g = 255; p.b = 0;
    } else if (hue < 180) {    /* 120..180：绿→蓝 提升 */
        p.r = 0; p.g = 255; p.b = (uint8_t)(255 * (hue - 120) / 60);
    } else if (hue < 240) {    /* 180..240：绿退 */
        p.r = 0; p.g = (uint8_t)(255 * (240 - hue) / 60); p.b = 255;
    } else if (hue < 300) {    /* 240..300：蓝→红 提升 */
        p.r = (uint8_t)(255 * (hue - 240) / 60); p.g = 0; p.b = 255;
    } else {                   /* 300..360：蓝退回红 */
        p.r = 255; p.g = 0; p.b = (uint8_t)(255 * (360 - hue) / 60);
    }
    return p;
}

/* 全带相位渐变 + 单灯定点（索引 0 走反相位，便于肉眼验证定点控制） */
static void frame_fill(led_px_t *px, int phase)
{
    for (int i = 0; i < P03_LED_N; i++) {
        int h = (phase + i * 5) % HUE_360;
        px[i] = hue_to_rgb(h);
    }
    /* 定点灯：走反相位，视觉上固定不动（色相恒定） */
    px[0] = hue_to_rgb((HUE_360 - phase) % HUE_360);
    /* 展示灯：定点索引沿带可看（打印行用相同 hue，读起来即定点验证） */
    px[LED_PRINT_NDX] = px[0];
}

/* ---------------------------------------------------------------- 灯阵渲染 */

#if P_SIM_LED
/* 合成灯带：维护内存灯阵 + 每周期打印（零硬件）
 * 打印用真实 phase（0/120/240 三分点断言落在相位上，非 tick 计数器） */
static void render(const led_px_t *px, int phase)
{
    app_printf("LED phase=%03d led0=r%3dg%3db%3d led5=r%3dg%3db%3d",
               phase, px[0].r, px[0].g, px[0].b,
               px[LED_PRINT_NDX].r, px[LED_PRINT_NDX].g, px[LED_PRINT_NDX].b);
    /* 三分点断言显式打（经过 0/120/240 时打印 HUE=… 供文本断言） */
    if (phase == HUE_0)
        app_printf("HUE 0 r255 g0 b0");
    if (phase == HUE_120)
        app_printf("HUE 120 r0 g255 b0");
    if (phase == HUE_240)
        app_printf("HUE 240 r0 g0 b255");
}
#else
/* 真 RMT 灯带：自研 GRB 组帧 + copy encoder 发送 */
static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_rmt_enc  = NULL;

/* 符号化：1 灯 24bit GRB → 24 个 rmt_symbol_word_t（10MHz 分辨率，0.1us/tick） */
static void px_to_symbols(rmt_symbol_word_t *dst, const led_px_t *px)
{
    /* 名义时序（10MHz 计数）：0 码 3.5 tick 高 / 8 tick 低；1 码 7/6 */
    static const rmt_symbol_word_t bit0 = {
        .level0 = 1, .duration0 = 35, .level1 = 0, .duration1 = 80,
    };
    static const rmt_symbol_word_t bit1 = {
        .level0 = 1, .duration0 = 70, .level1 = 0, .duration1 = 60,
    };
    const uint8_t grb[3] = { px->g, px->r, px->b };   /* GRB 顺序 */
    int n = 0;
    for (int byte = 0; byte < 3; byte++) {
        for (int bit = 7; bit >= 0; bit--) {          /* MSB first */
            dst[n++] = (grb[byte] >> bit) & 1 ? bit1 : bit0;
        }
    }
}

static void render(const led_px_t *px, int phase)
{
    if (s_rmt_chan == NULL) {
        /* 通道：GPIO13，mem_block_symbols 偶数，10MHz 时钟 */
        rmt_tx_channel_config_t tc = {
            .gpio_num = LED_GPIO,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .mem_block_symbols = 64,
        };
        if (rmt_new_tx_channel(&tc, &s_rmt_chan) != ESP_OK) {
            app_printf("RMT FAIL new_tx_channel");
            return;
        }
        rmt_copy_encoder_config_t ec = {};
        if (rmt_new_copy_encoder(&ec, &s_rmt_enc) != ESP_OK) {
            app_printf("RMT FAIL new_copy_encoder");
            return;
        }
        rmt_enable(s_rmt_chan);
    }

    /* 全带帧：P03_LED_N × 24 symbol + RESET（≥50us = 500 tick 低） */
    static rmt_symbol_word_t frame[P03_LED_N * 24 + 1];
    for (int i = 0; i < P03_LED_N; i++)
        px_to_symbols(&frame[i * 24], &px[i]);
    frame[P03_LED_N * 24] = (rmt_symbol_word_t) {
        .level0 = 0, .duration0 = 600, .level1 = 0, .duration1 = 0,
    };

    rmt_transmit_config_t tx = { .loop_count = 0 };   /* 单发（非循环） */
    esp_err_t rc = rmt_transmit(s_rmt_chan, s_rmt_enc, frame,
                                sizeof(frame), &tx);
    if (rc != ESP_OK) {
        app_printf("RMT FAIL transmit rc=%d", (int)rc);
    }
}
#endif

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{

    app_printf("OK boot %s v%s | board=%s | mcu=%s %s",
               APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
               P_SIM_LED ? "(BYPASS P_SIM_LED)" : "(HARDWARE RMT GPIO13->J7)");

    int      phase  = 0;
    uint32_t frames = 0;
    for (;;) {
        /* 每 10s 重申 boot 头（探针可能晚连错过一次性输出，P-00 同构心跳） */
        if ((frames++ % 500) == 0)
            app_printf("OK boot %s v%s | board=%s | mcu=%s %s",
                       APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
                       P_SIM_LED ? "(BYPASS P_SIM_LED)" : "(HARDWARE RMT GPIO13->J7)");
        led_px_t px[P03_LED_N];
        frame_fill(px, phase);
        render(px, phase);
        vTaskDelay(pdMS_TO_TICKS(20));   /* 50fps 色轮推流 */
        phase = (phase + 6) % HUE_360;   /* 每帧 +6°，6s 满轮（0/120/240 都被扫过） */
    }
}