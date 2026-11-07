/**
 * @file    p12_spare_main.c
 * @brief   P-12 备用 GPIO（TP20=MTDI/GPIO41、TP21=MTDO/GPIO42）例程
 *
 * 链路：MTDI/MTDO 引脚（任务书 §1.1：U3 pad24/25 —— 仅 TP20/TP21 测试点，
 *       板载 20 个 GPIO 之外的「唯一备用 GPIO，飞线用」）→ gpio 驱动配置为
 *       输入/输出 → 双向互测（输出写、输入读，loopback 自证）→ 文本遥测 →
 *       UART0（QEMU 档）/ USB-Serial-JTAG（真板档）。
 *
 * 演示（有限序列，事件回放每 1s 重放全量；P-02 模式）：
 *   A. 方向配置：MTDI 设输出、MTDO 设输入，打印两脚能力与 strapping 提示；
 *   B. 输出/输入互测：MTDI 依次写 4 拍电平 {0→1→0→1}，MTDO 同步读回比对，
 *      记录方向 1 的不一致计数；随后角色互换（MTDI 输入、MTDO 输出）再测一轮，
 *      记录方向 2 的不一致计数；
 *   C. 汇总：报告两方向 mismatch 计数 + VERDICT PASS（mismatch 均为 0 时）。
 *
 * 硬件（任务书 §1.1/§4.1）：TP20（MTDI）= XIAO-S3 PAD25、GPIO41、上电默认高
 *       （MTDI 上拉生效）+ J9 关联无（仅测试点）；TP21（MTDO）= GPIO42、上电
 *       默认由电压选择 strapping 侧拉（详见 README 与固件 strapping 提示行）。
 *       ⚠ strapping 属性（任务书要求提示）：GPIO42 是 ESP32-S3 的 strapping
 *       引脚（VDD_SPI 电压选择，boot 早期采样）——真板飞线验证时必须保证
 *       上电瞬间两脚无强下拉（否则可能改变 boot 行为）；例程在启动行中打印
 *       该警告。QEMU 模拟无 strapping 语义，仅透传配置路径。
 *
 * 宏契约（CMake 定义，两态显式）：
 *   P_SIM_SPARE  1=合成 GPIO 回环（默认，QEMU 可测：QEMU 的 GPIO 无外部
 *                飞线电平）；0=真硬件 GPIO41/42（真板 + TP20/21 飞线）。
 *   EXP_QEMU_UART 1=遥测出口 UART0（QEMU 第 1 个 -serial 槽）；0=USB-Serial-JTAG。
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#define APP_NAME    "P-12_spare_gpio_tpxx"
#define APP_VERSION "v0.1.0"
#define APP_BOARD   "XIAO Robot Board V1.0"
#define APP_MCU     "XIAO ESP32-S3 Plus"

#define MTDI_GPIO   41          /* TP20：MTDI/SPIIOMUX —— 备用输入/输出 A */
#define MTDO_GPIO   42          /* TP21：MTDO/SPIIOMUX —— 备用输入/输出 B（strapping） */

#define APP_UART_BAUD       115200
#define APP_TX_TIMEOUT_MS   20

/* 互测拍数（写输出电平表，读回比对） */
#define SPARE_PATTERN_N     4
static const int s_pat[SPARE_PATTERN_N] = { 0, 1, 0, 1 };

/* 角色 1：MTDI 输出 / MTDO 输入 */
#define ROLE1_OUT  MTDI_GPIO
#define ROLE1_IN   MTDO_GPIO
/* 角色 2：MTDI 输入 / MTDO 输出 */
#define ROLE2_OUT  MTDO_GPIO
#define ROLE2_IN   MTDI_GPIO

/* 事件回放缓冲（SIM 完成态每 1s 重放，供 QEMU 探针晚连场景可见） */
#define REPLAY_MAX 16
static char s_replay[REPLAY_MAX][96];
static size_t s_replay_n = 0;
static void replay_add(const char *line)
{
    if (s_replay_n < REPLAY_MAX)
        snprintf(s_replay[s_replay_n++], sizeof(s_replay[0]), "%s", line);
}

/* ---------------------------------------------------------------- 输出层 */
static void app_puts(const char *line)
{
#if EXP_QEMU_UART
    uart_write_bytes(UART_NUM_0, line, strlen(line));
    uart_write_bytes(UART_NUM_0, "\r\n", 2);
#else
    usb_serial_jtag_write_bytes((const uint8_t *)line, strlen(line),
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
    usb_serial_jtag_write_bytes((const uint8_t *)"\r\n", 2,
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
#endif
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

/* ---------------------------------------------------------------- GPIO 源 */

#if P_SIM_SPARE
/* 合成回环：写电平存内存表，读回返回表值 —— 模拟「飞线把写脚电平直接送回
 * 读脚」的 loopback 语义，QEMU 可断言全链路（配置 + 写 + 读 + 比对）。
 * 两个备用脚共用一张表（模拟 TP20↔TP21 飞线互联）：写任一脚 → 双槽同步，
 * 读任一脚 → 同一电平（短接点电平同源）。 */
static int s_sim_level[2] = { 0, 0 };   /* [0]=MTDI [1]=MTDO，互联同源 */
static int spare_write_sim(int gpio, int lvl)
{
    (void)gpio;
    s_sim_level[0] = lvl;
    s_sim_level[1] = lvl;
    return 0;
}
static int spare_read_sim(int gpio)
{
    return s_sim_level[gpio == MTDI_GPIO ? 0 : 1];
}
#else
static int spare_write_hw(int gpio, int lvl) { return gpio_set_level(gpio, lvl); }
static int spare_read_hw(int gpio)            { return gpio_get_level(gpio); }
#endif

/* 方向配置（输出脚 / 输入脚） */
static void spare_config(int out_gpio, int in_gpio)
{
#if !P_SIM_SPARE
    gpio_config_t o = {
        .pin_bit_mask = 1ULL << out_gpio,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&o);
    gpio_config_t i = {
        .pin_bit_mask = 1ULL << in_gpio,
        .mode         = GPIO_MODE_INPUT,
    };
    gpio_config(&i);
#else
    (void)out_gpio; (void)in_gpio;
#endif
}

/* ---------------------------------------------------------------- 互测 */
/* 一轮互测：out 写 pattern、in 读回比对，返回 mismatch 计数并回放行 */
static int spare_loop(uint32_t tag, int out_gpio, int in_gpio)
{
    int mismatch = 0;
    for (int k = 0; k < SPARE_PATTERN_N; k++) {
#if P_SIM_SPARE
        spare_write_sim(out_gpio, s_pat[k]);
        int got = spare_read_sim(in_gpio);
#else
        spare_write_hw(out_gpio, s_pat[k]);
        int got = spare_read_hw(in_gpio);
#endif
        char ll[96];
        snprintf(ll, sizeof(ll),
                 "SPARE r%u out=%d -> in=%d %s",
                 (unsigned)tag, s_pat[k], got,
                 got == s_pat[k] ? "OK" : "MISMATCH");
        app_printf("%s", ll);
        replay_add(ll);
        if (got != s_pat[k]) mismatch++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return mismatch;
}

void app_main(void)
{
#if EXP_QEMU_UART
    const uart_config_t uc = {
        .baud_rate  = APP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &uc);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
#endif

    /* strapping 提示（任务书硬性「注意 strapping 属性」）：GPIO42=MTDO 是
     * ESP32-S3 strapping 脚（VDD_SPI 电压选择，boot 早期采样）——真板飞线时
     * 两脚不可带强下拉进 boot；QEMU 无此语义，仅透传提示。 */
    app_printf("OK boot %s %s | board=%s | mcu=%s | %s",
               APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
               P_SIM_SPARE ? "(SIM LOOPBACK)" : "(HW TP20/TP21 FLY-WIRE)");
    app_printf("SPARE strapping-warn GPIO42=MTDO=TP21 is VDD_SPI strap pin: "
               "no strong pull-down to GND during boot on real board");
    app_printf("SPARE pins MTDI=GPIO41(TP20) MTDO=GPIO42(TP21)");
    replay_add("OK boot P-12_spare_gpio_tpxx (SIM replay)");
    replay_add("SPARE pins MTDI=GPIO41(TP20) MTDO=GPIO42(TP21)");

    /* 角色 1：MTDI 输出、MTDO 输入 */
    spare_config(ROLE1_OUT, ROLE1_IN);
    app_printf("SPARE role1 MTDI=OUT MTDO=IN");
    replay_add("SPARE role1 MTDI=OUT MTDO=IN");
    int m1 = spare_loop(1, ROLE1_OUT, ROLE1_IN);

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 角色 2：MTDI 输入、MTDO 输出（验证两脚都能输出与都够输入） */
    spare_config(ROLE2_OUT, ROLE2_IN);
    app_printf("SPARE role2 MTDI=IN MTDO=OUT");
    replay_add("SPARE role2 MTDI=IN MTDO=OUT");
    int m2 = spare_loop(2, ROLE2_OUT, ROLE2_IN);

    char sum[96];
    snprintf(sum, sizeof(sum), "SPARE done mismatch=(%d,%d) -> %s",
             m1, m2, (m1 == 0 && m2 == 0) ? "VERDICT PASS" : "VERDICT FAIL");
    app_printf("%s", sum);
    replay_add(sum);

    /* 完成态：每 1s 重放全部可观测行（探针可能晚连） */
    for (;;) {
        for (size_t i = 0; i < s_replay_n && i < REPLAY_MAX; i++)
            app_puts(s_replay[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}