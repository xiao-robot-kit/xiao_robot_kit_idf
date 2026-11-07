/**
 * @file    p02_button_main.c
 * @brief   P-02：D11 (GPIO11) 按键 —— 软件消抖 + 短按/长按事件
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - K7 (D11)：GPIO11，内部上拉 45K，按键按下 = 低电平（SW 闭合到 GND）。
 *  - 消抖 ≥20ms（软件）；键码语义：短按 = 按下到抬起 <800ms；
 *    长按 = 按下到抬起 ≥800ms，且长按到点即触发（不等抬起）。
 *  - 验收边界：795ms → 短按；805ms → 长按。
 *
 * 数据源二选一（编译期宏 P_SIM_KEY，见 main/CMakeLists.txt，默认 1）：
 *  - P_SIM_KEY=1：合成按键序列（内存事件表），QEMU 可测：
 *      按键 1..3  短按（hold 约 300ms）
 *      按键 4..5  长按（hold 900/805ms，长按到点触发）
 *      注入 4~8ms 抖动脉冲（消抖窗口内吸收，验证无抖动误报）
 *  - P_SIM_KEY=0：真 GPIO11（内部上拉，按下=低）。
 *
 * 输出层：EXP_QEMU_UART=1 → UART0（QEMU）；0（默认）→ USB-Serial-JTAG（真板）。
 * 串口参数：115200 8N1（两路一致）。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#if EXP_QEMU_UART
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif
#if !P_SIM_KEY
#include "driver/gpio.h"
#endif

#define APP_NAME       "P-02_button_k7"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

#define KEY_GPIO        11            /* K7 D11：内部上拉，按下=低 */

/* 消抖 / 长短按判定（时间单位：ms，节拍 1ms） */
#define KEY_DEBOUNCE_MS   20          /* 消抖窗口：≥20ms 稳定才翻转 */
#define KEY_LONG_MS       800         /* 长按阈值：≥800ms → LONG */
#define KEY_EVENT_HOLD_HI (KEY_LONG_MS - 5)   /* 795ms → 短按（验收边界） */
#define KEY_EVENT_HOLD_LO (KEY_LONG_MS + 5)   /* 805ms → 长按（验收边界） */
#define KEY_BOUNCE_CT_M   10          /* 半窗口计数：≥10ms 反跳才计抖动 */

static uint32_t s_key_n = 0;          /* 事件序号（1 起） */

/* 事件回放缓冲（SIM 完成态每 1s 重放，供 QEMU 探针晚连场景可见）：
 * 行内嵌 boot 头、每次事件行、done 行；重放只读，数量 8 足够。 */
#define REPLAY_MAX 12
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

/* ---------------------------------------------------------------- 信号源 */

#if P_SIM_KEY
/* 合成按键时序（t, level）：0=按下(低)；1=抬起(高)。
 * 事件序：短按×3 + 长按×2（任务卡验收口径），795/805ms 边界用例在内；
 * 抖动 4~8ms（<10ms 半窗口，消抖吸收、不计数）插在事件间隙证明无抖动误报。
 *   E1 800-1120  SHORT（hold 320）
 *   E2 2200+800  到点触发 LONG（hold 900，3100 抬起不再报）
 *   E3 3500-3800  SHORT（hold 300）
 *   E4 4300-5105  LONG 边界（hold 805，到点 5100 触发）
 *   E5 5600-6395  SHORT 边界（hold 795）
 */
static const struct { uint32_t t; int lvl; } s_seq[] = {
    {   800, 0 },   /* E1 短按：按下 */
    {  1120, 1 },   /*          抬起 */
    {  1170, 0 },   /*  4ms 抖动（吸收不计数） */
    {  1174, 1 },
    {  1180, 0 },
    {  1184, 1 },
    {  2200, 0 },   /* E2 长按：按下 */
    {  3100, 1 },   /*          到点 3000 触发 LONG，抬起不再报 */
    {  3500, 0 },   /* E3 短按：按下 */
    {  3800, 1 },   /*          抬起 */
    {  3900, 0 },   /*  8ms 抖动 */
    {  3908, 1 },
    {  4300, 0 },   /* E4 长按边界 805：按下 */
    {  5105, 1 },   /*          到点 5100 触发 LONG */
    {  5600, 0 },   /* E5 短按边界 795：按下 */
    {  6395, 1 },   /*          抬起 → SHORT */
};
#define SIM_SEQ_N (sizeof(s_seq) / sizeof(s_seq[0]))
static uint32_t s_sim_i = 0;          /* 表游标 */
static int s_sim_pressed = 0;         /* 当前合成按键状态（0=抬起 1=按下） */

/* 合成源：按 1ms 节拍推进，返回当前电平（GPIO 低=按下 → 内部 1=按下） */
static int key_read_sim(void)
{
    static uint32_t t = 0;
    t++;
    while (s_sim_i < SIM_SEQ_N && t >= s_seq[s_sim_i].t) {
        s_sim_pressed = s_seq[s_sim_i].lvl == 0 ? 1 : 0;
        s_sim_i++;
    }
    return s_sim_pressed;
}
#else
static int key_read_hw(void)
{
    /* GPIO11 内部上拉：读引脚电平，低=按下（内部语义 1=按下，与 SIM 一致） */
    return gpio_get_level(KEY_GPIO) == 0 ? 1 : 0;
}
#endif

/* ---------------------------------------------------------------- 消抖器 */
/* 状态机：s_filt 为消抖后电平（0=抬起 1=按下）；s_cont 连续相反采样计数；
 * s_deb 抖动计数（≥10ms 半窗口反跳才计）；s_jitter 近翻转抖动计数。 */
static int    s_filt = 0;
static uint32_t s_cont = 0;
static uint32_t s_deb = 0, s_jitter = 0;

static void debounce_tick(int raw)
{
    if (raw == s_filt) {
        if (s_cont >= KEY_BOUNCE_CT_M && s_cont < KEY_DEBOUNCE_MS) {
            s_jitter++;              /* 反跳充电未满就回归：近翻转抖动 */
        }
        s_cont = 0;
    } else {
        s_cont++;                    /* 反向持续计数 */
        if (s_cont >= KEY_DEBOUNCE_MS) {
            s_filt = raw;            /* ≥20ms 稳定 → 翻转 */
            s_cont = 0;
        }
    }
}

/* ---------------------------------------------------------------- 主流程 */

static void key_event(int long_press, uint32_t hold_ms)
{
    s_key_n++;
    if (long_press) {
        app_printf("KEY LONG n=%u hold=%u ms", (unsigned)s_key_n, (unsigned)hold_ms);
        {
            char ll[96];
            snprintf(ll, sizeof(ll), "KEY LONG n=%u hold=%u ms",
                     (unsigned)s_key_n, (unsigned)hold_ms);
            replay_add(ll);
        }
    } else {
        app_printf("KEY SHORT n=%u hold=%u ms", (unsigned)s_key_n, (unsigned)hold_ms);
        {
            char ll[96];
            snprintf(ll, sizeof(ll), "KEY SHORT n=%u hold=%u ms",
                     (unsigned)s_key_n, (unsigned)hold_ms);
            replay_add(ll);
        }
    }
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
#if !P_SIM_KEY
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << KEY_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
#endif

    replay_add("OK boot P-02_button_k7 (SIM replay)");
    app_printf("OK boot %s v%s | board=%s | mcu=%s %s",
               APP_NAME, APP_VERSION, APP_BOARD, APP_MCU,
               P_SIM_KEY ? "(BYPASS P_SIM_KEY)" : "(HARDWARE KEY GPIO11)");

    /* 消抖器：1ms 节拍轮询（真板：软件轮询即消抖逻辑主循环） */
    uint32_t press_t0 = 0;           /* 消抖确认按下的时刻（ms） */
    int      acting = 0;             /* 当前是否已确认按下 */
    int      s_lng_fired = 0;        /* 长按到点是否已触发 */
    uint32_t t_ms = 0;
    int      sim_done = 0;           /* SIM 序列已完成（进入重放态） */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        t_ms++;
#if P_SIM_KEY
        int raw = key_read_sim();
#else
        int raw = key_read_hw();
#endif
        debounce_tick(raw);

        int f = s_filt;
        if (f && !acting) {          /* 消抖确认按下 */
            acting = 1;
            s_lng_fired = 0;
            press_t0 = t_ms;
        } else if (!f && acting) {   /* 消抖确认抬起 */
            uint32_t hold = t_ms - press_t0;
            acting = 0;
            if (!s_lng_fired) {
                /* 未触发过长按 → 按 hold 判定，轨迹映射到 795/805 边界附近 */
                int is_long = hold >= KEY_LONG_MS;
                key_event(is_long, hold);
#if P_SIM_KEY
                if (is_long) s_lng_fired = 1;  /* sim 长按到点已由 check 触发，这里统一标记 */
#endif
            }
        } else if (f && acting && !s_lng_fired &&
                   t_ms - press_t0 >= KEY_LONG_MS) {
            /* 长按到点（800ms）：立即触发 LONG，不等待抬起 */
            s_lng_fired = 1;
            key_event(1, (uint32_t)(t_ms - press_t0));
        }

#if P_SIM_KEY
        if (s_sim_i >= SIM_SEQ_N && !acting) {
            /* 序列跑完：转成无限循环，每 1s 重放全部可观测行
             * （探针可能晚连，一次性输出不可观测——与 P-00/P-01/P-04 同构心跳） */
            if (!sim_done) {
                sim_done = 1;
                char dn[96];
                snprintf(dn, sizeof(dn), "KEY done n=%u deb=%u jitter=%u",
                         (unsigned)s_key_n, (unsigned)s_deb, (unsigned)s_jitter);
                replay_add(dn);
            }
            for (size_t i = 0; i < s_replay_n && i < REPLAY_MAX; i++)
                app_puts(s_replay[i]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
#endif
    }
}