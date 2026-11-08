/* SPDX-License-Identifier: MIT */
/**
 * @file    p04_battery_main.c
 * @brief   P-04 电池电压 / 剩余电量例程主程序
 *
 * 链路：D16 电池采样（板上分压 100K:30K）→ ADC2_CH5 (GPIO16)
 *       → battery 组件（分压换算 → SOC 折线 → 剩余 mAh）→ 串口周期打印
 *
 * 数据源二选一（编译期宏 P_ADC_RAW，main/CMakeLists.txt 定义）：
 *   - P_ADC_RAW=1：合成 ADC 原始值（无电池跑通换算链路；沿 exp2 EXP_SIM 同构）
 *   - P_ADC_RAW=0：真 ADC2_CH5 采样（真板 + 分压电阻）
 *
 * 输出（每 1s 一行）：
 *   BATT v=3.85 soc=68% mah=952 alarm=0
 *   v=电池电压 V（4.2 满 / 3.0 截止）；soc 剩余电量 %；mah 剩余容量；
 *   alarm 低电告警（≤3.30V 置 1，预设 v_alarm）
 *
 * 验收口径（任务书 P-04）：真板接电池 → 电压/SOC 数值合理、随放电单调下降、
 * 低电告警生效；无电池 SIM 档（QEMU）验证换算链路双向正确。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "battery.h"

static const char *TAG = "p04";

/* 采样周期（s） */
#define P04_PERIOD_S    1

#if P_ADC_RAW
/* ==== 合成 ADC 原始值档（无电池跑通链路；QEMU/SIM 用） ==== */
/* 模拟放电：4.2V → 3.0V 逐步下降，覆盖全 SOC 区间 */
static uint32_t sim_raw_step(uint32_t tick)
{
    /* 预设 4.2→3.0 线性下降，走 20 步（每步约 0.06V），alarm(≤3.3V) 在第 ~15s 触发，
     * 20s QEMU 断言窗口内可见；走完回满电重来 */
    const float vref    = 3.3f;
    const int   scale   = (1 << 12) - 1;              /* 12bit */
    const float ratio   = (100.0f + 30.0f) / 30.0f;   /* 分压比 4.3333 */
    float v_bat = 4.2f - 1.2f * (float)(tick % 20) / 20.0f;
    float v_adc = v_bat / ratio;                      /* 反推 ADC 输入电压 */
    uint32_t raw = (uint32_t)(v_adc / vref * (float)scale + 0.5f);
    if (raw > (uint32_t)scale) raw = scale;
    return raw;
}

static esp_err_t sample_battery_raw(uint32_t *raw)
{
    static uint32_t tick = 0;
    *raw = sim_raw_step(tick++);
    return ESP_OK;
}
#else
/* ==== 真 ADC 档：ADC2_CH5 (GPIO16) 单次采样（esp_adc oneshot API，v6.1） ==== */
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define P04_ADC_UNIT     ADC_UNIT_2            /* GPIO16 = ADC2 通道（ADC2_GPIO16_CHANNEL 5） */
#define P04_ADC_CHANNEL  ADC_CHANNEL_5
#define P04_ADC_BITS     ADC_BITWIDTH_12
#define P04_ADC_ATTEN    ADC_ATTEN_DB_11       /* 输入量程 0~~3.3V(11dB)；分压后满电 1.26V 居中 */

static adc_oneshot_unit_handle_t s_adc_unit = NULL;

static esp_err_t adc_init_once(void)
{
    if (s_adc_unit != NULL) {
        return ESP_OK;
    }
    /* ADC2 与 Wi-Fi 互斥（硬件共享）；本例程不启用 Wi-Fi */
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = P04_ADC_UNIT,
    };
    esp_err_t rc = adc_oneshot_new_unit(&ucfg, &s_adc_unit);
    if (rc != ESP_OK) {
        return rc;
    }
    adc_oneshot_chan_cfg_t ccfg = {
        .atten = P04_ADC_ATTEN,
        .bitwidth = P04_ADC_BITS,
    };
    rc = adc_oneshot_config_channel(s_adc_unit, P04_ADC_CHANNEL, &ccfg);
    if (rc != ESP_OK) {
        adc_oneshot_del_unit(s_adc_unit);
        s_adc_unit = NULL;
    }
    return rc;
}

static esp_err_t sample_battery_raw(uint32_t *raw)
{
    esp_err_t rc = adc_init_once();
    if (rc != ESP_OK) {
        return rc;
    }
    int out = 0;
    rc = adc_oneshot_read(s_adc_unit, P04_ADC_CHANNEL, &out);   /* 阻塞读取；Wi-Fi 占用时返回错误 */
    if (rc != ESP_OK) {
        return rc;
    }
    *raw = (uint32_t)out;
    return ESP_OK;
}
#endif

#if P_QEMU_UART
#include "driver/uart.h"
static void console_init(void)
{
    /* QEMU 档：UART0 115200 8N1 显式初始化（USB-Serial-JTAG 无 chardev 出口） */
    uart_config_t u = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &u);
}
#else
#include "driver/usb_serial_jtag.h"
static void console_init(void)
{
    /* 真板档：USB-Serial-JTAG 控制台（默认启用） */
    usb_serial_jtag_driver_config_t u = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&u);
}
#endif

/* 输出层：与 P-00 同构——UART0(QEMU) / USB-Serial-JTAG(真板) 显式路由，
 * 防 printf 走默认 console 在 QEMU 下无 chardev 出口而丢输出 */
static void app_puts(const char *line)
{
#if P_QEMU_UART
    uart_write_bytes(UART_NUM_0, line, strlen(line));
    uart_write_bytes(UART_NUM_0, "\r\n", 2);
#else
    usb_serial_jtag_write_bytes((const uint8_t *)line, strlen(line), pdMS_TO_TICKS(100));
    usb_serial_jtag_write_bytes((const uint8_t *)"\r\n", 2, pdMS_TO_TICKS(100));
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

void app_main(void)
{
    console_init();

    /* 电池组件：板上分压 + 典型 1S 锂电参数（容量 1400mAh 可调） */
    const battery_cfg_preset_t *p = battery_cfg_find("xiao_1s_1400mah");
    if (p == NULL) {
        app_printf("BATT ERR preset-not-found");
        return;
    }
    battery_t bat;
    if (battery_init(&bat, &p->cfg) != 0) {
        app_printf("BATT ERR init-failed");
        return;
    }

    app_printf("OK boot p04_battery_voltage | divider=100K:30K v_full=%.2f v_cutoff=%.2f cap=%g mAh",
               p->cfg.v_full, p->cfg.v_cutoff, p->cfg.capacity_mah);
#if P_ADC_RAW
    app_printf("SRC synthetic-adc (no battery)");
#else
    app_printf("SRC adc2_ch5 GPIO16");
#endif

    uint32_t tick = 0;
    for (;;) {
        uint32_t raw = 0;
        esp_err_t rc = sample_battery_raw(&raw);
        if (rc != ESP_OK) {
            app_printf("ADC ERR %s", esp_err_to_name(rc));
        } else {
            battery_update_adc(&bat, raw, 3.3f, 12);
            app_printf("BATT v=%.2f soc=%.0f%% mah=%.0f alarm=%d",
                       bat.v_last, bat.soc_last, bat.mah_last, battery_is_alarm(&bat));
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(P04_PERIOD_S * 1000));
        /* QEMU 文本断言窗口：探针可能晚连错过启动头，每 10s 重申一次（验收仍要求上电 1s 内首打） */
        if (tick % 10 == 0) {
            app_printf("OK boot p04_battery_voltage | divider=100K:30K v_full=%.2f v_cutoff=%.2f cap=%g mAh",
                       p->cfg.v_full, p->cfg.v_cutoff, p->cfg.capacity_mah);
#if P_ADC_RAW
            app_printf("SRC synthetic-adc (no battery)");
#endif
        }
    }
}