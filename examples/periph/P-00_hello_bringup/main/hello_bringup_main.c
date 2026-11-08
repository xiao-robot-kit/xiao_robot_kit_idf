/* SPDX-License-Identifier: MIT */
/**
 * @file    hello_bringup_main.c
 * @brief   P-00：工程脚手架 + USB 串口日志
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - GPIO 无空余：扩展只能走 I2C（J11/J12，与 IMU 共享）或 UART（J10）；本例程零 GPIO 占用。
 *  - 板载无固件可控 LED（D1 绿灯/充电红×2 均为硬件驱动）→ 调试可视输出一律串口 (+J7 灯带)。
 *  - 舵机与电机共 Motor_5V 轨；本例程不驱动任何外设，无功耗担忧。
 *
 * 验收（任务书 §3 P-00）：上电 1s 内串口打出版本/复位原因；连跑 10min 无重启。
 *
 * 输出层：EXP_QEMU_UART=1 → UART0（QEMU 第 1 个 -serial 槽）；0（默认）→ USB-Serial-JTAG。
 * 串口参数：115200 8N1（两路一致）。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"        /* esp_reset_reason()（v6.1：声明在 esp_system.h，无独立 esp_reset_reason.h） */
#if EXP_QEMU_UART
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif

#define APP_NAME        "P-00_hello_bringup"
#define APP_VERSION     "0.1.0"
#define APP_BOARD       "XIAO Robot Board V1.0"
#define APP_MCU         "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD   115200
#define APP_HEARTBEAT_MS 1000

/* UART0 与 USB-Serial-JTAG 共用的输出原语（print → either 层） */
static void app_puts(const char *line)
{
#if EXP_QEMU_UART
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

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:       return "POWERON";     /* 上电/复位键 */
    case ESP_RST_SW:            return "SOFTWARE";    /* esp_restart() */
    case ESP_RST_PANIC:         return "PANIC";       /* 严重错误重启 */
    case ESP_RST_INT_WDT:       return "INTERRUPT_WDT";
    case ESP_RST_TASK_WDT:      return "TASK_WDT";
    case ESP_RST_WDT:           return "WATCHDOG";
    case ESP_RST_DEEPSLEEP:     return "DEEPSLEEP";   /* 定时/按键唤醒 */
    case ESP_RST_BROWNOUT:      return "BROWNOUT";    /* 欠压（警惕供电轨） */
    case ESP_RST_SDIO:          return "SDIO";
    case ESP_RST_USB:           return "USB";
    case ESP_RST_UNKNOWN:       return "UNKNOWN";
    default:                    return "?";
    }
}

void app_main(void)
{
    /* UART0 仅 QEMU 档需要显式初始化（USB-Serial-JTAG 免配置） */
#if EXP_QEMU_UART
    uart_config_t cfg = {
        .baud_rate = APP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);
#endif

    esp_reset_reason_t reason = esp_reset_reason();

    /* 上电 1s 内：版本 + 复位原因（验收可见输出） */
    app_printf("OK boot %s v%s | board=%s | mcu=%s", APP_NAME, APP_VERSION,
               APP_BOARD, APP_MCU);
    app_printf("INFO reset_reason=%s", reset_reason_str(reason));
    app_printf("HELLO workflow: USB-CDC log link up");

    /* 心跳：连跑判据（10min 无重启 = 无行中断/无 watchdog/无 panic） */
    uint64_t t0 = esp_timer_get_time();
    uint32_t he = 1;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(APP_HEARTBEAT_MS));
        uint64_t sec = (esp_timer_get_time() - t0) / 1000000ull;
        app_printf("HEART %u uptime=%llu s", (unsigned)he++, (unsigned long long)sec);
        /* QEMU 文本断言窗口：探针可能晚连错过 boot 头，每 10s 重申一次版本头（验收仍要求上电 1s 内首打） */
        if (he % 10 == 0) {
            app_printf("OK boot %s v%s | board=%s | mcu=%s", APP_NAME, APP_VERSION,
                       APP_BOARD, APP_MCU);
        }
    }
}