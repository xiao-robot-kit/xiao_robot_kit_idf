/* SPDX-License-Identifier: MIT */
/**
 * @file    store_demo_main.c
 * @brief   pstore 独立例程主程序（T-006/T-018）
 *
 * 演示目标（链路优先：最小闭环先行）：
 *  1. **flash 裸分区后端**：pstore 落在自定义 data 分区（label "store"），
 *     操作经 esp_partition 直写（无 NVS 中间层）——examples/_shared/pstore_flash.c；
 *  2. **断电续跑**：boot_count 每次开机 +1 落盘；QEMU 下 kill → 重启同一镜像，
 *     boot 值跨掉电保持（run_qemu 实验脚本 + 手动两段 KVM 验证）；
 *  3. **命令通道（MOD_STORE 3 命令）**：GET / STATUS / SET（无 ERASE，
 *     见 store_demo.h 注记）；上位机页三按钮与图标区使用同一编码表；
 *  4. 周期 TEXT 状态行，probe 可直接断言。
 *
 * 工程形态：同 exp1/exp2 —— EXP_QEMU_UART=1 走 UART0（QEMU chardev 有出口），
 * 0（默认）走 USB-Serial-JTAG（真板链路）。无 EXP_SIM：例程不采传感器，
 * 数据面只有 pstore 参数，模拟/真板共用同一份逻辑（flash 后端两处都真）。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if EXP_QEMU_UART
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif
#include "store_demo.h"
#include "proto/frame.h"
#include "pstore_flash.h"      /* port/esp_idf：flash 裸分区后端（本例程的介质） */

#define APP_NS_PART_LABEL   "store"            /* 分区表 label（见 partitions.csv） */
#define APP_STORE_CAPACITY  4096               /* 存储区字节数（分区 64KB 内留裕量） */
#define APP_UART_BAUD       115200
#define APP_UART_RX_BUF     256
#define APP_TX_TIMEOUT_MS   20
#define APP_STATUS_PERIOD_MS 1000              /* 周期状态行间隔 */

static pstore_backend_t s_be;
static pstore_flash_t   s_flash;
static store_demo_t    *s_sd;
static uint8_t          s_seq;

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n == 0) return;
#if EXP_QEMU_UART
    uart_write_bytes(UART_NUM_0, buf, n);
#else
    usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
#endif
}

static void send_text(const char *msg)
{
    size_t n = strlen(msg);
    if (n > PROTO_PAYLOAD_MAX) n = PROTO_PAYLOAD_MAX;
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)n);
}

/* 帧出口回调（store_demo 会话的 ACK/TEXT 出口） */
static int tx_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    send_frame(type, payload, len);
    return 0;
}

/** 收一拍下行字节交给会话（帧组装/分发在 store_demo 会话内，非阻塞） */
static void service_rx(void)
{
    uint8_t buf[64];
#if EXP_QEMU_UART
    const int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), 0);
#else
    const int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
#endif
    if (n > 0) store_demo_rx(s_sd, buf, (uint8_t)n, tx_send);
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
    uart_driver_install(UART_NUM_0, APP_UART_RX_BUF, 0, 0, NULL, 0);
#else
    usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&uc);
#endif

    /* 1) flash 裸分区后端（T-006 核心：介质 = 自定义 data 分区，esp_partition 直写） */
    const int be_rc = pstore_flash_open(&s_be, &s_flash, APP_NS_PART_LABEL);
    if (be_rc != 0) {
        send_text("FATAL store partition not found (partitions.csv?)");
        return;
    }

    /* 2) 会话（创建 → 开机加载 → boot_count++ 落盘） */
    const store_demo_cfg_t cfg = { .backend = &s_be, .capacity = APP_STORE_CAPACITY };
    s_sd = store_demo_create(&cfg);
    if (s_sd == NULL) {
        send_text("FATAL store demo create failed");
        return;
    }
    const int boot_rc = store_demo_boot(s_sd);
    if (boot_rc == -2) send_text("ERROR store save failed at boot");

    /* 3) 上电证据行（probe 断言面） */
    {
        char hi[96];
        if (boot_rc == 0) {
            snprintf(hi, sizeof(hi), "OK store boot: count=%u load=%d len=%u kb=%u",
                     (unsigned)s_sd->boot_count, s_sd->load_state,
                     (unsigned)s_sd->file_len, (unsigned)s_sd->capacity_kb);
        } else {
            snprintf(hi, sizeof(hi), "ERR store boot failed rc=%d", boot_rc);
            send_text(hi);
            for (;;) vTaskDelay(pdMS_TO_TICKS(APP_STATUS_PERIOD_MS));
        }
        send_text(hi);
    }

    /* 4) 周期状态行 + 下行命令服务 */
    uint32_t ticks = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(APP_STATUS_PERIOD_MS));
        ticks++;
        service_rx();
#if 0
        /* 周期状态行默认关闭（probe 只断言开机行足矣；需要时开） */
        if (ticks % 5 == 0) {
            char line[128];
            store_demo_text(s_sd, line, sizeof(line));
            send_text(line);
        }
#endif
    }
}