/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_main.c
 * @brief   实验电机：QEMU 演示固件（demo S 梯度 + T-112 四专项场景运行时）
 *
 * @note    链路：电机库(the `xiao_motor` component) + 合成 PWM/编码器 HAL
 *          (examples/_shared/motor_hal_sim.c) → 500Hz 控制环 → 帧协议 0x05/0x06
 *          → UART0(console) → QEMU -serial tcp:5558 → host 示波器
 *          (xiao_robot_kit_host/backend/scope_motor.py)。
 * @note    控制台 = UART0（sdkconfig.defaults，QEMU 第一 serial 槽；ROM 启动
 *          横幅与帧流同端口，上位机按 SOF+CRC8 重同步）。USB-Serial-JTAG
 *          在 QEMU esp32s3 无路由（exp1 教训），本实验显式改走 UART0。
 * @note    双运行时（T-112+DC 收满）：boot 默认 demo（motor_demo，0x05 帧
 *          4 槽四类齐）；UART0 收「SCEN <n>」行（RX 任务解析→邮箱）后主循环
 *          切换到 motor_scen 场景运行时（0x06 帧带场景号+定时器快照）。
 *          切换在主循环上下文执行（邮箱读写各在临界区内，零数据竞争）——
 *          demo/scen 的 tick+打帧永远同任务串行，无字节级交错。命令重发即
 *          场景重启（start 全量重置语义）；SCEN 0 在切换后不可回 demo
 *          （pwm_hal_register 进程级一次，重启 QEMU 回 demo 面）。
 * @note    参数零写死：demo 走 motor_demo_table；四场景走 motor_scen_table
 *          （电机/定时器底版 = 库预设，wiring+步进轨迹全表驱动）。
 * @note    验收（host 示波器断言）：demo = S 梯度形状（0x05，原有断言集）；
 *          场景 = 四专项断言集（scope_motor.py --scenario，0x06 帧解码）。
 * @note    TEXT 帧消息限长：payload ≤ PROTO_PAYLOAD_MAX(96B)，超长帧被编码器
 *          拒发（静默丢）——本文件所有 TEXT 字面量按 ≤96B 构造并留裕量。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"       /* 任务卡指定输出面：uart_write_bytes → UART_NUM_0 */
#include "driver/uart_vfs.h"   /* console VFS 并入同一驱动：两条写路径串行不交错 */
#include "proto/frame.h"
#include "motor_demo.h"
#include "motor_scen.h"

static uint8_t s_seq = 0;

/**
 * 帧直写 UART0（任务卡指定机制：driver/uart.h 的 uart_write_bytes）。
 * @note  UART0 同时是 console（ROM/bootloader 横幅同口先出）；安装驱动后调
 *        uart_vfs_dev_use_driver 把 console 写并入驱动路径——帧流与诊断输出
 *        同一 TX 通道串行下发，不发生字节级交错。写失败无补救面：不中断
 *        控制环（帧序号 SEQ 可检丢帧）。
 */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n) {
        int w = uart_write_bytes(UART_NUM_0, (const char *)buf, n);
        (void)w;
    }
}

static void send_text(const char *msg)
{
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)strlen(msg));
}

/* ── 场景命令邮箱（T-112）：RX 任务单写、主循环单取 ──────────────────────
 * 读写均在 portMUX 临界区内（读=取走并清零，原子）——跨任务共享零竞争；
 * 「最新命令胜出」语义正合「重发即重启场景」（上位机 0.5s 重发自愈）。 */
static portMUX_TYPE s_scen_mux = portMUX_INITIALIZER_UNLOCKED;
static int s_scen_req = -1;    /**< -1 = 无请求；场景号 ≥ 0 生效 */
static int s_rt = 0;           /**< 活动运行时：0=demo 1=场景 -1=失败态 */

#define RX_LINE_MAX 16         /* 「SCEN <n>」最长 8B；裕量 16 */

/** @brief 邮箱投递（RX 任务侧；临界区内单写） */
static void scen_req_post(int id)
{
    portENTER_CRITICAL(&s_scen_mux);
    s_scen_req = id;
    portEXIT_CRITICAL(&s_scen_mux);
}

/** @brief 邮箱取件（主循环侧；取走并清零，原子） */
static int scen_req_take(void)
{
    portENTER_CRITICAL(&s_scen_mux);
    int req = s_scen_req;
    s_scen_req = -1;
    portEXIT_CRITICAL(&s_scen_mux);
    return req;
}

/**
 * @brief   UART0 RX 任务：行协议「SCEN <n>」→ 邮箱（不直接切换——切换在
 *          主循环上下文做，与 tick/打帧同任务串行，见文件头双运行时注）。
 * @note    输入按不可信处理：strtol 全串校验 + 值域 0..99，垃圾行整体忽略；
 *          100ms 读超时兜底空转；行溢出丢弃（等 '\n' 重新同步）。
 */
static void scen_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    char line[RX_LINE_MAX];
    int len = 0;
    for (;;) {
        int n = uart_read_bytes(UART_NUM_0, buf, sizeof buf,
                                pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                line[len] = '\0';
                if (strncmp(line, "SCEN ", 5) == 0) {
                    char *end = NULL;
                    long v = strtol(line + 5, &end, 10);
                    /* 全串数值 + 值域内才投递（0=demo 查询；1..count=切换） */
                    if (end != NULL && *end == '\0' && v >= 0 && v <= 99) {
                        scen_req_post((int)v);
                    }
                }
                len = 0;
            } else if (len < RX_LINE_MAX - 1) {
                line[len++] = c;
            } else {
                len = 0;   /* 溢出行整体丢弃，等下一个 '\n' 重新同步 */
            }
        }
    }
}

/**
 * @brief   执行场景切换（主循环上下文；失败留 -1 态等下一命令重试）
 * @note    结果经 TEXT 帧回报（人读，≤96B 构造）；机器判据 = 0x06 帧 scen_id。
 */
static void apply_scen_request(int id)
{
    char msg[96];   /* 最坏展开（含 %d 极值）< 96 ≤ TEXT payload 上限 */
    if (id == 0) {
        if (s_rt == 0) {
            send_text("SCEN OK 0 demo already active");
            return;
        }
        send_text("SCEN ERR 0: demo re-entry needs reboot (HAL once-only)");
        return;
    }
    if (id > motor_scen_def_count) {
        snprintf(msg, sizeof msg, "SCEN ERR id %d (valid 0..%d)",
                 id, motor_scen_def_count);
        send_text(msg);
        return;
    }
    if (s_rt == 0) {
        (void)motor_demo_deinit();   /* demo 让位：电机/plant/定时器销毁，HAL 留驻 */
    }
    if (motor_scen_start(id) != 0) {
        s_rt = -1;
        snprintf(msg, sizeof msg, "SCEN ERR %d start failed (console)", id);
        send_text(msg);
        return;
    }
    s_rt = 1;
    const motor_scen_def_t *d = motor_scen_def_get(id);
    if (d != NULL) {
        snprintf(msg, sizeof msg, "SCEN OK %d %s (%d slots, %d timers)",
                 d->id, d->name, d->n_slots, d->n_timer_presets);
        send_text(msg);
    } else {
        send_text("SCEN ERR def lookup inconsistent");
    }
}

void app_main(void)
{
    /* UART0 驱动（console 口）：RX 小缓冲兜 console 输入 + 场景命令；TX ring 0 =
     * uart_write_bytes 阻塞直推 FIFO。带宽如实：500Hz×68B(0x06 最大) = 34KB/s
     * ——QEMU espressif fork 的 UART 模型不模拟 TX 波特定时（chardev 直通
     * TCP），此速率可行；实机 115200 波特下该帧流会反压限流，实机移植需
     * 降帧率或提波特率（QEMU 演示不背此约束）。 */
    esp_err_t rc = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (rc == ESP_OK) {
        uart_vfs_dev_use_driver(UART_NUM_0);
    }
    /* 驱动安装失败不致命：帧仍走 uart_write_bytes 返回错误 → SEQ 缺口可见 */

    if (motor_demo_init() != 0) {
        send_text("FATAL motor_demo_init failed (see stderr/console)");
        return;
    }
    char hi[96];   /* 最坏展开（含 %d 极值）< 96 ≤ TEXT payload 上限 */
    snprintf(hi, sizeof hi,
             "OK motor demo: %d slots, %dHz poll (SCEN 1..%d: pwm/servo/enc/nested/dc)",
             MOTOR_DEMO_SLOTS, 1000000 / MOTOR_DEMO_POLL_US,
             motor_scen_def_count);
    send_text(hi);

    /* RX 任务：场景命令解析（阻塞于驱动读，勿扰控制环；见 scen_rx_task 注） */
    xTaskCreate(scen_rx_task, "scen_rx", 2048, NULL, 2, NULL);

    TickType_t next = xTaskGetTickCount();
    uint32_t now_us = 0;
    /* 两运行时共用遥测缓冲：容量 = max(demo 槽数, 场景槽上限)——demo 扩 4 槽
     * 后由 demo 侧定容量（场景 ≤3 槽恒在其内），防 demo_tick 越界写 */
#if MOTOR_DEMO_SLOTS > MOTOR_SCEN_MAX_SLOTS
    motor_demo_tele_t tele[MOTOR_DEMO_SLOTS];
#else
    motor_demo_tele_t tele[MOTOR_SCEN_MAX_SLOTS];
#endif

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(MOTOR_DEMO_POLL_US / 1000));
        now_us += MOTOR_DEMO_POLL_US;   /* 单调时基：tick 计数推演（零回退） */

        int req = scen_req_take();
        if (req >= 0) {
            apply_scen_request(req);
            continue;   /* 切换拍不发帧：下一拍起新运行时的干净帧流 */
        }

        if (s_rt == 0) {
            /* demo 运行时：0x05 帧（64B，4 槽；记录布局不变，帧长随槽数） */
            motor_demo_tick(now_us, tele);
            uint8_t payload[MOTOR_DEMO_SLOTS * 16];
            motor_demo_pack(tele, payload);   /* 布局单源在 motor_demo.c */
            send_frame(PROTO_TYPE_MOTOR, payload, sizeof payload);
        } else if (s_rt == 1) {
            /* 场景运行时：0x06 帧（场景头 + 定时器快照 + 电机记录） */
            int n = 0;
            motor_scen_tick(now_us, tele, &n);
            uint8_t payload[3 + 6 * MOTOR_SCEN_MAX_TIMERS
                              + 16 * MOTOR_SCEN_MAX_SLOTS];
            size_t plen = motor_scen_pack(tele, n, payload, sizeof payload);
            if (plen > 0) {
                send_frame(PROTO_TYPE_SCEN, payload, (uint8_t)plen);
            }
        }
        /* s_rt == -1：失败态无帧流（TEXT 已报因；等下一 SCEN 命令重试恢复） */
    }
}
