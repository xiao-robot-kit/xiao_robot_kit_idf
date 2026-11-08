/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_servo_main.c
 * @brief   实验电机·单实例三：舵机五点角度标定（QEMU 演示固件）
 *
 * @note    链路：电机库(the `xiao_motor` component) + 合成 PWM HAL
 *          (examples/_shared/motor_hal_sim.c) → 500Hz 控制环 → 0x05 帧协议
 *          → UART0(console) → QEMU -serial tcp → host 探针
 *          (testing/qemu/probe_sim.py --expect-type 0x05)。
 * @note    控制台 = UART0（sdkconfig.defaults，QEMU 第一 serial 槽；ROM 启动
 *          横幅与帧流同端口，探针按 SOF+CRC8 重同步）。USB-Serial-JTAG 在
 *          QEMU esp32s3 无路由（exp1 教训），本实验显式走 UART0。
 * @note    场景固化：无 SCEN 选择器——exp_motor_qemu 综合演示的场景 2 单例
 *          工程化（用户定案：电机五实例展开）；五场景切换面看综合演示工程。
 * @note    双宏契约（ERR-004 铁律）：本实验仅提供合成 HAL 与 UART0 出口
 *          （EXP_SIM / EXP_QEMU_UART 恒 1，main/CMakeLists.txt 默认即此）；
 *          置 0 = 构建期 #error 显式拒绝——不静默编出「假硬件」变体。
 * @note    TEXT 帧消息限长：payload ≤ PROTO_PAYLOAD_MAX(96B)，本文件字面量
 *          按 ≤96B 构造并留裕量。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"       /* 任务卡指定输出面：uart_write_bytes → UART_NUM_0 */
#include "driver/uart_vfs.h"   /* console VFS 并入同一驱动：两条写路径串行不交错 */
#include "proto/frame.h"
#include "exp_motor_servo_scen.h"

#if !EXP_SIM
#error "exp_motor_servo: 真硬件电机 HAL 未提供（本实验输出面 = motor_hal_sim 合成存储）"
#endif
#if !EXP_QEMU_UART
#error "exp_motor_servo: 帧出口仅实现 UART0（EXP_QEMU_UART=1）"
#endif

static uint8_t s_seq = 0;

/**
 * @brief 帧直写 UART0（同 exp_motor_qemu：console 与帧流同口，驱动安装后
 *        uart_vfs_dev_use_driver 并入同一 TX 通道，无字节级交错）。
 */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n) {
        int w = uart_write_bytes(UART_NUM_0, (const char *)buf, n);
        (void)w;   /* 写失败无补救面：不中断控制环（SEQ 缺口可检丢帧） */
    }
}

static void send_text(const char *msg)
{
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)strlen(msg));
}

void app_main(void)
{
    /* UART0 驱动（console 口）：TX ring 0 = uart_write_bytes 阻塞直推 FIFO。
     * 带宽如实：500Hz × 21B(0x05 单槽) ≈ 10.5KB/s——QEMU UART 模型 chardev
     * 直通可行；实机 115200 波特下须降帧率或提波特率（QEMU 演示不背此约束）。 */
    esp_err_t rc = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (rc == ESP_OK) {
        uart_vfs_dev_use_driver(UART_NUM_0);
    }
    /* 驱动安装失败不致命：帧仍走 uart_write_bytes 返回错误 → SEQ 缺口可见 */

    if (exp_motor_servo_scen_init() != 0) {
        send_text("FATAL exp_motor_servo scen init failed (console)");
        return;
    }
    char hi[96];   /* 最坏展开（含 %d 极值）< 96 ≤ TEXT payload 上限 */
    snprintf(hi, sizeof hi,
             "OK exp_motor_servo: 1 slot servo, %dHz poll (5-point 0/45/90/135/180deg, 2s/pt)",
             1000000 / EXP_MOTOR_SERVO_POLL_US);
    send_text(hi);

    TickType_t next = xTaskGetTickCount();
    uint32_t now_us = 0;
    exp_motor_tele_t tele[EXP_MOTOR_SERVO_SLOTS];

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(EXP_MOTOR_SERVO_POLL_US / 1000));
        now_us += EXP_MOTOR_SERVO_POLL_US;   /* 单调时基：tick 计数推演（零回退） */

        exp_motor_servo_scen_tick(now_us, tele);
        uint8_t payload[EXP_MOTOR_SERVO_SLOTS * 16];
        exp_motor_servo_scen_pack(tele, payload);
        send_frame(PROTO_TYPE_MOTOR, payload, sizeof payload);
    }
}
