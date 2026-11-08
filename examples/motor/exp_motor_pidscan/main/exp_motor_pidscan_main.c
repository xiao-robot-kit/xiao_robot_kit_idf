/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_pidscan_main.c
 * @brief   实验电机·单实例五：PID 参数网格扫描（3×3=9 组阶跃响应对比表，
 *          QEMU 演示固件）
 *
 * @note    链路：电机库(the `xiao_motor` component，速度环 PID) + 合成
 *          编码器 plant HAL (examples/_shared/motor_hal_sim.c) → 500Hz 控制环 →
 *          0x05 帧（slot=组号）+ TEXT 汇总帧（对比表逐行，最优 '*' 标注）
 *          → UART0(console) → QEMU -serial tcp → host 查看
 *          (xiao_robot_kit_host backend/scope_pidscan.py；Embedded 验收走 probe
 *          testing/qemu/probe_sim.py --expect-type 0x05)。
 * @note    控制台 = UART0（sdkconfig.defaults，QEMU 第一 serial 槽；ROM 启动
 *          横幅与帧流同端口，探针按 SOF+CRC8 重同步；USB-Serial-JTAG 在 QEMU
 *          esp32s3 无路由——exp1 教训，本实验显式走 UART0）。
 * @note    扫描循环：9 组 × 3s（低段 1s 前导 + 高段 2s 统计窗）= 27s 一轮；
 *          轮末发对比表（表头 + 9 组行 + BEST + END，12 行 TEXT ≤ probe
 *          TEXT 保留上限 16 行），随后无限循环重扫——host 后连亦能见整表。
 * @note    双宏契约（ERR-004 铁律）：本实验仅提供合成 HAL 与 UART0 出口
 *          （EXP_SIM / EXP_QEMU_UART 恒 1，main/CMakeLists.txt 默认即此）；
 *          置 0 = 构建期 #error 显式拒绝——不静默编出「假硬件」变体。
 * @note    TEXT 帧消息限长：payload ≤ PROTO_PAYLOAD_MAX(96B)，本文件字面量
 *          按 ≤96B 构造并留裕量（组行 ~45B、BEST 行 ~40B）。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"       /* 任务卡指定输出面：uart_write_bytes → UART_NUM_0 */
#include "driver/uart_vfs.h"   /* console VFS 并入同一驱动：两条写路径串行不交错 */
#include "proto/frame.h"
#include "exp_motor_pidscan_scen.h"

#if !EXP_SIM
#error "exp_motor_pidscan: 真硬件电机/编码器 HAL 未提供（本实验输出面 = motor_hal_sim 合成存储）"
#endif
#if !EXP_QEMU_UART
#error "exp_motor_pidscan: 帧出口仅实现 UART0（EXP_QEMU_UART=1）"
#endif

static uint8_t s_seq = 0;

/**
 * @brief 帧直写 UART0（同 exp_motor_enc：console 与帧流同口，驱动安装后
 *        uart_vfs_dev_use_driver 并入同一 TX 通道，无字节级交错）。
 */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n) {
        int w = uart_write_bytes(UART_NUM_0, (const char *)buf, n);
        (void)w;   /* 写失败无补救面：不中断扫描（SEQ 缺口可检丢帧） */
    }
}

static void send_text(const char *msg)
{
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)strlen(msg));
}

void app_main(void)
{
    /* UART0 驱动（console 口）：TX ring 0 = uart_write_bytes 阻塞直推 FIFO。
     * 带宽如实：500Hz × 21B(0x05 单槽) ≈ 10.5KB/s（同 enc 实验），QEMU
     * UART 模型 chardev 直通可行。 */
    esp_err_t rc = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (rc == ESP_OK) {
        uart_vfs_dev_use_driver(UART_NUM_0);
    }
    /* 驱动安装失败不致命：帧仍走 uart_write_bytes 返回错误 → SEQ 缺口可见 */

    if (exp_motor_pidscan_scen_init() != 0) {
        send_text("FATAL exp_motor_pidscan scen init failed (console)");
        return;
    }
    const uint32_t group_ticks = exp_motor_pidscan_group_ticks();
    char hi[96];   /* 最坏展开（含 %lu 极值）< 96 ≤ TEXT payload 上限 */
    snprintf(hi, sizeof hi,
             "OK exp_motor_pidscan: 3x3 grid, %d groups x %lu ticks, %dHz poll",
             EXP_MOTOR_PIDSCAN_GROUPS, (unsigned long)group_ticks,
             1000000 / EXP_MOTOR_PIDSCAN_POLL_US);
    send_text(hi);

    exp_motor_tele_t tele;
    TickType_t next = xTaskGetTickCount();
    uint32_t now_us = 0;

    for (;;) {   /* 轮循环：无限重扫（host 后连亦见整表；设计一轮 = 9 组 × 3s，
                  * QEMU esp32s3 实测拍率 ~231Hz → 实际 ~60s，host 采集窗按此留量） */
        char line[96];   /* 组行最坏 ~45B + BEST ~40B（summary_line 契约） */
        char prog[96];   /* 进度行 = "# " + 组行（流式进度，不带最优标注） */
        size_t n = exp_motor_pidscan_header_line(line, sizeof line);
        if (n > 0 && n < sizeof line) send_text(line);

        for (int g = 0; g < EXP_MOTOR_PIDSCAN_GROUPS; g++) {
            if (exp_motor_pidscan_group_begin(g) != 0) {
                send_text("FATAL exp_motor_pidscan group_begin failed (console)");
                return;
            }
            for (uint32_t t = 0; t < group_ticks; t++) {
                vTaskDelayUntil(&next,
                                pdMS_TO_TICKS(EXP_MOTOR_PIDSCAN_POLL_US / 1000));
                now_us += EXP_MOTOR_PIDSCAN_POLL_US;   /* 单调时基（零回退） */

                exp_motor_pidscan_scen_tick(now_us, &tele);
                uint8_t payload[16];
                exp_motor_pidscan_scen_pack(&tele, payload);
                send_frame(PROTO_TYPE_MOTOR, payload, sizeof payload);
            }
            exp_pid_result_t r;
            (void)exp_motor_pidscan_group_end(g, &r);
            /* 流式进度行（'#' 前缀，无最优标注）：此时网格只完成一部分，
             * best_group 只见部分结果——逐组行不携带 '*'（QEMU 实跑暴露：
             * 流式行带部分最优标注会让整表出现多个 '*'，违反唯一标注契约） */
            n = exp_motor_pidscan_summary_line(g, line, sizeof line, 0);
            if (n > 0 && n + 2 < sizeof prog) {
                prog[0] = '#'; prog[1] = ' ';
                memcpy(prog + 2, line, n + 1);
                send_text(prog);
            }
        }

        /* 轮末完整对比表：9 组结果全齐，best_group 全表定案——唯一 '*' */
        for (int g = 0; g < EXP_MOTOR_PIDSCAN_GROUPS; g++) {
            n = exp_motor_pidscan_summary_line(g, line, sizeof line, 1);
            if (n > 0 && n < sizeof line) send_text(line);
        }

        int best = exp_motor_pidscan_best_group();
        if (best >= 0) {
            exp_pid_result_t r;
            exp_motor_pidscan_get_result(best, &r);
            snprintf(line, sizeof line,
                     "BEST G%d score%.3fs (st+ov%%/100+se%%/100 min)",
                     best, r.score);
        } else {
            snprintf(line, sizeof line, "BEST none (no group converged)");
        }
        send_text(line);
        send_text("PIDSCAN round done 9 groups");
    }
}
