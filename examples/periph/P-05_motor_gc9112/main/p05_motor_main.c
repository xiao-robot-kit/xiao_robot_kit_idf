/* SPDX-License-Identifier: MIT */
/**
 * @file    p05_motor_main.c
 * @brief   P-05 motor_gc9112：双路 GC9112 电机驱动例程（QEMU 文本遥测版）
 *
 * @note    类目：外设例程（periph）——有限序列演示 + 事件回放（P-02 模式）：
 *          序列 = M1/M2 各自 正转(+50%)→反转(−50%)→停(0)→调速(±25/50/75/100% 扫
 *          6 档) → DONE；完成态每 1s 重放 boot + 全部事件 + DONE（探针晚连可观测）。
 * @note    输出契约 = 组件 API：motor.dc.generic 预设（MOTOR_DC：target=带符号
 *          归一化占空 −1..+1，符号=桥向）；桥输出回读 sim_pwm_bridge_duty（物理
 *          输出面 = 合成 PWM 存储），断言两臂恰一非零（H 桥直通零容忍）。
 * @note    双宏契约：P_SIM_MOTOR=1 合成 HAL（唯一实现）；置 0 = 构建期 #error
 *          （真板 LEDC HAL 未提供）。EXP_QEMU_UART=1 → UART0；0 → USB-Serial-JTAG。
 * @note    真板未验证（任务卡标注）：D1/D2=M1 IN1/IN2、D19/D18=M2 IN1/IN2，
 *          GC9112 单 H 桥，20kHz 载波，MOTOR_5V 电源轨（与舵机共轨，见任务书 5.3）。
 * @note    观察点：每档 = 目标占空 → poll 驱动桥输出 → sim 回读两臂；调速扫档
 *          每股记录 (duty, rpm_model)——rpm 合成：duty×3600（12V N20 全速示例），
 *          真板上该曲线由 P-06 encoder 实测替换（本为例程演示，rpm 标注 model）。
 * @note    事件回放（S_REPLAY_MAX=32×80）：boot 行 + 每档 1 行 + CURVE + DONE；
 *          完成态每 1s 重放全部——QEMU TCP 探针何时连上都拿得到全序列。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "motor.h"
#include "motor_hal_sim.h"

#if !P_SIM_MOTOR
#error "P-05: 真硬件 LEDC 电机 HAL 未提供（本例程输出面 = motor_hal_sim 合成存储）"
#endif

/* ── 输出面：EXP_QEMU_UART=1 → UART0（QEMU 第一 serial 槽）；0 → USB-Serial-JTAG ── */
#if EXP_QEMU_UART
#define P05_OUT_UART  UART_NUM_0
#else
#define P05_OUT_UART  UART_NUM_0
#endif

/* ── 例程常量（参数零写死，全在此段） ───────────────────────────────────── */
#define POLL_MS       2          /* 500Hz 控制环 */
#define STEP_MS       1200       /* 每档停留 1.2s */
#define DUTY_MAX      1.0f       /* 桥占空上限 */
#define N_SLOTS       2          /* M1 / M2 */
#define N_CURVE       3          /* 调速扫档数/电机 */
#define RPM_MODEL_FULL 3600.0f   /* duty=1 的模型转速 rpm（真板待 P-06 实测替换） */

/* ── 场景表（正转→反转→停→调速扫档，两电机同表但方向可反向） ────────────── */
typedef struct {
    float   target;      /* 带符号占空（MOTOR_DC 域）；0=停 */
    const char *label;   /* 事件标签（回放行用） */
} step_t;

static const step_t s_track[] = {
    {  0.50f, "fwd+50" },
    { -0.50f, "rev-50" },
    {  0.00f, "stop" },
    {  0.25f, "spd+25" },
    {  0.50f, "spd+50" },
    {  0.75f, "spd+75" },
    { -0.75f, "spd-75" },
    {  0.75f, "spd+75b" },
    { -1.00f, "spd-100" },
    {  1.00f, "spd+100" },
};
#define N_TRACK (int)(sizeof(s_track) / sizeof(s_track[0]))

/* 曲线采样点（调速扫档：duty 0.25/0.50/0.75/1.00 的 model rpm） */
static const float s_curve_duty[N_CURVE] = { 0.25f, 0.50f, 0.75f };

/* ── 槽描述（wiring：真板 D1/D2 与 D19/D18；sim 档引脚仅签入，无硬件语义） ── */
typedef struct {
    int   pin_a;        /* GC9112 IN1（真板） */
    int   pin_b;        /* GC9112 IN2（真板） */
    int   sign;         /* +1 正向 / −1 反向（M2 反转方向演示） */
} slot_cfg_t;
static const slot_cfg_t s_slots[N_SLOTS] = {
    { 1,  2,  +1 },   /* M1: D1/D2 */
    { 19, 18, -1 },   /* M2: D19/D18 */
};

/* ── 运行时状态 ── */
static pwm_timer_t *s_timer;
static motor_t     *s_motors[N_SLOTS];
static uint8_t      s_ch[N_SLOTS];        /* 桥正向通道（回读键） */
static int          s_dir[N_SLOTS];       /* 槽方向符号（回读签名） */

/* ── 事件回放缓冲（P-02 模式：完成态每 1s 重放全序列） ─────────────────────── */
#define S_REPLAY_MAX 32
#define S_REPLAY_LEN 80
static char s_replay[S_REPLAY_MAX][S_REPLAY_LEN];
static int  s_replay_n = 0;
static void replay_add(const char *line)
{
    if (s_replay_n < S_REPLAY_MAX) {
        snprintf(s_replay[s_replay_n], S_REPLAY_LEN, "%s", line);
        s_replay_n++;
    }
}

/* ── 输出（统一走 HOST 串口；放行首打 = boot 行先入缓冲再打） ────────────────── */
static void app_puts(const char *s)
{
    uart_write_bytes(P05_OUT_UART, s, strlen(s));
}

/* ── 装配（P-00 同款：display 先断言核心，失败即停不静默） ─────────────────── */
static int p05_init(void)
{
    /* 输出口（QEMU 档 UART0 安装驱动；真板 USB 由 app_puts 直推） */
#if EXP_QEMU_UART
    esp_err_t rc = uart_driver_install(P05_OUT_UART, 256, 0, 0, NULL, 0);
    if (rc != ESP_OK) return -1;
    uart_vfs_dev_use_driver(P05_OUT_UART);
#endif

    /* 注册合成 PWM HAL（进程级一次） */
    if (pwm_hal_register(&sim_pwm_hal) != 0) return -1;

    /* 定时器：20kHz / 4 通道（两电机 × 桥对 2 通道） */
    s_timer = pwm_timer_create(20000, 4);
    if (s_timer == NULL) return -1;

    /* 双电机（motor.dc.generic 底版；channel 成对占用 ch/ch+1） */
    for (int i = 0; i < N_SLOTS; i++) {
        const motor_cfg_t *base = motor_cfg_find("motor.dc.generic");
        if (base == NULL) return -1;
        uint8_t ch = (uint8_t)(i * 2);
        motor_cfg_t cfg = *base;
        cfg.timer  = s_timer;
        cfg.channel = ch;
        cfg.pin_a  = s_slots[i].pin_a;
        cfg.pin_b  = s_slots[i].pin_b;
        motor_t *m = motor_create(&cfg);
        if (m == NULL) return -1;
        s_motors[i] = m;
        s_ch[i]     = ch;
        s_dir[i]    = s_slots[i].sign;
    }
    return 0;
}

/* ── 每档执行：set_target → poll 一拍输出 → 回读桥两臂（直通零容忍断言） ────── */
static void run_step(int slot, const step_t *st, uint32_t now_us, char *out, int out_len)
{
    motor_t *m = s_motors[slot];
    float tgt = st->target * (float)s_dir[slot];
    motor_set_target(m, tgt);
    /* 档内持续 poll（受控推进；s_curve 每档末读一次模型转速） */
    uint32_t end = now_us + STEP_MS * 1000;
    float rpm_model = 0.0f;
    while (now_us < end) {
        motor_poll(m, now_us);
        now_us += POLL_MS * 1000;
    }
    /* 回读桥输出（物理输出面） */
    sim_bridge_duty_t bd = {0};
    if (sim_pwm_bridge_duty(s_timer, s_ch[slot], (uint8_t)(s_ch[slot] + 1), &bd) == 0) {
        rpm_model = bd.signed_duty * (float)s_dir[slot] * RPM_MODEL_FULL;
        if (rpm_model < 0) rpm_model = -rpm_model;
    }
    const char *name = (slot == 0) ? "M1" : "M2";
    snprintf(out, (size_t)out_len, "MOT %s %s duty=%+.2f rpm=%d (sim)", name,
             st->label, tgt, (int)rpm_model);
}

void app_main(void)
{
    if (p05_init() != 0) {
        uart_write_bytes(P05_OUT_UART, "FATAL P-05 init failed\r\n", 25);
        return;
    }
    char line[96];
    const char *boot = "OK boot P-05_motor_gc9112 (M1D1/2 M2D19/18 sim)";
    app_puts(boot); app_puts("\r\n");
    replay_add(boot);

    /* 双电机同步推进（同一时基；两电机每条 track 各自跑一遍） */
    uint32_t t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t now_us = t0 * 1000;
    for (int s = 0; s < N_SLOTS; s++) {
        for (int k = 0; k < N_TRACK; k++) {
            char ev[S_REPLAY_LEN];
            run_step(s, &s_track[k], now_us + k * (STEP_MS * 1000), ev, sizeof ev);
            snprintf(line, sizeof line, "%s\r\n", ev);
            app_puts(line);
            replay_add(ev);
        }
    }
    /* 调速曲线（每电机 3 点：duty→model rpm） */
    for (int s = 0; s < N_SLOTS; s++) {
        for (int c = 0; c < N_CURVE; c++) {
            int rpm = (int)(s_curve_duty[c] * RPM_MODEL_FULL);
            snprintf(line, sizeof line, "CURVE %s duty=%.2f rpm=%d (model)\r\n",
                     (s == 0) ? "M1" : "M2", s_curve_duty[c], rpm);
            app_puts(line);
            replay_add(line);
        }
    }
    snprintf(line, sizeof line, "MOT DONE m1=%d m2=%d\r\n", N_TRACK, N_TRACK);
    app_puts(line);
    replay_add(line);

    /* 完成态：每 1s 重放全部事件（探针晚连可观测） */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (int i = 0; i < s_replay_n; i++) {
            app_puts(s_replay[i]);
            app_puts("\r\n");
        }
    }
}