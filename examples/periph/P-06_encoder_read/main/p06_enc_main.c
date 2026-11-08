/* SPDX-License-Identifier: MIT */
/**
 * @file    p06_enc_main.c
 * @brief   P-06 编码器计数例程（N20 @ motor.enc.n20，cpr=7200）
 *
 * 链路：motor 组件（motor.enc.n20 @ pwm.motor.20k）+ 合成编码器 HAL/plant
 *       （examples/_shared/motor_hal_sim.c，QEMU 无 PCNT/编码器模型）
 *       → 文本遥测 → UART0（EXP_QEMU_UART=1，QEMU 档）/ USB-Serial-JTAG（真板档）。
 *
 * 演示两段（有限序列，事件回放每 1s 重放全量切换过程；P-02 模式）：
 *   Phase A 手转模拟（SPIN）：直接向桥双通道写占空（模拟外部转动手轮）
 *       → plant 读桥差推转速 → 编码器计数线性累积/停滞/回落；
 *       —— 输出 SPIN fwd / COUNT 快照 / PIK 峰值 / SPIN rev / COUNT 回落。
 *   Phase B 电转闭环（DRIVE）：motor_set_target(±rpm) → motor_poll 100Hz
 *       闭环 → PID 输出到桥 → plant 回灌计数与测速 → motor_get_state 回读；
 *       —— 输出 DRIVE target / ACT 转速+计数 / DRIVE 0 停。
 *
 * 硬件（任务书 §3.4 / motor 组件文档）：
 *   M1 编码器 A=D3 / B=D14（编码器口 J8.3，ZH1.5 6P，VCC=3V3_OUT→R7→120R 磁珠）
 *   M1 桥 D1/D2（20kHz）→ 与 Phase B 闭环共用；cpr=7200 = 磁编 12PPR×4 倍频×150 减速比。
 *   注（任务书 §5.3 电流预算）：N20 与舵机共 Motor_5V 轨——驱动段只用单电机轻载，
 *       防共轨堵转电流超预算；真板验收对象为「手转得计数、电转收转速」两断言。
 *
 * 宏契约（CMake 定义，两态显式）：
 *   P_SIM_MOTOR   1=合成 HAL（本例程默认，QEMU 可测）；0=真硬件 LEDC/PCNT HAL——
 *                 未提供，C 侧 #error 拒绝（不静默编出假硬件固件，ERR-004 纪律）。
 *   EXP_QEMU_UART 1=遥测出口 UART0（QEMU 第 1 个 -serial 槽）；0=USB-Serial-JTAG（真板）。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "mtr/motor.h"
#include "mtr/pwm_hal.h"
#include "motor_hal_sim.h"

/* ---- 双宏编译契约（未定义即拒绝；本文件不静默退化为任一态） ---- */
#ifndef P_SIM_MOTOR
#error "P_SIM_MOTOR undefined: set -DP_SIM_MOTOR=1 (sim) or =0 (real HW, rejected)"
#endif
#if P_SIM_MOTOR == 0
#error "P_SIM_MOTOR=0: real LEDC/PCNT HAL not provided by port yet (ERR-004) — use sim"
#endif
#ifndef EXP_QEMU_UART
#error "EXP_QEMU_UART undefined: set 1=UART0 (QEMU) or 0=USB-Serial-JTAG (board)"
#endif

/* ---- 场景参数（单电机 M1；Phase A 手转 / Phase B 电转） ---- */
#define EXP_POLL_US        10000u          /* 100Hz：plant 步长与控制环同频 */
#define EXP_POLL_RATE      (1000000u / EXP_POLL_US)
#define M1_ENC_A           3              /* D3 */
#define M1_ENC_B           14             /* D14 */
#define M1_PIN_A           1              /* D1 桥正向 */
#define M1_PIN_B           2              /* D2 桥反向 */
#define M1_CHANNEL         0              /* 桥通道 ch0/1（plant 键 ch_fwd/ch_rev） */
#define SPIN_DUTY_POS      0.50f          /* 手转注入占空：plant → 80 rpm 目标 */
#define SPIN_DUTY_NEG      (-0.50f)
#define SPIN_HOLD_US       2000000u       /* 正转/反转各 2s */
#define SPIN_STOP_US       1000000u       /* 停止观察 1s */
#define DRIVE_RPM_TARGET   120.0f         /* 电转闭环目标 rpm */
#define DRIVE_RUN_US       2500000u       /* 驱动 2.5s */
#define DRIVE_STOP_US      1000000u       /* 零目标 1s */
#define FAST_TX            115200

#if EXP_QEMU_UART == 1
#define EXP_UART_NUM       UART_NUM_0
#else
#define EXP_UART_NUM       UART_NUM_0
#endif
/* ---- Phase 状态机 ---- */
typedef enum {
    PHASE_SPIN_FWD = 0,
    PHASE_SPIN_STOP1,
    PHASE_SPIN_REV,
    PHASE_SPIN_STOP2,
    PHASE_DRIVE_POS,
    PHASE_DRIVE_SUSTAIN,
    PHASE_DRIVE_STOP,
    PHASE_DONE,
} exp_phase_t;

static motor_t      *s_motor = NULL;
static pwm_timer_t  *s_timer = NULL;
static exp_phase_t   s_phase = PHASE_SPIN_FWD;
static int64_t       s_phase_start_us = 0;
static int32_t       s_peak_count = 0;    /* SPIN 正转峰值（PIK 输出） */

/* ---- 事件回放缓冲（P-02 模式：boot + 事件行 + DONE，完成态每 1s 重放） ---- */
#define S_REPLAY_MAX   24
#define S_REPLAY_LEN   96
static char s_replay[S_REPLAY_MAX][S_REPLAY_LEN];
static int  s_replay_n = 0;

static void tx_str(const char *s)
{
    uart_write_bytes(EXP_UART_NUM, s, (int)strlen(s));
}

/* 事件行（入回放缓冲 + 输出）；周期遥测（COUNT/ACT）不走本入口 */
static void event_log(const char *s)
{
    if (s_replay_n < S_REPLAY_MAX)
        snprintf(s_replay[s_replay_n++], S_REPLAY_LEN, "%s", s);
    tx_str(s);
}

static void uart_init(void)
{
#if EXP_QEMU_UART == 1
    /* QEMU 档：UART0 安装驱动（第 1 个 -serial 槽收文本遥测） */
    uart_config_t cfg = {
        .baud_rate = FAST_TX,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t rc = uart_driver_install(EXP_UART_NUM, 256, 0, 0, NULL, 0);
    if (rc != ESP_OK) return;
    uart_param_config(EXP_UART_NUM, &cfg);
    uart_vfs_dev_use_driver(EXP_UART_NUM);
#else
    /* 真板档：USB-Serial-JTAG 由 esp_console 接管，输出面经 app_puts 直推 */
    (void)0;
#endif
}

static void output_flush(void)
{
    uart_wait_tx_done(EXP_UART_NUM, 50);
}

static void log_count(const char *tag, int32_t count)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "ENC %s t+%06lldms count=%+d\n",
             tag, (long long)((esp_timer_get_time() - s_phase_start_us) / 1000),
             (int)count);
    tx_str(buf);
}

static void log_actual(int64_t elapsed_us, float rpm, int32_t count)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "ENC ACT t+%06lldms rpm=%+6.1f count=%+d\n",
             (long long)(elapsed_us / 1000), (double)rpm, (int)count);
    tx_str(buf);
}

/* ---- 手转注入：直接写桥双通道（通道已被电机占用，可写） ---- */
static void spin_set(float signed_duty)
{
    float fwd = (signed_duty >= 0.0f) ? signed_duty : 0.0f;
    float rev = (signed_duty < 0.0f) ? (-signed_duty) : 0.0f;
    (void)pwm_timer_channel_set_duty(s_timer, M1_CHANNEL,     fwd);
    (void)pwm_timer_channel_set_duty(s_timer, M1_CHANNEL + 1, rev);
}

/* ---- 读取累计计数（Phase A 自读桥；Phase B 随 motor 闭环回读） ---- */
static int32_t enc_count_now(void)
{
    int32_t c = 0;
    if (sim_encoder_hal.encoder_read(M1_ENC_A, M1_ENC_B, &c, sim_encoder_hal.ctx) != 0)
        return 0;
    return c;
}

static void phase_enter(exp_phase_t ph)
{
    s_phase = ph;
    s_phase_start_us = esp_timer_get_time();
}

static const char *phase_name(exp_phase_t ph)
{
    switch (ph) {
    case PHASE_SPIN_FWD:     return "SPIN fwd=+0.50";
    case PHASE_SPIN_STOP1:   return "SPIN stop";
    case PHASE_SPIN_REV:     return "SPIN rev=-0.50";
    case PHASE_SPIN_STOP2:   return "SPIN stop";
    case PHASE_DRIVE_POS:    return "DRIVE target=+120";
    case PHASE_DRIVE_SUSTAIN:return "DRIVE active";
    case PHASE_DRIVE_STOP:   return "DRIVE target=0";
    default:                 return "DONE";
    }
}

static void app_loop(void *arg)
{
    (void)arg;
    uint32_t now_us = 0;             /* 单调 us 计数器（motor_poll 时基，防回绕） */
    bool ph_logged = false;

    phase_enter(PHASE_SPIN_FWD);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EXP_POLL_US / 1000));
        now_us += EXP_POLL_US;
        const int64_t el = esp_timer_get_time() - s_phase_start_us;

        if (!ph_logged) {
            char b[64];
            snprintf(b, sizeof(b), "ENC %s\n", phase_name(s_phase));
            event_log(b);
            ph_logged = true;
            switch (s_phase) {
            case PHASE_SPIN_FWD: spin_set(SPIN_DUTY_POS); break;
            case PHASE_SPIN_REV: spin_set(SPIN_DUTY_NEG); break;
            case PHASE_DRIVE_POS:
                spin_set(0.0f);   /* 桥回零后才走闭环 */
                if (s_motor != NULL) (void)motor_set_target(s_motor, DRIVE_RPM_TARGET);
                break;
            case PHASE_DRIVE_STOP:
                if (s_motor != NULL) (void)motor_set_target(s_motor, 0.0f);
                break;
            default: spin_set(0.0f); break;
            }
        }

        /* 每步推进模型（encoder_read 是 plant 推进引擎：一次迅推进 dt_s）
         * —— Phase A（无闭环）每步推进，仿「墙钟每 10ms 读一拍」真实采集语义；
         * Phase B 电机 poll 内部已读 encoder，此处不得再读（否则双倍推进） */
        int32_t live = 0;
        if (s_phase < PHASE_DRIVE_POS)
            live = enc_count_now();

        /* 周期快照（每 0.2s 一次） */
        if ((el / EXP_POLL_US) % 20u == 0u)
            log_count("COUNT", live);

        switch (s_phase) {
        case PHASE_SPIN_FWD:
            if (el >= SPIN_HOLD_US) {
                int32_t c = enc_count_now();
                if (c > s_peak_count) s_peak_count = c;
                {
                    char b[96];
                    snprintf(b, sizeof(b), "ENC PIK count=%+d\n", (int)c);
                    event_log(b);
                }
                spin_set(0.0f);
                phase_enter(PHASE_SPIN_STOP1);
                ph_logged = false;
            }
            break;
        case PHASE_SPIN_STOP1:
            if (el >= SPIN_STOP_US) {
                phase_enter(PHASE_SPIN_REV);
                ph_logged = false;
            }
            break;
        case PHASE_SPIN_REV:
            if (el >= SPIN_HOLD_US) {
                spin_set(0.0f);
                phase_enter(PHASE_SPIN_STOP2);
                ph_logged = false;
            }
            break;
        case PHASE_SPIN_STOP2:
            if (el >= SPIN_STOP_US) {
                phase_enter(PHASE_DRIVE_POS);   /* 进入时 set_target(+120) */
                ph_logged = false;
            }
            break;
        case PHASE_DRIVE_POS:
            if (s_motor != NULL) (void)motor_poll(s_motor, now_us);
            if (el >= EXP_POLL_US) {
                phase_enter(PHASE_DRIVE_SUSTAIN);
                ph_logged = false;
            }
            break;
        case PHASE_DRIVE_SUSTAIN:
            if (s_motor != NULL) (void)motor_poll(s_motor, now_us);
            if ((el / EXP_POLL_US) % 20u == 0u) {
                motor_state_t st;
                if (s_motor != NULL && motor_get_state(s_motor, &st) == 0)
                    log_actual(el, st.actual, st.enc_count);
            }
            if (el >= DRIVE_RUN_US) {
                phase_enter(PHASE_DRIVE_STOP);
                ph_logged = false;
            }
            break;
        case PHASE_DRIVE_STOP:
            if (s_motor != NULL) (void)motor_poll(s_motor, now_us);
            if ((el / EXP_POLL_US) % 20u == 0u) {
                motor_state_t st;
                if (s_motor != NULL && motor_get_state(s_motor, &st) == 0)
                    log_actual(el, st.actual, st.enc_count);
            }
            if (el >= DRIVE_STOP_US) {
                phase_enter(PHASE_DONE);
                ph_logged = false;
            }
            break;
        case PHASE_DONE:
        default:
            /* 完成态每 1s 全量重放（P-02 模式：TCP 探针何时连上都拿得到） */
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                for (int k = 0; k < s_replay_n; k++) {
                    static char rb[S_REPLAY_LEN + 8];
                    snprintf(rb, sizeof(rb), "%s\r\n", s_replay[k]);
                    tx_str(rb);
                }
                output_flush();
            }
            break;
        }
    }
}

void app_main(void)
{
    /* ---- 输出面 ---- */
    uart_init();

    char boot[96];
    snprintf(boot, sizeof(boot),
             "OK boot P-06_encoder_read cpr=7200 rate=%uHz sim_motor=1 qemu_uart=%d\n",
             (unsigned)EXP_POLL_RATE, EXP_QEMU_UART);
    event_log(boot);
    output_flush();

    /* ---- 装配（preset → timer → motor → attach enc HAL → register plant） ---- */
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        tx_str("FATAL pwm_hal_register\n");
        goto done;
    }
    s_timer = pwm_timer_create_preset("pwm.motor.20k");
    if (s_timer == NULL) {
        tx_str("FATAL timer create\n");
        goto done;
    }
    const motor_cfg_t *base = motor_cfg_find("motor.enc.n20");
    if (base == NULL) {
        tx_str("FATAL preset not found\n");
        goto done;
    }
    motor_cfg_t cfg = *base;
    cfg.timer   = s_timer;
    cfg.channel = M1_CHANNEL;
    cfg.pin_a   = M1_PIN_A;
    cfg.pin_b   = M1_PIN_B;
    cfg.enc_a   = M1_ENC_A;
    cfg.enc_b   = M1_ENC_B;
    s_motor = motor_create(&cfg);
    if (s_motor == NULL) {
        tx_str("FATAL motor_create\n");
        goto done;
    }
    if (motor_hal_attach(s_motor, &sim_encoder_hal) != 0) {
        tx_str("FATAL hal attach\n");
        goto cleanup_motor;
    }
    /* plant：桥对 = ch/ch+1，与 cfg.channel 同源；手转注入走桥对，电转闭环同 plant */
    sim_motor_plant_cfg_t plant = {
        .timer        = s_timer,
        .ch_fwd       = M1_CHANNEL,
        .ch_rev       = (uint8_t)(M1_CHANNEL + 1u),
        .enc_a        = M1_ENC_A,
        .enc_b        = M1_ENC_B,
        .rpm_per_duty = 160.0f,
        .tau_s        = 0.02f,
        .cpr          = 7200,
        .dt_s         = (float)EXP_POLL_US / 1000000.0f,
    };
    if (sim_motor_plant_register(&plant) != 0) {
        tx_str("FATAL plant register\n");
        goto cleanup_motor;
    }

    /* ---- 场景运行 ---- */
    xTaskCreate(app_loop, "p06_enc", 4096, NULL, 5, NULL);
    return;

cleanup_motor:
    (void)motor_destroy(s_motor);
    s_motor = NULL;
done:
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));   /* FATAL 挂起（可见可查） */
}