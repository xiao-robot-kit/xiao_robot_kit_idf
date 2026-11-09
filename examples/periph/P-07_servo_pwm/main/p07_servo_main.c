/* SPDX-License-Identifier: MIT */
/**
 * @file    p07_servo_main.c
 * @brief   P-07 舵机 PWM 例程（4 路 50Hz 舵机，0°/90°/180° 定位 + 电压联动告警）
 *
 * 链路：motor 组件（motor.servo.sg90 @ pwm.servo.50，底版 500~2500us/0~180°）
 *       × 4 路（单定时器 4 通道，SERVO 单臂占 1 通道）→ 合成 PWM 存储回读
 *       USB-Serial-JTAG（真板档）。
 *
 * 演示两件事（有限序列，事件回放每 1s 重放全量；P-02 模式）：
 *   A. 四舵机错相扫角表 {0 → 90 → 180 → 90 → 0}（每槽起始相位错开，证明
 *      4 路独立驱动）；每 1s 事件行回读各通道 duty → 换算脉宽 us（50Hz 周期
 *      20ms：us = duty×20000）。
 *   B. 电压联动告警（任务书 §4.4 硬性）：每拍采样 D16 电池电压 —— v_bat
 *      低于告警阈值 6.20V 时输出 WARN 行 + 停扫保舵（防堵转拉垮 Motor_5V 轨）。
 *
 * 硬件（任务书 §3.1/§4.4/§5.3）：
 *   P-07 四舵机口 D15 = S1、D9 = S2、D0 = S3、D8 = S4（LEDC 50Hz 独立通道）。
 *   电源轨：Motor_5V（Q7 使能）同时供 4 舵机 + 2 电机驱动——USB 供电时轨功率
 *       受限，多舵机堵转会拉垮电压 → 例程硬性电压监测联动告警（本文件 B 段）；
 *       舵机空载峰流预算约 4×250mA=1A < USB 5V@1A，堵转单舵 0.9A 立超——故
 *       告警即全停（停摆护轨），不等二次超时。
 *       断言 WARN 行）；P_ADC_RAW=0 真板档 = ADC2_CH5（GPIO16，100K:30K 分压，
 *       换算 Vbat = Vadc×130/30，同 P-04）。
 *
 * 宏契约（CMake 定义，两态显式）：
 *                 C 侧 #error 拒绝（不静默编出假硬件固件，ERR-004 纪律）。
 *   P_ADC_RAW      1=合成电压演示曲线；0=真 ADC2_CH5 采样（真板，需 esp_adc）。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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
#error "P_SIM_MOTOR=0: real LEDC HAL not provided by port yet (ERR-004) — use sim"
#endif
#ifndef P_ADC_RAW
#error "P_ADC_RAW undefined: set 1=sim voltage demo or 0=real ADC (board)"
#endif

/* ---- 场景参数（4 舵机单定时器；错相扫角表 + 电压告警） ---- */
#define SERVO_N              4u            /* 4 路 */
#define SERVO_CH_BASE        0u            /* 定时器通道自 0 起 */
#define SERVO_FREQ           50u           /* 50Hz（= pwm.servo.50） */
#define SERVO_PERIOD_US      20000u        /* 20ms 周期（脉宽换算底座） */
#define STEP_HOLD_US         1000000u      /* 每角度档停留 1s */
#define PHASE_START_US       500000u       /* 相邻槽起始相位错开 0.5s（独立证明） */
#define ANGLE_TABLE_N        5u
static const float s_angle_table[ANGLE_TABLE_N] = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f };

#define VBAT_ALARM_V         6.20f         /* 告警阈值（低电联动） */
#define VBAT_DEMO_OK_V       7.40f         /* 合成正常电压 */
#define VBAT_DEMO_LOW_V      6.00f         /* 合成低电值（演示告警） */
#define VBAT_DEMO_LOW_AT_US  4000000u      /* 4s 后演示跌压（扫描中段，WARN 入回放） */
#define VBAT_DEMO_REC_AT_US  7000000u      /* 7s 后恢复 */
#define SAFE_BAUD            115200
#define S_REPLAY_MAX         48
#define S_REPLAY_LEN         96

#define EXP_UART_NUM         UART_NUM_0

/* ---- 引脚表（任务书 §3.1：D15/D9/D0/D8 = S1~S4） ---- */
static const int s_pins[SERVO_N] = { 15, 9, 0, 8 };

/* ---- 运行状态 ---- */
static motor_t     *s_servo[SERVO_N] = { NULL };
static pwm_timer_t *s_timer = NULL;
static int64_t      s_start_us = 0;
static float        s_vbat_now = VBAT_DEMO_OK_V;

/* ---- 回放缓冲（P-02 模式：boot + 每角度档 1 行 + WARN + DONE） ---- */
static char s_replay[S_REPLAY_MAX][S_REPLAY_LEN];
static int  s_replay_n = 0;

static void replay_add(const char *line)
{
    if (s_replay_n < S_REPLAY_MAX)
        snprintf(s_replay[s_replay_n++], S_REPLAY_LEN, "%s", line);
}

static void tx_str(const char *s)
{
    uart_write_bytes(EXP_UART_NUM, s, (int)strlen(s));
}

static void output_log(const char *line)
{
    static char b[S_REPLAY_LEN + 8];
    snprintf(b, sizeof(b), "%s\r\n", line);
    tx_str(b);
    replay_add(line);
    uart_wait_tx_done(EXP_UART_NUM, 50);
}

static void uart_init(void)
{
    (void)0;   /* 真板档 USB-Serial-JTAG 由 esp_console 接管，app_puts 直推 */
}

#if P_ADC_RAW == 1
static float sample_vbat_us(void)
{
    return s_vbat_now;
}
static void vbat_demo_update(int64_t el)
{
    if (el >= VBAT_DEMO_REC_AT_US)         s_vbat_now = VBAT_DEMO_OK_V;
    else if (el >= VBAT_DEMO_LOW_AT_US)    s_vbat_now = VBAT_DEMO_LOW_V;
    else                                   s_vbat_now = VBAT_DEMO_OK_V;
}
#else
/* ==== 真板 ADC 档：ADC2_CH5 (GPIO16) 单次采样 + 一阶滤波（同 P-04 底座） ==== */
#include "esp_adc/adc_oneshot.h"
#define P07_ADC_UNIT     ADC_UNIT_2
#define P07_ADC_CHANNEL  ADC_CHANNEL_5
#define P07_ADC_BITS     ADC_BITWIDTH_12
#define P07_ADC_ATTEN    ADC_ATTEN_DB_11
#define P07_DIV_HI       100u              /* 分压上臂 kΩ（板上 100K） */
#define P07_DIV_LO       30u               /* 分压下臂 kΩ（板上 30K） */
static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static float s_vbat_filt = VBAT_DEMO_OK_V;

static int adc_init_once(void)
{
    if (s_adc_unit != NULL) return 0;
    adc_oneshot_unit_init_cfg_t u = { .unit_id = P07_ADC_UNIT };
    if (adc_oneshot_new_unit(&u, &s_adc_unit) != ESP_OK) return -1;
    adc_oneshot_chan_cfg_t c = {
        .atten = P07_ADC_ATTEN, .bitwidth = P07_ADC_BITS,
    };
    if (adc_oneshot_config_channel(s_adc_unit, P07_ADC_CHANNEL, &c) != ESP_OK)
        return -1;
    return 0;
}
static float sample_vbat_us(void)
{
    if (adc_init_once() != 0) return s_vbat_filt;
    int raw = 0;
    if (adc_oneshot_read(s_adc_unit, P07_ADC_CHANNEL, &raw) != ESP_OK)
        return s_vbat_filt;
    /* Vbat = Vadc × (100+30)/30 = Vadc × 130/30；Vadc = raw/4096×3.3 */
    float v_adc = (float)raw / 4096.0f * 3.3f;
    float v_bat = v_adc * ((float)(P07_DIV_HI + P07_DIV_LO) / (float)P07_DIV_LO);
    s_vbat_filt = 0.9f * s_vbat_filt + 0.1f * v_bat;   /* 一阶滤波（抖噪） */
    return s_vbat_filt;
}
static void vbat_demo_update(int64_t el) { (void)el; }
#endif

/* ---- 四槽推进：错相扫角表（步进 = s_step[i] 指向表元素；hold 后 +1） ---- */
static uint32_t angle_index(int slot, int64_t el)
{
    /* 槽 i 起跑相位 = i×0.5s：step 计数 = (el + i×0.5s)/1s（先钳到表长） */
    int64_t shifted = el + (int64_t)slot * (int64_t)PHASE_START_US;
    if (shifted < 0) shifted = 0;
    uint32_t idx = (uint32_t)(shifted / STEP_HOLD_US);
    return (idx < ANGLE_TABLE_N) ? idx : (uint32_t)(ANGLE_TABLE_N - 1u);
}

/* ---- 场景一帧：各槽 set_target + poll + 回读（物理输出面 = 合成存储） ---- */
static void servo_frame(const int64_t el, char *out, size_t out_len)
{
    float duty[SERVO_N];
    float us[SERVO_N];
    float ang[SERVO_N];
    for (uint32_t i = 0; i < SERVO_N; i++) {
        if (s_servo[i] == NULL) { duty[i] = 0.0f; us[i] = 0.0f; ang[i] = 0.0f; continue; }
        ang[i] = s_angle_table[angle_index(i, el)];
        (void)motor_set_target(s_servo[i], ang[i]);
        (void)motor_poll(s_servo[i], 0);
        if (sim_pwm_channel_duty(s_timer, (uint8_t)i, &duty[i]) != 0)
            duty[i] = 0.0f;
        us[i] = duty[i] * (float)SERVO_PERIOD_US;   /* 50Hz：duty×20ms → us */
    }
    snprintf(out, out_len,
             "SERVO vbat=%.2f ang0=%2.0f ang1=%2.0f ang2=%2.0f ang3=%2.0f (us:%0.0f/%0.0f/%0.0f/%0.0f)",
             (double)s_vbat_now,
             (double)ang[0], (double)ang[1], (double)ang[2], (double)ang[3],
             (double)us[0], (double)us[1], (double)us[2], (double)us[3]);
}
/* ---- 主循环：1s 事件档 + 周期遥测 + 电压告警联动 ---- */
static void app_loop(void *arg)
{
    (void)arg;
    char line[S_REPLAY_LEN + 8];
    bool warned = false;
    s_start_us = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const int64_t el = esp_timer_get_time() - s_start_us;

        vbat_demo_update(el);
        s_vbat_now = sample_vbat_us();

        /* 电压联动告警（任务书 §4.4 硬性）：v_bat 低于阈值 → WARN + 全停扫 */
        if (s_vbat_now < VBAT_ALARM_V) {
            if (!warned) {
                snprintf(line, sizeof(line), "WARN vbat=%.2f low (cutoff %.2fV) servo hold",
                         (double)s_vbat_now, (double)VBAT_ALARM_V);
                output_log(line);
                warned = true;
            }
            /* 告警中：各舵机目标钳 0°（停摆保位，防堵转拉垮 Motor_5V 轨） */
            for (uint32_t i = 0; i < SERVO_N; i++)
                if (s_servo[i] != NULL) {
                    (void)motor_set_target(s_servo[i], 0.0f);
                    (void)motor_poll(s_servo[i], 0);
                }
            continue;
        }
        if (warned) {
            snprintf(line, sizeof(line), "OK vbat=%.2f recovered (%.2fV ok)",
                     (double)s_vbat_now, (double)VBAT_ALARM_V);
            output_log(line);
            warned = false;
        }

        /* 正常档：每 1s 一帧事件（四槽角度 + 脉宽回读） */
        if ((el / 500000u) % 2u == 0u) {
            servo_frame(el, line, sizeof(line) - 4);
            output_log(line);
        }

        /* 完成判定：全部槽走完表尾 → DONE */
        uint32_t done_all = 1;
        for (uint32_t i = 0; i < SERVO_N; i++)
            if (angle_index(i, el) < ANGLE_TABLE_N - 1u) { done_all = 0; break; }
        if (done_all) {
            snprintf(line, sizeof(line), "SERVO DONE n=%u hold=%.1fs",
                     (unsigned)SERVO_N, (double)STEP_HOLD_US / 1000000.0);
            output_log(line);
            /* 完成态每 1s 重放全量（P-02 模式：TCP 探针何时连上都拿得到） */
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                for (int k = 0; k < s_replay_n; k++) {
                    static char rb[S_REPLAY_LEN + 8];
                    snprintf(rb, sizeof(rb), "%s\r\n", s_replay[k]);
                    tx_str(rb);
                }
                uart_wait_tx_done(EXP_UART_NUM, 50);
            }
        }
    }
}

void app_main(void)
{
    uart_init();

    char boot[S_REPLAY_LEN];
    snprintf(boot, sizeof(boot),
             "OK boot P-07_servo_pwm n=%u freq=%u sim_motor=1 adc_raw=%d",
             (unsigned)SERVO_N, (unsigned)SERVO_FREQ, P_ADC_RAW);
    output_log(boot);

    /* ---- 装配（timer → 4 × motor preset → 无 plant（SERVO 无编码器）） ---- */
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        output_log("FATAL pwm_hal_register");
        goto done;
    }
    s_timer = pwm_timer_create_preset("pwm.servo.50");
    if (s_timer == NULL) {
        output_log("FATAL timer create");
        goto done;
    }
    for (uint32_t i = 0; i < SERVO_N; i++) {
        const motor_cfg_t *base = motor_cfg_find("motor.servo.sg90");
        if (base == NULL) { output_log("FATAL preset not found"); goto done; }
        motor_cfg_t cfg = *base;
        cfg.timer   = s_timer;
        cfg.channel = (uint8_t)(SERVO_CH_BASE + i);
        cfg.pin_a   = s_pins[i];
        cfg.pin_b   = -1;
        cfg.enc_a   = -1;
        cfg.enc_b   = -1;
        s_servo[i] = motor_create(&cfg);
        if (s_servo[i] == NULL) { output_log("FATAL motor_create"); goto done; }
    }

    /* ---- 场景运行 ---- */
    xTaskCreate(app_loop, "p07_servo", 4096, NULL, 5, NULL);
    return;

done:
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));   /* FATAL 挂起（可见可查） */
}
