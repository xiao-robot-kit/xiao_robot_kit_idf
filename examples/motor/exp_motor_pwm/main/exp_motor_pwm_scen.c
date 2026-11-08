/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_pwm_scen.c
 * @brief   PWM 占空阶梯扫描实验运行时实现（契约见 exp_motor_pwm_scen.h）
 *
 * @note    场景固化：exp_motor_qemu 综合演示 motor_scen_table.c 场景 1 的单例
 *          化（步表/保持拍数/wiring 同源搬移：0/25/50/75/100% 五档、每档 2s、
 *          motor.pwm.generic @ pwm.motor.20k ch0）。改实验只改本文件的表。
 * @note    状态有家：本模块持有实验的全部实例句柄与步进状态，static 限文件内
 *          ——固件与 host gcc 自检各自链接一份，互不干扰（同 motor_scen.c）。
 * @note    失败原子性：init 中途失败立即全量回滚（销毁已建部分）——不留半挂
 *          占用；固件侧 init 失败即停（FATAL TEXT 帧），不进入半装配运行。
 */

#include <stdio.h>
#include <string.h>
#include "exp_motor_pwm_scen.h"
#include "motor_hal_sim.h"
#include "proto/frame.h"

/* ── 实验表（参数零写死：改实验只改本段） ──────────────────────────────── */
static const char *const s_timer_presets[] = { "pwm.motor.20k" };   /* 库预设名 */
enum { S_N_TIMERS = 1 };                                            /* 定时器数 */

/** 单电机槽定义（wiring + 实验轨迹；底版 = 库预设） */
typedef struct {
    const char   *motor_preset;   /**< motor_cfg_find 键（库预设底版） */
    uint8_t       channel;        /**< 定时器上的通道 */
    int           pin_a;          /**< PWM 正向引脚 */
    int           pin_b;          /**< PWM 反向引脚（非桥类 -1） */
    int           enc_a;          /**< 编码器 A 相（非 ENC 类 -1） */
    int           enc_b;          /**< 编码器 B 相（非 ENC 类 -1） */
    float         plant_rpm_per_duty;  /**< ENC 合成 plant 增益（非 ENC 槽 0） */
    float         plant_tau_s;         /**< 一阶滞后，s（非 ENC 槽 0） */
    const float  *steps;          /**< 步进目标序列（循环推进；域 = set_target 域） */
    int           n_steps;        /**< 序列长度 */
    uint32_t      hold_ticks;     /**< 每步保持拍数（EXP_MOTOR_PWM_POLL_US 计） */
} exp_slot_t;

static const float s_steps[] = { 0.00f, 0.25f, 0.50f, 0.75f, 1.00f };
static const exp_slot_t s_slots[EXP_MOTOR_PWM_SLOTS] = {
    { "motor.pwm.generic", 0, 4, -1, -1, -1,
      0.0f, 0.0f,
      s_steps, 5, 1000 },   /* 5 档 × 2s = 10s 一循环 */
};

/* ── 实例句柄与步进状态（装配序建立，运行期只读） ── */
static pwm_timer_t *s_timers[S_N_TIMERS];            /**< 实验定时器 */
static motor_t     *s_motors[EXP_MOTOR_PWM_SLOTS];   /**< 每槽一台电机 */
static pwm_timer_t *s_mot_timer[EXP_MOTOR_PWM_SLOTS]; /**< 各槽定时器（回读键） */
static uint8_t      s_mot_ch[EXP_MOTOR_PWM_SLOTS];    /**< 各槽通道（回读键） */
static int          s_kind[EXP_MOTOR_PWM_SLOTS];      /**< 各槽 kind（底版单源，帧用） */
static int          s_step[EXP_MOTOR_PWM_SLOTS];      /**< 当前步下标（帧 phase 字段） */
static uint32_t     s_ticks[EXP_MOTOR_PWM_SLOTS];     /**< 起拍累计（整除换步） */

/**
 * @brief   按名查电机预设整项（cfg 底版 + timer_preset 建议名一起拿）
 * @note    motor_cfg_find 只回 cfg（timer_preset 不在其返回面）——wiring 需要
 *          预设项的建议定时器名，故直接遍历库导出的预设表（零库改动）。
 */
static const motor_cfg_preset_t *preset_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < motor_cfg_preset_count; i++)
        if (strcmp(motor_cfg_presets[i].name, name) == 0)
            return &motor_cfg_presets[i];
    return NULL;
}

/** @brief 全量反装配（电机 + 定时器）；对「未装配」态幂等（回滚路径用） */
static void scen_destroy(void)
{
    for (int i = 0; i < EXP_MOTOR_PWM_SLOTS; i++) {
        if (s_motors[i] == NULL) continue;
        (void)motor_destroy(s_motors[i]);
        s_motors[i] = NULL;
        s_mot_timer[i] = NULL;
    }
    for (int t = 0; t < S_N_TIMERS; t++) {
        if (s_timers[t] == NULL) continue;
        (void)pwm_timer_destroy(s_timers[t]);
        s_timers[t] = NULL;
    }
}

int exp_motor_pwm_scen_init(void)
{
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        fprintf(stderr, "pwm exp: pwm_hal_register failed\n");
        return -1;
    }
    for (int t = 0; t < S_N_TIMERS; t++) {
        s_timers[t] = pwm_timer_create_preset(s_timer_presets[t]);
        if (s_timers[t] == NULL) {
            fprintf(stderr, "pwm exp: timer %s create failed\n",
                    s_timer_presets[t]);
            goto fail;
        }
    }
    for (int i = 0; i < EXP_MOTOR_PWM_SLOTS; i++) {
        const exp_slot_t *slot = &s_slots[i];
        const motor_cfg_preset_t *preset = preset_find(slot->motor_preset);
        if (preset == NULL) {
            fprintf(stderr, "pwm exp: motor preset %s not found\n",
                    slot->motor_preset);
            goto fail;
        }
        /* wiring：底版建议定时器在本实验名表中的下标（查名） */
        int t_idx = -1;
        for (int t = 0; t < S_N_TIMERS; t++) {
            if (strcmp(preset->timer_preset, s_timer_presets[t]) == 0) {
                t_idx = t;
                break;
            }
        }
        if (t_idx < 0) {
            fprintf(stderr, "pwm exp: slot %d timer preset %s not in table\n",
                    i, preset->timer_preset);
            goto fail;
        }

        motor_cfg_t cfg = preset->cfg;   /* 值语义拷底版，再填 wiring */
        cfg.timer = s_timers[t_idx];
        cfg.channel = slot->channel;
        cfg.pin_a = slot->pin_a;
        cfg.pin_b = slot->pin_b;
        cfg.enc_a = slot->enc_a;
        cfg.enc_b = slot->enc_b;

        s_motors[i] = motor_create(&cfg);
        if (s_motors[i] == NULL) {
            fprintf(stderr, "pwm exp: slot %d motor_create failed (%s)\n",
                    i, slot->motor_preset);
            goto fail;
        }
        s_mot_timer[i] = cfg.timer;
        s_mot_ch[i] = cfg.channel;
        s_kind[i] = (int)cfg.kind;   /* 帧 payload 的 kind 字段（底版单源） */
        s_step[i] = 0;
        s_ticks[i] = 0;
    }
    return 0;

fail:
    scen_destroy();
    return -1;
}

void exp_motor_pwm_scen_tick(uint32_t now_us,
                             exp_motor_tele_t out[EXP_MOTOR_PWM_SLOTS])
{
    if (out == NULL) return;
    for (int i = 0; i < EXP_MOTOR_PWM_SLOTS; i++) {
        const exp_slot_t *slot = &s_slots[i];

        /* 步进发生器：步号 = 累计拍 / hold（整除换步，循环取模）——步 k 恰占
         * hold 拍（先取步后计数，无 off-by-one；host 自检逐拍核对） */
        s_step[i] = (int)((s_ticks[i] / slot->hold_ticks)
                          % (uint32_t)slot->n_steps);
        s_ticks[i]++;

        float sp = slot->steps[s_step[i]];
        (void)motor_set_target(s_motors[i], sp);
        (void)motor_poll(s_motors[i], now_us);

        motor_state_t st;
        if (motor_get_state(s_motors[i], &st) != 0) {
            st.target = 0.0f; st.actual = 0.0f; st.duty = 0.0f;
            st.enc_count = 0; st.fault = 0xFFFFFFFFu;
        }
        out[i].setpoint = sp;
        /* duty = 存储回读（物理输出面）：域 [0,1] 外哨兵 -1 = 回读失败，
         * 界内断言必抓，不静默 */
        out[i].duty = -1.0f;
        (void)sim_pwm_channel_duty(s_mot_timer[i], s_mot_ch[i], &out[i].duty);
        out[i].actual = st.actual;
        out[i].enc_count = st.enc_count;
        out[i].fault = st.fault;
        out[i].phase = s_step[i];
        out[i].kind = s_kind[i];
    }
}

void exp_motor_pwm_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_PWM_SLOTS],
                             uint8_t payload[EXP_MOTOR_PWM_SLOTS * 16])
{
    /* 布局单源：proto/frame.h PROTO_TYPE_MOTOR 注释（N × 16B 小端记录，
     * 解码器按 payload 长度自适配——单实例 = 1×16B） */
    for (int i = 0; i < EXP_MOTOR_PWM_SLOTS; i++) {
        uint8_t *p = payload + i * 16;
        p[0] = (uint8_t)i;
        p[1] = (uint8_t)tele[i].kind;
        memcpy(p + 2, &tele[i].setpoint, 4);   /* host/x86 与 Xtensa 均小端 */
        memcpy(p + 6, &tele[i].duty, 4);
        memcpy(p + 10, &tele[i].actual, 4);
        p[14] = (uint8_t)tele[i].phase;
        p[15] = (uint8_t)(tele[i].fault & 0xFFu);
    }
}
