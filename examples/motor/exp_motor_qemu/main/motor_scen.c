/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_scen.c
 * @brief   电机实验场景运行时实现（契约见 motor_scen.h；表驱动，零 magic number）
 *
 * @note    状态有家：本模块持有场景的全部实例句柄与步进状态，static 限文件内
 *          ——固件与 host 自检各自链接一份，互不干扰（同 motor_demo.c）。
 * @note    失败原子性：start 中途失败立即全量回滚（destroy 已建部分），
 *          active 保持 -1（「已停」态）——不留半挂占用，重试安全。
 * @note    遥测 duty = sim PWM 合成存储回读（物理输出面）：nested 场景中
 *          若电机 A 的写串扰到 B 的通道（隔离破坏），B 的回读立即可见；
 *          DC 桥对槽走 sim_pwm_bridge_duty 带符号回读（符号 = 方向位）；
 *          采信存储而非电机自身状态，回读失败留域外哨兵（断言必抓）。
 */

#include <stdio.h>
#include <string.h>
#include "motor_scen.h"
#include "motor_hal_sim.h"
#include "proto/frame.h"

/* ── 实例句柄与步进状态（装配序建立，运行期只读；start 全量重建） ── */
static const motor_scen_def_t *s_def;               /**< 活动场景定义（NULL=停） */
static pwm_timer_t *s_timers[MOTOR_SCEN_MAX_TIMERS]; /**< 场景定时器 */
static int          s_n_timers;                     /**< 已建定时器数（回滚用） */
static motor_t     *s_motors[MOTOR_SCEN_MAX_SLOTS]; /**< 每槽一台电机 */
static int          s_n_motors;                     /**< 已建电机数（回滚用） */
static pwm_timer_t *s_mot_timer[MOTOR_SCEN_MAX_SLOTS]; /**< 各槽定时器（回读键） */
static uint8_t      s_mot_ch[MOTOR_SCEN_MAX_SLOTS];    /**< 各槽通道（回读键） */
static int          s_kind[MOTOR_SCEN_MAX_SLOTS];      /**< 各槽 kind（帧/回滚用） */
static int          s_enc_a[MOTOR_SCEN_MAX_SLOTS];     /**< 各槽 enc_a（plant 注销键） */
static int          s_enc_b[MOTOR_SCEN_MAX_SLOTS];     /**< 各槽 enc_b（plant 注销键） */
static int          s_step[MOTOR_SCEN_MAX_SLOTS];      /**< 当前步下标（帧 phase 字段） */
static uint32_t     s_ticks[MOTOR_SCEN_MAX_SLOTS];     /**< 场景起拍累计（整除换步） */

const motor_scen_def_t *motor_scen_def_get(int id)
{
    for (int i = 0; i < motor_scen_def_count; i++)
        if (motor_scen_defs[i].id == id)
            return &motor_scen_defs[i];
    return NULL;
}

/** @brief 定时器预设名 → 通道数（库预设表查名；0x06 快照 channels 字段源） */
static uint8_t preset_channels(const char *name)
{
    if (name == NULL) return 0;
    for (int i = 0; i < pwm_timer_preset_count; i++)
        if (strcmp(pwm_timer_presets[i].name, name) == 0)
            return pwm_timer_presets[i].channels;
    return 0;
}

/**
 * @brief   按名查电机预设整项（cfg 底版 + timer_preset 建议名一起拿）
 * @note    motor_cfg_find 只回 cfg（timer_preset 不在其返回面）——wiring 需要
 *          预设项的建议定时器名，故直接遍历库导出的预设表（零库改动，
 *          与 motor_demo.c 同款）。
 */
static const motor_cfg_preset_t *preset_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < motor_cfg_preset_count; i++)
        if (strcmp(motor_cfg_presets[i].name, name) == 0)
            return &motor_cfg_presets[i];
    return NULL;
}

/** @brief 全量反装配（电机 + plant + 定时器）；对「已停」态幂等 */
static void scen_destroy(void)
{
    for (int i = 0; i < s_n_motors; i++) {
        if (s_motors[i] == NULL) continue;
        if (s_kind[i] == (int)MOTOR_ENC_CLOSED)
            (void)sim_motor_plant_unregister(s_enc_a[i], s_enc_b[i]);
        (void)motor_destroy(s_motors[i]);
        s_motors[i] = NULL;
        s_mot_timer[i] = NULL;
    }
    s_n_motors = 0;
    for (int t = 0; t < s_n_timers; t++) {
        if (s_timers[t] == NULL) continue;
        (void)pwm_timer_destroy(s_timers[t]);
        s_timers[t] = NULL;
    }
    s_n_timers = 0;
}

int motor_scen_start(int id)
{
    const motor_scen_def_t *def = motor_scen_def_get(id);
    if (def == NULL) return -1;

    /* 全量重置：旧场景（若有）销毁；失败中途保持「已停」态 */
    scen_destroy();
    s_def = NULL;

    for (int t = 0; t < def->n_timer_presets; t++) {
        s_timers[t] = pwm_timer_create_preset(def->timer_presets[t]);
        if (s_timers[t] == NULL) {
            fprintf(stderr, "scen %d: timer %s create failed\n",
                    def->id, def->timer_presets[t]);
            goto fail;
        }
        s_n_timers = t + 1;
    }

    for (int i = 0; i < def->n_slots; i++) {
        const motor_scen_slot_t *slot = &def->slots[i];
        const motor_cfg_preset_t *preset = preset_find(slot->motor_preset);
        if (preset == NULL) {
            fprintf(stderr, "scen %d: motor preset %s not found\n",
                    def->id, slot->motor_preset);
            goto fail;
        }
        const motor_cfg_t *base = &preset->cfg;   /* 底版（timer_preset 走 preset 项） */

        /* wiring：底版建议定时器在本场景名表中的下标（与 demo 同款查名） */
        int t_idx = -1;
        for (int t = 0; t < def->n_timer_presets; t++) {
            if (strcmp(preset->timer_preset, def->timer_presets[t]) == 0) {
                t_idx = t;
                break;
            }
        }
        if (t_idx < 0) {
            fprintf(stderr, "scen %d: slot %d timer preset %s not in scen table\n",
                    def->id, i, preset->timer_preset);
            goto fail;
        }

        motor_cfg_t cfg = *base;   /* 值语义拷底版，再填 wiring */
        cfg.timer = s_timers[t_idx];
        cfg.channel = slot->channel;
        cfg.pin_a = slot->pin_a;
        cfg.pin_b = slot->pin_b;
        cfg.enc_a = slot->enc_a;
        cfg.enc_b = slot->enc_b;

        s_motors[i] = motor_create(&cfg);
        if (s_motors[i] == NULL) {
            fprintf(stderr, "scen %d: slot %d motor_create failed (%s)\n",
                    def->id, i, slot->motor_preset);
            goto fail;
        }
        s_n_motors = i + 1;
        s_mot_timer[i] = cfg.timer;
        s_mot_ch[i] = cfg.channel;
        s_kind[i] = (int)base->kind;
        s_enc_a[i] = slot->enc_a;
        s_enc_b[i] = slot->enc_b;

        /* ENC：先挂编码器 HAL（无副作用）再注册 plant（若其后失败，
         * plant 未挂上/或本槽失败即整体回滚——attach 失败不留 plant，
         * plant 失败不留电机，序见 motor_scen.h 装配注） */
        if (base->kind == MOTOR_ENC_CLOSED) {
            if (motor_hal_attach(s_motors[i], &sim_encoder_hal) != 0) {
                fprintf(stderr, "scen %d: slot %d hal attach failed\n", def->id, i);
                goto fail;
            }
            sim_motor_plant_cfg_t plant = {
                .timer = cfg.timer,
                .ch_fwd = cfg.channel,
                .ch_rev = (uint8_t)(cfg.channel + 1u),
                .enc_a = cfg.enc_a,
                .enc_b = cfg.enc_b,
                .rpm_per_duty = slot->plant_rpm_per_duty,
                .tau_s = slot->plant_tau_s,
                .cpr = base->cpr,
                .dt_s = (float)MOTOR_DEMO_POLL_US / 1000000.0f,
            };
            if (sim_motor_plant_register(&plant) != 0) {
                fprintf(stderr, "scen %d: slot %d plant register failed\n",
                        def->id, i);
                goto fail;
            }
        }

        s_step[i] = 0;
        s_ticks[i] = 0;
    }
    s_def = def;
    return 0;

fail:
    scen_destroy();
    return -1;
}

int motor_scen_stop(void)
{
    scen_destroy();
    s_def = NULL;
    return 0;
}

int motor_scen_active(void)
{
    return (s_def != NULL) ? s_def->id : -1;
}

void motor_scen_tick(uint32_t now_us,
                     motor_demo_tele_t out[MOTOR_SCEN_MAX_SLOTS], int *n_out)
{
    if (n_out == NULL) return;
    if (s_def == NULL || out == NULL) {
        *n_out = 0;
        return;
    }
    for (int i = 0; i < s_def->n_slots; i++) {
        const motor_scen_slot_t *slot = &s_def->slots[i];

        /* 步进发生器：步号 = 累计拍 / hold（整除换步，循环取模）——步 k
         * 恰占 hold 拍（先取步后计数，无 off-by-one；host 自检逐拍核对） */
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
        /* duty 回读分支（物理输出面）：单通道槽 = 通道占空（0~1）；DC 桥对槽 =
         * 方向桥带符号回读（−1~+1，符号 = 方向位——正反转切换的可观测锚）。
         * 哨兵取各自域外值：回读失败可被界内断言抓住，不静默。 */
        if (s_kind[i] == (int)MOTOR_DC) {
            out[i].duty = -2.0f;
            sim_bridge_duty_t bd;
            if (sim_pwm_bridge_duty(s_mot_timer[i], s_mot_ch[i],
                                    (uint8_t)(s_mot_ch[i] + 1u), &bd) == 0) {
                out[i].duty = bd.signed_duty;
            }
        } else {
            out[i].duty = -1.0f;
            (void)sim_pwm_channel_duty(s_mot_timer[i], s_mot_ch[i], &out[i].duty);
        }
        out[i].actual = st.actual;
        out[i].enc_count = st.enc_count;
        out[i].fault = st.fault;
        out[i].phase = s_step[i];   /* 场景帧 phase = 步进下标（帧头注释契约） */
        out[i].kind = s_kind[i];
    }
    *n_out = s_def->n_slots;
}

size_t motor_scen_pack(const motor_demo_tele_t tele[], int n,
                       uint8_t *payload, size_t cap)
{
    if (s_def == NULL || tele == NULL || payload == NULL) return 0;
    if (n != s_def->n_slots) return 0;

    const int n_tim = s_def->n_timer_presets;
    const size_t need = 3u + 6u * (size_t)n_tim + 16u * (size_t)n;
    if (cap < need) return 0;

    payload[0] = (uint8_t)s_def->id;
    payload[1] = (uint8_t)n_tim;
    for (int t = 0; t < n_tim; t++) {
        uint8_t *p = payload + 2 + t * 6;
        uint32_t freq = pwm_timer_freq(s_timers[t]);   /* 活实例现取（单源） */
        p[0] = (uint8_t)t;
        p[1] = preset_channels(s_def->timer_presets[t]);
        memcpy(p + 2, &freq, 4);   /* host/x86 与 Xtensa 均小端 */
    }
    payload[2 + 6 * n_tim] = (uint8_t)n;

    /* 电机记录：与 0x05 的 16B 记录逐字节同构（布局单源 frame.h 注释） */
    for (int i = 0; i < n; i++) {
        uint8_t *p = payload + 3 + 6 * n_tim + i * 16;
        p[0] = (uint8_t)i;
        p[1] = (uint8_t)tele[i].kind;
        memcpy(p + 2, &tele[i].setpoint, 4);
        memcpy(p + 6, &tele[i].duty, 4);
        memcpy(p + 10, &tele[i].actual, 4);
        p[14] = (uint8_t)tele[i].phase;
        p[15] = (uint8_t)(tele[i].fault & 0xFFu);
    }
    return need;
}
