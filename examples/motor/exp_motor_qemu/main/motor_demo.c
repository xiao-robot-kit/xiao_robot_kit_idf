/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_demo.c
 * @brief   电机演示运行时实现（契约见 motor_demo.h；表驱动，零 magic number）
 *
 * @note    状态有家：本模块持有全部实例句柄（定时器/电机/S 梯度/dwell 计数），
 *          static 限文件内——固件与 host 自检各自链接一份，互不干扰。
 * @note    dwell/retarget 状态机：一腿走完（scurve_done）→ 保持
 *          MOTOR_DEMO_DWELL_TICKS 拍（匀速/停驻段）→ retarget 交替目标。
 *          scurve_retarget 语义「先停稳再走」——dwell 后速度恰为 0，衔接干净。
 */

#include <stdio.h>
#include <string.h>
#include "motor_demo.h"
#include "motor_hal_sim.h"
#include "proto/frame.h"

/* ── 实例句柄（装配序建立，运行期只读） ── */
static pwm_timer_t *s_timers[4];                       /**< 去重预设名建出的定时器 */
static motor_t     *s_motors[MOTOR_DEMO_SLOTS];        /**< 每槽一台电机 */
static scurve_t     s_sc[MOTOR_DEMO_SLOTS];            /**< 每槽一条 S 梯度 */
static int          s_kind[MOTOR_DEMO_SLOTS];          /**< 每槽 motor_kind_t（帧 payload 用） */
static int          s_dwell[MOTOR_DEMO_SLOTS];         /**< 剩余保持拍数 */
static int          s_tgt_idx[MOTOR_DEMO_SLOTS];       /**< 下一腿目标下标（0/1） */
static pwm_timer_t *s_mot_timer[MOTOR_DEMO_SLOTS];     /**< 各槽定时器（DC 桥回读键） */
static uint8_t      s_mot_ch[MOTOR_DEMO_SLOTS];        /**< 各槽通道（DC 桥回读键） */

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

int motor_demo_init(void)
{
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        fprintf(stderr, "demo: pwm_hal_register failed\n");
        return -1;
    }

    /* 定时器：按预设名表去重建（同频只建一个——sim PWM 存储约束 + 嵌套共享语义） */
    for (int t = 0; t < motor_demo_timer_preset_count; t++) {
        s_timers[t] = pwm_timer_create_preset(motor_demo_timer_presets[t]);
        if (s_timers[t] == NULL) {
            fprintf(stderr, "demo: timer preset %s create failed\n",
                    motor_demo_timer_presets[t]);
            return -1;
        }
    }

    /* 每槽：预设底版 + wiring → create（ENC 再挂 plant 与编码器 HAL）→ S 梯度 init */
    for (int i = 0; i < MOTOR_DEMO_SLOTS; i++) {
        const motor_demo_slot_t *slot = &motor_demo_slots[i];
        const motor_cfg_preset_t *preset = preset_find(slot->motor_preset);
        if (preset == NULL) {
            fprintf(stderr, "demo: motor preset %s not found\n", slot->motor_preset);
            return -1;
        }
        const motor_cfg_t *base = &preset->cfg;   /* 底版（timer_preset 走 preset 项） */

        /* wiring：定时器名 = 槽表 override 优先，否则底版建议（名表即去重后的
         * 「建议集 + override 集」；override 唯一用途见 motor_demo_table.c 表头注） */
        const char *timer_name = (slot->timer_override != NULL)
                               ? slot->timer_override : preset->timer_preset;
        int t_idx = -1;
        for (int t = 0; t < motor_demo_timer_preset_count; t++) {
            if (strcmp(timer_name, motor_demo_timer_presets[t]) == 0) {
                t_idx = t;
                break;
            }
        }
        if (t_idx < 0) {
            fprintf(stderr, "demo: slot %d timer preset %s not in demo table\n",
                    i, timer_name);
            return -1;
        }

        motor_cfg_t cfg = *base;   /* 值语义拷贝底版，再填 wiring */
        cfg.timer = s_timers[t_idx];
        cfg.channel = slot->channel;
        cfg.pin_a = slot->pin_a;
        cfg.pin_b = slot->pin_b;
        cfg.enc_a = slot->enc_a;
        cfg.enc_b = slot->enc_b;

        s_motors[i] = motor_create(&cfg);
        if (s_motors[i] == NULL) {
            fprintf(stderr, "demo: slot %d motor_create failed (preset %s)\n",
                    i, slot->motor_preset);
            return -1;
        }
        s_mot_timer[i] = cfg.timer;   /* DC 槽桥回读键（其余槽未用，顺手记齐） */
        s_mot_ch[i] = cfg.channel;

        if (base->kind == MOTOR_ENC_CLOSED) {
            /* 合成 plant：cpr 单源自预设底版；桥对 = (channel, channel+1) */
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
                fprintf(stderr, "demo: slot %d plant register failed\n", i);
                return -1;
            }
            if (motor_hal_attach(s_motors[i], &sim_encoder_hal) != 0) {
                fprintf(stderr, "demo: slot %d hal attach failed\n", i);
                return -1;
            }
        }

        if (scurve_init(&s_sc[i], &slot->scurve, slot->target[0]) != 0) {
            fprintf(stderr, "demo: slot %d scurve_init failed\n", i);
            return -1;
        }
        s_kind[i] = (int)base->kind;   /* 帧 payload 的 kind 字段（底版单源） */
        s_tgt_idx[i] = 1;      /* 首腿 target[0] 在 init 已定；下一腿交替到 1 */
        s_dwell[i] = MOTOR_DEMO_DWELL_TICKS;   /* 首腿走完同样保持 N 拍（与
                                                  motor_demo_table.h「一腿走完
                                                  后保持」契约一致；旧值 0 会
                                                  让首腿无停驻段直接换向——
                                                  500 拍自检实测暴露） */
    }
    return 0;
}

int motor_demo_deinit(void)
{
    int rc = 0;
    /* 电机先行销毁（释放通道占用——定时器 destroy 的前置）；ENC 槽的合成
     * plant 一并注销（引脚对 8/9 与场景 enc 电机同键，不注销则再注册被拒） */
    for (int i = 0; i < MOTOR_DEMO_SLOTS; i++) {
        if (s_motors[i] == NULL) continue;
        if (s_kind[i] == (int)MOTOR_ENC_CLOSED) {
            (void)sim_motor_plant_unregister(motor_demo_slots[i].enc_a,
                                             motor_demo_slots[i].enc_b);
        }
        if (motor_destroy(s_motors[i]) != 0) rc = -1;
        s_motors[i] = NULL;
    }
    /* 定时器销毁（通道已全还——destroy 前置由电机销毁保证） */
    for (int t = 0; t < motor_demo_timer_preset_count; t++) {
        if (s_timers[t] == NULL) continue;
        if (pwm_timer_destroy(s_timers[t]) != 0) rc = -1;
        s_timers[t] = NULL;
    }
    /* HAL 留驻（进程级一次）；scurve/dwell 状态由下次 init 全量重建，无需清 */
    return rc;
}

void motor_demo_tick(uint32_t now_us, motor_demo_tele_t out[MOTOR_DEMO_SLOTS])
{
    if (out == NULL) return;
    for (int i = 0; i < MOTOR_DEMO_SLOTS; i++) {
        const motor_demo_slot_t *slot = &motor_demo_slots[i];

        /* dwell/retarget 状态机（腿走完 → 保持 N 拍 → 交替目标重规划） */
        if (scurve_done(&s_sc[i])) {
            if (s_dwell[i] > 0) {
                s_dwell[i]--;
            } else {
                if (scurve_retarget(&s_sc[i], slot->target[s_tgt_idx[i]]) != 0) {
                    fprintf(stderr, "demo: slot %d retarget failed\n", i);
                }
                s_tgt_idx[i] ^= 1;   /* 两腿交替 */
                s_dwell[i] = MOTOR_DEMO_DWELL_TICKS;
            }
        }

        /* S 梯度推进 → 设定值喂电机 → 一拍输出 → 遥测 */
        float sp = scurve_step(&s_sc[i]);
        (void)motor_set_target(s_motors[i], sp);
        (void)motor_poll(s_motors[i], now_us);

        motor_state_t st;
        if (motor_get_state(s_motors[i], &st) != 0) {
            st.target = 0.0f; st.actual = 0.0f; st.duty = 0.0f;
            st.enc_count = 0; st.fault = 0xFFFFFFFFu;
        }
        out[i].setpoint = sp;
        out[i].duty = st.duty;
        /* DC 槽：duty = 方向桥带符号回读（模拟 HAL DC 分支——符号 = 方向位，
         * 幅值 = 输出占空；与 st.duty 的幅值语义不同，帧契约见 frame.h 0x05 注）。
         * 域 [−1,1] 外哨兵 −2：回读失败可被界内断言抓住，不静默 */
        if (s_kind[i] == (int)MOTOR_DC) {
            out[i].duty = -2.0f;
            sim_bridge_duty_t bd;
            if (sim_pwm_bridge_duty(s_mot_timer[i], s_mot_ch[i],
                                    (uint8_t)(s_mot_ch[i] + 1u), &bd) == 0) {
                out[i].duty = bd.signed_duty;
            }
        }
        out[i].actual = st.actual;
        out[i].enc_count = st.enc_count;
        out[i].fault = st.fault;
        out[i].phase = (int)scurve_phase(&s_sc[i]);
        out[i].kind = s_kind[i];
    }
}

void motor_demo_pack(const motor_demo_tele_t tele[MOTOR_DEMO_SLOTS],
                     uint8_t payload[MOTOR_DEMO_SLOTS * 16])
{
    /* 布局单源：proto/frame.h PROTO_TYPE_MOTOR 注释（4 × 16B 小端记录） */
    for (int i = 0; i < MOTOR_DEMO_SLOTS; i++) {
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
