/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_scurve_scen.c
 * @brief   S 梯度轨迹实验运行时实现（契约见 exp_motor_scurve_scen.h）
 *
 * @note    场景来源：阶段维补齐实验（用户「每一阶段×每一电机」矩阵的 S 曲线
 *          阶段）——非 exp_motor_qemu 场景表固化（五场景无 S 曲线专项），wiring
 *          沿用综合演示槽位约定（PWM @ 20k ch0/pin4；ENC @ 20k 桥 ch1+2/
 *          pin 6/7/enc 8/9、plant 160 rpm/duty、τ=20ms——同 motor_demo_table.c
 *          槽 0/槽 2 的库预设底版）。改实验只改本文件的表。
 * @note    状态有家：本模块持有实验的全部实例句柄与腿状态，static 限文件内
 *          ——固件与 host gcc 自检各自链接一份，互不干扰（同 motor_demo.c）。
 * @note    失败原子性：init 中途失败立即全量回滚（销毁已建部分，plant 先注销）
 *          ——不留半挂占用；固件侧 init 失败即停（FATAL TEXT 帧）。
 * @note    腿边界（自检断言的分段依据）：scurve_init 落点即新腿起点——ENC 槽
 *          腿基线（enc_count）与 PWM 槽积分器在 rewind 同拍清零；此后 phase
 *          0..6 为运动段、7 为停驻段，一腿速度面恰历加速/匀速/减速三段。
 */

#include <stdio.h>
#include <string.h>
#include "exp_motor_scurve_scen.h"
#include "motor_hal_sim.h"
#include "proto/frame.h"

/* ── 实验表（参数零写死：改实验只改本段） ──────────────────────────────── */
static const char *const s_timer_presets[] = { "pwm.motor.20k" };   /* 库预设名 */
enum { S_N_TIMERS = 1 };                                            /* 定时器数 */

/**
 * @brief   单电机槽定义（wiring + S 梯度动力学 + 域换算 + plant；底版 = 库预设）
 * @note    scurve.dt 必须等于 EXP_MOTOR_SCURVE_POLL_US 对应秒值（时基单源）；
 *          vel_scale：u/s → motor_set_target 域（PWM 槽 1.0 = duty 直通；
 *          ENC 槽 60.0 = rev/s → rpm）。
 */
typedef struct {
    const char   *motor_preset;   /**< motor_cfg_find 键（库预设底版） */
    uint8_t       channel;        /**< 定时器上的通道（桥类自动占 channel+1） */
    int           pin_a;          /**< PWM 正向引脚 */
    int           pin_b;          /**< PWM 反向引脚（非桥类 -1） */
    int           enc_a;          /**< 编码器 A 相引脚（非 ENC 类 -1） */
    int           enc_b;          /**< 编码器 B 相引脚（非 ENC 类 -1） */
    scurve_cfg_t  scurve;         /**< 轨迹动力学（vmax/amax/jerk/dt） */
    float         target_pos;     /**< 腿终点位置 [u]（>0：正向点对点扫描） */
    float         vel_scale;      /**< 速度前馈换算：v [u/s] × 此值 = 电机设定 */
    float         plant_rpm_per_duty; /**< ENC 合成 plant 增益（非 ENC 槽 0） */
    float         plant_tau_s;        /**< ENC 一阶滞后，s（非 ENC 槽 0） */
} exp_slot_t;

static const exp_slot_t s_slots[EXP_MOTOR_SCURVE_SLOTS] = {
    /* 槽0：开环 PWM（u = duty·s）——S 梯度直接整形速度命令（= duty），
     * 匀速段即占空平台；腿终点 2.0 duty·s ≈ 4s 一腿（见 scurve 注） */
    { "motor.pwm.generic", 0, 4, -1, -1, -1,
      { .vmax = 0.6f, .amax = 1.5f, .jerk = 6.0f, .dt = 0.002f },
      2.0f, 1.0f, 0.0f, 0.0f },
    /* 槽1：N20 编码器闭环（u = rev）——速度前馈 ×60 = rpm；vmax 1.2 rev/s
     * = 72 rpm → 稳态 duty 0.45（plant 160 rpm/duty，界内不饱和）；
     * 腿终点 5.0 rev ≈ 5s 一腿 */
    { "motor.enc.n20", 1, 6, 7, 8, 9,
      { .vmax = 1.2f, .amax = 2.0f, .jerk = 8.0f, .dt = 0.002f },
      5.0f, 60.0f, 160.0f, 0.02f },
};

/* ── 实例句柄与腿状态（装配序建立，运行期只读） ── */
static pwm_timer_t *s_timers[S_N_TIMERS];             /**< 实验定时器 */
static motor_t     *s_motors[EXP_MOTOR_SCURVE_SLOTS]; /**< 每槽一台电机 */
static scurve_t     s_sc[EXP_MOTOR_SCURVE_SLOTS];     /**< 每槽一条 S 梯度 */
static pwm_timer_t *s_mot_timer[EXP_MOTOR_SCURVE_SLOTS]; /**< 各槽定时器（回读键） */
static uint8_t      s_mot_ch[EXP_MOTOR_SCURVE_SLOTS];    /**< 各槽通道（回读键） */
static int          s_kind[EXP_MOTOR_SCURVE_SLOTS];      /**< 各槽 kind（帧用） */
static int          s_dwell[EXP_MOTOR_SCURVE_SLOTS];     /**< 剩余停驻拍数 */
static int32_t      s_leg_base_enc[EXP_MOTOR_SCURVE_SLOTS]; /**< ENC 腿基线计数 */
static double       s_pos_int[EXP_MOTOR_SCURVE_SLOTS];    /**< PWM 位置积分器 [u] */
static int32_t      s_cpr[EXP_MOTOR_SCURVE_SLOTS];       /**< 各槽 cpr（预设底版单源） */

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

/** @brief 全量反装配（plant 注销 + 电机 + 定时器）；对「未装配」态幂等 */
static void scen_destroy(void)
{
    for (int i = 0; i < EXP_MOTOR_SCURVE_SLOTS; i++) {
        if (s_motors[i] == NULL) continue;
        if (s_kind[i] == (int)MOTOR_ENC_CLOSED)
            (void)sim_motor_plant_unregister(s_slots[i].enc_a, s_slots[i].enc_b);
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

int exp_motor_scurve_scen_init(void)
{
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        fprintf(stderr, "scurve exp: pwm_hal_register failed\n");
        return -1;
    }
    for (int t = 0; t < S_N_TIMERS; t++) {
        s_timers[t] = pwm_timer_create_preset(s_timer_presets[t]);
        if (s_timers[t] == NULL) {
            fprintf(stderr, "scurve exp: timer %s create failed\n",
                    s_timer_presets[t]);
            goto fail;
        }
    }
    for (int i = 0; i < EXP_MOTOR_SCURVE_SLOTS; i++) {
        const exp_slot_t *slot = &s_slots[i];
        const motor_cfg_preset_t *preset = preset_find(slot->motor_preset);
        if (preset == NULL) {
            fprintf(stderr, "scurve exp: motor preset %s not found\n",
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
            fprintf(stderr, "scurve exp: slot %d timer preset %s not in table\n",
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
            fprintf(stderr, "scurve exp: slot %d motor_create failed (%s)\n",
                    i, slot->motor_preset);
            goto fail;
        }
        s_mot_timer[i] = cfg.timer;
        s_mot_ch[i] = cfg.channel;
        s_kind[i] = (int)cfg.kind;   /* 帧 payload 的 kind 字段（底版单源） */
        s_cpr[i] = cfg.cpr;          /* 位置换算 cpr（底版单源；非 ENC 槽 0 不用） */

        /* ENC 专属步（序同 motor_scen.c）：先挂编码器 HAL（无副作用）再注册
         * plant——若其后失败即整体回滚（attach 失败不留 plant） */
        if (cfg.kind == MOTOR_ENC_CLOSED) {
            if (motor_hal_attach(s_motors[i], &sim_encoder_hal) != 0) {
                fprintf(stderr, "scurve exp: slot %d hal attach failed\n", i);
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
                .cpr = cfg.cpr,   /* cpr 单源自电机预设底版 */
                .dt_s = (float)EXP_MOTOR_SCURVE_POLL_US / 1000000.0f,
            };
            if (sim_motor_plant_register(&plant) != 0) {
                fprintf(stderr, "scurve exp: slot %d plant register failed\n", i);
                goto fail;
            }
        }

        /* S 梯度首腿 + 腿基线清零（PWM 积分器 / ENC 编码器基线） */
        if (scurve_init(&s_sc[i], &slot->scurve, slot->target_pos) != 0) {
            fprintf(stderr, "scurve exp: slot %d scurve_init failed\n", i);
            goto fail;
        }
        s_dwell[i] = EXP_MOTOR_SCURVE_DWELL_TICKS;   /* 首腿走完同样停驻 N 拍
                                                       （同 motor_demo 首腿 dwell
                                                       初始化——旧值 0 会让首腿
                                                       无停驻段立即换腿，自检实测
                                                       暴露） */
        s_leg_base_enc[i] = 0;   /* motor_create 后 enc_count 基线即 0 */
        s_pos_int[i] = 0.0;
    }
    return 0;

fail:
    scen_destroy();
    return -1;
}

/**
 * @brief   腿收尾 rewind：清位置基线 + scurve_init 复位重走（同一拍起新腿）
 * @note    停驻期满才触发；ENC 腿基线取当前编码器累计（上一腿已收敛于此），
 *          新腿位移从 0 重新起算——「到目标收敛」断言的采样基准。
 */
static void leg_rewind(int i)
{
    const exp_slot_t *slot = &s_slots[i];
    s_pos_int[i] = 0.0;
    if (s_kind[i] == (int)MOTOR_ENC_CLOSED) {
        motor_state_t st;
        if (motor_get_state(s_motors[i], &st) == 0)
            s_leg_base_enc[i] = st.enc_count;
    }
    if (scurve_init(&s_sc[i], &slot->scurve, slot->target_pos) != 0) {
        /* rewind 失败仅打印（scurve 安全态 = 不动），下一拍 done 分支重试 */
        fprintf(stderr, "scurve exp: slot %d rewind failed\n", i);
    }
    s_dwell[i] = EXP_MOTOR_SCURVE_DWELL_TICKS;
}

void exp_motor_scurve_scen_tick(uint32_t now_us,
                                exp_motor_tele_t out[EXP_MOTOR_SCURVE_SLOTS])
{
    (void)now_us;
    if (out == NULL) return;
    for (int i = 0; i < EXP_MOTOR_SCURVE_SLOTS; i++) {
        const exp_slot_t *slot = &s_slots[i];

        /* 腿状态机：走完（done）→ 停驻 N 拍（速度 0，闭环收敛）→ rewind */
        if (scurve_done(&s_sc[i])) {
            if (s_dwell[i] > 0) {
                s_dwell[i]--;
            } else {
                leg_rewind(i);
            }
        }

        /* S 梯度推进 → 速度前馈喂电机 → 一拍输出 → 遥测 */
        float pos = scurve_step(&s_sc[i]);
        scurve_state_t sc;
        if (scurve_get(&s_sc[i], &sc) != 0) {
            sc.pos = 0.0f; sc.vel = 0.0f; sc.acc = 0.0f;
            sc.phase = SCURVE_PHASE_DONE;
        }
        float vel_cmd = sc.vel * slot->vel_scale;
        (void)motor_set_target(s_motors[i], vel_cmd);
        (void)motor_poll(s_motors[i], now_us);

        motor_state_t st;
        if (motor_get_state(s_motors[i], &st) != 0) {
            st.target = 0.0f; st.actual = 0.0f; st.duty = 0.0f;
            st.enc_count = 0; st.fault = 0xFFFFFFFFu;
        }
        out[i].setpoint = vel_cmd;

        /* duty = 存储回读（物理输出面），PWM 槽单通道 / ENC 槽方向桥带符号：
         * 域外哨兵（-1 / -2）= 回读失败，界内断言必抓，不静默 */
        out[i].duty = -2.0f;
        if (s_kind[i] == (int)MOTOR_ENC_CLOSED) {
            sim_bridge_duty_t bd;
            if (sim_pwm_bridge_duty(s_mot_timer[i], s_mot_ch[i],
                                    (uint8_t)(s_mot_ch[i] + 1u), &bd) == 0)
                out[i].duty = bd.signed_duty;
        } else {
            out[i].duty = -1.0f;
            (void)sim_pwm_channel_duty(s_mot_timer[i], s_mot_ch[i], &out[i].duty);
        }

        out[i].actual = st.actual;
        out[i].enc_count = st.enc_count;
        out[i].fault = st.fault;
        out[i].phase = (int)sc.phase;
        out[i].kind = s_kind[i];
        out[i].pos_target = pos;

        /* 实际位置 [u]：PWM 槽 = duty 回读逐拍积分（开环可见位置面）；
         * ENC 槽 = 编码器位移 / cpr（腿基线起算，cpr 底版单源） */
        if (s_kind[i] == (int)MOTOR_ENC_CLOSED) {
            out[i].pos_actual =
                (float)(st.enc_count - s_leg_base_enc[i]) / (float)s_cpr[i];
        } else {
            float d = (out[i].duty >= 0.0f && out[i].duty <= 1.0f)
                    ? out[i].duty : 0.0f;   /* 哨兵拍不积分（断言必抓） */
            s_pos_int[i] += (double)d * (EXP_MOTOR_SCURVE_POLL_US / 1000000.0);
            out[i].pos_actual = (float)s_pos_int[i];
        }
    }
}

void exp_motor_scurve_scen_pack(const exp_motor_tele_t tele[EXP_MOTOR_SCURVE_SLOTS],
                                uint8_t payload[EXP_MOTOR_SCURVE_SLOTS * 16])
{
    /* 布局单源：proto/frame.h PROTO_TYPE_MOTOR 注释（N × 16B 小端记录，
     * 解码器按 payload 长度自适配——本实验 = 2×16B） */
    for (int i = 0; i < EXP_MOTOR_SCURVE_SLOTS; i++) {
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
