/* SPDX-License-Identifier: MIT */
/**
 * @file    exp_motor_pidscan_scen.c
 * @brief   PID 参数网格扫描实验运行时实现（契约见 exp_motor_pidscan_scen.h）
 *
 * @note    参数源同构：实验表 = 本文件网格/步表/hold/wiring/plant 段（参数零
 *          写死，改实验只改本段）；电机底版 = 库预设 motor.enc.n20（wiring
 *          填法与整定来源注释同 exp_motor_enc_scen.c，scenario 3 同源）。
 * @note    组间轮转语义：每组 group_begin 用网格值覆盖 vel_pid.kp/ki 重建电机
 *          （其余字段 = 底版原值：kd=0/限幅 ±1/duty_max=1）；同一 plant 常驻
 *          （引脚对 8/9 不变，控制环 PID→桥双通道 与 反馈环 编码器差分 在纯
 *          RAM 闭环）。motor_destroy 清零桥双通道占空（motor.c:189-197）——
 *          下一组低段前导（1s）远大于 τ=20ms，组间无残留耦合。
 * @note    状态有家：本模块持全部实例句柄与扫描状态，static 限文件内——
 *          固件与 host gcc 自检各自链接一份，互不干扰（同 exp_motor_enc）。
 * @note    失败原子性：init 中途失败全量回滚；group_begin 失败不留半挂
 *          （旧组已销毁，新组未建）；group_end 后电机必然已销毁。
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "exp_motor_pidscan_scen.h"
#include "motor_hal_sim.h"
#include "proto/frame.h"

/* ── 实验表（参数零写死：改实验只改本段） ──────────────────────────────── */

/** 步进目标（组内两段：低段前导 + 高段统计窗；域 = rpm） */
static const float s_steps[2] = { 30.0f, 90.0f };
/** 每段保持拍数（2ms 拍：低段 1s 前导 / 高段 2s 统计窗） */
static const uint32_t s_hold[2] = { 500u, 1000u };
enum { S_LO = 0, S_HI = 1 };       /* 步号（phase 字段域） */

/** 组描述（网格展开表：g = kp_idx*3 + ki_idx） */
typedef struct {
    float kp;   /**< 覆盖 vel_pid.kp */
    float ki;   /**< 覆盖 vel_pid.ki */
} exp_pid_grid_t;

/**
 * @brief   3×3 网格（行 = kp {0.006, 0.010, 0.015} 升序（上限 = 底版 kp
 *          0.015，向下采样），列 = ki {1.0, 1.5, 2.0} 升序（下限 = 底版
 *          ki 1.0，向上采样））
 * @note    参数区选择（整定依据，改网格须重验单调性断言）：本 plant
 *          （160 rpm/duty、τ=20ms、2ms 拍）的闭环特征方程
 *          0.02s²+(1+160kp)s+160ki=0，ζ = (1+160kp)/(2·√(3.2·ki))——本网格
 *          全体 ζ ∈ [0.39, 0.95] 欠阻尼：同 ki 列 kp↑ → ζ↑ → 到带时间单调
 *          缩短（「kp 增大响应加快」成立的参数区；ki ≤0.5 且 kp ≥0.022 会
 *          落入深过阻尼区，慢极点反而随 kp 增大而变慢——host 自检实测
 *          暴露，勿退回该区）。
 */
static const exp_pid_grid_t s_grid[EXP_MOTOR_PIDSCAN_GROUPS] = {
    /* 行主序展开（行 = kp 升序，列 = ki 升序） */
    { 0.006f, 1.00f }, { 0.006f, 1.50f }, { 0.006f, 2.00f },
    { 0.010f, 1.00f }, { 0.010f, 1.50f }, { 0.010f, 2.00f },
    { 0.015f, 1.00f }, { 0.015f, 1.50f }, { 0.015f, 2.00f },
};

/** 单槽 wiring + plant（全组共用：底版 motor.enc.n20 @ pwm.motor.20k ch0） */
static const char *const s_timer_preset = "pwm.motor.20k";
static const char *const s_motor_preset = "motor.enc.n20";
static const uint8_t s_channel = 0;   /* 桥正向通道（+1 = 反向臂） */
static const int    s_pin_a = 6, s_pin_b = 7;
static const int    s_enc_a = 8, s_enc_b = 9;
static const float  s_plant_rpm_per_duty = 160.0f;   /* 同 enc 实验（同源） */
static const float  s_plant_tau_s = 0.02f;

/* ── 实例句柄与扫描状态（装配序建立，运行期只读/组内推进） ── */
static pwm_timer_t *s_timer = NULL;          /* 唯一定时器（全组共用） */
static motor_t     *s_motor = NULL;          /* 当前组电机（组间轮转） */
static int          s_cur_group = -1;        /* 当前组号（-1 = 无活组） */
static int          s_kind = 0;              /* motor_kind_t（底版单源，帧用） */
static int          s_phase = 0;             /* 组内步号（0 低段 / 1 高段） */
static uint32_t     s_phase_ticks = 0;       /* 当前步已拍数 */

/* 组指标累积态（进高段清零；group_end 定案进 s_results） */
static float    s_peak = 0.0f;       /* 高段 actual 峰值 */
static int32_t  s_last_out_ph = -1;  /* 最后一次出 ±3% 带的高段内拍号（-1 = 未出） */
static double   s_tail_sum = 0.0;    /* 高段末半段 actual 累计 */
static uint32_t s_tail_cnt = 0;      /* 高段末半段拍数 */

static exp_pid_result_t s_results[EXP_MOTOR_PIDSCAN_GROUPS];   /* 组结果表 */

/**
 * @brief   按名查电机预设整项（cfg 底版 + timer_preset 建议名一起拿；
 *          motor_cfg_find 只回 cfg——wiring 需要预设项的建议定时器名）
 */
static const motor_cfg_preset_t *preset_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < motor_cfg_preset_count; i++)
        if (strcmp(motor_cfg_presets[i].name, name) == 0)
            return &motor_cfg_presets[i];
    return NULL;
}

/** @brief 结果表清哨兵（init 一次；未跑组如实 = 不收敛态） */
static void results_reset(void)
{
    for (int g = 0; g < EXP_MOTOR_PIDSCAN_GROUPS; g++) {
        s_results[g].settle_s = -1.0f;
        s_results[g].overshoot_pct = -1.0f;
        s_results[g].ss_err_pct = -1.0f;
        s_results[g].score = -1.0f;
        s_results[g].ok = 0;
    }
}

/** @brief 组统计态清零（group_begin 与进高段时） */
static void stats_reset(void)
{
    s_peak = 0.0f;
    s_last_out_ph = -1;
    s_tail_sum = 0.0;
    s_tail_cnt = 0;
}

/** @brief 销毁活电机（若电机活着；不动定时器与 plant——组间常驻语义） */
static void motor_teardown(void)
{
    if (s_motor != NULL) {
        (void)motor_destroy(s_motor);   /* 内部清零桥双通道 + 还通道 */
        s_motor = NULL;
    }
    s_cur_group = -1;
}

int exp_motor_pidscan_scen_init(void)
{
    if (pwm_hal_register(&sim_pwm_hal) != 0) {
        fprintf(stderr, "pidscan exp: pwm_hal_register failed\n");
        return -1;
    }
    s_timer = pwm_timer_create_preset(s_timer_preset);
    if (s_timer == NULL) {
        fprintf(stderr, "pidscan exp: timer %s create failed\n", s_timer_preset);
        return -1;
    }

    const motor_cfg_preset_t *preset = preset_find(s_motor_preset);
    if (preset == NULL) {
        fprintf(stderr, "pidscan exp: motor preset %s not found\n", s_motor_preset);
        goto fail;
    }
    s_kind = (int)preset->cfg.kind;   /* 帧 payload kind 字段（底版单源） */

    /* plant 常驻注册一次（全组共用：引脚对 8/9 不变，与电机实例解耦——
     * register 只查 enc 引脚对唯一性，不依赖电机存在）。每组电机由
     * group_begin 轮转建立（首组与后续同路径）；组末 motor_destroy 清零
     * 桥双通道，plant 反馈环起点为零速——组间无残留耦合。 */
    {
        sim_motor_plant_cfg_t plant = {
            .timer = s_timer,
            .ch_fwd = s_channel,
            .ch_rev = (uint8_t)(s_channel + 1u),
            .enc_a = s_enc_a,
            .enc_b = s_enc_b,
            .rpm_per_duty = s_plant_rpm_per_duty,
            .tau_s = s_plant_tau_s,
            .cpr = preset->cfg.cpr,   /* cpr 单源自电机预设底版 */
            .dt_s = (float)EXP_MOTOR_PIDSCAN_POLL_US / 1000000.0f,
        };
        if (sim_motor_plant_register(&plant) != 0) {
            fprintf(stderr, "pidscan exp: plant register failed\n");
            goto fail;
        }
    }

    results_reset();
    s_cur_group = -1;
    return 0;

fail:
    motor_teardown();
    if (s_timer != NULL) {
        (void)pwm_timer_destroy(s_timer);
        s_timer = NULL;
    }
    return -1;
}

void exp_motor_pidscan_scen_destroy(void)
{
    motor_teardown();
    if (s_timer != NULL) {
        (void)sim_motor_plant_unregister(s_enc_a, s_enc_b);
        (void)pwm_timer_destroy(s_timer);
        s_timer = NULL;
    }
}

uint32_t exp_motor_pidscan_group_ticks(void)
{
    return s_hold[S_LO] + s_hold[S_HI];   /* 500 + 1000 = 1500（表单源） */
}

int exp_motor_pidscan_group_begin(int g)
{
    if (g < 0 || g >= EXP_MOTOR_PIDSCAN_GROUPS) return -1;
    if (s_timer == NULL) return -1;   /* 未 init（装配序守护） */

    motor_teardown();   /* 销毁上一组（若活）；桥双通道清零、还通道 */

    const motor_cfg_preset_t *preset = preset_find(s_motor_preset);
    if (preset == NULL) {
        fprintf(stderr, "pidscan exp: motor preset %s not found\n", s_motor_preset);
        return -1;
    }

    motor_cfg_t cfg = preset->cfg;         /* 值语义拷底版，再填 wiring + 网格 */
    cfg.timer = s_timer;
    cfg.channel = s_channel;
    cfg.pin_a = s_pin_a;
    cfg.pin_b = s_pin_b;
    cfg.enc_a = s_enc_a;
    cfg.enc_b = s_enc_b;
    cfg.vel_pid.kp = s_grid[g].kp;         /* ★ 网格覆盖（其余 vel_pid 字段 = 底版） */
    cfg.vel_pid.ki = s_grid[g].ki;

    s_motor = motor_create(&cfg);
    if (s_motor == NULL) {
        fprintf(stderr, "pidscan exp: group %d motor_create failed (kp=%g ki=%g)\n",
                g, s_grid[g].kp, s_grid[g].ki);
        return -1;   /* 旧组已销毁 + 新组未建：状态干净，可重试 group_begin */
    }
    s_kind = (int)cfg.kind;

    if (motor_hal_attach(s_motor, &sim_encoder_hal) != 0) {
        fprintf(stderr, "pidscan exp: group %d hal attach failed\n", g);
        motor_teardown();
        return -1;
    }

    s_cur_group = g;
    s_phase = 0;
    s_phase_ticks = 0;
    stats_reset();
    return 0;
}

void exp_motor_pidscan_scen_tick(uint32_t now_us, exp_motor_tele_t *out)
{
    (void)now_us;
    if (out == NULL) return;

    if (s_motor == NULL || s_cur_group < 0) {
        out->setpoint = 0.0f;
        out->duty = -1.0f;
        out->actual = 0.0f;
        out->enc_count = 0;
        out->fault = 0xFFFFFFFFu;   /* 无活组：哨兵行（调序错误可见） */
        out->phase = 0;
        out->kind = s_kind;
        return;
    }

    /* 步进发生器：当前步拍满 → 换步（两步循环：低段前导 → 高段统计窗） */
    if (s_phase_ticks >= s_hold[s_phase]) {
        s_phase = (s_phase == S_LO) ? S_HI : S_LO;
        s_phase_ticks = 0;
        if (s_phase == S_HI) stats_reset();   /* 进高段：统计窗起点 */
    }
    const uint32_t ph_t = s_phase_ticks;
    s_phase_ticks++;

    const float sp = s_steps[s_phase];
    (void)motor_set_target(s_motor, sp);
    (void)motor_poll(s_motor, now_us);

    motor_state_t st;
    if (motor_get_state(s_motor, &st) != 0) {
        st.target = 0.0f; st.actual = 0.0f; st.duty = 0.0f;
        st.enc_count = 0; st.fault = 0xFFFFFFFFu;
    }
    out->setpoint = sp;
    out->duty = -1.0f;   /* 存储回读（物理输出面）：界外哨兵 -1 = 回读失败 */
    (void)sim_pwm_channel_duty(s_timer, s_channel, &out->duty);
    out->actual = st.actual;
    out->enc_count = st.enc_count;
    out->fault = st.fault;
    out->phase = s_phase;
    out->kind = s_kind;

    /* 指标累积（高段统计窗；判据口径与 exp_motor_enc 同源：±3% 带） */
    if (s_phase == S_HI) {
        const float hi = s_steps[S_HI];
        const float band = hi * (EXP_MOTOR_PIDSCAN_SS_TOL_PCT / 100.0f);
        if (out->actual > s_peak) s_peak = out->actual;
        if (out->actual < hi - band || out->actual > hi + band)
            s_last_out_ph = (int32_t)ph_t;   /* 出带时刻（收敛时间分母） */
        if (ph_t >= s_hold[S_HI] / 2u) {
            s_tail_sum += out->actual;       /* 末半段（稳态误差窗） */
            s_tail_cnt++;
        }
    }
}

int exp_motor_pidscan_group_end(int g, exp_pid_result_t *out)
{
    if (out == NULL || g < 0 || g >= EXP_MOTOR_PIDSCAN_GROUPS) return -1;
    if (s_motor == NULL || s_cur_group != g) return -1;

    const float hi = s_steps[S_HI];
    const float step_amp = s_steps[S_HI] - s_steps[S_LO];   /* 阶跃幅度（超调分母） */

    /* 稳态误差：高段末半段均值（窗至少 1 拍——hold 表保证 1000/2 拍） */
    double mean = (s_tail_cnt > 0) ? (s_tail_sum / (double)s_tail_cnt) : 0.0;
    double ss_err = (s_tail_cnt > 0)
                  ? (fabs(mean - (double)hi) / (double)hi * 100.0) : 1e9;

    /* 收敛时间 = 末次出带时刻 + 1 拍（从阶跃起点到「进入 ±3% 带且此后不出」
     * 的时长；末拍在带内才有效）。末次出带涵盖超调回落穿带——超调大的组
     * 收敛时间如实更长。poll_enc 首拍建基线不控 → 低段首拍误差大不碍：
     * 统计窗只在高段（进高段 stats_reset）。 */
    int32_t last_hold = (int32_t)s_hold[S_HI] - 1;
    exp_pid_result_t r;
    r.overshoot_pct = (s_peak > hi)
                    ? (float)((s_peak - (double)hi) / (double)step_amp * 100.0)
                    : 0.0f;
    r.ss_err_pct = (float)ss_err;
    r.settle_s = -1.0f;
    r.score = -1.0f;
    r.ok = 0;
    if (s_last_out_ph >= 0 && s_last_out_ph < last_hold &&
        ss_err <= (double)EXP_MOTOR_PIDSCAN_SS_TOL_PCT) {
        double settle_ticks = (double)(s_last_out_ph + 1);
        r.settle_s = (float)(settle_ticks * (EXP_MOTOR_PIDSCAN_POLL_US / 1000000.0));
        r.score = r.settle_s + r.overshoot_pct / 100.0f + r.ss_err_pct / 100.0f;
        r.ok = 1;
    }

    motor_teardown();   /* 销毁电机（桥双通道清零 + 还通道）；plant/定时器常驻 */
    s_results[g] = r;
    *out = r;
    return 0;
}

void exp_motor_pidscan_get_result(int g, exp_pid_result_t *out)
{
    if (out == NULL) return;
    if (g < 0 || g >= EXP_MOTOR_PIDSCAN_GROUPS) {
        out->settle_s = -1.0f; out->overshoot_pct = -1.0f;
        out->ss_err_pct = -1.0f; out->score = -1.0f; out->ok = 0;
        return;
    }
    *out = s_results[g];
}

int exp_motor_pidscan_best_group(void)
{
    int best = -1;
    for (int g = 0; g < EXP_MOTOR_PIDSCAN_GROUPS; g++) {
        if (!s_results[g].ok) continue;   /* 不收敛组不参评（如实） */
        if (best < 0 || s_results[g].score < s_results[best].score)
            best = g;   /* 严格小于：并列取组号小（可复现） */
    }
    return best;
}

size_t exp_motor_pidscan_summary_line(int g, char *buf, size_t cap, int mark_best)
{
    if (buf == NULL || cap == 0 || g < 0 || g >= EXP_MOTOR_PIDSCAN_GROUPS)
        return 0;
    exp_pid_result_t r = s_results[g];
    const char *mark = "";
    if (mark_best && r.ok) {
        int best = exp_motor_pidscan_best_group();
        if (g == best) mark = " *";
    } else if (!r.ok) {
        mark = " !";   /* 不收敛组如实：' !' 标注（对比表不撒谎） */
    }
    int n;
    if (r.ok) {
        n = snprintf(buf, cap, "G%d kp%g ki%g st%.3fs ov%.1f%% se%.2f%%%s",
                     g, s_grid[g].kp, s_grid[g].ki,
                     r.settle_s, r.overshoot_pct, r.ss_err_pct, mark);
    } else {
        n = snprintf(buf, cap, "G%d kp%g ki%g stN/A ov%.1f%% se%.2f%%%s",
                     g, s_grid[g].kp, s_grid[g].ki,
                     r.overshoot_pct, r.ss_err_pct, mark);
    }
    return (n < 0) ? 0 : (size_t)n;
}

size_t exp_motor_pidscan_header_line(char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return 0;
    int n = snprintf(buf, cap, "PIDSCAN 3x3: g kp ki settle_s ov%% se%%");
    return (n < 0) ? 0 : (size_t)n;
}

void exp_motor_pidscan_scen_pack(const exp_motor_tele_t *tele, uint8_t payload[16])
{
    /* 布局单源：proto/frame.h PROTO_TYPE_MOTOR 注释（1×16B 小端记录）。
     * 本工程 slot 域 = 扫描组号（单电机轮转；见 scen.h 契约注）。 */
    uint8_t *p = payload;
    p[0] = (uint8_t)(s_cur_group & 0xFFu);   /* 组号（-1 时 0xFF 哨兵可见） */
    p[1] = (uint8_t)tele->kind;
    memcpy(p + 2, &tele->setpoint, 4);   /* host/x86 与 Xtensa 均小端 */
    memcpy(p + 6, &tele->duty, 4);
    memcpy(p + 10, &tele->actual, 4);
    p[14] = (uint8_t)tele->phase;
    p[15] = (uint8_t)(tele->fault & 0xFFu);
}
