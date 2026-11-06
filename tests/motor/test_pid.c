/* SPDX-License-Identifier: MIT */
/**
 * @file    test_pid.c
 * @brief   PID 库单测——闭环收敛 / 预设注册表 / 抗饱和有界 / 回灌方向【关键】/
 *          饱和锁定 / 复位清态 / 参数非法拒 / 多实例独立
 *
 * 场景参数一律以命名常量写在各用例头部（场景即文档）；被控对象为合成一阶模型
 * dv = (u − v)/tau（欧拉离散），闭环结构为合成对象仿真。
 * 抗饱和回灌方向的断言用【带符号精确值】锁定（符号反了必红——量具先能红）。
 *
 * 运行（库根目录下）：
 *   gcc -std=c11 -Wall -Wextra -Isrc/motor \
 *       src/motor/pid.c src/motor/tests/test_pid.c -lm -o build/test_pid
 *   ./build/test_pid    （退出码 0 = 全绿）
 */
#include <math.h>     /* NAN / INFINITY 宏（编译期内建，无库调用） */
#include <stdio.h>
#include <string.h>   /* memset */

#include "pid.h"

static int g_failed = 0;   /* 失败断言计数（测试文件自有，非库状态） */
static int g_total = 0;

static void check(int cond, const char *what)
{
    g_total++;
    if (cond) {
        printf("  ok   %s\n", what);
    } else {
        g_failed++;
        printf("  FAIL %s\n", what);
    }
}

static int near_f(float a, float b, float tol)
{
    float d = a - b;
    if (d < 0.0f) {
        d = -d;
    }
    return d <= tol;
}

/* ---------- 1. 阶跃收敛：合成一阶被控对象闭环 ---------- */
static void test_step_convergence(void)
{
    /* 增益使闭环 v/sp = 20/(s+20)（tau=0.1, kp=2, ki=20 ⇒ 0.1s²+3s+20 =
       (s+10)(s+20)，kp/ki 的零点 -10 恰对消极点 -10）：一阶纯收敛、无超调 */
    const pid_cfg_t cfg = { .kp = 2.0f, .ki = 20.0f, .kd = 0.0f,
                            .out_min = -3.0f, .out_max = 3.0f, .kb = 1.0f };
    const float plant_tau = 0.1f;       /* 一阶时间常数 s */
    const float dt = 0.001f;            /* 步长 s */
    const float sp = 1.0f;              /* 阶跃目标 */
    const int   n_steps = 1000;         /* 总时长 1 s */
    const float tol_final = 1e-3f;      /* 终值容差 */
    const float tol_overshoot = 0.15f;  /* 超调上限 15% */

    pid_t pid;
    float v = 0.0f, v_peak = 0.0f, u_max_seen = -1e30f;
    int k, rc_all = 0;

    printf("[1] 阶跃收敛（一阶被控对象 dv=(u-v)/tau）\n");
    check(pid_init(&pid, &cfg) == 0, "init 成功");
    for (k = 0; k < n_steps; k++) {
        float e = sp - v;
        if (pid_step(&pid, e, dt) != 0) {
            rc_all = 1;
            break;
        }
        v = v + (pid.out - v) / plant_tau * dt;   /* 欧拉：dv=(u-v)/tau */
        if (v > v_peak) {
            v_peak = v;
        }
        if (pid.out > u_max_seen) {
            u_max_seen = pid.out;
        }
    }
    check(rc_all == 0, "1000 拍 step 全部返回 0");
    check(near_f(v, sp, tol_final), "终值收敛 |v - sp| < 1e-3");
    check(v_peak <= sp * (1.0f + tol_overshoot), "超调 < 15%");
    check(u_max_seen <= cfg.out_max + 1e-6f, "全程输出不越上限（限幅不变式）");
    printf("       v=%.6f  v_peak=%.6f  u_max=%.6f\n", v, v_peak, u_max_seen);
}

/* ---------- 2. 预设注册表 + 预设底版闭环冒烟 ---------- */
static void test_preset_registry(void)
{
    const float plant_tau = 0.1f;
    const float dt = 0.001f;
    const float sp = 40.0f;        /* duty% 量纲（与预设同口径） */
    const int   n_steps = 2000;    /* 2 s */
    const float tol_final = 0.5f;  /* 跟踪容差（err_ss < 0.5） */

    pid_t pid;
    const pid_cfg_t *found = pid_cfg_find("pid.vel_pi.default");
    float v = 0.0f;
    int k, rc_all = 0;

    printf("[2] 预设注册表\n");
    check(found != NULL, "find(\"pid.vel_pi.default\") 命中");
    check(pid_cfg_find(NULL) == NULL, "find(NULL) 拒");
    check(pid_cfg_find("") == NULL, "find(空串) 拒");
    check(pid_cfg_find("pid.no.such") == NULL, "find(未知名) 拒");
    check(found != NULL && near_f(found->kp, 2.0f, 1e-6f)
              && near_f(found->ki, 20.0f, 1e-6f)
              && near_f(found->out_min, -100.0f, 1e-6f)
              && near_f(found->out_max, 100.0f, 1e-6f)
              && near_f(found->kb, 1.0f, 1e-6f),
          "预设值锁定（kp=2/ki=20/±100/kb=1）");

    {   /* 全部预设 cfg 必须能通过 pid_init（预设合法性不变式） */
        int all_ok = 1;
        int i;
        for (i = 0; i < pid_cfg_preset_count; i++) {
            pid_t t;
            if (pid_init(&t, &pid_cfg_presets[i].cfg) != 0) {
                all_ok = 0;
            }
        }
        check(all_ok, "全部预设 cfg 可 init（合法性不变式）");
    }

    if (found != NULL) {
        check(pid_init(&pid, found) == 0, "预设 init 实例");
        for (k = 0; k < n_steps; k++) {
            if (pid_step(&pid, sp - v, dt) != 0) {
                rc_all = 1;
                break;
            }
            v = v + (pid.out - v) / plant_tau * dt;
        }
        check(rc_all == 0, "2000 拍 step 全部返回 0");
        check(near_f(v, sp, tol_final), "预设闭环 2s 收敛 |v - sp| < 0.5");
        printf("       v=%.4f（sp=%.1f）\n", v, sp);
    }
}

/* ---------- 3. 抗饱和有界：sp=100 连推 400 拍 ---------- */
static void test_windup_bounded(void)
{
    /* 隔离积分通道（kp=0 ⇒ u_unsat 即积分项）；误差恒 sp=100（meas=0）。
       饱和平衡：i_eq = out_max + ki·e/kb = 1 + 1000/5 = 201；
       无抗饱和线性外推：i_lin = ki·e·dt·N = 10·100·0.001·400 = 400 */
    const pid_cfg_t cfg = { .kp = 0.0f, .ki = 10.0f, .kd = 0.0f,
                            .out_min = -1.0f, .out_max = 1.0f, .kb = 5.0f };
    const float dt = 0.001f;
    const float sp = 100.0f;
    const int   n_ticks = 400;
    const float i_eq_bound = 201.0f;   /* 饱和平衡界（理论上界） */
    const float i_linear = 400.0f;     /* 无抗饱和线性外推值 */
    const float i_lo = 150.0f;         /* 实测约 174；过低 = 抑制过度 */

    pid_t pid;
    int k, rc_all = 0;

    printf("[3] 抗饱和有界（sp=100 连推 400 拍）\n");
    check(pid_init(&pid, &cfg) == 0, "init 成功");
    for (k = 0; k < n_ticks; k++) {
        if (pid_step(&pid, sp, dt) != 0) {   /* error = sp - 0 = 100 恒定 */
            rc_all = 1;
            break;
        }
    }
    check(rc_all == 0, "400 拍 step 全部返回 0");
    printf("       积分=%.3f  平衡界=%.1f  无抗饱和线性外推=%.1f\n",
           pid.integral, i_eq_bound, i_linear);
    check(pid.integral < i_eq_bound, "积分 < 饱和平衡界 201（有界）");
    check(pid.integral < i_linear, "积分 < 无抗饱和线性外推 400（对照更大）");
    check(pid.integral > i_lo, "积分 > 150（仍在积分，未被过度抑制）");
    check(near_f(pid.out, cfg.out_max, 1e-6f), "输出锁定 +out_max（深度饱和）");
}

/* ---------- 4. 【关键】饱和期间回灌方向 ---------- */
static void test_backfeed_direction(void)
{
    const pid_cfg_t cfg = { .kp = 1.0f, .ki = 1.0f, .kd = 0.0f,
                            .out_min = -10.0f, .out_max = 10.0f, .kb = 10.0f };
    const float e_big = 100.0f;   /* 远超限幅的恒定误差 */
    const float dt = 0.01f;
    pid_t pid;
    float i_before;

    printf("[4] 饱和期间回灌方向【关键】\n");
    check(pid_init(&pid, &cfg) == 0, "init 成功");

    /* 正向饱和：u_unsat = kp·e = 100 > 10 ⇒ 回灌 kb·(10-100)·dt = -9；
       净积分增量 = ki·e·dt + (-9) = 1 - 9 = -8 < 0（正误差下积分不增反降） */
    check(pid_step(&pid, e_big, dt) == 0, "正向饱和 step 返回 0");
    check(near_f(pid.out, 10.0f, 1e-5f), "正向饱和输出钳在 +out_max");
    check(near_f(pid.out_unsat, 100.0f, 1e-4f), "u_unsat = kp·e = 100");
    check(pid.integral < 0.0f, "正向饱和：积分被抑制（净减少，< 0）");
    check(near_f(pid.integral, -8.0f, 1e-4f),
          "回灌量精确锁定：i = ki·e·dt + kb·(u_sat-u_unsat)·dt = -8");

    /* 饱和持续：第二拍积分继续被压（仍 < 上一拍） */
    i_before = pid.integral;
    check(pid_step(&pid, e_big, dt) == 0, "第二拍 step 返回 0");
    check(pid.integral < i_before, "持续饱和：积分逐拍继续下降");

    /* 负向饱和镜像：u_unsat = -100 < -10 ⇒ 回灌为正，积分被顶回（> 0） */
    check(pid_init(&pid, &cfg) == 0, "重新 init");
    check(pid_step(&pid, -e_big, dt) == 0, "负向饱和 step 返回 0");
    check(near_f(pid.out, -10.0f, 1e-5f), "负向饱和输出钳在 out_min");
    check(pid.integral > 0.0f, "负向饱和：积分被抑制（净增加，> 0）");
    check(near_f(pid.integral, 8.0f, 1e-4f), "镜像回灌量 = +8 精确锁定");

    /* 不饱和边界：u == u_unsat ⇒ 回灌恰为 0（纯积分推进，无误伤） */
    check(pid_init(&pid, &cfg) == 0, "重新 init");
    check(pid_step(&pid, 5.0f, dt) == 0, "不饱和 step 返回 0");
    check(near_f(pid.out, pid.out_unsat, 0.0f), "不饱和时 u == u_unsat（零回灌）");
    check(near_f(pid.integral, 0.05f, 1e-6f), "不饱和时纯积分推进 ki·e·dt");

    /* 温和参数（净增量仍为正的常见工况）：饱和拍增量严格小于纯积分增量 */
    {
        const pid_cfg_t soft = { .kp = 0.0f, .ki = 10.0f, .kd = 0.0f,
                                 .out_min = -1.0f, .out_max = 1.0f, .kb = 5.0f };
        const float e_wind = 100.0f;
        const float dt_w = 0.001f;
        const int   warmup = 3;    /* 3 拍进入饱和（i=2.995 > out_max） */
        pid_t pid2;
        float i_prev, i_new;
        int j;

        check(pid_init(&pid2, &soft) == 0, "温和工况 init");
        for (j = 0; j < warmup; j++) {
            (void)pid_step(&pid2, e_wind, dt_w);
        }
        i_prev = pid2.integral;
        check(pid_step(&pid2, e_wind, dt_w) == 0, "温和工况饱和拍 step 返回 0");
        i_new = pid2.integral;
        check(i_new - i_prev < soft.ki * e_wind * dt_w - 0.005f,
              "饱和拍积分增量 < 纯积分增量（抑制方向成立）");
    }
}

/* ---------- 5. 闭环饱和锁定物理上限（不可达目标） ---------- */
static void test_saturation_lock(void)
{
    /* sp=1.0 需 u_ss=1.0，但 out_max=0.3（需求 333% ⇒ 长期饱和）；
       稳态 v 锁在 0.3（物理上限），积分有界（平衡 i≈12.3）。
       无抗饱和时积分线性外推 ≥ ki·e·t = 20·0.7·3 = 42。 */
    const pid_cfg_t cfg = { .kp = 2.0f, .ki = 20.0f, .kd = 0.0f,
                            .out_min = -0.3f, .out_max = 0.3f, .kb = 1.0f };
    const float plant_tau = 0.1f;
    const float dt = 0.001f;
    const float sp = 1.0f;
    const int   n_steps = 3000;    /* 3 s */
    const float v_limit = 0.3f;
    const float tol = 0.01f;
    const float i_bound = 20.0f;   /* > 平衡值约 12.3 */
    const float i_lo = 10.0f;      /* < 平衡值约 12.3 */

    pid_t pid;
    float v = 0.0f, v_peak = 0.0f;
    int k, rc_all = 0;

    printf("[5] 闭环饱和锁定（不可达目标）\n");
    check(pid_init(&pid, &cfg) == 0, "init 成功");
    for (k = 0; k < n_steps; k++) {
        if (pid_step(&pid, sp - v, dt) != 0) {
            rc_all = 1;
            break;
        }
        v = v + (pid.out - v) / plant_tau * dt;
        if (v > v_peak) {
            v_peak = v;
        }
    }
    check(rc_all == 0, "3000 拍 step 全部返回 0");
    check(near_f(v, v_limit, tol), "稳态锁定物理上限 v≈0.3");
    check(v_peak <= v_limit + tol, "全程不超上限（v_peak ≤ 0.31）");
    check(pid.integral < i_bound, "长期饱和下积分有界（< 20，线性外推 ≥ 42）");
    check(pid.integral > i_lo, "积分 > 10（方向为正且未被过度抑制）");
    printf("       v=%.5f  v_peak=%.5f  积分=%.3f\n", v, v_peak, pid.integral);
}

/* ---------- 6. pid_reset 清状态 ---------- */
static void test_reset_clears_state(void)
{
    const pid_cfg_t cfg = { .kp = 1.0f, .ki = 1.0f, .kd = 1.0f,
                            .out_min = -1000.0f, .out_max = 1000.0f, .kb = 1.0f };
    const float e_pre = 50.0f;
    const float dt = 0.01f;
    const int   warmup = 5;
    pid_t pid;
    int k, rc_all = 0;

    printf("[6] pid_reset 清状态\n");
    check(pid_init(&pid, &cfg) == 0, "init 成功");
    for (k = 0; k < warmup; k++) {
        if (pid_step(&pid, e_pre, dt) != 0) {
            rc_all = 1;
        }
    }
    check(rc_all == 0, "预热 5 拍返回 0");
    check(pid.integral != 0.0f && pid.out != 0.0f && pid.have_prev != 0,
          "复位前状态非零");

    check(pid_reset(&pid) == 0, "reset 返回 0");
    check(pid.integral == 0.0f && pid.out == 0.0f && pid.out_unsat == 0.0f
              && pid.have_prev == 0,
          "积分/输出/上拍误差全清零");
    check(near_f(pid.cfg.kp, 1.0f, 0.0f) && near_f(pid.cfg.kd, 1.0f, 0.0f),
          "配置保留（不随 reset 丢）");

    /* 复位后首拍须回到「首拍语义」：D 项 = 0。若 prev_error 状态泄漏，
       d = kd·(1-50)/0.05 = -980 ⇒ out_unsat 远偏离 1.0，断言即红 */
    check(pid_step(&pid, 1.0f, 0.05f) == 0, "复位后首拍 step 返回 0");
    check(near_f(pid.out_unsat, 1.0f, 1e-4f),
          "复位后首拍 = 纯 P（D 项确为 0，prev_error 真清了）");

    check(pid_reset(NULL) == -1, "reset(NULL) 拒");
}

/* ---------- 7. 参数非法拒 ---------- */
static void test_invalid_params(void)
{
    const pid_cfg_t good = { .kp = 1.0f, .ki = 1.0f, .kd = 0.0f,
                             .out_min = -10.0f, .out_max = 10.0f, .kb = 1.0f };
    pid_cfg_t bad;
    pid_t pid, probe;

    printf("[7] 参数非法拒\n");
    check(pid_init(NULL, &good) == -1, "init(NULL, cfg) 拒");
    check(pid_init(&pid, NULL) == -1, "init(pid, NULL) 拒");

    bad = good;  bad.out_min = bad.out_max;      /* out_min == out_max */
    check(pid_init(&probe, &bad) == -1, "out_min == out_max 拒");
    bad = good;  bad.out_min = 15.0f;            /* out_min > out_max */
    check(pid_init(&probe, &bad) == -1, "out_min > out_max 拒");
    bad = good;  bad.kb = -0.1f;
    check(pid_init(&probe, &bad) == -1, "kb < 0 拒");
    bad = good;  bad.kp = NAN;
    check(pid_init(&probe, &bad) == -1, "kp = NAN 拒");
    bad = good;  bad.ki = INFINITY;
    check(pid_init(&probe, &bad) == -1, "ki = INFINITY 拒");
    bad = good;  bad.out_max = NAN;
    check(pid_init(&probe, &bad) == -1, "out_max = NAN 拒");

    /* init 失败不得污染实例内容 */
    probe.integral = 12345.0f;
    probe.cfg.kp = 7.0f;
    bad = good;  bad.kb = -1.0f;
    check(pid_init(&probe, &bad) == -1, "init 失败返回 -1");
    check(near_f(probe.integral, 12345.0f, 0.0f)
              && near_f(probe.cfg.kp, 7.0f, 0.0f),
          "init 失败实例内容不变");

    check(pid_init(&pid, &good) == 0, "正常 init");
    check(pid_step(&pid, 1.0f, 0.01f) == 0, "正常 step 一拍");

    check(pid_step(NULL, 1.0f, 0.01f) == -1, "step(NULL, ...) 拒");
    check(pid_step(&pid, 1.0f, 0.0f) == -1, "dt = 0 拒");
    check(pid_step(&pid, 1.0f, -0.01f) == -1, "dt < 0 拒");
    check(pid_step(&pid, NAN, 0.01f) == -1, "error = NAN 拒");
    check(pid_step(&pid, INFINITY, 0.01f) == -1, "error = INFINITY 拒");
    check(pid_step(&pid, 1.0f, NAN) == -1, "dt = NAN 拒");

    {   /* 非法 step 不得改动状态 */
        float i_keep = pid.integral;
        (void)pid_step(&pid, NAN, 0.01f);
        check(near_f(pid.integral, i_keep, 0.0f), "非法 step 状态不变");
    }

    /* 零初始化（BSS/memset 语义）实例未 init 就 step ⇒ 拒 */
    memset(&probe, 0, sizeof probe);
    check(pid_step(&probe, 1.0f, 0.01f) == -1, "未 init（零初始化）的 step 拒");
}

/* ---------- 8. 多实例独立（零共享状态不变式） ---------- */
static void test_instances_independent(void)
{
    const pid_cfg_t cfg_a = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
                              .out_min = -100.0f, .out_max = 100.0f, .kb = 1.0f };
    const pid_cfg_t cfg_b = { .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,
                              .out_min = -100.0f, .out_max = 100.0f, .kb = 1.0f };
    const float dt = 0.01f;
    const float errs[4] = { 1.0f, -2.0f, 0.5f, 4.0f };
    pid_t pa, pb;
    int k, all_match = 1;

    printf("[8] 多实例互不干扰\n");
    check(pid_init(&pa, &cfg_a) == 0 && pid_init(&pb, &cfg_b) == 0,
          "双实例 init");
    for (k = 0; k < 4; k++) {
        (void)pid_step(&pa, errs[k], dt);
        (void)pid_step(&pb, errs[k], dt);
        /* ki=0 且不饱和 ⇒ out 恒 = kp·error（纯 P，逐拍独立可算） */
        if (!near_f(pa.out, cfg_a.kp * errs[k], 1e-5f) ||
            !near_f(pb.out, cfg_b.kp * errs[k], 1e-5f)) {
            all_match = 0;
        }
    }
    check(all_match, "交错步进下两实例各自精确跟随 kp·e（无共享状态）");
    check(pa.integral == 0.0f && pb.integral == 0.0f,
          "ki=0 且不饱和 ⇒ 积分恒 0（零回灌不变式）");
}

int main(void)
{
    printf("=== motor/pid 单测 ===\n");
    test_step_convergence();
    test_preset_registry();
    test_windup_bounded();
    test_backfeed_direction();
    test_saturation_lock();
    test_reset_clears_state();
    test_invalid_params();
    test_instances_independent();
    printf("=== 结果：%d/%d 断言通过，失败 %d ===\n",
           g_total - g_failed, g_total, g_failed);
    return g_failed == 0 ? 0 : 1;
}
