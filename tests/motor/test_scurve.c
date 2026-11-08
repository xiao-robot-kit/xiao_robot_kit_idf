/* SPDX-License-Identifier: MIT */
/**
 * @file    test_scurve.c
 * @brief   S 梯度轨迹发生器单测（旧式实现 test_scurve.c 全用例移植 + 任务指定新边界）
 *
 * 旧→新用例映射（旧式实现 embedded/testing/test_scurve.c，表回放语义 → 闭式生成语义）：
 *   旧① 逐步表回放 + STOP 停机  → 块1 七相位全程回放（边界逐一断言）+ done 恒终值
 *   旧② 分级表重复步数          → 块2 匀加/匀速段多步恒斜率（每步 Δv/Δp 恒定）
 *   旧③ HOLD_CRUISE 末项保持    → 块3 巡航段 100 步 v 恒 vmax、p 线性推进
 *   旧④ rewind 复位重走加速     → 块4 重新 init 复位重走（前缀逐拍一致）
 *   旧⑤ 空表合法立即 done       → 块5 零行程 target=0 合法、恒 0
 *   旧⑥ 表内 0 项拒 + 安全态    → 块6 非法 cfg/target 拒 + 不动安全态
 *   旧⑦ NULL 表拒               → 块7 NULL / 零初始化结构体全防御
 * 新增边界（任务指定）：
 *   块8  dt 整除总时长末点不重复（防旧式实现 host poly_profile「整除末点重复」雷）
 *   块9  短行程退化（三角 6 相 / 超短 4 相 / d_full 精确边界 / vmax<vswitch 5 相）
 *   块10 中途改 target 重规划（保守停稳再走；非法 retarget 保旧计划）
 *   块11 负方向镜像
 *   块12 预设表
 *
 * 网格对齐主 cfg（dyadic：所有段时长/边界值/速度均为 2 的幂次分母，双精度与
 * float32 均精确表示，相位翻转步数可精确断言）：
 *   {vmax=2, amax=4, jerk=256, dt=1/64}：tj=amax/jerk=1/64（1 步）、
 *   v_switch=amax²/jerk=0.0625、ta=(2−0.0625)/4=31/64（31 步）、
 *   d(vmax)=33/64、d_full=66/64=1.03125。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mtr/scurve.h"

static int s_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); s_fail = 1; } \
} while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

/** dyadic 网格对齐主配置（见文件头注） */
static const scurve_cfg_t CFG_D = { 2.0f, 4.0f, 256.0f, 0.015625f };

static const char *ph_name(scurve_phase_t ph)
{
    switch (ph) {
    case SCURVE_PHASE_JERK_UP1: return "JERK_UP1";
    case SCURVE_PHASE_ACC:      return "ACC";
    case SCURVE_PHASE_JERK_DN1: return "JERK_DN1";
    case SCURVE_PHASE_CRUISE:   return "CRUISE";
    case SCURVE_PHASE_JERK_DN2: return "JERK_DN2";
    case SCURVE_PHASE_DEC:      return "DEC";
    case SCURVE_PHASE_JERK_UP2: return "JERK_UP2";
    default:                    return "DONE";
    }
}

/** 走到 done：返回总步数（超 max_steps 未完成返回 -1）；记录观测 |v|/|a| 峰值 */
static int run_to_done(scurve_t *s, int max_steps, double *v_pk, double *a_pk)
{
    scurve_state_t st;
    double vp = 0.0, ap = 0.0;
    int k = 0;
    while (!scurve_done(s)) {
        if (k >= max_steps)
            return -1;
        (void)scurve_step(s);
        if (scurve_get(s, &st) == 0) {
            if (fabs((double)st.vel) > vp) vp = fabs((double)st.vel);
            if (fabs((double)st.acc) > ap) ap = fabs((double)st.acc);
        }
        k++;
    }
    if (v_pk) *v_pk = vp;
    if (a_pk) *a_pk = ap;
    return k;
}

/** 相位序列采样（含 t=0 初相与 DONE 终相，相邻去重）；返回总步数 */
static int phase_seq(scurve_t *s, int max_steps, scurve_phase_t *seq, int *n_seq)
{
    scurve_state_t st;
    int k = 0, n = 0;
    seq[n++] = scurve_phase(s);
    while (!scurve_done(s)) {
        if (k >= max_steps)
            return -1;
        (void)scurve_step(s);
        if (scurve_get(s, &st) == 0 && st.phase != seq[n - 1])
            seq[n++] = st.phase;
        k++;
    }
    if (seq[n - 1] != SCURVE_PHASE_DONE)
        seq[n++] = SCURVE_PHASE_DONE;
    *n_seq = n;
    return k;
}

static int seq_has(const scurve_phase_t *seq, int n, scurve_phase_t ph)
{
    int i;
    for (i = 0; i < n; i++)
        if (seq[i] == ph) return 1;
    return 0;
}

int main(void)
{
    /* ①（旧①）七相位全程回放：dyadic 网格对齐，切换点/取值/界内/单调逐一断言 */
    {
        scurve_t s;
        scurve_state_t st;
        const int    bk[7]  = { 1, 32, 33, 65, 66, 97, 98 };
        const scurve_phase_t bp[7] = {
            SCURVE_PHASE_ACC, SCURVE_PHASE_JERK_DN1, SCURVE_PHASE_CRUISE,
            SCURVE_PHASE_JERK_DN2, SCURVE_PHASE_DEC,
            SCURVE_PHASE_JERK_UP2, SCURVE_PHASE_DONE };
        const double bv[7]  = { 0.03125, 1.96875, 2.0, 2.0, 1.96875, 0.03125, 0.0 };
        const double ba[7]  = { 4.0, 4.0, 0.0, 0.0, -4.0, -4.0, 0.0 };
        const double bps[7] = { 1.6276041666666667e-4, 0.48453776041666667,
                                0.515625, 1.515625, 1.5467122395833333,
                                2.0310872395833333, 2.03125 };
        double vp = 0.0, ap = 0.0;
        float prev_p = 0.0f;
        int k, bi = 0;

        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        CHECK(scurve_done(&s) == 0);
        CHECK(scurve_get(&s, &st) == 0);                    /* t=0 静止起点 */
        NEAR(st.pos, 0.0, 1e-12); NEAR(st.vel, 0.0, 1e-12); NEAR(st.acc, 0.0, 1e-12);
        CHECK(st.phase == SCURVE_PHASE_JERK_UP1);

        for (k = 1; k <= 98; k++) {
            float p = scurve_step(&s);
            CHECK(scurve_get(&s, &st) == 0);
            CHECK(p > prev_p);                              /* 严格单调：无平台无重复 */
            prev_p = p;
            CHECK((double)st.vel <= 2.0 + 1e-9 && (double)st.vel >= -1e-9);
            CHECK(fabs((double)st.acc) <= 4.0 + 1e-9);
            if (fabs((double)st.vel) > vp) vp = fabs((double)st.vel);
            if (fabs((double)st.acc) > ap) ap = fabs((double)st.acc);
            if (bi < 7 && k == bk[bi]) {                    /* 相位切换点断言 */
                if (st.phase != bp[bi])
                    printf("  k=%d phase=%s want=%s\n", k,
                           ph_name(st.phase), ph_name(bp[bi]));
                CHECK(st.phase == bp[bi]);
                NEAR(st.vel, bv[bi], 1e-9);
                NEAR(st.acc, ba[bi], 1e-9);
                NEAR(st.pos, bps[bi], 1e-6);
                NEAR(p, bps[bi], 1e-6);
                bi++;
            }
        }
        CHECK(bi == 7);
        CHECK(scurve_done(&s) == 1);
        NEAR(vp, 2.0, 1e-9);
        NEAR(ap, 4.0, 1e-9);
        NEAR(scurve_step(&s), 2.03125, 0.0);                /* 旧①『走完恒 0』→ 恒终值 */
        NEAR(scurve_step(&s), 2.03125, 0.0);
        NEAR(scurve_step(&s), 2.03125, 0.0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 2.03125f && st.vel == 0.0f && st.acc == 0.0f);
        CHECK(st.phase == SCURVE_PHASE_DONE);
        printf("ok: 七相位全程回放（7 切换点/界内/单调/done 恒终值）\n");
    }

    /* ②（旧②）匀加/匀速段多步恒斜率：分级重复的闭式同构 */
    {
        scurve_t s;
        scurve_state_t st, prev;
        int k;

        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        (void)scurve_step(&s);                              /* k=1 进 ACC */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.phase == SCURVE_PHASE_ACC);
        prev = st;
        for (k = 2; k <= 31; k++) {                         /* 匀加窗：每步 Δv = amax·dt */
            (void)scurve_step(&s);
            CHECK(scurve_get(&s, &st) == 0);
            CHECK(st.phase == SCURVE_PHASE_ACC);
            NEAR((double)st.vel - (double)prev.vel, 0.0625, 1e-12);
            prev = st;
        }
        (void)scurve_step(&s);                              /* k=32：切 JERK_DN1，斜率不变 */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.phase == SCURVE_PHASE_JERK_DN1);
        NEAR((double)st.vel - (double)prev.vel, 0.0625, 1e-12);
        NEAR(st.vel, 1.96875, 1e-9);
        prev = st;
        (void)scurve_step(&s);                              /* k=33：进 CRUISE，v 到 vmax */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.phase == SCURVE_PHASE_CRUISE);
        NEAR(st.vel, 2.0, 1e-12);
        NEAR(st.pos, 0.515625, 1e-9);
        prev = st;
        for (k = 34; k <= 65; k++) {                        /* 匀速窗：每步 Δp = vmax·dt */
            (void)scurve_step(&s);
            CHECK(scurve_get(&s, &st) == 0);
            NEAR(st.vel, 2.0, 1e-12);
            NEAR((double)st.pos - (double)prev.pos, 0.03125, 1e-12);
            if (k <= 64) CHECK(st.phase == SCURVE_PHASE_CRUISE);
            if (k == 65) CHECK(st.phase == SCURVE_PHASE_JERK_DN2);
            prev = st;
        }
        printf("ok: 匀加每步 Δv=amax·dt / 匀速全平坦 Δp=vmax·dt（旧②同构）\n");
    }

    /* ③（旧③ HOLD_CRUISE）巡航保持：长巡航 100 步 v 恒 vmax、p 线性推进 */
    {
        scurve_t s;
        scurve_state_t st, prev;
        double vp = 0.0, ap = 0.0;
        int i, rest;

        CHECK(scurve_init(&s, &CFG_D, 5.03125f) == 0);      /* 巡航 128 步：T=194·dt */
        for (i = 0; i < 33; i++) (void)scurve_step(&s);     /* 跳到巡航起点 */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.phase == SCURVE_PHASE_CRUISE);
        NEAR(st.vel, 2.0, 1e-9);
        prev = st;
        for (i = 0; i < 100; i++) {                          /* 旧③的 100 次恒值 */
            (void)scurve_step(&s);
            CHECK(scurve_get(&s, &st) == 0);
            NEAR(st.vel, 2.0, 1e-9);
            NEAR((double)st.pos - (double)prev.pos, 0.03125, 1e-9);
            prev = st;
        }
        rest = run_to_done(&s, 200, &vp, &ap);
        CHECK(rest == 61);                                   /* 194 − 133 剩余步数 */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 5.03125f && st.vel == 0.0f);
        printf("ok: 巡航 100 步恒 vmax 线性推进（旧③ HOLD_CRUISE 同构）\n");
    }

    /* ④（旧④ rewind）重新 init = 复位重走：前缀逐拍一致 */
    {
        scurve_t s;
        float p1[20], p2[20];
        int i;
        double vp = 0.0, ap = 0.0;

        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (i = 0; i < 20; i++) p1[i] = scurve_step(&s);
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);      /* rewind 同构：重装计划 */
        CHECK(scurve_done(&s) == 0);
        for (i = 0; i < 20; i++) {
            p2[i] = scurve_step(&s);
            CHECK(p2[i] == p1[i]);                           /* 同确定性计算：逐拍相等 */
        }
        CHECK(run_to_done(&s, 200, &vp, &ap) == 78);         /* 98 − 20 */
        CHECK(scurve_done(&s) == 1);
        printf("ok: 重新 init 复位重走（旧④ rewind 同构）\n");
    }

    /* ⑤（旧⑤ 空表）零行程：target=0 合法、立即 done、恒 0 */
    {
        scurve_t s;
        scurve_state_t st;

        CHECK(scurve_init(&s, &CFG_D, 0.0f) == 0);
        CHECK(scurve_done(&s) == 1);
        NEAR(scurve_step(&s), 0.0, 0.0);
        NEAR(scurve_step(&s), 0.0, 0.0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 0.0f && st.vel == 0.0f && st.acc == 0.0f);
        CHECK(st.phase == SCURVE_PHASE_DONE);
        printf("ok: 零行程合法立即 done（旧⑤ 空表同构）\n");
    }

    /* ⑥（旧⑥ 表内 0 项）非法 cfg 拒 + 不动安全态（一步都不输出） */
    {
        scurve_t s;
        scurve_state_t st;
        const scurve_cfg_t bad[] = {
            { 0.0f, 4.0f, 256.0f, 0.015625f },   /* vmax = 0 */
            { 2.0f, 0.0f, 256.0f, 0.015625f },   /* amax = 0 */
            { 2.0f, 4.0f, 0.0f, 0.015625f },     /* jerk = 0（除零源） */
            { 2.0f, 4.0f, 256.0f, 0.0f },        /* dt = 0 */
            { -2.0f, 4.0f, 256.0f, 0.015625f },  /* 负参数 */
            { 2.0f, 4.0f, 256.0f, -0.01f },      /* 负 dt */
            { NAN, 4.0f, 256.0f, 0.015625f },    /* NaN */
            { 2.0f, 4.0f, 256.0f, NAN },         /* NaN dt */
            { HUGE_VALF, 4.0f, 256.0f, 0.015625f }, /* Inf */
        };
        const int n_bad = (int)(sizeof bad / sizeof bad[0]);
        int i;

        for (i = 0; i < n_bad; i++) {
            CHECK(scurve_init(&s, &bad[i], 1.0f) == -1);
            CHECK(scurve_done(&s) == 1);
            NEAR(scurve_step(&s), 0.0, 0.0);
            CHECK(scurve_get(&s, &st) == 0);
            NEAR(st.pos, 0.0, 0.0); NEAR(st.vel, 0.0, 0.0); NEAR(st.acc, 0.0, 0.0);
            CHECK(st.phase == SCURVE_PHASE_DONE);
        }
        CHECK(scurve_init(&s, &CFG_D, HUGE_VALF) == -1);    /* 非有限 target 拒 */
        CHECK(scurve_init(&s, &CFG_D, NAN) == -1);
        printf("ok: 参数越界拒 + 不动安全态（旧⑥同构，%d 例）\n", n_bad);
    }

    /* ⑦（旧⑦ NULL）NULL / 零初始化结构体全接口防御 */
    {
        scurve_t s, z;
        scurve_state_t st;

        CHECK(scurve_init(NULL, &CFG_D, 1.0f) == -1);
        CHECK(scurve_init(&s, NULL, 1.0f) == -1);
        NEAR(scurve_step(NULL), 0.0, 0.0);
        CHECK(scurve_done(NULL) == 1);
        CHECK(scurve_phase(NULL) == SCURVE_PHASE_DONE);
        CHECK(scurve_get(NULL, &st) == -1);
        CHECK(scurve_get(&s, NULL) == -1);
        CHECK(scurve_retarget(NULL, 1.0f) == -1);
        memset(&z, 0, sizeof z);                            /* BSS/静态零初始化语义 */
        NEAR(scurve_step(&z), 0.0, 0.0);
        CHECK(scurve_done(&z) == 1);
        CHECK(scurve_get(&z, &st) == -1);
        CHECK(scurve_retarget(&z, 1.0f) == -1);
        CHECK(scurve_phase(&z) == SCURVE_PHASE_DONE);
        printf("ok: NULL/零初始化防御（旧⑦同构）\n");
    }

    /* ⑧（新）dt 整除总时长末点不重复：T = 98·dt 精确整除 */
    {
        scurve_t s;
        scurve_cfg_t c = CFG_D;
        float p[200];
        float last;
        int n, k, dup;

        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        n = 0;
        while (!scurve_done(&s) && n < 200) { p[n] = scurve_step(&s); n++; }
        CHECK(n == 98);                                     /* 步数恰 = T/dt：无额外一拍 */
        last = 0.0f; dup = 0;
        for (k = 0; k < n; k++) {
            CHECK(p[k] > last);                             /* 严格递增：无重复无平台 */
            last = p[k];
            if ((double)p[k] == 2.03125) dup++;
        }
        CHECK(dup == 1);                                    /* 终值恰发一次 */
        NEAR(p[96], 2.0310872395833333, 1e-6);             /* 倒数第二拍未到终点 */
        CHECK((double)p[96] < 2.03125);
        CHECK((double)p[97] == 2.03125);
        /* done 恰在第 98 拍翻转（不是 97 也不是 99） */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (k = 0; k < 97; k++) (void)scurve_step(&s);
        CHECK(scurve_done(&s) == 0);
        (void)scurve_step(&s);
        CHECK(scurve_done(&s) == 1);
        /* 变体：非整除 T/dt = 76.5625 → ceil = 77 拍，末点仍恰一次 */
        c.dt = 0.02f;
        CHECK(scurve_init(&s, &c, 2.03125f) == 0);
        n = 0;
        while (!scurve_done(&s) && n < 200) { p[n] = scurve_step(&s); n++; }
        CHECK(n == 77);
        last = 0.0f; dup = 0;
        for (k = 0; k < n; k++) {
            CHECK(p[k] > last);
            last = p[k];
            if ((double)p[k] == 2.03125) dup++;
        }
        CHECK(dup == 1);
        printf("ok: dt 整除总时长末点不重复（98 拍恰 98、终值一次；非整除 77 拍同性质）\n");
    }

    /* ⑨（新）短行程退化 */
    {
        scurve_t s;
        scurve_state_t st;
        scurve_phase_t seq[16];
        int nq;
        double vp, ap;

        /* 9a 三角 6 相（无巡航）：target=d(v_peak=1)·2=0.265625 → v_peak=1.0 精确 */
        {
            const int    bk[6]  = { 1, 16, 17, 18, 33, 34 };
            const scurve_phase_t bp[6] = {
                SCURVE_PHASE_ACC, SCURVE_PHASE_JERK_DN1, SCURVE_PHASE_JERK_DN2,
                SCURVE_PHASE_DEC, SCURVE_PHASE_JERK_UP2, SCURVE_PHASE_DONE };
            const double bv[6]  = { 0.03125, 0.96875, 1.0, 0.96875, 0.03125, 0.0 };
            const double ba[6]  = { 4.0, 4.0, 0.0, -4.0, -4.0, 0.0 };
            const double bps[6] = { 1.6276041666666667e-4, 0.11735026041666667,
                                    0.1328125, 0.14827473958333333,
                                    0.26546223958333333, 0.265625 };
            int k, bi = 0;
            CHECK(scurve_init(&s, &CFG_D, 0.265625f) == 0);
            for (k = 1; k <= 34; k++) {
                (void)scurve_step(&s);
                CHECK(scurve_get(&s, &st) == 0);
                CHECK((double)st.vel <= 1.0 + 1e-9);        /* 峰值被压到 v_max 以下 */
                if (bi < 6 && k == bk[bi]) {
                    CHECK(st.phase == bp[bi]);
                    NEAR(st.vel, bv[bi], 1e-9);
                    NEAR(st.acc, ba[bi], 1e-9);
                    NEAR(st.pos, bps[bi], 1e-6);
                    bi++;
                }
            }
            CHECK(bi == 6);
            CHECK(scurve_done(&s) == 1);
            CHECK(st.pos == 0.265625f && st.vel == 0.0f);
            /* 相位序列：无 CRUISE */
            CHECK(scurve_init(&s, &CFG_D, 0.265625f) == 0);
            CHECK(phase_seq(&s, 1000, seq, &nq) == 34);
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_CRUISE));
            CHECK(seq_has(seq, nq, SCURVE_PHASE_ACC) && seq_has(seq, nq, SCURVE_PHASE_DEC));
            /* 闭式对拍：v²+(amax²/jerk)·v−2·amax·d_half=0 的正根 = 采样峰值 */
            {
                double b = 16.0 / 256.0, d_half = 0.1328125;
                double v_cf = (-b + sqrt(b * b + 8.0 * 4.0 * d_half)) / 2.0;
                NEAR(v_cf, 1.0, 1e-12);
                CHECK(scurve_init(&s, &CFG_D, 0.265625f) == 0);
                CHECK(run_to_done(&s, 1000, &vp, &ap) == 34);
                NEAR(vp, v_cf, 1e-9);                       /* 采样峰值 == 闭式根 */
                NEAR(ap, 4.0, 1e-9);
                CHECK(scurve_get(&s, &st) == 0);
                CHECK(st.pos == 0.265625f);
            }
            printf("ok: 短行程退化 a) 三角 6 相（v_peak=1.0 闭式对拍精确命中）\n");
        }

        /* 9b 超短 4 相（匀加/匀减也消失）：dt=1/128, target=1/4096, v_peak=1/64 */
        {
            scurve_cfg_t c = CFG_D;
            const int    bk[4]  = { 1, 2, 3, 4 };
            const scurve_phase_t bp[4] = {
                SCURVE_PHASE_JERK_DN1, SCURVE_PHASE_JERK_DN2,
                SCURVE_PHASE_JERK_UP2, SCURVE_PHASE_DONE };
            const double bv[4]  = { 0.0078125, 0.015625, 0.0078125, 0.0 };
            const double ba[4]  = { 2.0, 0.0, -2.0, 0.0 };
            const double bps[4] = { 2.0345052083333333e-5, 1.220703125e-4,
                                    2.2379557291666667e-4, 2.44140625e-4 };
            int k, bi = 0;
            c.dt = 0.0078125f;
            CHECK(scurve_init(&s, &c, 0.000244140625f) == 0);
            for (k = 1; k <= 4; k++) {
                (void)scurve_step(&s);
                CHECK(scurve_get(&s, &st) == 0);
                if (bi < 4 && k == bk[bi]) {
                    CHECK(st.phase == bp[bi]);
                    NEAR(st.vel, bv[bi], 1e-9);
                    NEAR(st.acc, ba[bi], 1e-9);
                    NEAR(st.pos, bps[bi], 1e-6);
                    bi++;
                }
            }
            CHECK(bi == 4);
            CHECK(scurve_done(&s) == 1);
            CHECK(st.pos == 0.000244140625f && st.vel == 0.0f);
            CHECK(scurve_init(&s, &c, 0.000244140625f) == 0);
            CHECK(phase_seq(&s, 100, seq, &nq) == 4);
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_ACC));
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_DEC));
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_CRUISE));
            /* 闭式对拍：v_peak = (jerk·d_half²)^(1/3)；a_peak = √(jerk·v) < amax */
            NEAR(cbrt(256.0 * 0.0001220703125 * 0.0001220703125), 0.015625, 1e-12);
            CHECK(scurve_init(&s, &c, 0.000244140625f) == 0);
            CHECK(run_to_done(&s, 100, &vp, &ap) == 4);
            NEAR(vp, 0.015625, 1e-9);
            NEAR(ap, 2.0, 1e-9);                            /* a_peak=2 < amax=4 */
            printf("ok: 短行程退化 b) 超短 4 相（v_peak/a_peak 闭式对拍）\n");
        }

        /* 9c d_full 精确边界：target=d_full=1.03125 → 零长巡航裁掉（6 相），
         *    v 仍精确到达 vmax（梯形↔三角的连续过渡点） */
        {
            const int    bk[6]  = { 1, 32, 33, 34, 65, 66 };
            const scurve_phase_t bp[6] = {
                SCURVE_PHASE_ACC, SCURVE_PHASE_JERK_DN1, SCURVE_PHASE_JERK_DN2,
                SCURVE_PHASE_DEC, SCURVE_PHASE_JERK_UP2, SCURVE_PHASE_DONE };
            const double bv[6]  = { 0.03125, 1.96875, 2.0, 1.96875, 0.03125, 0.0 };
            const double bps[6] = { 1.6276041666666667e-4, 0.48453776041666667,
                                    0.515625, 0.5467122395833333,
                                    1.0310872395833333, 1.03125 };
            int k, bi = 0;
            CHECK(scurve_init(&s, &CFG_D, 1.03125f) == 0);
            for (k = 1; k <= 66; k++) {
                (void)scurve_step(&s);
                CHECK(scurve_get(&s, &st) == 0);
                if (bi < 6 && k == bk[bi]) {
                    CHECK(st.phase == bp[bi]);
                    NEAR(st.vel, bv[bi], 1e-9);
                    NEAR(st.pos, bps[bi], 1e-6);
                    bi++;
                }
            }
            CHECK(bi == 6);
            CHECK(scurve_done(&s) == 1);
            CHECK(st.pos == 1.03125f && st.vel == 0.0f);
            CHECK(scurve_init(&s, &CFG_D, 1.03125f) == 0);
            CHECK(phase_seq(&s, 1000, seq, &nq) == 66);
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_CRUISE));  /* 零长巡航被裁 */
            CHECK(seq_has(seq, nq, SCURVE_PHASE_ACC));
            printf("ok: 短行程退化 c) d_full 精确边界 6 相（零长巡航裁掉，v 仍到 vmax）\n");
        }

        /* 9d vmax < amax²/jerk：恒为无匀加段形态（jerk 受限、a 峰值不达 amax） */
        {
            scurve_cfg_t c = { 0.03125f, 4.0f, 256.0f, 0.015625f };
            CHECK(scurve_init(&s, &c, 0.001f) == 0);
            CHECK(phase_seq(&s, 100, seq, &nq) == 4);
            CHECK(seq_has(seq, nq, SCURVE_PHASE_CRUISE));
            CHECK(seq_has(seq, nq, SCURVE_PHASE_JERK_UP1));
            CHECK(seq_has(seq, nq, SCURVE_PHASE_JERK_DN1));
            CHECK(seq_has(seq, nq, SCURVE_PHASE_JERK_UP2));
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_ACC));
            CHECK(!seq_has(seq, nq, SCURVE_PHASE_DEC));
            CHECK(scurve_get(&s, &st) == 0);
            CHECK(st.pos == 0.001f && st.vel == 0.0f && st.acc == 0.0f);
            CHECK(scurve_init(&s, &c, 0.001f) == 0);
            CHECK(run_to_done(&s, 100, &vp, &ap) == 4);
            CHECK(vp <= 0.03125 + 1e-9);                    /* 峰值速度守界 */
            printf("ok: 短行程退化 d) vmax<vswitch 5 相（无匀加/匀减段）\n");
        }
    }

    /* ⑩（新）中途改 target 重规划（保守停稳再走） */
    {
        scurve_t s;
        scurve_state_t st, st0;
        double vp, ap;
        int i;

        /* a) 前向改近：k=16（匀加段中途，v=0.96875, a=4）→ 1.75 */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (i = 0; i < 16; i++) (void)scurve_step(&s);
        CHECK(scurve_get(&s, &st0) == 0);
        NEAR(st0.vel, 0.96875, 1e-9);
        NEAR(st0.acc, 4.0, 1e-9);
        NEAR(st0.pos, 0.11735026041666667, 1e-6);
        CHECK(scurve_retarget(&s, 1.75f) == 0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == st0.pos && st.vel == st0.vel && st.acc == st0.acc); /* 态精确连续 */
        CHECK(st.phase == SCURVE_PHASE_JERK_DN2);            /* 停稳段第一相 */
        CHECK(scurve_done(&s) == 0);
        (void)scurve_step(&s);                               /* 停稳动力学连续：v 续爬到 1.0 */
        CHECK(scurve_get(&s, &st) == 0);
        NEAR(st.vel, 1.0, 1e-9);
        NEAR(st.acc, 0.0, 1e-12);
        CHECK(run_to_done(&s, 2000, &vp, &ap) > 0);
        CHECK(vp <= 2.0 + 1e-6 && ap <= 4.0 + 1e-6);        /* 全程守界 */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 1.75f && st.vel == 0.0f && st.acc == 0.0f); /* 精确收敛新目标 */
        CHECK(st.phase == SCURVE_PHASE_DONE);
        printf("ok: retarget a) 前向改近（态连续/守界/精确收敛 1.75）\n");

        /* b) 反向改远小于停稳点：先刹停（18 步精确）到 0.265625 再倒走 */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (i = 0; i < 16; i++) (void)scurve_step(&s);
        CHECK(scurve_retarget(&s, 0.0625f) == 0);
        for (i = 0; i < 18; i++) (void)scurve_step(&s);     /* 停稳表 18·dt 精确走完 */
        CHECK(scurve_get(&s, &st) == 0);
        NEAR(st.pos, 0.265625, 1e-9);
        NEAR(st.vel, 0.0, 1e-9);
        CHECK(st.phase == SCURVE_PHASE_JERK_UP1);           /* 新计划（反向）第一相 */
        CHECK(run_to_done(&s, 2000, &vp, &ap) > 0);
        CHECK(vp <= 2.0 + 1e-6 && ap <= 4.0 + 1e-6);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 0.0625f && st.vel == 0.0f && st.acc == 0.0f);
        printf("ok: retarget b) 反向（刹停点 0.265625 精确 / 收敛 0.0625）\n");

        /* c) 静止改目标：纯新计划（无停稳段） */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        CHECK(scurve_retarget(&s, 1.0f) == 0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 0.0f && st.vel == 0.0f && st.acc == 0.0f);
        CHECK(st.phase == SCURVE_PHASE_JERK_UP1);
        CHECK(run_to_done(&s, 2000, &vp, &ap) > 0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 1.0f && st.vel == 0.0f);
        printf("ok: retarget c) 静止改目标（1.0）\n");

        /* d) 运动中改回同目标：刹停后重走原目标 */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (i = 0; i < 16; i++) (void)scurve_step(&s);
        CHECK(scurve_retarget(&s, 2.03125f) == 0);
        CHECK(run_to_done(&s, 2000, &vp, &ap) > 0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 2.03125f && st.vel == 0.0f);
        printf("ok: retarget d) 改回同目标（2.03125）\n");

        /* e) 非法 retarget：拒改且旧计划原样保留（validate-then-commit） */
        CHECK(scurve_init(&s, &CFG_D, 2.03125f) == 0);
        for (i = 0; i < 16; i++) (void)scurve_step(&s);
        CHECK(scurve_retarget(&s, NAN) == -1);
        CHECK(scurve_retarget(&s, HUGE_VALF) == -1);
        CHECK(scurve_done(&s) == 0);                        /* 运动未被打断 */
        CHECK(run_to_done(&s, 2000, &vp, &ap) == 82);       /* 98 − 16：旧计划照走 */
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 2.03125f && st.vel == 0.0f);
        printf("ok: retarget e) 非法目标拒改、旧计划保走到 2.03125\n");
    }

    /* ⑪（新）负方向镜像：相位序不变、v/a 取镜像符号、精确收敛 */
    {
        scurve_t s;
        scurve_state_t st;
        scurve_phase_t seq[16];
        int nq, k;
        const int bk[4] = { 1, 33, 65, 98 };
        const double bv[4] = { -0.03125, -2.0, -2.0, 0.0 };
        const double bps[4] = { -1.6276041666666667e-4, -0.515625, -1.515625, -2.03125 };
        int bi = 0;
        double vp = 0.0, ap = 0.0;
        float prev_p = 0.0f;

        CHECK(scurve_init(&s, &CFG_D, -2.03125f) == 0);
        for (k = 1; k <= 98; k++) {
            float p = scurve_step(&s);
            CHECK(scurve_get(&s, &st) == 0);
            CHECK(p < prev_p);                              /* 严格递减（反向单调） */
            prev_p = p;
            CHECK((double)st.vel >= -2.0 - 1e-9 && (double)st.vel <= 1e-9);
            CHECK(fabs((double)st.acc) <= 4.0 + 1e-9);
            if (fabs((double)st.vel) > vp) vp = fabs((double)st.vel);
            if (fabs((double)st.acc) > ap) ap = fabs((double)st.acc);
            if (bi < 4 && k == bk[bi]) {
                NEAR(st.vel, bv[bi], 1e-9);
                NEAR(st.pos, bps[bi], 1e-6);
                if (k == 33) CHECK(st.phase == SCURVE_PHASE_CRUISE);
                bi++;
            }
        }
        CHECK(bi == 4);
        CHECK(scurve_done(&s) == 1);
        NEAR(vp, 2.0, 1e-9);
        NEAR(ap, 4.0, 1e-9);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == -2.03125f && st.vel == 0.0f && st.acc == 0.0f);
        CHECK(scurve_init(&s, &CFG_D, -2.03125f) == 0);
        CHECK(phase_seq(&s, 1000, seq, &nq) == 98);         /* 相位序与正向一致 */
        CHECK(seq[0] == SCURVE_PHASE_JERK_UP1);
        CHECK(seq_has(seq, nq, SCURVE_PHASE_CRUISE));
        CHECK(seq_has(seq, nq, SCURVE_PHASE_ACC));
        CHECK(seq_has(seq, nq, SCURVE_PHASE_DEC));
        printf("ok: 负方向镜像（相位序同正向、收敛 -2.03125）\n");
    }

    /* ⑫（新）预设表：find/字段校验/实走一条 */
    {
        scurve_t s;
        scurve_state_t st;
        const scurve_cfg_t *c;
        double vp, ap;
        int i;

        CHECK(scurve_cfg_find("scurve.rpm.gentle") != NULL);
        CHECK(scurve_cfg_find("scurve.rpm.brisk") != NULL);
        CHECK(scurve_cfg_find("scurve.step.gentle") != NULL);
        CHECK(scurve_cfg_find("scurve.no.such") == NULL);
        CHECK(scurve_cfg_find(NULL) == NULL);
        CHECK(scurve_cfg_preset_count == 3);
        for (i = 0; i < scurve_cfg_preset_count; i++) {
            CHECK(scurve_cfg_presets[i].name != NULL);
            CHECK(scurve_cfg_presets[i].cfg.vmax > 0.0f);
            CHECK(scurve_cfg_presets[i].cfg.amax > 0.0f);
            CHECK(scurve_cfg_presets[i].cfg.jerk > 0.0f);
            CHECK(scurve_cfg_presets[i].cfg.dt > 0.0f);
        }
        c = scurve_cfg_find("scurve.rpm.gentle");
        CHECK(scurve_init(&s, c, 50.0f) == 0);
        CHECK(run_to_done(&s, 20000, &vp, &ap) > 0);
        CHECK(scurve_get(&s, &st) == 0);
        CHECK(st.pos == 50.0f && st.vel == 0.0f && st.acc == 0.0f);
        CHECK(vp <= 120.0 + 1e-3);                          /* 预设界内走通 */
        CHECK(ap <= 240.0 + 1e-3);
        printf("ok: 预设表 find/字段/走通（rpm.gentle → 50.0）\n");
    }

    if (!s_fail)
        printf("test_scurve: ALL PASS\n");
    return s_fail;
}
