/* SPDX-License-Identifier: MIT */
/**
 * @file    scurve.c
 * @brief   S 梯度（七相位恒定 jerk）轨迹发生器实现（纯计算，零硬件依赖）
 *
 * @note    资产定性（如实）：提炼自旧式实现冻结资产，仅单测覆盖，无实机验证。
 *          闭式数学提炼自旧式实现 host 侧 motion_planner.py 的解析基准
 *          （_accel_segs / _mirror_to_decel / _plan_move_analytic），
 *          表回放状态机形态提炼自旧式实现 embedded 侧 scurve_exec.c。
 */

#include "mtr/scurve.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ---- 内部算法常量（命名常量，非业务可调参数——数值精度阈值） ---- */

/** 段长裁剪阈值 [s]：低于此的相位视为未使用（对齐旧式实现 _SWITCH_TOL 量级） */
#define SCURVE_T_EPS 1e-12
/** 构建自检相对容差：位移闭合 / 终态静止（解析构造实际误差 ~1e-12 量级） */
#define SCURVE_CHK_REL 1e-9
/** 构建自检边界余量：|a| ≤ amax、|j| ≤ jerk 的相对松弛 */
#define SCURVE_BOUND_SLACK 1e-6

/* ── 段解析原语（v(t)=v0+a0·t+j·t²/2 族） ───────────────────── */

static scurve_seg_t seg_make(scurve_phase_t ph, double T, double v0, double a0, double j)
{
    scurve_seg_t g;
    g.phase = ph;
    g.T = T;
    g.v0 = v0;
    g.a0 = a0;
    g.j = j;
    return g;
}

static double seg_v(const scurve_seg_t *g, double t)
{
    return g->v0 + g->a0 * t + g->j * t * t / 2.0;
}

static double seg_a(const scurve_seg_t *g, double t)
{
    return g->a0 + g->j * t;
}

static double seg_s(const scurve_seg_t *g, double t)
{
    return g->v0 * t + g->a0 * t * t / 2.0 + g->j * t * t * t / 6.0;
}

/** 方向折叠：折叠域段表翻回真实符号域（sgn=+1 恒等；v0/a0/j 取负） */
static void negate_seg(scurve_seg_t *g, double sgn)
{
    if (sgn < 0.0) {
        g->v0 = -g->v0;
        g->a0 = -g->a0;
        g->j = -g->j;
    }
}

/* ── 加速段表（0→v，折叠域 v>0；提炼自旧式实现 _accel_segs） ──────── */

/**
 * @brief  0→v 的时间最优加速段（cruise 前半的原子件，减速侧取其镜像）
 * @param  out  段表出口（容量 ≥3）
 * @return 段数（2 = 三角加速 a_peak<amax；3 = 含匀加段）
 * @note   两形态：v ≥ amax²/jerk → jerk+/匀加/jerk−；
 *         v < amax²/jerk → jerk+/jerk−（三角加速，a_peak = √(jerk·v) < amax）。
 */
static int accel_segs(scurve_seg_t *out, double v, double amax, double jerk)
{
    int n = 0;
    const double v_switch = amax * amax / jerk; /* 两个 jerk 段合计的速度贡献 */

    if (v >= v_switch) {
        const double tj = amax / jerk;
        const double ta = (v - v_switch) / amax;
        out[n++] = seg_make(SCURVE_PHASE_JERK_UP1, tj, 0.0, 0.0, jerk);
        if (ta > SCURVE_T_EPS)
            out[n++] = seg_make(SCURVE_PHASE_ACC, ta, jerk * tj * tj / 2.0, amax, 0.0);
        /* jerk− 段初速 = v − 该段贡献（j·tj²/2 = amax²/2j），末态恰为 (v, 0) */
        out[n++] = seg_make(SCURVE_PHASE_JERK_DN1, tj, v - jerk * tj * tj / 2.0, amax, -jerk);
    } else {
        const double tj = sqrt(v / jerk); /* a_peak = jerk·tj < amax */
        out[n++] = seg_make(SCURVE_PHASE_JERK_UP1, tj, 0.0, 0.0, jerk);
        out[n++] = seg_make(SCURVE_PHASE_JERK_DN1, tj, jerk * tj * tj / 2.0, jerk * tj, -jerk);
    }
    return n;
}

/* ── 镜像减速（提炼自旧式实现 _mirror_to_decel） ──────────────────── */

/** 加速侧相位名 → 减速侧镜像相位名（匀速不进镜像） */
static scurve_phase_t mirror_phase(scurve_phase_t ph)
{
    switch (ph) {
    case SCURVE_PHASE_JERK_UP1: return SCURVE_PHASE_JERK_UP2;
    case SCURVE_PHASE_ACC:      return SCURVE_PHASE_DEC;
    case SCURVE_PHASE_JERK_DN1: return SCURVE_PHASE_JERK_DN2;
    default:                    return ph;
    }
}

/**
 * @brief  把 0→v_top 加速段表镜像成 v_top→0 减速段表（段序倒置）
 * @note   时间反演 + 取负：jerk 符号不变、初加速度 = 原段末加速度取反、
 *         初速 = 原段末速（段间 v/a 由边界态数组精确链接）。
 * @note   up/out 不重叠区间（调用方保证 out = up 表之后的空位）；
 *         n_up ≤ 3（accel_segs 契约），边界数组按此定容。
 */
static int mirror_decel(const scurve_seg_t *up, int n_up, scurve_seg_t *out)
{
    double bv[4], ba[4]; /* 边界态：bv[0]=ba[0]=(0,0) 静止起点；[i+1]=段 i 末态 */
    int i;

    bv[0] = 0.0;
    ba[0] = 0.0;
    for (i = 0; i < n_up; i++) {
        bv[i + 1] = seg_v(&up[i], up[i].T);
        ba[i + 1] = seg_a(&up[i], up[i].T);
    }
    for (i = n_up - 1; i >= 0; i--)
        out[n_up - 1 - i] = seg_make(mirror_phase(up[i].phase), up[i].T,
                                     bv[i + 1], -ba[i + 1], up[i].j);
    return n_up;
}

/* ── 闭式反解（修雷点：旧式实现三角峰值用 80 次二分逼近，此处全闭式） ── */

/**
 * @brief  0→v 加速距离闭式 d(v)（v>0；分支在 v=amax²/jerk 处解析连续）
 * @note   v ≥  v_switch：d = v²/(2·amax) + v·amax/(2·jerk)（a-profile 对称 ⇒ d = v·T/2）
 *         v < v_switch：d = √(v³/jerk)（三角 a-profile）
 */
static double d_of_v(double v, double amax, double jerk)
{
    if (v >= amax * amax / jerk)
        return v * v / (2.0 * amax) + v * amax / (2.0 * jerk);
    return sqrt(v * v * v / jerk);
}

/**
 * @brief  三角行程峰值速度闭式反解：2·d(v_peak) = dist ⇒ d(v_peak) = d_half
 * @note   d_half ≤ amax³/jerk²：v_peak = (jerk·d_half²)^(1/3)（4 相形态）
 *         否则：v² + (amax²/jerk)·v − 2·amax·d_half = 0 取正根（6 相形态）
 */
static double v_peak_for(double d_half, double amax, double jerk)
{
    if (d_half <= amax * amax * amax / (jerk * jerk))
        return cbrt(jerk * d_half * d_half);
    {
        const double b = amax * amax / jerk;
        return (-b + sqrt(b * b + 8.0 * amax * d_half)) / 2.0;
    }
}

/* ── 七相位主计划（折叠域 dist>0；梯形/三角自动判别） ─────────── */

/**
 * @brief  rest-to-rest 主计划构建（折叠域，方向由调用方 negate 翻回）
 * @param  out  段表出口（容量 ≥7：加速 ≤3 + 匀速 ≤1 + 镜像 ≤3）
 * @return 段数
 */
static int build_move(scurve_seg_t *out, double dist, const scurve_cfg_t *cfg)
{
    const double amax = (double)cfg->amax;
    const double jerk = (double)cfg->jerk;
    const double vmax = (double)cfg->vmax;
    int n_up, n, i;
    double d_up;

    if (dist >= 2.0 * d_of_v(vmax, amax, jerk)) {
        /* 梯形：能加速到 vmax（d_full ≤ dist） */
        double t_cruise;
        n_up = accel_segs(out, vmax, amax, jerk);
        d_up = 0.0;
        for (i = 0; i < n_up; i++)
            d_up += seg_s(&out[i], out[i].T);
        t_cruise = (dist - 2.0 * d_up) / vmax;
        n = n_up;
        if (t_cruise > SCURVE_T_EPS) {
            out[n] = seg_make(SCURVE_PHASE_CRUISE, t_cruise, vmax, 0.0, 0.0);
            n++;
        } /* else：恰达 d_full 的行程——零长匀速段裁掉（6 相退化） */
    } else {
        /* 三角：距离不足，峰值闭式反解（旧式实现此处为 80 次二分） */
        const double v_peak = v_peak_for(dist / 2.0, amax, jerk);
        n_up = accel_segs(out, v_peak, amax, jerk);
        n = n_up;
    }
    n += mirror_decel(out, n_up, &out[n]);
    return n;
}

/* ── 保守停稳段（retarget 原子件：从 (v0,a0) 刹停到 (0,0)） ────── */

/**
 * @brief  停稳段表构建（折叠域 v0 ≥ 0；从 (v0, a0) 到 (0, 0) 的三形态闭式）
 * @param  out  段表出口（容量 ≥3）
 * @return 段数（0 = 已静止）
 * @note   形态A 深制动（v0 ≥ (2·amax²−a0²)/(2·jerk)）：
 *           jerk−（a0→−amax）/ 匀减（−amax 保持）/ jerk+（−amax→0）；
 *         形态B 三角凹陷（制动需求浅）：jerk−（a0→a_pk<0）/ jerk+（a_pk→0），
 *           a_pk = −√((a0²+2·j·v0)/2)，面积恰 = −v0；
 *         形态C 过制动回收（a0<0 且 v0 ≤ a0²/(2·j)：直接松 a 会冲过 v=0）：
 *           jerk+（a0→+B）/ jerk−（+B→0），B = √((a0²−2·j·v0)/2)。
 *           折叠域内 v 短暂过零回落（|回落| ≤ a0²/(2·jerk)，动力学唯一解）。
 * @note   段间 (v,a) 用增量公式精确链推；解析恒等式保证末态 (0,0)
 *         （commit 自检验证，失配拒装表）。
 */
static int build_stop(scurve_seg_t *out, double v0, double a0, const scurve_cfg_t *cfg)
{
    const double amax = (double)cfg->amax;
    const double jerk = (double)cfg->jerk;
    const double v_case_a = (2.0 * amax * amax - a0 * a0) / (2.0 * jerk);
    int n = 0;
    double v = v0, a = a0;

    if (v0 == 0.0 && a0 == 0.0)
        return 0; /* 已静止 */

    if (v0 >= v_case_a) {
        /* 形态A */
        const double t1 = (a0 + amax) / jerk; /* a0 → −amax（a0=−amax 时为 0，裁掉） */
        const double t_hold = (v0 + (a0 * a0 - 2.0 * amax * amax) / (2.0 * jerk)) / amax;
        const double t2 = amax / jerk; /* −amax → 0 */
        if (t1 > SCURVE_T_EPS) {
            out[n++] = seg_make(SCURVE_PHASE_JERK_DN2, t1, v, a, -jerk);
            v += a * t1 - jerk * t1 * t1 / 2.0;
            a -= jerk * t1;
        }
        if (t_hold > SCURVE_T_EPS) {
            out[n++] = seg_make(SCURVE_PHASE_DEC, t_hold, v, a, 0.0);
            v += a * t_hold;
        }
        out[n++] = seg_make(SCURVE_PHASE_JERK_UP2, t2, v, a, jerk);
    } else if (a0 >= 0.0 || v0 > a0 * a0 / (2.0 * jerk)) {
        /* 形态B：三角凹陷 */
        const double a_pk = -sqrt((a0 * a0 + 2.0 * jerk * v0) / 2.0);
        const double t1 = (a0 - a_pk) / jerk;
        const double t2 = -a_pk / jerk;
        out[n++] = seg_make(SCURVE_PHASE_JERK_DN2, t1, v, a, -jerk);
        v += a * t1 - jerk * t1 * t1 / 2.0;
        a -= jerk * t1;
        out[n++] = seg_make(SCURVE_PHASE_JERK_UP2, t2, v, a, jerk);
    } else {
        /* 形态C：过制动回收（a0 < 0 且 v0 ≤ a0²/(2·jerk)） */
        const double b_pk = sqrt((a0 * a0 - 2.0 * jerk * v0) / 2.0);
        const double t1 = (b_pk - a0) / jerk;
        const double t2 = b_pk / jerk;
        out[n++] = seg_make(SCURVE_PHASE_JERK_UP2, t1, v, a, jerk);
        v += a * t1 + jerk * t1 * t1 / 2.0;
        a += jerk * t1;
        if (t2 > SCURVE_T_EPS)
            out[n++] = seg_make(SCURVE_PHASE_JERK_DN2, t2, v, a, -jerk);
    }
    return n;
}

/* ── 计划装表 + 构建自检（validate-then-commit 的 validate 半边） ── */

/**
 * @brief  裁零长段装表 + 四重自检；全过才落游标（失败返回 -1，实例半改弃用）
 * @param  p_origin  计划起点的绝对位置 [u]（init=0；retarget=当前采样位）
 * @param  target    计划终点 [u]（p_end 钉死于此——到目标收敛的精确锚）
 * 自检：① 段参数有限且 |a0|、|a(T)| ≤ amax、|j| ≤ jerk；
 *       ② 段间 v/a 连续（无跳变——分段边界正确性的直接断言）；
 *       ③ 位移闭合：Σ s(T) ≈ target − p_origin；
 *       ④ 终态静止：末段 (v,a) → (0,0)。
 */
static int commit_plan(scurve_t *s, const scurve_seg_t *tab, int n,
                       double p_origin, double target)
{
    const double amax = (double)s->cfg.amax;
    const double jerk = (double)s->cfg.jerk;
    const double a_lim = amax * (1.0 + SCURVE_BOUND_SLACK) + 1e-9;
    const double j_lim = jerk * (1.0 + SCURVE_BOUND_SLACK) + 1e-9;
    const double tol_scale = fabs(target - p_origin);
    const double dist_tol = SCURVE_CHK_REL * (tol_scale > 1.0 ? tol_scale : 1.0);
    double t_total = 0.0, dist_sum = 0.0, v_end = 0.0, a_end = 0.0;
    int m = 0, i;

    for (i = 0; i < n; i++) {
        if (!isfinite(tab[i].T) || !(tab[i].T > SCURVE_T_EPS))
            continue; /* 零长/非有限段：未使用的相位，裁掉 */
        s->seg[m++] = tab[i];
    }

    for (i = 0; i < m; i++) {
        const scurve_seg_t *g = &s->seg[i];
        const double vT = g->v0 + g->a0 * g->T + g->j * g->T * g->T / 2.0;
        const double aT = g->a0 + g->j * g->T;

        if (!isfinite(g->v0) || !isfinite(g->a0) || !isfinite(g->j) || !isfinite(g->T))
            return -1;
        if (fabs(g->a0) > a_lim || fabs(aT) > a_lim)
            return -1;
        if (fabs(g->j) > j_lim)
            return -1;
        if (i > 0) { /* 段间连续性：上段末态 == 本段初态 */
            const scurve_seg_t *pv = &s->seg[i - 1];
            const double pv_vT = pv->v0 + pv->a0 * pv->T + pv->j * pv->T * pv->T / 2.0;
            const double pv_aT = pv->a0 + pv->j * pv->T;
            if (fabs(pv_vT - g->v0) > 1e-9 * (1.0 + fabs(g->v0)))
                return -1;
            if (fabs(pv_aT - g->a0) > 1e-9 * (1.0 + amax))
                return -1;
        }
        t_total += g->T;
        dist_sum += seg_s(g, g->T);
        v_end = vT;
        a_end = aT;
    }
    if (!isfinite(t_total) || !isfinite(dist_sum))
        return -1;
    if (fabs(dist_sum - (target - p_origin)) > dist_tol)
        return -1;
    if (fabs(v_end) > SCURVE_CHK_REL * (1.0 + (double)s->cfg.vmax))
        return -1;
    if (fabs(a_end) > SCURVE_CHK_REL * (1.0 + amax))
        return -1;

    s->n_seg = m;
    s->t_total = t_total;
    s->p_end = target; /* 终点钉死：末拍返回值与 target 精确相等（收敛锚） */
    s->t = 0.0;
    s->idx = 0;
    s->t_acc = 0.0;
    s->p_acc = p_origin;
    s->done = (uint8_t)(m == 0);
    if (m == 0) {
        /* 空计划（零行程/停稳即达）：立即终态，采样态 = 起点静止 */
        s->p = p_origin;
        s->v = 0.0;
        s->a = 0.0;
        s->phase = SCURVE_PHASE_DONE;
    } else {
        /* t=0 采样态 = 计划起点态（init 为静止；retarget 为当前采样态——精确连续） */
        s->p = p_origin;
        s->v = s->seg[0].v0;
        s->a = s->seg[0].a0;
        s->phase = s->seg[0].phase;
    }
    return 0;
}

/* ── 入口参数防御 ───────────────────────────────────────────── */

/** cfg 四字段全 > 0 且有限（旧式实现 algo cfg 零防御雷的对症防御） */
static int cfg_valid(const scurve_cfg_t *cfg)
{
    return isfinite((double)cfg->vmax) && cfg->vmax > 0.0f
        && isfinite((double)cfg->amax) && cfg->amax > 0.0f
        && isfinite((double)cfg->jerk) && cfg->jerk > 0.0f
        && isfinite((double)cfg->dt) && cfg->dt > 0.0f;
}

/** 非法入参 → 不动的安全态（旧式实现「非法表回 -1 + 空表安全态」同构） */
static void safe_state(scurve_t *s, const scurve_cfg_t *cfg)
{
    memset(s, 0, sizeof *s);
    if (cfg != NULL)
        s->cfg = *cfg;
    s->valid = 1;
    s->done = 1;
    s->phase = SCURVE_PHASE_DONE; /* step 恒 0 / done 恒 1：一步都不输出 */
}

/* ── 游标求值（step 的计算半边） ─────────────────────────────── */

/** 推进游标到含 t 的段并求值采样态（前置：done==0 且 t < t_total） */
static void eval_at(scurve_t *s, double t)
{
    int i;
    double tl;
    const scurve_seg_t *g;

    while (s->idx + 1 < s->n_seg && t >= s->t_acc + s->seg[s->idx].T) {
        s->p_acc += seg_s(&s->seg[s->idx], s->seg[s->idx].T);
        s->t_acc += s->seg[s->idx].T;
        s->idx++;
    }
    i = (s->idx < s->n_seg) ? s->idx : s->n_seg - 1; /* 尾段舍入保护 */
    g = &s->seg[i];
    tl = t - s->t_acc;
    if (tl < 0.0)
        tl = 0.0;
    if (tl > g->T)
        tl = g->T; /* t 落段边界上：取下一段 tl=0（v/a 连续，两段同值） */
    s->p = s->p_acc + seg_s(g, tl);
    s->v = seg_v(g, tl);
    s->a = seg_a(g, tl);
    s->phase = g->phase;
}

/* ── 对外 API ───────────────────────────────────────────────── */

int scurve_init(scurve_t *s, const scurve_cfg_t *cfg, float target)
{
    scurve_t tmp;
    scurve_seg_t tab[SCURVE_SEGS_MAX];
    int n = 0;
    const double dist = (double)target;

    if (s == NULL || cfg == NULL)
        return -1;
    if (!cfg_valid(cfg) || !isfinite(dist)) {
        safe_state(s, cfg);
        return -1;
    }

    memset(&tmp, 0, sizeof tmp);
    tmp.cfg = *cfg;

    if (dist != 0.0) {
        const double sgn = (dist > 0.0) ? 1.0 : -1.0;
        int i;
        n = build_move(tab, fabs(dist), cfg);
        for (i = 0; i < n && i < SCURVE_SEGS_MAX; i++)
            negate_seg(&tab[i], sgn);
    }
    /* dist == 0：零行程合法（旧式实现空表同构）——空表走 commit 的空计划分支 */

    if (commit_plan(&tmp, tab, n, 0.0, dist) != 0) {
        safe_state(s, cfg);
        return -1;
    }
    tmp.valid = 1;
    *s = tmp;
    return 0;
}

float scurve_step(scurve_t *s)
{
    if (s == NULL)
        return 0.0f;         /* 编程错误防御（float 返回面无错误码通道） */
    if (!s->valid)
        return 0.0f;         /* 未 init（零初始化）：安全不动 */
    if (s->done)
        return (float)s->p;  /* 终值锁存：done 后恒返回终值 */

    s->t += (double)s->cfg.dt;
    if (s->n_seg == 0 || s->t >= s->t_total) {
        /* 到时（或空计划）：钉死终态。t 单调 + 本拍一次置 done ⇒ 末点只发
         * 一次——dt 整除总时长时也**不会**多发一个重复末点（旧 host 雷）。 */
        s->t = s->t_total;
        s->done = 1;
        s->p = s->p_end;
        s->v = 0.0;
        s->a = 0.0;
        s->phase = SCURVE_PHASE_DONE;
        return (float)s->p;
    }
    eval_at(s, s->t);
    return (float)s->p;
}

int scurve_done(const scurve_t *s)
{
    if (s == NULL || !s->valid)
        return 1; /* 防御：视为终态（不推进、不运动） */
    return s->done ? 1 : 0;
}

int scurve_retarget(scurve_t *s, float target)
{
    scurve_t tmp;
    scurve_seg_t tab[SCURVE_SEGS_MAX];
    int n = 0, i;
    double d_stop = 0.0, dist2;
    const double tgt = (double)target;

    if (s == NULL || !s->valid)
        return -1;
    if (!isfinite(tgt))
        return -1; /* 非法目标：旧计划原样保留（validate 前 early-out） */

    tmp = *s; /* 草稿实例：失败则丢弃，s 不动（validate-then-commit） */
    {
        const double p0 = tmp.p, v0 = tmp.v, a0 = tmp.a;

        if (v0 != 0.0 || a0 != 0.0) {
            /* 保守停稳：先从当前 (v,a) 闭式刹停（折叠 v0 ≥ 0 域再翻符号） */
            double fv = v0, fa = a0, sgn = 1.0;
            scurve_seg_t st[3];
            int ns;
            if (fv < 0.0) {
                fv = -fv;
                fa = -fa;
                sgn = -1.0;
            }
            ns = build_stop(st, fv, fa, &tmp.cfg);
            for (i = 0; i < ns && n < SCURVE_SEGS_MAX; i++) {
                tab[n] = st[i];
                negate_seg(&tab[n], sgn);
                n++;
            }
        }
        for (i = 0; i < n; i++)
            d_stop += seg_s(&tab[i], tab[i].T);

        dist2 = tgt - (p0 + d_stop);
        if (dist2 != 0.0) {
            /* 停稳点 → 新目标的完整七相位主计划（方向折叠再翻符号） */
            const double sgn2 = (dist2 > 0.0) ? 1.0 : -1.0;
            scurve_seg_t mv[SCURVE_SEGS_MAX];
            int nm = build_move(mv, fabs(dist2), &tmp.cfg);
            for (i = 0; i < nm && n < SCURVE_SEGS_MAX; i++) {
                tab[n] = mv[i];
                negate_seg(&tab[n], sgn2);
                n++;
            }
        }
        /* dist2 == 0：停稳点恰为新目标——停稳表本身就是完整计划 */

        if (commit_plan(&tmp, tab, n, p0, tgt) != 0)
            return -1; /* 自检失败：丢弃草稿，旧计划原样继续（运动不中断） */
    }
    *s = tmp;
    return 0;
}

scurve_phase_t scurve_phase(const scurve_t *s)
{
    if (s == NULL || !s->valid)
        return SCURVE_PHASE_DONE; /* 防御终态标签 */
    return s->phase;
}

int scurve_get(const scurve_t *s, scurve_state_t *out)
{
    if (s == NULL || out == NULL || !s->valid)
        return -1;
    out->pos = (float)s->p;
    out->vel = (float)s->v;
    out->acc = (float)s->a;
    out->phase = s->phase;
    return 0;
}

/* ── 配置预设注册表（与 pwm_timer 同构：定义即交付，追加只改本文件） ── */

const scurve_cfg_preset_t scurve_cfg_presets[] = {
    /* rpm 域（u = rpm）：N20 级闭环电机的温和底版（0.75s 到峰值速度） */
    { "scurve.rpm.gentle", { 120.0f, 240.0f, 960.0f, 0.002f } },
    /* rpm 域：敏捷底版（0.35s 到峰值速度；dt 匹配 1kHz 控制环） */
    { "scurve.rpm.brisk", { 600.0f, 2400.0f, 24000.0f, 0.001f } },
    /* step 域（u = 步数）：步进电机速度整形底版（dt 匹配 10kHz 脉冲环） */
    { "scurve.step.gentle", { 3200.0f, 6400.0f, 128000.0f, 0.0001f } },
};

const int scurve_cfg_preset_count =
    (int)(sizeof(scurve_cfg_presets) / sizeof(scurve_cfg_presets[0]));

const scurve_cfg_t *scurve_cfg_find(const char *name)
{
    int i;
    if (name == NULL)
        return NULL;
    for (i = 0; i < scurve_cfg_preset_count; i++)
        if (strcmp(scurve_cfg_presets[i].name, name) == 0)
            return &scurve_cfg_presets[i].cfg;
    return NULL;
}
