/* SPDX-License-Identifier: MIT */
/**
 * @file    motor.c
 * @brief   电机实例核心实现（四类 kind 同一 create 接口分派；POSIX 风格）
 *
 * @note    原创律：自研。提炼自旧式实现（旧式实现路径
 *          - motor_lib.c 的「配置→算法→驱动」分派思路（静态注册制 s_devs
 *            槽位数组是重构对象，不照抄：本实现 malloc 实例、零静态零上限）；
 *          - motor_algo.c 的换算思路（舵机角→脉宽线性、直流符号/幅值拆分；
 *            其 cfg 零防御（NULL 解引用）不照抄：本实现入口全参数校验）。
 *          按审查报告修雷重组：
 *          - 0xFF 哨兵撞值域 → motor_state_t 显式 fault 位图 + float 真值；
 *          - 编码器两步读/复位竞态 → HAL 单调用原子快照 + 差分（motor_hal.h 契约）；
 *          - 换频不贯通 → 频率单源 pwm_timer_t 实例，本文件从不缓存频率，
 *            SERVO 换算一律经 pwm_timer_freq() 现取（pwm_hal.h 同款铁律）；
 *          - PID 抗饱和：消费并行任务交付的 pid.h/pid.c（back-calculation
 *            回灌方向按旧式实现 speed_control.py:149 代码义，其 docstring 符号雷
 *            已在 pid.h:73-76 正确重写——本文件不重复实现算法）。
 *
 * @note    输出面（对齐并行任务交付的 pwm_hal.h 通道面）：
 *          - 一通道同时只属一台电机（pwm_timer 通道占用执法，单源）；
 *          - PWM_OPEN/SERVO 占单通道（cfg.channel ↔ pin_a）；
 *          - DC/ENC_CLOSED 方向桥按 pwm_hal.h「桥向由上层双通道表达」：
 *            占 (channel, channel+1) 成对通道（↔ pin_a 正向 / pin_b 反向），
 *            正转 = 正向通道出幅值、反向通道出 0，反转反对称；
 *          - SERVO 经角度→脉宽→占空换算走 set_duty（通道面无 set_us 入口，
 *            换算在电机层做、频率经句柄现取——不引入第二频率事实源）。
 *
 * @note    S 梯度轨迹：motor.h poll 注释提及，由并行任务 scurve 库交付后
 *          挂接（motor_cfg_t 现无梯度参数字段且契约冻结，按「参数零写死」
 *          铁律不预埋 magic number）；当前 poll 目标直通速度环。
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mtr/motor.h"
#include "mtr/motor_hal.h"
#include "mtr/pwm_hal.h"    /* 通道面组件 API（acquire/release/set_duty/enable） */
#include "mtr/pid.h"        /* 速度环（并行任务交付的通用 PID 小件） */

/* ── 单位换算常量（物理定义，非可调参数——不属 magic number 范畴） ── */
#define MTR_US_PER_SEC_F   1000000.0f   /**< us/s */
#define MTR_RPM_PER_RPS_F  60.0f        /**< rpm/(rev/s) */

/* ── 速度环配置映射（motor.h 冻结的 motor_pid_cfg_t → pid.h 的 pid_cfg_t） ──
 * kb（回灌系数）在 motor_pid_cfg_t 无字段（契约冻结不加）：按 Åström 推荐
 * 跟踪时间常数 Tt=kp/ki 取 kb=ki/kp（旧式实现 PIParams docstring「常取
 * ki/kp 量级」）；kp<=0 退 kb=ki（纯积分环等比回灌），ki=0 得 kb=0=
 * 关闭抗饱和（纯 P/PD 无积分）。合法性（有限值/界序/kb>=0）由 create 期
 * pid_init 单源执法——本文件不重复第二份校验表。 */
static void pid_cfg_from_motor(pid_cfg_t *out, const motor_pid_cfg_t *in)
{
    out->kp = in->kp;
    out->ki = in->ki;
    out->kd = in->kd;
    out->out_min = in->out_min;
    out->out_max = in->out_max;
    out->kb = (in->kp > 0.0f) ? (in->ki / in->kp) : in->ki;
}

/* ═══ 电机实例 ═══ */

/** @brief 电机实例（不透明结构实体，对外只暴露 motor_t*） */
struct motor {
    motor_cfg_t        cfg;        /**< 深拷配置（值语义；timer 为句柄引用——嵌套模型） */
    const motor_hal_t *hal;        /**< 编码器 HAL（attach 挂载；const，调用方保活） */
    uint8_t            ch_rev;     /**< 桥类反向通道（cfg.channel+1；非桥类未用） */
    float              target;     /**< 当前目标（rpm 或 deg，按 kind 语义） */
    float              applied;    /**< actual 真值：开环=上次施加值（钳位后），ENC=测量 rpm */
    float              duty;       /**< 当前输出占空比幅值 0.0~1.0 */
    int32_t            enc_base;   /**< 编码器差分基线（最近一次快照） */
    int32_t            enc_count;  /**< 编码器累计（基线以来差分累加） */
    int                has_enc_base; /**< 基线是否已建立（首次成功读时建） */
    uint32_t           last_us;    /**< 上拍时标 */
    int                has_last_us;  /**< 首拍无 dt（基线拍不控） */
    pid_t              pid;        /**< 速度环（pid.h 实例：自持积分/误差/输出状态） */
    uint32_t           fault;      /**< 活状态故障位图（非锁存，见 motor_types.h） */
};

/** @brief 桥类判定（DC/ENC 走双通道方向桥，见文件头输出面注） */
static int uses_bridge(motor_kind_t k)
{
    return (k == MOTOR_DC) || (k == MOTOR_ENC_CLOSED);
}

/**
 * @brief   配置全参数校验（create 入口防御——防「algo cfg 零防御」雷）
 * @note    channel 范围/占用冲突不在本校验：由 pwm_timer 通道面 acquire
 *          统一执法（单一事实源，避免第二份范围表漂移）。
 * @note    float 字段的 NaN 一并拒绝（NaN 比较恒假 → 走拒绝分支）。
 */
static int cfg_valid(const motor_cfg_t *c)
{
    if (c->timer == NULL) return 0;
    if (c->kind != MOTOR_PWM_OPEN && c->kind != MOTOR_DC &&
        c->kind != MOTOR_SERVO && c->kind != MOTOR_ENC_CLOSED)
        return 0;
    if (!(c->duty_max > 0.0f && c->duty_max <= 1.0f)) return 0;
    if (c->pin_a < 0) return 0;

    switch (c->kind) {
    case MOTOR_PWM_OPEN:
        break;
    case MOTOR_SERVO:
        if (c->pulse_min_us == 0 || c->pulse_min_us >= c->pulse_max_us) return 0;
        if (!(c->angle_min < c->angle_max)) return 0;
        /* 脉宽须落在载波周期内（整数精确判界防浮点边界抖动；
         * 频率经句柄现取——单源，实例频率不可变故 create 期检查恒有效） */
        if ((uint64_t)c->pulse_max_us * (uint64_t)pwm_timer_freq(c->timer)
            > 1000000ull)
            return 0;
        break;
    case MOTOR_DC:
        if (c->pin_b < 0) return 0;
        break;
    case MOTOR_ENC_CLOSED:
        if (c->cpr <= 0) return 0;
        if (c->pin_b < 0 || c->enc_a < 0 || c->enc_b < 0) return 0;
        /* vel_pid 合法性不在本校验：create 期 pid_init 单源执法
         * （非有限值/界序倒置/负 kb 一并拒，避免第二份校验表漂移）。 */
        break;
    }
    return 1;
}

motor_t *motor_create(const motor_cfg_t *cfg)
{
    if (cfg == NULL || !cfg_valid(cfg)) return NULL;

    /* 通道占用：桥类占 (channel, channel+1) 成对；范围与冲突由
     * pwm_timer 通道面执法（acquire 越界/重占即拒）。 */
    if (pwm_timer_channel_acquire(cfg->timer, cfg->channel) != 0)
        return NULL;
    uint8_t ch_rev = 0;
    if (uses_bridge(cfg->kind)) {
        /* channel ≤ 254（uint8 通道数上限 255），+1 不回绕；
         * 越界由 acquire 拒 */
        ch_rev = (uint8_t)(cfg->channel + 1u);
        if (pwm_timer_channel_acquire(cfg->timer, ch_rev) != 0) {
            (void)pwm_timer_channel_release(cfg->timer, cfg->channel);
            return NULL;
        }
    }

    motor_t *m = (motor_t *)calloc(1, sizeof *m);
    if (m == NULL) {   /* 资源不足：回滚占用，不留半挂实例 */
        (void)pwm_timer_channel_release(cfg->timer, cfg->channel);
        if (uses_bridge(cfg->kind))
            (void)pwm_timer_channel_release(cfg->timer, ch_rev);
        return NULL;
    }
    m->cfg = *cfg;          /* 深拷：之后改 cfg 不影响已建实例 */
    m->ch_rev = ch_rev;

    /* 速度环参数单源校验（pid_init 拒非有限值/界序倒置/负 kb）→ 拒建。
     * 仅 ENC_CLOSED 消费速度环：其余 kind 的 vel_pid 是未用字段
     * （如零值），不校验不初始化（poll 也绝不触碰）。 */
    if (cfg->kind == MOTOR_ENC_CLOSED) {
        pid_cfg_t pcfg;
        pid_cfg_from_motor(&pcfg, &cfg->vel_pid);
        if (pid_init(&m->pid, &pcfg) != 0) {
            free(m);
            (void)pwm_timer_channel_release(cfg->timer, cfg->channel);
            if (uses_bridge(cfg->kind))
                (void)pwm_timer_channel_release(cfg->timer, ch_rev);
            return NULL;
        }
    }

    /* 通道使能随实例生命周期（占空 0 = 恒低/无脉冲——上电安全态，
     * 见 pwm_hal.h 平台契约）；失败走完整销毁回滚路径。 */
    if (pwm_timer_channel_enable(cfg->timer, cfg->channel, 1) != 0) {
        motor_destroy(m);
        return NULL;
    }
    if (uses_bridge(cfg->kind) &&
        pwm_timer_channel_enable(cfg->timer, ch_rev, 1) != 0) {
        motor_destroy(m);
        return NULL;
    }
    return m;
}

int motor_destroy(motor_t *m)
{
    if (m == NULL) return -1;

    /* 安全收尾：先清两通道占空再停输出再还通道（还了就不能再写）。
     * 失败不短路：能保的先保住，最后如实汇报（透传语义）。 */
    int rc = 0;
    if (uses_bridge(m->cfg.kind)) {
        if (pwm_timer_channel_set_duty(m->cfg.timer, m->ch_rev, 0.0f) != 0) rc = -1;
        if (pwm_timer_channel_enable(m->cfg.timer, m->ch_rev, 0) != 0)     rc = -1;
        if (pwm_timer_channel_release(m->cfg.timer, m->ch_rev) != 0)       rc = -1;
    }
    if (pwm_timer_channel_set_duty(m->cfg.timer, m->cfg.channel, 0.0f) != 0) rc = -1;
    if (pwm_timer_channel_enable(m->cfg.timer, m->cfg.channel, 0) != 0)     rc = -1;
    if (pwm_timer_channel_release(m->cfg.timer, m->cfg.channel) != 0)       rc = -1;
    free(m);
    return rc;   /* 不动定时器——定时器由业务层自管（motor.h 契约） */
}

int motor_set_target(motor_t *m, float target)
{
    if (m == NULL || isnan(target)) return -1;
    m->target = target;   /* 越界值不拒：输出时钳位，actual 反映钳后真值 */
    return 0;
}

/* ── 各 kind 一拍输出（poll 分派目标）── */

/** @brief 开环 PWM：target 即归一化占窗（cfg 无刻度字段→线性斜率 1 直控） */
static int poll_pwm_open(motor_t *m)
{
    float d = m->target;
    if (d > m->cfg.duty_max) d = m->cfg.duty_max;   /* 上限钳位 */
    if (d < 0.0f) d = 0.0f;     /* 单向输出：负目标按 0 */
    if (pwm_timer_channel_set_duty(m->cfg.timer, m->cfg.channel, d) != 0) {
        m->fault |= MOTOR_FAULT_PWM_WRITE;
        return -1;
    }
    m->duty = d;
    m->applied = d;
    return 0;
}

/** @brief 方向桥双通道写（正=ch 幅值/ch+1 零；反对称）。失败置位不静默。 */
static int bridge_out(motor_t *m, int forward, float mag)
{
    float d_f = forward ? mag : 0.0f;
    float d_r = forward ? 0.0f : mag;
    if (pwm_timer_channel_set_duty(m->cfg.timer, m->cfg.channel, d_f) != 0 ||
        pwm_timer_channel_set_duty(m->cfg.timer, m->ch_rev, d_r) != 0) {
        m->fault |= MOTOR_FAULT_PWM_WRITE;
        return -1;
    }
    return 0;
}

/** @brief DC：target = 带符号归一化占窗（符号=桥向，幅值钳 duty_max） */
static int poll_dc(motor_t *m)
{
    int forward = (m->target >= 0.0f);
    float mag = forward ? m->target : -m->target;
    if (mag > m->cfg.duty_max) mag = m->cfg.duty_max;
    if (bridge_out(m, forward, mag) != 0) return -1;
    m->duty = mag;
    m->applied = forward ? mag : -mag;
    return 0;
}

/**
 * @brief   SERVO：角度→区间钳位→线性脉宽→占空
 * @note    换算：us = pmin + (a-amin)*(pmax-pmin)/(amax-amin)；
 *          duty = us*freq/1e6（频率经 pwm_timer_freq 现取——单源无分叉）。
 */
static int poll_servo(motor_t *m)
{
    float a = m->target;
    if (a < m->cfg.angle_min) a = m->cfg.angle_min;   /* 角度区间钳位 */
    if (a > m->cfg.angle_max) a = m->cfg.angle_max;
    float span_us = (float)(m->cfg.pulse_max_us - m->cfg.pulse_min_us);
    float span_a  = m->cfg.angle_max - m->cfg.angle_min;
    float us = (float)m->cfg.pulse_min_us
             + (a - m->cfg.angle_min) * span_us / span_a;
    if (us < (float)m->cfg.pulse_min_us) us = (float)m->cfg.pulse_min_us;  /* 浮点兜底 */
    if (us > (float)m->cfg.pulse_max_us) us = (float)m->cfg.pulse_max_us;
    float duty = us * (float)pwm_timer_freq(m->cfg.timer) / MTR_US_PER_SEC_F;
    if (duty > 1.0f) duty = 1.0f;   /* create 已验脉宽在周期内，冗余护栏 */

    if (pwm_timer_channel_set_duty(m->cfg.timer, m->cfg.channel, duty) != 0) {
        m->fault |= MOTOR_FAULT_PWM_WRITE;
        return -1;
    }
    m->duty = duty;
    m->applied = a;
    return 0;
}

/**
 * @brief   ENC_CLOSED：编码器差分→rpm→PID→方向桥输出
 * @note    首次成功读建基线（该拍不控——无差分无速度）；dt<=0 的重复拍
 *          只推进计数不控（无速度可算，保持上拍输出）。
 */
static int poll_enc(motor_t *m, float dt_s)
{
    int32_t count = 0;
    if (m->hal == NULL || m->hal->encoder_read == NULL) {
        m->fault |= MOTOR_FAULT_ENC_HAL;
        return -1;
    }
    if (m->hal->encoder_read(m->cfg.enc_a, m->cfg.enc_b, &count,
                             m->hal->ctx) != 0) {
        m->fault |= MOTOR_FAULT_ENC_READ;
        return -1;
    }
    if (!m->has_enc_base) {   /* 基线建立（含首拍失败后的迟到重建） */
        m->enc_base = count;
        m->has_enc_base = 1;
        return 0;
    }

    /* 差分：无符号减法（回绕安全）后回投 int32（模 2^32 语义）；
     * 驱动永不复位计数器——原子性契约见 motor_hal.h */
    int32_t delta = (int32_t)((uint32_t)count - (uint32_t)m->enc_base);
    m->enc_base = count;
    m->enc_count += delta;

    if (dt_s <= 0.0f) return 0;   /* 同拍重复：无速度，保持上拍输出 */

    /* 计数差 → rpm：rev = delta/cpr；rpm = rev/dt * 60 */
    float rpm = ((float)delta * MTR_RPM_PER_RPS_F)
              / ((float)m->cfg.cpr * dt_s);
    m->applied = rpm;   /* actual = 编码器测量真值 */

    /* 速度环：PID 出带符号占窗命令（pid.h 实例；S 梯度轨迹待 scurve
     * 库收编后挂此——见文件头注） */
    if (pid_step(&m->pid, m->target - rpm, dt_s) != 0)
        return -1;   /* 不可达（cfg 经 pid_init 校验 + dt>0 已前置）——防御性拒 */
    float u = m->pid.out;
    int forward = (u >= 0.0f);
    float mag = forward ? u : -u;
    if (mag > m->cfg.duty_max) mag = m->cfg.duty_max;
    if (bridge_out(m, forward, mag) != 0) return -1;
    m->duty = mag;
    return 0;
}

int motor_poll(motor_t *m, uint32_t now_us)
{
    if (m == NULL) return -1;

    /* 时基回退：拒绝推进（状态不动，位亮到下一次成功推进） */
    if (m->has_last_us && now_us < m->last_us) {
        m->fault |= MOTOR_FAULT_TIME_REGRESS;
        return -1;
    }
    m->fault = 0u;   /* 活状态位：本拍重估（可恢复故障自清） */

    int first = !m->has_last_us;
    uint32_t dt_us = first ? 0u : (now_us - m->last_us);
    m->last_us = now_us;
    m->has_last_us = 1;
    float dt_s = (float)dt_us / MTR_US_PER_SEC_F;

    switch (m->cfg.kind) {
    case MOTOR_PWM_OPEN:   return poll_pwm_open(m);
    case MOTOR_DC:         return poll_dc(m);
    case MOTOR_SERVO:      return poll_servo(m);
    case MOTOR_ENC_CLOSED: return poll_enc(m, dt_s);
    }
    return -1;   /* 不可达（kind 经 create 校验）；防御性拒 */
}

int motor_get_state(const motor_t *m, motor_state_t *out)
{
    if (m == NULL || out == NULL) return -1;
    out->target    = m->target;
    out->actual    = m->applied;
    out->duty      = m->duty;
    out->enc_count = m->enc_count;
    out->fault     = m->fault;
    return 0;
}

/* ── 编码器 HAL 挂载（motor_hal.h 契约的实现面——实例自持指针，零全局态） ── */

int motor_hal_attach(motor_t *m, const motor_hal_t *hal)
{
    if (m == NULL || hal == NULL) return -1;
    if (m->cfg.kind == MOTOR_ENC_CLOSED && hal->encoder_read == NULL)
        return -1;   /* 闭环必需成员缺位：fail-fast（入口全参数校验） */
    m->hal = hal;
    return 0;
}
