/* SPDX-License-Identifier: MIT */
/**
 * @file    test_motor.c
 * @brief   电机实例核心无硬件单测（mock 平台层：pwm_hal + 编码器 HAL + 合成对象）
 *
 * @note    pwm_timer.c 为真实链接件——本文件**不**mock 它；
 *          mock 的是平台层两条缝：pwm_hal_t（PWM 平台表，启动期注册）与
 *          motor_hal_t（编码器表，实例 attach）。与 test_pwm_timer.c 同构分层。
 * @note    自研极简断言（不引第三方测试框架，风格对齐 test_imu.c）。
 * @note    验证范围：create 全参数校验逐项拒 / 四类 kind 分派换算数值断言
 *          （PWM 钳位、SERVO 角线性、DC 方向桥双通道）/ ENC 合成编码器
 *          （恒速注入测量 + 一阶对象闭环收敛 ±5%）/ 时基回退拒 / HAL
 *          挂载校验与故障位 / 生命周期（destroy 停输出还通道）/ 多电机
 *          共定时器分通道互不干扰 / 预设表。
 *          不覆盖：真实 LEDC/PCNT 时序（需实机验证）；S 梯度轨迹由
 *          test_scurve.c 单测覆盖。
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "motor.h"
#include "motor_hal.h"
#include "pwm_timer.h"
#include "pwm_hal.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)
#define CHECK_EQ(a, b, msg) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a == _b) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s: %ld != %ld (line %d)\n", msg, _a, _b, __LINE__); } \
} while (0)
/** @brief 浮点等值（绝对误差 eps——真值断言用，禁止以值域哨兵判态） */
static int feqe(float a, float b, float eps)
{
    float d = a - b;
    return (d < eps) && (d > -eps);
}

/* ═══ mock PWM 平台（pwm_hal_t 契约的测试实现：资源池 + 观测面） ═══ */

#define MOCK_TIMERS 16   /* 平台定时器池容量（测试够用即可） */
#define MOCK_CHS    8    /* 单定时器通道上限 */

typedef struct {
    int      live;               /* 1 = 平台定时器存在；release 置 0 */
    float    duty[MOCK_CHS];     /* 每通道最后占空（观测点） */
    int      enabled[MOCK_CHS];  /* 每通道使能态（观测点） */
} mt_t;

static mt_t mt_pool[MOCK_TIMERS];
static int  mt_last = -1;                    /* 最近一次 timer_config 占的池位 */
static mt_t *g_plat[MOCK_TIMERS];            /* 业务层定时器 → 池位（建测序记录） */
static int  g_nplat;

static int mock_timer_config(uint32_t freq_hz, uint8_t channels,
                             void **out_plat, void *ctx)
{
    (void)ctx;
    if (out_plat == NULL) return -1;
    if (freq_hz == 0u || channels == 0u || channels > MOCK_CHS) return -1;
    for (int i = 0; i < MOCK_TIMERS; i++) {
        if (mt_pool[i].live) continue;
        mt_pool[i].live = 1;
        for (int c = 0; c < MOCK_CHS; c++) {
            mt_pool[i].duty[c] = 0.0f;      /* 平台契约：上电安全态 */
            mt_pool[i].enabled[c] = 0;
        }
        mt_last = i;
        *out_plat = &mt_pool[i];
        return 0;
    }
    return -1;   /* 池满 */
}

static int mock_timer_release(void *plat, void *ctx)
{
    (void)ctx;
    mt_t *m = (mt_t *)plat;
    if (m == NULL || !m->live) return -1;
    m->live = 0;
    return 0;
}

static int mock_channel_set_duty(void *plat, uint8_t channel, float duty, void *ctx)
{
    (void)ctx;
    mt_t *m = (mt_t *)plat;
    if (m == NULL || !m->live) return -1;
    if (channel >= MOCK_CHS) return -1;
    m->duty[channel] = duty;
    return 0;
}

static int mock_channel_enable(void *plat, uint8_t channel, int enable, void *ctx)
{
    (void)ctx;
    mt_t *m = (mt_t *)plat;
    if (m == NULL || !m->live) return -1;
    if (channel >= MOCK_CHS) return -1;
    m->enabled[channel] = enable ? 1 : 0;
    return 0;
}

static const pwm_hal_t mock_pwm_hal = {
    mock_timer_config, mock_timer_release,
    mock_channel_set_duty, mock_channel_enable, NULL,
};

/** @brief 建定时器并登记池位（观测序号 = 创建序） */
static pwm_timer_t *mk_timer(uint32_t freq_hz, uint8_t channels)
{
    pwm_timer_t *t = pwm_timer_create(freq_hz, channels);
    if (t != NULL && g_nplat < MOCK_TIMERS)
        g_plat[g_nplat++] = &mt_pool[mt_last];
    return t;
}
/** @brief 观测：第 which 个（mk_timer 创建序）定时器的 ch 通道占空 */
static float mock_duty(int which, uint8_t ch)
{
    return g_plat[which]->duty[ch];
}
static int mock_enabled(int which, uint8_t ch)
{
    return g_plat[which]->enabled[ch];
}

/* ═══ mock 编码器 HAL（motor_hal_t 契约）+ 合成电机对象 ═══ */

/*
 * 合成对象（闭环用例）：一阶转速模型 rpm →(K,tau)← 桥驱动（正反通道占空差），
 * 编码器计数 = 转速积分（浮点累加器 → 整数快照，模拟计数器量化）。
 */
#define PLANT_K_RPM_PER_DUTY 100.0   /* 满占空稳态转速（合成对象增益） */
#define PLANT_TAU_S          0.05    /* 一阶机械时间常数 s */
#define ENC_CPR_TEST         6000    /* 合成电机输出轴每转计数 */
#define POLL_DT_US           10000u  /* 控制节拍 10ms（100Hz） */
#define LOOP_STEPS           800     /* 闭环仿真窗 8s */
#define TARGET_RPM           60.0f   /* 闭环目标 */
#define TOL_RPM              3.0f    /* 收敛容差（目标 ±5%） */

static struct {
    double   rpm;        /* 对象当前转速 rpm */
    double   acc;        /* 编码器累计（浮点累加器，count） */
    int32_t  snap;       /* 上次读出的整数快照（count） */
    int      fail_next;  /* 注入接下来 N 次编码器读失败 */
} plant;

static int mock_enc_read(int pin_a, int pin_b, int32_t *count, void *ctx)
{
    (void)ctx;
    if (plant.fail_next > 0) { plant.fail_next--; return -1; }
    if (pin_a < 0 || pin_b < 0 || count == NULL) return -1;  /* 平台防御 */
    *count = plant.snap;
    return 0;
}

static const motor_hal_t mock_enc_hal = { mock_enc_read, NULL };

static void plant_reset(void)
{
    memset(&plant, 0, sizeof plant);
}

/* ═══ 用例 ═══ */

int main(void)
{
    motor_state_t st;

    CHECK_EQ(pwm_hal_register(&mock_pwm_hal), 0, "register mock pwm platform");

    /* ---- 1. create 全参数校验逐项拒（先建合法对照占位，再逐项坏） ---- */
    {
        pwm_timer_t *t20 = mk_timer(20000u, 4u);   /* idx 0 */
        pwm_timer_t *t50 = mk_timer(50u, 4u);      /* idx 1 */
        pwm_timer_t *t20b = mk_timer(20000u, 4u);  /* idx 2（enc 对照用） */
        CHECK(t20 != NULL && t50 != NULL && t20b != NULL, "control timers created");

        motor_cfg_t pwm_base = {
            .kind = MOTOR_PWM_OPEN, .timer = t20, .channel = 0,
            .pin_a = 5, .pin_b = -1, .enc_a = -1, .enc_b = -1, .cpr = 0,
            .pulse_min_us = 0, .pulse_max_us = 0,
            .angle_min = 0.0f, .angle_max = 0.0f,
            .vel_pid = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
            .duty_max = 1.0f,
        };
        motor_cfg_t dc_base = pwm_base;
        dc_base.kind = MOTOR_DC; dc_base.channel = 1;
        dc_base.pin_a = 10; dc_base.pin_b = 11;
        motor_cfg_t sv_base = pwm_base;
        sv_base.kind = MOTOR_SERVO; sv_base.timer = t50; sv_base.channel = 0;
        sv_base.pin_a = 6;
        sv_base.pulse_min_us = 500; sv_base.pulse_max_us = 2500;
        sv_base.angle_min = 0.0f; sv_base.angle_max = 180.0f;
        motor_cfg_t enc_base = pwm_base;
        enc_base.kind = MOTOR_ENC_CLOSED; enc_base.timer = t20b;
        enc_base.channel = 0; enc_base.pin_a = 12; enc_base.pin_b = 13;
        enc_base.enc_a = 2; enc_base.enc_b = 3; enc_base.cpr = 6000;
        enc_base.vel_pid = (motor_pid_cfg_t){ 0.015f, 1.0f, 0.0f, -1.0f, 1.0f };

        /* 合法对照（四类各一，占位供冲突用例） */
        motor_t *mp = motor_create(&pwm_base);
        motor_t *md = motor_create(&dc_base);     /* 占通道对 (1,2) */
        motor_t *ms = motor_create(&sv_base);
        motor_t *me = motor_create(&enc_base);    /* 占 t20b 通道对 (0,1) */
        CHECK(mp != NULL, "control pwm created");
        CHECK(md != NULL, "control dc created");
        CHECK(ms != NULL, "control servo created");
        CHECK(me != NULL, "control enc created");

        /* 通用面 */
        CHECK(motor_create(NULL) == NULL, "reject NULL cfg");
        {
            motor_cfg_t c = pwm_base; c.timer = NULL;
            CHECK(motor_create(&c) == NULL, "reject NULL timer");
        }
        {
            motor_cfg_t c = pwm_base; c.kind = (motor_kind_t)99;
            CHECK(motor_create(&c) == NULL, "reject bad kind");
        }
        {
            motor_cfg_t c = pwm_base; c.pin_a = -1;
            CHECK(motor_create(&c) == NULL, "reject pin_a < 0");
        }
        {
            motor_cfg_t c = pwm_base; c.duty_max = 0.0f;
            CHECK(motor_create(&c) == NULL, "reject duty_max = 0");
        }
        {
            motor_cfg_t c = pwm_base; c.duty_max = 1.5f;
            CHECK(motor_create(&c) == NULL, "reject duty_max > 1");
        }
        {
            motor_cfg_t c = pwm_base; c.duty_max = (float)NAN;
            CHECK(motor_create(&c) == NULL, "reject duty_max NaN");
        }
        {
            motor_cfg_t c = pwm_base; c.channel = 4;   /* 4 通道定时器：0..3 */
            CHECK(motor_create(&c) == NULL, "reject channel out of range");
        }
        {
            motor_cfg_t c = pwm_base; c.channel = 2;   /* 被 DC 对照的桥对占用 */
            CHECK(motor_create(&c) == NULL, "reject occupied channel");
        }

        /* DC 面（方向桥双通道：channel+1 也须在界内且空闲） */
        {
            motor_cfg_t c = dc_base; c.pin_b = -1;
            CHECK(motor_create(&c) == NULL, "reject dc pin_b < 0");
        }
        {
            motor_cfg_t c = dc_base; c.channel = 3;    /* 反向通道 4 越界 */
            CHECK(motor_create(&c) == NULL, "reject dc bridge pair out of range");
        }

        /* SERVO 面 */
        {
            motor_cfg_t c = sv_base; c.pulse_min_us = 0;
            CHECK(motor_create(&c) == NULL, "reject servo pulse_min = 0");
        }
        {
            motor_cfg_t c = sv_base; c.pulse_min_us = 2500;
            CHECK(motor_create(&c) == NULL, "reject servo pulse_min >= max");
        }
        {
            motor_cfg_t c = sv_base; c.angle_min = 5.0f; c.angle_max = 5.0f;
            CHECK(motor_create(&c) == NULL, "reject servo angle min == max");
        }
        {
            motor_cfg_t c = sv_base; c.angle_min = 10.0f; c.angle_max = 0.0f;
            CHECK(motor_create(&c) == NULL, "reject servo angle min > max");
        }
        {
            motor_cfg_t c = sv_base; c.timer = t20;    /* 2500us*20kHz > 周期 50us */
            CHECK(motor_create(&c) == NULL, "reject servo pulse wider than carrier period");
        }
        {
            motor_cfg_t c = sv_base;   /* t50 ch0 已被对照舵机占用 */
            CHECK(motor_create(&c) == NULL, "reject servo channel conflict");
        }

        /* ENC 面 */
        {
            motor_cfg_t c = enc_base; c.cpr = 0;
            CHECK(motor_create(&c) == NULL, "reject enc cpr = 0");
        }
        {
            motor_cfg_t c = enc_base; c.cpr = -100;
            CHECK(motor_create(&c) == NULL, "reject enc cpr < 0");
        }
        {
            motor_cfg_t c = enc_base; c.enc_a = -1;
            CHECK(motor_create(&c) == NULL, "reject enc enc_a < 0");
        }
        {
            motor_cfg_t c = enc_base; c.enc_b = -1;
            CHECK(motor_create(&c) == NULL, "reject enc enc_b < 0");
        }
        {
            motor_cfg_t c = enc_base; c.pin_b = -1;
            CHECK(motor_create(&c) == NULL, "reject enc pin_b < 0");
        }
        {
            motor_cfg_t c = enc_base;
            c.vel_pid.out_min = 1.0f; c.vel_pid.out_max = -1.0f;
            CHECK(motor_create(&c) == NULL, "reject pid out_min > out_max");
        }
        {
            motor_cfg_t c = enc_base;
            c.vel_pid.out_min = 0.5f; c.vel_pid.out_max = 0.5f;
            CHECK(motor_create(&c) == NULL, "reject pid out_min == out_max");
        }
        {
            motor_cfg_t c = enc_base; c.vel_pid.kp = (float)NAN;
            CHECK(motor_create(&c) == NULL, "reject pid kp NaN");
        }

        /* 深拷证据：改源 cfg 不影响已建实例（对照电机的状态独立取数） */
        {
            motor_cfg_t mutated = pwm_base; mutated.duty_max = 0.123f;
            (void)mutated;   /* 源结构体怎么改都不触及实例内部 */
            CHECK_EQ(motor_set_target(mp, 0.6f), 0, "control target set");
            CHECK_EQ(motor_poll(mp, 100u), 0, "control poll");
            CHECK_EQ(motor_get_state(mp, &st), 0, "control get_state");
            CHECK(feqe(st.duty, 0.6f, 0.000001f), "control duty unaffected by cfg mutation");
        }

        CHECK_EQ(motor_destroy(mp), 0, "destroy control pwm");
        CHECK_EQ(motor_destroy(md), 0, "destroy control dc");
        CHECK_EQ(motor_destroy(ms), 0, "destroy control servo");
        CHECK_EQ(motor_destroy(me), 0, "destroy control enc");
        CHECK_EQ(pwm_timer_destroy(t20), 0, "cleanup t20");
        CHECK_EQ(pwm_timer_destroy(t50), 0, "cleanup t50");
        CHECK_EQ(pwm_timer_destroy(t20b), 0, "cleanup t20b");
    }

    /* ---- 2. PWM_OPEN：target→duty 线性 + duty_max 钳位 ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 3 */
        motor_cfg_t c = *motor_cfg_find("motor.pwm.generic");
        c.timer = t; c.channel = 0; c.pin_a = 5;
        c.duty_max = 0.7f;                        /* 业务层改钳位上限 */
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "pwm motor created");

        CHECK_EQ(motor_set_target(m, 1.0f), 0, "pwm set target 1.0");
        CHECK_EQ(motor_poll(m, 100u), 0, "pwm poll");
        CHECK(feqe(mock_duty(3, 0), 0.7f, 0.000001f), "pwm duty clamped to duty_max");
        CHECK_EQ(motor_get_state(m, &st), 0, "pwm get_state");
        CHECK(feqe(st.duty, 0.7f, 0.000001f), "pwm state duty clamped");
        CHECK(feqe(st.actual, 0.7f, 0.000001f), "pwm state actual = applied");
        CHECK(feqe(st.target, 1.0f, 0.000001f), "pwm state target raw");
        CHECK_EQ(st.fault, 0u, "pwm fault clean");

        CHECK_EQ(motor_set_target(m, -0.5f), 0, "pwm set negative target");
        CHECK_EQ(motor_poll(m, 200u), 0, "pwm poll 2");
        CHECK(feqe(mock_duty(3, 0), 0.0f, 0.000001f), "pwm negative target -> duty 0");

        CHECK_EQ(motor_set_target(m, 0.3f), 0, "pwm set target 0.3");
        CHECK_EQ(motor_poll(m, 300u), 0, "pwm poll 3");
        CHECK(feqe(mock_duty(3, 0), 0.3f, 0.000001f), "pwm linear passthrough");

        CHECK_EQ(motor_destroy(m), 0, "pwm destroy");
        CHECK(feqe(mock_duty(3, 0), 0.0f, 0.000001f), "pwm destroy stops output");
        CHECK_EQ(mock_enabled(3, 0), 0, "pwm destroy disables channel");
        CHECK_EQ(pwm_timer_destroy(t), 0, "pwm cleanup timer");
    }

    /* ---- 3. DC：方向桥双通道 + duty 幅值钳位 ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 4；桥对 (2,3) */
        motor_cfg_t c = *motor_cfg_find("motor.dc.generic");
        c.timer = t; c.channel = 2; c.pin_a = 10; c.pin_b = 11;
        c.duty_max = 0.7f;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "dc motor created");

        CHECK_EQ(motor_set_target(m, -1.0f), 0, "dc set target -1.0 (reverse)");
        CHECK_EQ(motor_poll(m, 100u), 0, "dc poll reverse");
        CHECK(feqe(mock_duty(4, 2), 0.0f, 0.000001f), "dc reverse: fwd channel 0");
        CHECK(feqe(mock_duty(4, 3), 0.7f, 0.000001f), "dc reverse: rev channel = duty_max");
        CHECK_EQ(motor_get_state(m, &st), 0, "dc get_state");
        CHECK(feqe(st.duty, 0.7f, 0.000001f), "dc state duty magnitude");
        CHECK(feqe(st.actual, -0.7f, 0.000001f), "dc state actual signed");
        CHECK_EQ(st.fault, 0u, "dc fault clean");

        CHECK_EQ(motor_set_target(m, 0.4f), 0, "dc set target 0.4 (forward)");
        CHECK_EQ(motor_poll(m, 200u), 0, "dc poll forward");
        CHECK(feqe(mock_duty(4, 2), 0.4f, 0.000001f), "dc forward: fwd channel 0.4");
        CHECK(feqe(mock_duty(4, 3), 0.0f, 0.000001f), "dc forward: rev channel 0");
        CHECK_EQ(motor_get_state(m, &st), 0, "dc get_state forward");
        CHECK(feqe(st.duty, 0.4f, 0.000001f), "dc state duty forward");
        CHECK(feqe(st.actual, 0.4f, 0.000001f), "dc state actual forward");

        CHECK_EQ(motor_set_target(m, 0.0f), 0, "dc set target 0");
        CHECK_EQ(motor_poll(m, 300u), 0, "dc poll stop");
        CHECK(feqe(mock_duty(4, 2), 0.0f, 0.000001f) &&
              feqe(mock_duty(4, 3), 0.0f, 0.000001f), "dc zero target both channels 0");

        CHECK_EQ(motor_destroy(m), 0, "dc destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "dc cleanup timer");
    }

    /* ---- 4. SERVO：角度→脉宽→占空（角线性数值断言） ---- */
    {
        pwm_timer_t *t = mk_timer(50u, 4u);      /* idx 5：50Hz，周期 20000us */
        motor_cfg_t c = *motor_cfg_find("motor.servo.sg90");
        c.timer = t; c.channel = 0; c.pin_a = 6;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "servo motor created");

        /* 期望占空 = 期望脉宽 us * 50Hz / 1e6（与驱动同式换算） */
        float d000 = 500.0f  * 50.0f / 1000000.0f;
        float d045 = 1000.0f * 50.0f / 1000000.0f;
        float d090 = 1500.0f * 50.0f / 1000000.0f;
        float d135 = 2000.0f * 50.0f / 1000000.0f;
        float d180 = 2500.0f * 50.0f / 1000000.0f;

        CHECK_EQ(motor_set_target(m, 0.0f), 0, "servo set 0");
        CHECK_EQ(motor_poll(m, 100u), 0, "servo poll 0");
        CHECK(feqe(mock_duty(5, 0), d000, 0.000001f), "servo 0deg -> 500us duty");

        CHECK_EQ(motor_set_target(m, 45.0f), 0, "servo set 45");
        CHECK_EQ(motor_poll(m, 200u), 0, "servo poll 45");
        CHECK(feqe(mock_duty(5, 0), d045, 0.000001f), "servo 45deg -> 1000us duty");

        CHECK_EQ(motor_set_target(m, 90.0f), 0, "servo set 90");
        CHECK_EQ(motor_poll(m, 300u), 0, "servo poll 90");
        CHECK(feqe(mock_duty(5, 0), d090, 0.000001f), "servo 90deg -> 1500us duty");

        CHECK_EQ(motor_set_target(m, 135.0f), 0, "servo set 135");
        CHECK_EQ(motor_poll(m, 400u), 0, "servo poll 135");
        CHECK(feqe(mock_duty(5, 0), d135, 0.000001f), "servo 135deg -> 2000us duty");

        CHECK_EQ(motor_set_target(m, 180.0f), 0, "servo set 180");
        CHECK_EQ(motor_poll(m, 500u), 0, "servo poll 180");
        CHECK(feqe(mock_duty(5, 0), d180, 0.000001f), "servo 180deg -> 2500us duty");

        /* 角线性：中点占空 = 端点占差的算术中点（45/90/135 三点共线） */
        CHECK(feqe(d090, (d045 + d135) * 0.5f, 0.000001f), "servo midpoint linearity");

        /* 区间钳位：越界目标压回边界（真值钳位，不哨兵） */
        CHECK_EQ(motor_set_target(m, 200.0f), 0, "servo set 200 (over)");
        CHECK_EQ(motor_poll(m, 600u), 0, "servo poll over-range");
        CHECK(feqe(mock_duty(5, 0), d180, 0.000001f), "servo 200deg clamped to 2500us");
        CHECK_EQ(motor_get_state(m, &st), 0, "servo get_state clamped");
        CHECK(feqe(st.actual, 180.0f, 0.000001f), "servo actual clamped angle");
        CHECK(feqe(st.target, 200.0f, 0.000001f), "servo target raw kept");

        CHECK_EQ(motor_set_target(m, -30.0f), 0, "servo set -30 (under)");
        CHECK_EQ(motor_poll(m, 700u), 0, "servo poll under-range");
        CHECK(feqe(mock_duty(5, 0), d000, 0.000001f), "servo -30deg clamped to 500us");
        CHECK_EQ(motor_get_state(m, &st), 0, "servo get_state under");
        CHECK(feqe(st.actual, 0.0f, 0.000001f), "servo actual clamped low angle");

        CHECK_EQ(motor_destroy(m), 0, "servo destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "servo cleanup timer");
    }

    /* ---- 5. ENC 合成编码器（恒转速注入）：差分→rpm 测量数值 ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 6 */
        motor_cfg_t c;
        memset(&c, 0, sizeof c);
        c.kind = MOTOR_ENC_CLOSED; c.timer = t; c.channel = 0;
        c.pin_a = 12; c.pin_b = 13; c.enc_a = 2; c.enc_b = 3;
        c.cpr = ENC_CPR_TEST;
        c.vel_pid = (motor_pid_cfg_t){ 0.015f, 1.0f, 0.0f, -1.0f, 1.0f };
        c.duty_max = 1.0f;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "enc motor created");
        CHECK_EQ(motor_hal_attach(m, &mock_enc_hal), 0, "enc hal attached");

        plant_reset();
        CHECK_EQ(motor_set_target(m, 0.0f), 0, "enc set target 0");
        uint32_t now = 0u;
        /* 恒转速注入：每拍 +120 count（cpr 6000、dt 10ms → 恒 120 rpm） */
        CHECK_EQ(motor_poll(m, now), 0, "enc baseline poll");
        for (int i = 1; i <= 12; i++) {
            plant.snap += 120;
            now += POLL_DT_US;
            CHECK_EQ(motor_poll(m, now), 0, "enc measure poll");
        }
        CHECK_EQ(motor_get_state(m, &st), 0, "enc get_state");
        CHECK(feqe(st.actual, 120.0f, 0.01f), "enc measured rpm = 120 (constant rate)");
        CHECK_EQ(st.enc_count, 120 * 12, "enc cumulative count");
        CHECK_EQ(st.fault, 0u, "enc fault clean");

        CHECK_EQ(motor_destroy(m), 0, "enc measure destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "enc measure cleanup timer");
    }

    /* ---- 6. ENC 闭环：一阶对象收敛到目标 ±5% ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 7；桥对 (0,1) */
        motor_cfg_t c;
        memset(&c, 0, sizeof c);
        c.kind = MOTOR_ENC_CLOSED; c.timer = t; c.channel = 0;
        c.pin_a = 12; c.pin_b = 13; c.enc_a = 2; c.enc_b = 3;
        c.cpr = ENC_CPR_TEST;
        c.vel_pid = (motor_pid_cfg_t){ 0.015f, 1.0f, 0.0f, -1.0f, 1.0f };
        c.duty_max = 1.0f;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "closed-loop motor created");
        CHECK_EQ(motor_hal_attach(m, &mock_enc_hal), 0, "closed-loop hal attached");

        plant_reset();
        CHECK_EQ(motor_set_target(m, TARGET_RPM), 0, "closed-loop target 60 rpm");

        uint32_t now = 0u;
        float actual_mid = 0.0f;
        for (int i = 0; i < LOOP_STEPS; i++) {
            CHECK_EQ(motor_poll(m, now), 0, "closed-loop poll");
            /* 合成对象推进：桥驱动 = 正反通道占空差；一阶趋近 + 计数积分 */
            double drive = (double)mock_duty(7, 0) - (double)mock_duty(7, 1);
            double dt = (double)POLL_DT_US / 1000000.0;
            plant.rpm += (PLANT_K_RPM_PER_DUTY * drive - plant.rpm) * dt / PLANT_TAU_S;
            plant.acc += plant.rpm / 60.0 * ENC_CPR_TEST * dt;
            plant.snap = (int32_t)plant.acc;
            now += POLL_DT_US;
            if (i == 199) {   /* 2s 处中期收敛证据 */
                CHECK_EQ(motor_get_state(m, &st), 0, "mid get_state");
                actual_mid = st.actual;
                CHECK(st.actual > TARGET_RPM - TOL_RPM &&
                      st.actual < TARGET_RPM + TOL_RPM, "closed-loop converged at 2s");
            }
        }
        CHECK_EQ(motor_get_state(m, &st), 0, "final get_state");
        CHECK(st.actual > TARGET_RPM - TOL_RPM &&
              st.actual < TARGET_RPM + TOL_RPM, "closed-loop final within +/-5%");
        CHECK_EQ(st.fault, 0u, "closed-loop fault clean");
        CHECK(st.enc_count > 0, "closed-loop encoder counting");
        CHECK(feqe(mock_duty(7, 1), 0.0f, 0.000001f), "closed-loop forward drive: rev channel idle");
        CHECK(mock_duty(7, 0) > 0.0f, "closed-loop forward drive: fwd channel active");
        printf("  enc closed-loop: target=%.1f mid=%.2f final=%.2f rpm (tol +/-%.1f)\n",
               (double)TARGET_RPM, (double)actual_mid, (double)st.actual, (double)TOL_RPM);

        CHECK_EQ(motor_destroy(m), 0, "closed-loop destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "closed-loop cleanup timer");
    }

    /* ---- 7. HAL 挂载校验 + 编码器故障位（非锁存，可恢复自清） ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 8 */
        motor_cfg_t c;
        memset(&c, 0, sizeof c);
        c.kind = MOTOR_ENC_CLOSED; c.timer = t; c.channel = 0;
        c.pin_a = 12; c.pin_b = 13; c.enc_a = 2; c.enc_b = 3;
        c.cpr = ENC_CPR_TEST;
        c.vel_pid = (motor_pid_cfg_t){ 0.015f, 1.0f, 0.0f, -1.0f, 1.0f };
        c.duty_max = 1.0f;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "hal-test motor created");

        /* 未挂载即 poll：拒 + ENC_HAL 位 */
        plant_reset();
        CHECK_EQ(motor_poll(m, 100u), -1, "poll without hal rejected");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after hal-miss");
        CHECK_EQ(st.fault & MOTOR_FAULT_ENC_HAL, MOTOR_FAULT_ENC_HAL, "ENC_HAL bit set");

        /* attach 校验：NULL / 缺 encoder_read 成员 */
        CHECK_EQ(motor_hal_attach(m, NULL), -1, "attach NULL hal rejected");
        {
            const motor_hal_t no_read = { NULL, NULL };
            CHECK_EQ(motor_hal_attach(m, &no_read), -1, "attach hal without encoder_read rejected");
        }
        CHECK_EQ(motor_hal_attach(NULL, &mock_enc_hal), -1, "attach to NULL motor rejected");

        /* 挂上后恢复：poll 成功、位自清（基线拍） */
        CHECK_EQ(motor_hal_attach(m, &mock_enc_hal), 0, "attach ok after validation");
        CHECK_EQ(motor_poll(m, 200u), 0, "poll ok after attach");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after recovery");
        CHECK_EQ(st.fault, 0u, "fault self-cleared after recovery");

        /* 读失败注入：拒 + ENC_READ 位，恢复后自清 */
        plant_reset();
        plant.fail_next = 1;
        CHECK_EQ(motor_poll(m, 300u), -1, "poll rejected on encoder read failure");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after read failure");
        CHECK_EQ(st.fault & MOTOR_FAULT_ENC_READ, MOTOR_FAULT_ENC_READ, "ENC_READ bit set");
        CHECK_EQ(motor_poll(m, 400u), 0, "poll recovers after read failure");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after read recovery");
        CHECK_EQ(st.fault, 0u, "ENC_READ self-cleared");

        CHECK_EQ(motor_destroy(m), 0, "hal-test destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "hal-test cleanup timer");
    }

    /* ---- 8. 时基回退拒 -1（位亮到下一次成功推进） ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 9 */
        motor_cfg_t c = *motor_cfg_find("motor.pwm.generic");
        c.timer = t; c.channel = 0; c.pin_a = 5;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "regress motor created");
        CHECK_EQ(motor_set_target(m, 0.5f), 0, "regress set target");

        CHECK_EQ(motor_poll(m, 5000u), 0, "poll at t=5000");
        CHECK_EQ(motor_poll(m, 4000u), -1, "time regress rejected");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after regress");
        CHECK_EQ(st.fault & MOTOR_FAULT_TIME_REGRESS, MOTOR_FAULT_TIME_REGRESS, "REGRESS bit set");
        CHECK(feqe(mock_duty(9, 0), 0.5f, 0.000001f), "output unchanged on rejected poll");
        CHECK_EQ(motor_poll(m, 6000u), 0, "monotonic poll recovers");
        CHECK_EQ(motor_get_state(m, &st), 0, "get_state after recover");
        CHECK_EQ(st.fault, 0u, "REGRESS bit self-cleared");

        CHECK_EQ(motor_destroy(m), 0, "regress destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "regress cleanup timer");
    }

    /* ---- 9. 生命周期：占用守卫 / destroy 停输出 / 通道可复用 ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 10 */
        motor_cfg_t c = *motor_cfg_find("motor.pwm.generic");
        c.timer = t; c.channel = 1; c.pin_a = 5;
        motor_t *m = motor_create(&c);
        CHECK(m != NULL, "lifecycle motor created");
        CHECK_EQ(motor_set_target(m, 0.8f), 0, "lifecycle set target");
        CHECK_EQ(motor_poll(m, 100u), 0, "lifecycle poll");
        CHECK(feqe(mock_duty(10, 1), 0.8f, 0.000001f), "lifecycle output before destroy");

        CHECK_EQ(pwm_timer_destroy(t), -1, "timer destroy blocked while motor alive");
        CHECK_EQ(motor_destroy(m), 0, "lifecycle destroy");
        CHECK(feqe(mock_duty(10, 1), 0.0f, 0.000001f), "destroy zeroes duty");
        CHECK_EQ(mock_enabled(10, 1), 0, "destroy disables channel");
        CHECK_EQ(pwm_timer_destroy(t), 0, "timer destroy ok after motor destroyed");
        CHECK_EQ(motor_destroy(NULL), -1, "destroy NULL rejected");

        /* 通道复用：destroy 已还，可再造 */
        pwm_timer_t *t2 = mk_timer(20000u, 4u);  /* idx 11 */
        c.timer = t2;
        motor_t *m2 = motor_create(&c);
        CHECK(m2 != NULL, "channel reusable after release");
        CHECK_EQ(motor_destroy(m2), 0, "reuse destroy");
        CHECK_EQ(pwm_timer_destroy(t2), 0, "reuse cleanup");
    }

    /* ---- 10. 多电机共定时器：同句柄不同通道，互不干扰（实例化设计） ---- */
    {
        pwm_timer_t *t = mk_timer(20000u, 4u);   /* idx 12 */
        motor_cfg_t c = *motor_cfg_find("motor.pwm.generic");
        c.timer = t; c.pin_a = 5;
        c.channel = 0; c.duty_max = 1.0f;
        motor_t *m1 = motor_create(&c);
        c.channel = 1;
        motor_t *m2 = motor_create(&c);
        CHECK(m1 != NULL && m2 != NULL, "two motors share one timer");
        c.channel = 0;
        CHECK(motor_create(&c) == NULL, "third motor on taken channel rejected");

        CHECK_EQ(motor_set_target(m1, 0.5f), 0, "m1 target 0.5");
        CHECK_EQ(motor_set_target(m2, 0.9f), 0, "m2 target 0.9");
        CHECK_EQ(motor_poll(m2, 100u), 0, "poll m2 first");
        CHECK_EQ(motor_poll(m1, 200u), 0, "poll m1 second");
        CHECK(feqe(mock_duty(12, 0), 0.5f, 0.000001f), "m1 duty isolated");
        CHECK(feqe(mock_duty(12, 1), 0.9f, 0.000001f), "m2 duty isolated");

        /* NULL 参数面（借活实例） */
        CHECK_EQ(motor_set_target(NULL, 1.0f), -1, "set_target NULL rejected");
        CHECK_EQ(motor_set_target(m1, (float)NAN), -1, "set_target NaN rejected");
        CHECK_EQ(motor_get_state(NULL, &st), -1, "get_state NULL motor rejected");
        CHECK_EQ(motor_get_state(m1, NULL), -1, "get_state NULL out rejected");
        CHECK_EQ(motor_poll(NULL, 0u), -1, "poll NULL rejected");

        CHECK_EQ(motor_destroy(m1), 0, "m1 destroy");
        CHECK_EQ(motor_destroy(m2), 0, "m2 destroy");
        CHECK_EQ(pwm_timer_destroy(t), 0, "shared timer cleanup");
    }

    /* ---- 11. 预设注册表 ---- */
    {
        CHECK_EQ(motor_cfg_preset_count, 4, "preset count");
        const motor_cfg_t *pp = motor_cfg_find("motor.pwm.generic");
        const motor_cfg_t *pd = motor_cfg_find("motor.dc.generic");
        const motor_cfg_t *ps = motor_cfg_find("motor.servo.sg90");
        const motor_cfg_t *pe = motor_cfg_find("motor.enc.n20");
        CHECK(pp != NULL && pd != NULL && ps != NULL && pe != NULL, "all presets found");
        CHECK(motor_cfg_find("no.such.preset") == NULL, "preset miss -> NULL");
        CHECK(motor_cfg_find(NULL) == NULL, "preset NULL name -> NULL");

        CHECK(pp->timer == NULL && pd->timer == NULL &&
              ps->timer == NULL && pe->timer == NULL, "preset timers left NULL for business layer");
        CHECK(pp->kind == MOTOR_PWM_OPEN, "preset pwm kind");
        CHECK(pd->kind == MOTOR_DC, "preset dc kind");
        CHECK(ps->kind == MOTOR_SERVO, "preset servo kind");
        CHECK(pe->kind == MOTOR_ENC_CLOSED, "preset enc kind");

        CHECK(ps->pulse_min_us == 500 && ps->pulse_max_us == 2500, "sg90 pulse range");
        CHECK(feqe(ps->angle_min, 0.0f, 0.000001f) &&
              feqe(ps->angle_max, 180.0f, 0.000001f), "sg90 angle range");
        CHECK_EQ(pe->cpr, 7200, "n20 cpr");
        CHECK(feqe(pe->vel_pid.kp, 0.015f, 0.000001f) &&
              feqe(pe->vel_pid.ki, 1.0f, 0.000001f), "n20 pid starter gains");
        CHECK(feqe(pp->duty_max, 1.0f, 0.000001f), "pwm preset duty_max");

        CHECK(strcmp(motor_cfg_presets[0].timer_preset, "pwm.motor.20k") == 0 &&
              strcmp(motor_cfg_presets[2].timer_preset, "pwm.servo.50") == 0,
              "timer preset names align with pwm_timer registry");
    }

    printf("== test_motor: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
