/* SPDX-License-Identifier: MIT */
/**
 * @file    test_pwm_timer.c
 * @brief   PWM 定时器实例组件无硬件单测（mock HAL：平台侧定时器资源池）
 *
 * @note    自研极简断言（原创律：不引第三方测试框架，风格对齐 test_imu.c）。
 * @note    验证范围：create 参数校验 / 注册表校验 / 通道占用与隔离 /
 *          多实例频率独立 / 预设表命中与未命中 / destroy 全语义
 *          （占用拦截、平台真实释放、释放失败可重试、销毁后句柄不可用）。
 *          不覆盖：malloc 失败注入（需分配器钩子）、真实 LEDC 时序（实机验证）。
 * @note    防雷特别项：旧式实现 LEDC timer 只增不减（无 deinit）——本测以
 *          「release 回调完成 + 池位归还可复用 + 4 位池 10 轮循环」三重证据
 *          验证 destroy 真实释放。
 * @note    销毁后句柄检查走护栏魔数（尽力防御）：正式契约与 free() 同——
 *          销毁后不得再使用句柄；主流分配器下该检查确定性走护栏分支
 *          （销毁前置 0 再 free，或被分配器元数据覆写——两者皆非活魔数）。
 */

#include <stdio.h>
#include <string.h>
#include "mtr/pwm_timer.h"
#include "mtr/pwm_hal.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)
#define CHECK_EQ(a, b, msg) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a == _b) { g_pass++; } \
    else          { g_fail++; printf("  [FAIL] %s: %ld != %ld (line %d)\n", msg, _a, _b, __LINE__); } \
} while (0)
static int feq(float a, float b) { float d = a - b; return d < 0.000001f && d > -0.000001f; }

/* ---------- mock 平台：定时器资源池（正确行为 = release 归还池位） ---------- */
#define MOCK_TIMERS_MAX    4        /* 模拟 LEDC 定时器池容量（S3 = 4） */
#define MOCK_CHANNELS_MAX  8        /* 模拟单定时器通道上限 */
#define MOCK_FREQ_MAX      1000000u /* 模拟平台载波频率上限（拒超限 = 平台能力路径） */

typedef struct {
    int      live;      /* 1 = 平台定时器存在；release 成功置 0（真实释放观测点） */
    uint32_t freq;
    uint8_t  channels;
    float    duty[MOCK_CHANNELS_MAX];
    int      enabled[MOCK_CHANNELS_MAX];
} mock_timer_t;

static mock_timer_t mock_pool[MOCK_TIMERS_MAX];
static int mock_config_calls;      /* timer_config 被调次数（含被平台拒绝的） */
static int mock_release_attempts;  /* timer_release 被调次数（含失败的） */
static int mock_release_ok;        /* timer_release 成功次数（真实释放证据） */
static int mock_last_idx = -1;     /* 最近一次 timer_config 占用的池位 */
static int mock_fail_release;      /* 注入：下次 timer_release 平台失败 */

static int mock_timer_config(uint32_t freq_hz, uint8_t channels,
                             void **out_plat, void *ctx)
{
    (void)ctx;
    mock_config_calls++;
    if (out_plat == 0) return -1;
    if (freq_hz == 0u || freq_hz > MOCK_FREQ_MAX) return -1;      /* 平台能力边界 */
    if (channels == 0u || channels > MOCK_CHANNELS_MAX) return -1;
    for (int i = 0; i < MOCK_TIMERS_MAX; i++) {
        if (mock_pool[i].live) continue;
        mock_pool[i].live     = 1;
        mock_pool[i].freq     = freq_hz;
        mock_pool[i].channels = channels;
        for (int c = 0; c < MOCK_CHANNELS_MAX; c++) {
            mock_pool[i].duty[c]    = 0.0f;   /* 平台契约：上电安全态 */
            mock_pool[i].enabled[c] = 0;
        }
        mock_last_idx = i;
        *out_plat = &mock_pool[i];
        return 0;
    }
    return -1;   /* 池满：平台资源不足 */
}

static int mock_timer_release(void *plat, void *ctx)
{
    (void)ctx;
    mock_timer_t *m = (mock_timer_t *)plat;
    mock_release_attempts++;
    if (mock_fail_release) return -1;        /* 注入的平台失败（不归还池位） */
    if (m == 0 || !m->live) return -1;       /* 无效句柄/重复释放拒绝 */
    m->live = 0;
    mock_release_ok++;
    return 0;
}

static int mock_channel_set_duty(void *plat, uint8_t channel, float duty, void *ctx)
{
    (void)ctx;
    mock_timer_t *m = (mock_timer_t *)plat;
    if (m == 0 || !m->live) return -1;       /* 已释放句柄拒绝（平台契约） */
    if (channel >= m->channels) return -1;
    m->duty[channel] = duty;
    return 0;
}

static int mock_channel_enable(void *plat, uint8_t channel, int enable, void *ctx)
{
    (void)ctx;
    mock_timer_t *m = (mock_timer_t *)plat;
    if (m == 0 || !m->live) return -1;
    if (channel >= m->channels) return -1;
    m->enabled[channel] = enable ? 1 : 0;
    return 0;
}

static const pwm_hal_t mock_hal = {
    mock_timer_config, mock_timer_release,
    mock_channel_set_duty, mock_channel_enable, 0,
};

static void mock_reset(void)
{
    memset(mock_pool, 0, sizeof mock_pool);
    mock_config_calls     = 0;
    mock_release_attempts = 0;
    mock_release_ok       = 0;
    mock_last_idx         = -1;
    mock_fail_release     = 0;
}

int main(void)
{
    /* ---- 1. 平台未注册：create 全拒（注册前窗口） ---- */
    CHECK(pwm_timer_create(20000u, 4u) == 0, "create rejected before hal registered");
    CHECK(pwm_timer_create_preset("pwm.motor.20k") == 0, "create_preset rejected before hal");

    /* ---- 2. 注册表校验：NULL / 表缺位 / 重复注册 ---- */
    CHECK_EQ(pwm_hal_register(0), -1, "register NULL rejected");
    {
        const pwm_hal_t incomplete = {
            0, mock_timer_release, mock_channel_set_duty, mock_channel_enable, 0,
        };
        CHECK_EQ(pwm_hal_register(&incomplete), -1, "register incomplete table rejected");
    }
    CHECK_EQ(pwm_hal_register(&mock_hal), 0, "register mock hal ok");
    CHECK_EQ(pwm_hal_register(&mock_hal), -1, "double register rejected");

    /* ---- 3. create 参数校验拒非法 ---- */
    mock_reset();
    CHECK(pwm_timer_create(0u, 4u) == 0, "create rejects freq=0");
    CHECK(pwm_timer_create(20000u, 0u) == 0, "create rejects channels=0");
    CHECK(pwm_timer_create(MOCK_FREQ_MAX + 1u, 4u) == 0, "create rejects freq over platform limit");
    CHECK(pwm_timer_create(20000u, MOCK_CHANNELS_MAX + 1u) == 0, "create rejects channels over platform limit");
    /* 前两项组件层拒（不触平台），后两项经平台拒——config 恰被调 2 次 */
    CHECK_EQ(mock_config_calls, 2, "only platform-side rejects reached hal");

    /* 平台池耗尽拒 + 释放后复用（真实释放第一证据） */
    pwm_timer_t *pool[MOCK_TIMERS_MAX];
    for (int i = 0; i < MOCK_TIMERS_MAX; i++)
        pool[i] = pwm_timer_create(20000u + (uint32_t)i, 4u);
    {
        int all = 1;
        for (int i = 0; i < MOCK_TIMERS_MAX; i++) if (pool[i] == 0) all = 0;
        CHECK(all, "platform pool of 4 timers fully created");
    }
    CHECK(pwm_timer_create(30000u, 2u) == 0, "create rejected when platform pool full");
    CHECK_EQ(pwm_timer_destroy(pool[2]), 0, "destroy frees a platform slot");
    {
        pwm_timer_t *again = pwm_timer_create(30000u, 2u);
        CHECK(again != 0, "create reuses released slot");
        CHECK_EQ(pwm_timer_destroy(again), 0, "cleanup reused slot");
    }
    for (int i = 0; i < MOCK_TIMERS_MAX; i++)
        if (i != 2) CHECK_EQ(pwm_timer_destroy(pool[i]), 0, "cleanup pool slot");

    /* 创建即生效：平台收到精确参数，通道全 0（上电安全态） */
    mock_reset();
    {
        pwm_timer_t *t0 = pwm_timer_create(20000u, 4u);
        CHECK(t0 != 0, "create ok");
        CHECK_EQ(mock_config_calls, 1, "platform timer_config called once");
        CHECK_EQ(mock_last_idx, 0, "platform slot 0 taken");
        CHECK_EQ((long)mock_pool[0].freq, 20000, "platform got exact freq");
        CHECK_EQ((long)mock_pool[0].channels, 4, "platform got exact channels");
        {
            int safe = 1;
            for (int c = 0; c < 4; c++) if (!feq(mock_pool[0].duty[c], 0.0f)) safe = 0;
            CHECK(safe, "power-on duties all zero");
        }
        CHECK_EQ(pwm_timer_destroy(t0), 0, "cleanup t0");
    }

    /* ---- 4. 同句柄分通道互不干扰（占用登记在实例内部） ---- */
    mock_reset();
    {
        pwm_timer_t *t = pwm_timer_create(20000u, 4u);
        int idx = mock_last_idx;
        CHECK(t != 0, "timer for channel test");
        CHECK_EQ(pwm_timer_channel_acquire(t, 0), 0, "acquire ch0");
        CHECK_EQ(pwm_timer_channel_acquire(t, 1), 0, "acquire ch1");
        CHECK_EQ(pwm_timer_channel_acquire(t, 0), -1, "double acquire ch0 rejected");
        CHECK_EQ(pwm_timer_channel_acquire(t, 4), -1, "acquire out of range rejected");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 2, 0.5f), -1, "set_duty on un-acquired channel rejected");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 0, 0.25f), 0, "set ch0 duty 0.25");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 1, 0.75f), 0, "set ch1 duty 0.75");
        CHECK(feq(mock_pool[idx].duty[0], 0.25f), "ch0 duty isolated");
        CHECK(feq(mock_pool[idx].duty[1], 0.75f), "ch1 duty isolated");
        CHECK(feq(mock_pool[idx].duty[2], 0.0f), "ch2 untouched");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 1, 0.10f), 0, "rewrite ch1 duty");
        CHECK(feq(mock_pool[idx].duty[0], 0.25f), "ch0 unaffected by ch1 rewrite");
        /* 占空越界 / NaN 拒绝且不写平台 */
        CHECK_EQ(pwm_timer_channel_set_duty(t, 0, 1.5f), -1, "duty > 1 rejected");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 0, -0.1f), -1, "duty < 0 rejected");
        {
            volatile float z = 0.0f;
            float nan_v = z / z;
            CHECK_EQ(pwm_timer_channel_set_duty(t, 0, nan_v), -1, "duty NaN rejected");
        }
        CHECK(feq(mock_pool[idx].duty[0], 0.25f), "platform duty unchanged on rejects");
        /* 使能面 */
        CHECK_EQ(pwm_timer_channel_enable(t, 0, 1), 0, "enable ch0");
        CHECK_EQ(mock_pool[idx].enabled[0], 1, "ch0 enabled on platform");
        CHECK_EQ(mock_pool[idx].enabled[1], 0, "ch1 still disabled");
        CHECK_EQ(pwm_timer_channel_enable(t, 3, 1), -1, "enable un-acquired rejected");
        /* 释放与复占 */
        CHECK_EQ(pwm_timer_channel_release(t, 3), -1, "release un-acquired rejected");
        CHECK_EQ(pwm_timer_channel_release(t, 0), 0, "release ch0");
        CHECK_EQ(pwm_timer_channel_set_duty(t, 0, 0.5f), -1, "set_duty on released channel rejected");
        CHECK_EQ(pwm_timer_channel_acquire(t, 0), 0, "re-acquire ch0 ok");
        CHECK_EQ(pwm_timer_channel_release(t, 0), 0, "re-release ch0");
        CHECK_EQ(pwm_timer_channel_release(t, 1), 0, "release ch1");
        CHECK_EQ(pwm_timer_destroy(t), 0, "destroy after all channels released");
    }

    /* ---- 5. 两实例各自频率独立（各占一个平台定时器） ---- */
    mock_reset();
    {
        pwm_timer_t *a = pwm_timer_create(20000u, 4u);
        int ia = mock_last_idx;
        pwm_timer_t *b = pwm_timer_create(50u, 6u);
        int ib = mock_last_idx;
        CHECK(a != 0 && b != 0, "two instances created");
        CHECK(ia != ib, "distinct platform timers");
        CHECK_EQ((long)pwm_timer_freq(a), 20000, "freq a = 20000 Hz");
        CHECK_EQ((long)pwm_timer_freq(b), 50, "freq b = 50 Hz");
        CHECK_EQ((long)mock_pool[ia].freq, 20000, "platform a freq");
        CHECK_EQ((long)mock_pool[ib].freq, 50, "platform b freq");
        /* 同通道号跨实例互不干扰（各自占用登记互不可见） */
        CHECK_EQ(pwm_timer_channel_acquire(a, 0), 0, "a acquires ch0");
        CHECK_EQ(pwm_timer_channel_acquire(b, 0), 0, "b acquires ch0 (same index, other timer)");
        CHECK_EQ(pwm_timer_channel_set_duty(a, 0, 0.5f), 0, "a ch0 duty");
        CHECK_EQ(pwm_timer_channel_set_duty(b, 0, 0.1f), 0, "b ch0 duty");
        CHECK(feq(mock_pool[ia].duty[0], 0.5f), "platform a duty");
        CHECK(feq(mock_pool[ib].duty[0], 0.1f), "platform b duty");
        CHECK_EQ(pwm_timer_channel_release(a, 0), 0, "a releases ch0");
        CHECK_EQ(pwm_timer_destroy(a), 0, "destroy a");
        CHECK(mock_pool[ia].live == 0, "platform a released");
        CHECK(mock_pool[ib].live == 1, "platform b alive");
        CHECK_EQ(pwm_timer_freq(a), 0, "destroyed a handle unusable (freq = 0)");
        CHECK_EQ((long)pwm_timer_freq(b), 50, "b freq still valid");
        CHECK_EQ(pwm_timer_channel_set_duty(b, 0, 0.9f), 0, "b still operable");
        CHECK_EQ(pwm_timer_channel_release(b, 0), 0, "b releases ch0");
        CHECK_EQ(pwm_timer_destroy(b), 0, "destroy b");
    }

    /* ---- 6. 预设表 find 命中 / 未命中 ---- */
    mock_reset();
    CHECK_EQ(pwm_timer_preset_count, 2, "preset count");
    CHECK(pwm_timer_presets[0].name != 0 && strcmp(pwm_timer_presets[0].name, "pwm.motor.20k") == 0, "preset[0] name");
    CHECK_EQ((long)pwm_timer_presets[0].freq_hz, 20000, "preset[0] freq");
    CHECK_EQ((long)pwm_timer_presets[0].channels, 4, "preset[0] channels");
    CHECK(pwm_timer_presets[1].name != 0 && strcmp(pwm_timer_presets[1].name, "pwm.servo.50") == 0, "preset[1] name");
    CHECK_EQ((long)pwm_timer_presets[1].freq_hz, 50, "preset[1] freq");
    {
        pwm_timer_t *pm = pwm_timer_create_preset("pwm.motor.20k");
        CHECK(pm != 0, "create_preset hit motor preset");
        CHECK_EQ((long)pwm_timer_freq(pm), 20000, "motor preset instance freq");
        CHECK_EQ((long)mock_pool[mock_last_idx].channels, 4, "motor preset instance channels");
        CHECK_EQ(pwm_timer_destroy(pm), 0, "cleanup motor preset");
    }
    {
        pwm_timer_t *ps = pwm_timer_create_preset("pwm.servo.50");
        CHECK(ps != 0, "create_preset hit servo preset");
        CHECK_EQ((long)pwm_timer_freq(ps), 50, "servo preset instance freq");
        CHECK_EQ(pwm_timer_destroy(ps), 0, "cleanup servo preset");
    }
    CHECK(pwm_timer_create_preset("no.such.preset") == 0, "preset miss -> NULL");
    CHECK(pwm_timer_create_preset(0) == 0, "preset NULL name -> NULL");

    /* ---- 7. destroy 语义：占用拦截 / 释放失败可重试 / 真实释放 / 句柄失效 ---- */
    mock_reset();
    {
        pwm_timer_t *d = pwm_timer_create(20000u, 4u);
        int di = mock_last_idx;
        CHECK(d != 0, "timer for destroy semantics");
        /* 占用拦截（契约：-1 = 仍有电机关联） */
        CHECK_EQ(pwm_timer_channel_acquire(d, 1), 0, "acquire to block destroy");
        CHECK_EQ(pwm_timer_destroy(d), -1, "destroy blocked while channel occupied");
        CHECK_EQ(mock_release_attempts, 0, "no platform release while occupied");
        CHECK_EQ((long)pwm_timer_freq(d), 20000, "instance alive after blocked destroy");
        CHECK_EQ(pwm_timer_channel_set_duty(d, 1, 0.3f), 0, "instance still operable");
        CHECK_EQ(pwm_timer_channel_release(d, 1), 0, "release to unblock");
        /* 平台释放失败：destroy -1，实例保留可重试 */
        mock_fail_release = 1;
        CHECK_EQ(pwm_timer_destroy(d), -1, "destroy fails when platform release fails");
        CHECK_EQ(mock_release_attempts, 1, "platform release attempted");
        CHECK_EQ(mock_release_ok, 0, "platform release not completed");
        CHECK_EQ((long)pwm_timer_freq(d), 20000, "instance kept alive on release failure");
        mock_fail_release = 0;
        /* 重试成功：release 回调被调（防旧式实现「LEDC timer 只增不减」雷） */
        CHECK_EQ(pwm_timer_destroy(d), 0, "retry destroy after platform recovered");
        CHECK_EQ(mock_release_attempts, 2, "release attempted twice in total");
        CHECK_EQ(mock_release_ok, 1, "release callback completed");
        CHECK(mock_pool[di].live == 0, "platform timer really released");
        /* destroy 后句柄不可用（护栏尽力拒绝；契约 = 销毁后勿用，同 free） */
        CHECK_EQ(pwm_timer_freq(d), 0, "freq(dead handle) = 0");
        CHECK_EQ(pwm_timer_destroy(d), -1, "double destroy rejected");
        CHECK_EQ(pwm_timer_channel_acquire(d, 0), -1, "acquire on dead handle rejected");
        CHECK_EQ(pwm_timer_channel_set_duty(d, 0, 0.5f), -1, "set_duty on dead handle rejected");
        CHECK_EQ(pwm_timer_channel_enable(d, 0, 1), -1, "enable on dead handle rejected");
        CHECK_EQ(mock_release_attempts, 2, "no extra platform release from dead-handle ops");
    }
    /* 真实释放循环证据：池容量 4，create/destroy 10 轮全成功
       （旧式实现「只增不减」行为下第 5 轮即资源耗尽失败） */
    mock_reset();
    {
        int cycle_ok = 1;
        for (int i = 0; i < 10; i++) {
            pwm_timer_t *c = pwm_timer_create(20000u, 2u);
            if (c == 0 || pwm_timer_destroy(c) != 0) { cycle_ok = 0; break; }
        }
        CHECK(cycle_ok, "10x create/destroy cycles on 4-slot platform pool");
        CHECK_EQ(mock_release_ok, 10, "10 successful platform releases");
    }

    /* ---- 8. NULL 参数面 ---- */
    CHECK_EQ(pwm_timer_freq(0), 0, "freq(NULL) = 0");
    CHECK_EQ(pwm_timer_destroy(0), -1, "destroy(NULL) rejected");
    CHECK_EQ(pwm_timer_channel_acquire(0, 0), -1, "acquire(NULL) rejected");
    CHECK_EQ(pwm_timer_channel_release(0, 0), -1, "release(NULL) rejected");
    CHECK_EQ(pwm_timer_channel_set_duty(0, 0, 0.5f), -1, "set_duty(NULL) rejected");
    CHECK_EQ(pwm_timer_channel_enable(0, 0, 1), -1, "enable(NULL) rejected");

    printf("== test_pwm_timer: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
