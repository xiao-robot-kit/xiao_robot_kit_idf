/* SPDX-License-Identifier: MIT */
/**
 * @file    test_imu_heater.c
 * @brief   IMU 恒温加热组件无硬件单测（纯算法状态机路径全覆盖）
 *
 * @note    自研极简断言（原创律：不引第三方测试框架）。
 * @note    覆盖：
 *          a) 失能恒 0（省电：OFF 态 duty=0、heating_s 冻结）；
 *          b) 使能后低温触发（temp ≤ target−hyst → WARMING 升温）；
 *          c) 阶段减缓（温差大 → 高档开度；逼近目标 → 逐档降）；
 *          d) 恒温微调（爬过 target → HOLD 锁 hold_duty；窗外回差与重升温）；
 *          e) 达标停机（temp ≥ target+hyst → IDLE 零开度省电）；
 *          f) 过热保护（temp ≥ overheat_c → OVERHEAT 断输出，回落自动复原）；
 *          g) heating_s 只累计加热档（WARMING/HOLD duty>0）；
 *          h) 非法参数拒绝（NULL/负滞回/过热阈值越界/空表）与预设查表。
 * @note    不覆盖：真实热惯性与温控品质（需实机 P-10 板上验证）。
 */

#include <stdio.h>
#include "imu_heater/imu_heater.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)

#define feq(a, b) (((a) - (b) < 1e-4f) && ((b) - (a) < 1e-4f))

int main(void)
{
    /* ---- d) 阶段表素材（温差降序，与预设同构） ---- */
    static const imu_heater_stage_t stg[] = {
        { 10.0f, 1.00f },
        {  6.0f, 0.80f },
        {  3.0f, 0.55f },
        {  1.5f, 0.35f },
        {  0.5f, 0.20f },
    };
    imu_heater_cfg_t cfg = {
        .target_temp_c   = 45.0f,
        .hyst_c          = 2.0f,
        .stage           = stg,
        .stage_n         = 5,
        .max_duty        = 1.0f,
        .hold_duty       = 0.12f,
        .overheat_c      = 60.0f,
        .sample_period_s = 1.0f,
    };

    imu_heater_t h;

    /* ---- NULL / 非法参数拒绝 ---- */
    CHECK(imu_heater_init(NULL, &cfg) == -1, "init NULL 实例拒绝");
    CHECK(imu_heater_init(&h, NULL) == -1, "init NULL cfg 拒绝");
    {
        imu_heater_cfg_t bad = cfg;
        bad.hyst_c = 0.0f;
        CHECK(imu_heater_init(&h, &bad) == -2, "非正滞回拒绝");
        bad = cfg;
        bad.overheat_c = 45.0f;   /* 必须 > target+hyst=47 */
        CHECK(imu_heater_init(&h, &bad) == -3, "过热阈值低于停机带拒绝");
        bad = cfg;
        bad.stage_n = 0;
        CHECK(imu_heater_init(&h, &bad) == -4, "指针表空长度拒绝");
    }

    /* ---- a) 初始化 + 失能恒 0（省电） ---- */
    CHECK(imu_heater_init(&h, &cfg) == 0, "init ok");
    CHECK(h.state == IMU_HEATER_OFF, "init 态 = OFF");
    CHECK(imu_heater_enable(&h) == 0, "enable ok");
    CHECK(h.state == IMU_HEATER_IDLE, "enable 后起步 IDLE");
    CHECK(imu_heater_update(&h, 25.0f) > 0.9f, "低温 25°C 温差 20 → 高档开度");

    /* ---- c) 阶段减缓：温差降 → duty 逐档降 ---- */
    /* 当前态：WARMING（低温已触发）。校验各温差档开度 */
    imu_heater_init(&h, &cfg);
    imu_heater_enable(&h);
    /* 温差 20（45−25）→ 命中第 0 档 1.00 */
    CHECK(feq(imu_heater_update(&h, 25.0f), 1.00f), "温差20 → duty 1.00");
    /* 温差 8（45−37）→ 命中第 1 档 0.80 */
    CHECK(feq(imu_heater_update(&h, 37.0f), 0.80f), "温差8 → duty 0.80");
    /* 温差 4（45−41）→ 第 2 档 0.55 */
    CHECK(feq(imu_heater_update(&h, 41.0f), 0.55f), "温差4 → duty 0.55");
    /* 温差 1（45−44）→ 第 4 档 0.20 */
    CHECK(feq(imu_heater_update(&h, 44.0f), 0.20f), "温差1 → duty 0.20");

    /* ---- d) 恒温微调：爬过 target → HOLD 锁 hold_duty ---- */
    CHECK(feq(imu_heater_update(&h, 46.0f), 0.12f), "46°C ≥ 45 → HOLD hold_duty 0.12");
    CHECK(h.state == IMU_HEATER_HOLD, "HOLD 态");

    /* 恒温窗内微调保持（46→47 没触停带 47.0：target+hyst=47，47 未 ≥ → 仍 HOLD） */
    CHECK(feq(imu_heater_update(&h, 46.5f), 0.12f), "46.5°C 窗内 → 保持 hold_duty");
    CHECK(feq(imu_heater_update(&h, 47.0f), 0.00f), "47°C ≥ 47 停带上沿 → 停机 IDLE");
    CHECK(h.state == IMU_HEATER_IDLE, "停机 IDLE");
    CHECK(feq(imu_heater_update(&h, 47.5f), 0.00f), "超标仍零开度（省电）");

    /* 回落低温 → 重触发（滞回下沿 43：45−2）→ WARMING 低档 */
    CHECK(imu_heater_update(&h, 43.0f) <= 0.351f, "回落到 43 重触发低档");
    CHECK(h.state == IMU_HEATER_WARMING, "重升温 WARMING");

    /* ---- f) 过热保护 ---- */
    imu_heater_init(&h, &cfg);
    imu_heater_enable(&h);
    imu_heater_update(&h, 25.0f);          /* 先加热 */
    CHECK(feq(imu_heater_update(&h, 60.0f), 0.0f), "60°C ≥ 60 → 断输出");
    CHECK(h.state == IMU_HEATER_OVERHEAT, "OVERHEAT 态");
    CHECK(feq(imu_heater_update(&h, 59.0f), 0.0f), "过热带内仍断（59 < 60）");
    CHECK(h.state == IMU_HEATER_OVERHEAT, "仍 OVERHEAT");
    /* 回落到安全带（< 47）→ 自动复原 IDLE */
    CHECK(feq(imu_heater_update(&h, 46.0f), 0.0f), "回落 46 → IDLE 零开度");
    CHECK(h.state == IMU_HEATER_IDLE, "过热回落自动复原 IDLE");

    /* ---- b) 使能/失能 + 省电（heating_s 冻结） ---- */
    imu_heater_init(&h, &cfg);
    imu_heater_enable(&h);
    imu_heater_update(&h, 25.0f);          /* 加热 1s */
    imu_heater_update(&h, 25.0f);          /* 加热 2s */
    CHECK(feq(h.heating_s, 2.0f), "两周期加热累计 2s");
    imu_heater_disable(&h);
    CHECK(h.state == IMU_HEATER_OFF, "disable → OFF");
    CHECK(feq(imu_heater_update(&h, 25.0f), 0.0f), "失能低温也 0 开度");
    CHECK(feq(h.heating_s, 2.0f), "失能 heating_s 冻结 2s");
    imu_heater_enable(&h);
    CHECK(h.state == IMU_HEATER_IDLE, "再 enable → IDLE（新起点）");
    imu_heater_update(&h, 20.0f);          /* 再加热 1s */
    CHECK(feq(h.heating_s, 3.0f), "重新加热累计到 3s");

    /* ---- g) 恒流档（stage=NULL）：全速开满 ---- */
    {
        imu_heater_cfg_t c2 = cfg;
        c2.stage = NULL; c2.stage_n = 0;
        imu_heater_t h2;
        imu_heater_init(&h2, &c2);
        imu_heater_enable(&h2);
        CHECK(feq(imu_heater_update(&h2, 30.0f), 1.0f), "恒流档低温满开");
    }

    /* ---- h) max_duty 上限钳制 ---- */
    {
        imu_heater_cfg_t c3 = cfg;
        c3.max_duty = 0.5f;
        imu_heater_t h3;
        imu_heater_init(&h3, &c3);
        imu_heater_enable(&h3);
        CHECK(feq(imu_heater_update(&h3, 25.0f), 0.5f), "max_duty 钳制 0.5");
    }

    /* ---- 预设查表 ---- */
    const imu_heater_cfg_preset_t *p = imu_heater_cfg_find("thermo45");
    CHECK(p != NULL && feq(p->cfg.target_temp_c, 45.0f), "预设 thermo45 存在且目标 45°C");
    p = imu_heater_cfg_find("thermo35_cold");
    CHECK(p != NULL && feq(p->cfg.target_temp_c, 35.0f), "预设 thermo35_cold 存在");
    CHECK(imu_heater_cfg_find("no_such") == NULL, "未知预设返回 NULL");
    CHECK(imu_heater_cfg_find(NULL) == NULL, "NULL 名字返回 NULL");

    printf("== test_imu_heater: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}