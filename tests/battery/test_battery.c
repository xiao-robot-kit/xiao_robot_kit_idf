/* SPDX-License-Identifier: MIT */
/**
 * @file    test_battery.c
 * @brief   电池换算组件无硬件单测（分压 / SOC 折线 / 毫安时 / 告警）
 *
 * @note    自研极简断言（原创律：不引第三方测试框架）。
 * @note    覆盖：
 *          a) 分压换算：板上 100K:30K（ratio=4.3333），ADC 全量程/半量程回算；
 *          b) SOC 硬锚：≥v_full→100%、≤v_cutoff→0%（残余电荷区不误报）；
 *          c) 折线插值：中间电压取曲线两点线性插值（3.80V→65%）；
 *          d) 剩余毫安时：容量参数可调（850/1400 两预设），SOC→mAh 线性；
 *          e) 告警阈值：≤v_alarm 告警、>v_alarm 不告警；
 *          f) 非法参数拒绝（NULL/零电阻/满电≤截止/零容量/空曲线）；
 *          g) 预设查表（容量差异验证「容量可调」需求落地）。
 * @note    不覆盖：真实 ADC 误差与电池内阻压降（需实机 P-04 板上验证）。
 */

#include <stdio.h>
#include "battery/battery.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)

#define feq(a, b) (((a) - (b) < 1e-3f) && ((b) - (a) < 1e-3f))

/* 板上分压 100K:30K → ratio 130/30 = 4.3333 */
#define RATIO (130.0f / 30.0f)

int main(void)
{
    battery_t b;
    const battery_curve_pt_t curve[] = {
        { 3.00f,   0.0f },
        { 3.50f,  20.0f },
        { 3.70f,  50.0f },
        { 3.90f,  80.0f },
        { 4.20f, 100.0f },
    };
    battery_cfg_t cfg = {
        .r_high_kohm  = 100.0f,
        .r_low_kohm   = 30.0f,
        .v_full       = 4.20f,
        .v_cutoff     = 3.00f,
        .capacity_mah = 1400.0f,
        .curve        = curve,
        .curve_n      = 5,
        .v_alarm      = 3.30f,
    };

    /* ---- f) 非法参数拒 ---- */
    CHECK(battery_init(NULL, &cfg) == -1, "init NULL 实例拒绝");
    CHECK(battery_init(&b, NULL) == -1, "init NULL cfg 拒绝");
    {
        battery_cfg_t bad = cfg;
        bad.r_low_kohm = 0.0f;
        CHECK(battery_init(&b, &bad) == -2, "零分压电阻拒绝");
        bad = cfg;
        bad.v_full = 3.00f;   /* 必须 > cutoff */
        CHECK(battery_init(&b, &bad) == -3, "满电≤截止拒绝");
        bad = cfg;
        bad.capacity_mah = 0.0f;
        CHECK(battery_init(&b, &bad) == -4, "零容量拒绝");
        bad = cfg;
        bad.curve_n = 0;
        CHECK(battery_init(&b, &bad) == -5, "曲线指针空长度拒绝");
    }

    CHECK(battery_init(&b, &cfg) == 0, "init ok");
    CHECK(feq(b.divider_ratio, RATIO), "分压比 130/30 = 4.3333");

    /* ---- a) 分压换算：ADC 12bit vref 3.3V ---- */
    /* raw=4095 → V_adc=3.3 → VBAT=3.3×4.3333=14.3（极限测试，正常不应出现） */
    CHECK(feq(battery_voltage_from_adc(&b, 4095, 3.3f, 12), 3.3f * RATIO),
          "raw 满量程 → 14.3V 换算正确");
    /* raw=2047 → V_adc≈1.65 → VBAT≈7.15 */
    CHECK(feq(battery_voltage_from_adc(&b, 2047, 3.3f, 12), 2047.0f / 4095.0f * 3.3f * RATIO),
          "raw 半量程换算正确");
    /* 真实满电场景：VBAT=4.2 → V_adc=4.2/4.3333≈0.969 → raw≈1203 */
    const uint32_t raw_full = (uint32_t)(4.2f / RATIO / 3.3f * 4095.0f + 0.5f);
    CHECK(feq(battery_voltage_from_adc(&b, raw_full, 3.3f, 12), 4.2f),
          "raw 满电反算 4.2V 正确");

    /* ---- b) SOC 硬锚 ---- */
    /* 4.2V = 满电 100%；3.0V = 0%（低于此仍有电荷但不计可用） */
    CHECK(feq(battery_soc_from_volt(&b, 4.20f), 100.0f), "4.2V → 100%");
    CHECK(feq(battery_soc_from_volt(&b, 4.40f), 100.0f), "超满电钳 100%");
    CHECK(feq(battery_soc_from_volt(&b, 3.00f), 0.0f), "3.0V → 0%");
    CHECK(feq(battery_soc_from_volt(&b, 2.80f), 0.0f), "低于截止钳 0%（残余电荷）");

    /* ---- c) 折线插值：3.80V 落在 3.90/80% 与 3.70/50% 之间 → 65% ---- */
    CHECK(feq(battery_soc_from_volt(&b, 3.80f), 65.0f), "3.80V 折线插值 65%");
    /* 3.60V → 3.70/50% 与 3.50/20% 之间 → 35% */
    CHECK(feq(battery_soc_from_volt(&b, 3.60f), 35.0f), "3.60V 折线插值 35%");
    /* 表边界：≥首点电压取首点 100% */
    CHECK(feq(battery_soc_from_volt(&b, 4.30f), 100.0f), "表外上限取满电");

    /* ---- d) 剩余毫安时 + 容量可调 ---- */
    /* 1400mAh @ 65% → 910mAh */
    CHECK(feq(battery_mah_from_soc(&b, 65.0f), 910.0f), "1400mAh×65% = 910mAh");
    CHECK(feq(battery_mah_from_soc(&b, 100.0f), 1400.0f), "满电 = 容量 1400mAh");
    CHECK(feq(battery_mah_from_soc(&b, 0.0f), 0.0f), "0% = 0mAh");

    /* 整链：update_volt 缓存三个量 */
    CHECK(battery_update_volt(&b, 3.80f) == 0, "update_volt ok");
    CHECK(feq(b.v_last, 3.80f), "v_last 缓存 3.80V");
    CHECK(feq(b.soc_last, 65.0f), "soc_last 65%");
    CHECK(feq(b.mah_last, 910.0f), "mah_last 910mAh");

    /* update_adc：raw=2047 → VBAT≈7.15V（0.7V 电池？上下界钳制后逻辑自洽） */
    CHECK(battery_update_adc(&b, 2047, 3.3f, 12) == 0, "update_adc ok");

    /* ---- e) 告警 ---- */
    battery_t b2;
    battery_cfg_t cfg2 = cfg;
    cfg2.v_alarm = 3.30f;
    battery_init(&b2, &cfg2);
    battery_update_volt(&b2, 3.40f);
    CHECK(battery_is_alarm(&b2) == 0, "3.40V 不告警");
    battery_update_volt(&b2, 3.30f);
    CHECK(battery_is_alarm(&b2) == 1, "3.30V 告警（≤阈值）");
    battery_update_volt(&b2, 3.10f);
    CHECK(battery_is_alarm(&b2) == 1, "3.10V 告警");
    /* 关闭告警：v_alarm=0 → 恒 0 */
    cfg2.v_alarm = 0.0f;
    battery_init(&b2, &cfg2);
    battery_update_volt(&b2, 3.10f);
    CHECK(battery_is_alarm(&b2) == 0, "v_alarm=0 关闭告警");

    /* ---- g) 预设查表：容量可调落地验证 ---- */
    const battery_cfg_preset_t *p = battery_cfg_find("xiao_1s_1400mah");
    CHECK(p != NULL && feq(p->cfg.capacity_mah, 1400.0f), "预设 1400mAh 存在");
    p = battery_cfg_find("xiao_1s_850mah");
    CHECK(p != NULL && feq(p->cfg.capacity_mah, 850.0f), "预设 850mAh 存在（容量可调）");
    CHECK(battery_cfg_find("no_such") == NULL, "未知预设返回 NULL");
    CHECK(battery_cfg_find(NULL) == NULL, "NULL 名字返回 NULL");

    /* 850mAh 电池满电剩余 = 850mAh（用预设初始化验证容量可调生效） */
    {
        battery_t b3;
        const battery_cfg_preset_t *p3 = battery_cfg_find("xiao_1s_850mah");
        battery_init(&b3, &p3->cfg);
        battery_update_volt(&b3, 4.20f);
        CHECK(feq(b3.mah_last, 850.0f), "850mAh 电池满电剩余 850mAh");
    }

    printf("== test_battery: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}