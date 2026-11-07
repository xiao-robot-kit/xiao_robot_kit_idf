/* SPDX-License-Identifier: MIT */
/**
 * @file    battery.c
 * @brief   电池电压 / SOC 计算组件实现（纯算法，毫米精度换算）
 *
 * @note    设计要点：
 *          1. 分压换算：VBAT = V_ADC × (R_hi + R_lo) / R_lo。
 *             ESP32-S3 ADC 12bit：V_ADC = raw/4095 × vref。
 *          2. SOC 折线：v_full→100%、v_cutoff→0% 两个硬锚（用户需求：
 *             「容量不表示能全部用掉」——低电压处设备已停但仍有残余电荷，
 *             所以 SOC 下限锚在截止电压而非 0V）；两锚之间多点线性插值。
 *          3. 超界钳制：电压 ≥ v_full → 100%；≤ v_cutoff → 0%（不欠压误报）。
 *          4. 无状态（除缓存）：换算纯函数，可多线程/中断安全调用。
 *
 * @note    原创：全部自研。预研仓与现有组件无电池换算实现可循
 *          （MagicRobot 原型无电池 ADC 引脚），本案为板级新特性。
 */

#include "battery.h"

/* ---------- 极简 strcmp（避免 string.h 依赖，纯算法组件零平台库） ---------- */
static int strcmp_impl(const char *a, const char *b)
{
    while (*a && (*a == *b)) { a++; b++; }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

/* ---------- 命名预设 ---------- */

/* 默认 5 点典型锂聚合物 / 18650 OCV 曲线（电压升序：3.00V/0% … 4.20V/100%） */
static const battery_curve_pt_t g_curve_liion_1s[] = {
    { 3.00f,   0.0f },
    { 3.50f,  20.0f },
    { 3.70f,  50.0f },
    { 3.90f,  80.0f },
    { 4.20f, 100.0f },
};

const battery_cfg_preset_t battery_presets[] = {
    {
        .name = "xiao_1s_1400mah",   /* XIAO 板典型 1S 锂电（板上 100K:30K 分压） */
        .cfg = {
            .r_high_kohm   = 100.0f,
            .r_low_kohm    = 30.0f,
            .v_full        = 4.20f,
            .v_cutoff      = 3.00f,
            .capacity_mah  = 1400.0f,
            .curve         = g_curve_liion_1s,
            .curve_n       = 5,
            .v_alarm       = 3.30f,
        },
    },
    {
        .name = "xiao_1s_850mah",    /* 后配 850mAh 电池：容量可调演示 */
        .cfg = {
            .r_high_kohm   = 100.0f,
            .r_low_kohm    = 30.0f,
            .v_full        = 4.20f,
            .v_cutoff      = 3.00f,
            .capacity_mah  = 850.0f,
            .curve         = g_curve_liion_1s,
            .curve_n       = 5,
            .v_alarm       = 3.30f,
        },
    },
};

const size_t battery_presets_n =
    sizeof(battery_presets) / sizeof(battery_presets[0]);

static int cfg_check(const battery_cfg_t *cfg)
{
    if (cfg == NULL) {
        return -1;
    }
    if (cfg->r_high_kohm <= 0.0f || cfg->r_low_kohm <= 0.0f) {
        return -2;                     /* 分压电阻必须为正 */
    }
    if (cfg->v_full <= cfg->v_cutoff || cfg->v_cutoff <= 0.0f) {
        return -3;                     /* 满电必须高于截止、截止必须为正 */
    }
    if (cfg->capacity_mah <= 0.0f) {
        return -4;                     /* 容量必须为正（可调参数） */
    }
    if (cfg->curve != NULL && cfg->curve_n == 0) {
        return -5;                     /* 给了曲线指针却没给长度 */
    }
    return 0;
}

int battery_init(battery_t *b, const battery_cfg_t *cfg)
{
    int rc = cfg_check(cfg);
    if (rc != 0) {
        return rc;
    }
    if (b == NULL) {
        return -1;
    }
    b->cfg          = *cfg;
    b->divider_ratio = (cfg->r_high_kohm + cfg->r_low_kohm) / cfg->r_low_kohm;
    b->v_last   = 0.0f;
    b->soc_last = 0.0f;
    b->mah_last = 0.0f;
    return 0;
}

float battery_voltage_from_adc(const battery_t *b, uint32_t adc_raw,
                               float adc_vref_v, uint8_t adc_bits)
{
    if (b == NULL || adc_bits == 0 || adc_bits > 16 || adc_vref_v <= 0.0f) {
        return 0.0f;
    }
    const uint32_t full = (1u << adc_bits) - 1u;
    if (full == 0) {
        return 0.0f;
    }
    const float v_adc = (float)adc_raw * adc_vref_v / (float)full;
    return v_adc * b->divider_ratio;
}

float battery_soc_from_volt(const battery_t *b, float v_bat)
{
    if (b == NULL) {
        return 0.0f;
    }
    const battery_cfg_t *cfg = &b->cfg;

    /* 硬锚：满电以上 = 100%，截止以下（残余电荷区） = 0% */
    if (v_bat >= cfg->v_full) {
        return 100.0f;
    }
    if (v_bat <= cfg->v_cutoff) {
        return 0.0f;
    }

    if (cfg->curve != NULL && cfg->curve_n >= 2) {
        /* 折线插值（表须电压单调升序，查区间） */
        if (cfg->curve[0].volt_c >= v_bat) {
            return cfg->curve[0].soc_pct;
        }
        for (size_t i = 0; i + 1 < cfg->curve_n; i++) {
            const float v0 = cfg->curve[i].volt_c;
            const float v1 = cfg->curve[i + 1].volt_c;
            if (v0 <= v_bat && v_bat <= v1) {
                if (v1 <= v0) {
                    return cfg->curve[i].soc_pct;   /* 防除零 */
                }
                const float t = (v_bat - v0) / (v1 - v0);
                return cfg->curve[i].soc_pct +
                       t * (cfg->curve[i + 1].soc_pct - cfg->curve[i].soc_pct);
            }
        }
        return cfg->curve[cfg->curve_n - 1].soc_pct;
    }

    /* 无表：线性近似（满电→截止直线） */
    const float t = (v_bat - cfg->v_cutoff) / (cfg->v_full - cfg->v_cutoff);
    return t * 100.0f;
}

float battery_mah_from_soc(const battery_t *b, float soc_pct)
{
    if (b == NULL) {
        return 0.0f;
    }
    if (soc_pct < 0.0f) {
        soc_pct = 0.0f;
    } else if (soc_pct > 100.0f) {
        soc_pct = 100.0f;
    }
    return b->cfg.capacity_mah * soc_pct / 100.0f;
}

int battery_update_volt(battery_t *b, float v_bat)
{
    if (b == NULL || v_bat < 0.0f) {
        return -1;
    }
    b->v_last   = v_bat;
    b->soc_last = battery_soc_from_volt(b, v_bat);
    b->mah_last = battery_mah_from_soc(b, b->soc_last);
    return 0;
}

int battery_update_adc(battery_t *b, uint32_t adc_raw, float adc_vref_v, uint8_t adc_bits)
{
    if (b == NULL) {
        return -1;
    }
    const float v = battery_voltage_from_adc(b, adc_raw, adc_vref_v, adc_bits);
    if (v <= 0.0f) {
        return -2;
    }
    return battery_update_volt(b, v);
}

int battery_is_alarm(const battery_t *b)
{
    if (b == NULL || b->cfg.v_alarm <= 0.0f) {
        return 0;
    }
    return b->v_last <= b->cfg.v_alarm ? 1 : 0;
}

const battery_cfg_preset_t *battery_cfg_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < battery_presets_n; i++) {
        if (battery_presets[i].name != NULL &&
            strcmp_impl(name, battery_presets[i].name) == 0) {
            return &battery_presets[i];
        }
    }
    return NULL;
}