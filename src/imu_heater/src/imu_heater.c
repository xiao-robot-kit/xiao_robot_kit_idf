/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_heater.c
 * @brief   IMU 恒温加热控制组件实现（滞回恒温状态机，纯算法）
 *
 * @note    设计要点：
 *          1. 滞回恒温：单一阈值会振荡（PID 又不必要）——回差 hyst_c 提供
 *             「启 < target−hyst，停 > target+hyst」双阈，恒温窗内 HOLD 微调。
 *          2. 阶段减缓：WARMING 按温差查阶段表（delta 大 → 全开，逼近目标 →
 *             逐档降 duty），表 NULL 时退化为恒流（全开 max_duty）。
 *          3. 恒温微调：HOLD 档用 hold_duty（最低档），只补偿热损不冲温；
 *             温度爬过 target 即锁此档，避免过冲。
 *          4. 安全护栏：overheat_c 过热强制断输出（OVERHEAT 态），回落后自动
 *             回 IDLE；max_duty 上限一并钳制（电气/热功耗双护栏）。
 *          5. 省电（使能/失能）：disable → OFF 态恒 0 开度，heating_s 冻结；
 *             IDLE 达标也不加热。任务书「避免不必要的电量浪费」由此覆盖。
 *
 * @note    原创：全部自研。旧式实现与现有组件无 IMU 加热实现可循
 *          （旧式实现 原型无 D12 加热电阻），本案为板级新特性。
 */

#include "imu_heater/imu_heater.h"

/* ---------- 极简 strcmp（避免 string.h 依赖，纯算法组件零平台库） ---------- */
static int strcmp_impl(const char *a, const char *b)
{
    while (*a && (*a == *b)) { a++; b++; }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

/* ---------- 命名预设（恒温目标 / 阶段递减档都是经验值，集中在一个文件可调） ---------- */

/* 通用 IMU 恒温：目标 45 °C（LSM6DS3 数据手册建议工作带内），回差 2 °C，
   阶段表 5 档：温差≥10 全开 → 逼近时逐档降 Duty（降幅 = 阶段减缓），
   恒温窗 hold 12%，过热 60 °C 保护。 */
static const imu_heater_stage_t g_stage_thermo45[] = {
    { 10.0f, 1.00f },   /* 温差 ≥10°C：接近满开升温 */
    {  6.0f, 0.80f },
    {  3.0f, 0.55f },
    {  1.5f, 0.35f },
    {  0.5f, 0.20f },   /* 已很近目标：小 Duty 收尾 */
};

const imu_heater_cfg_preset_t imu_heater_presets[] = {
    {
        .name = "thermo45",
        .cfg = {
            .target_temp_c   = 45.0f,
            .hyst_c          = 2.0f,
            .stage           = g_stage_thermo45,
            .stage_n         = 5,
            .max_duty        = 1.0f,
            .hold_duty       = 0.12f,
            .overheat_c      = 60.0f,
            .sample_period_s = 1.0f,
        },
    },
    /* 低温环境专用：目标 35 °C，全开升温更快（表更宽），hold 更高（低环温热损大） */
    {
        .name = "thermo35_cold",
        .cfg = {
            .target_temp_c   = 35.0f,
            .hyst_c          = 1.5f,
            .stage           = g_stage_thermo45,
            .stage_n         = 5,
            .max_duty        = 1.0f,
            .hold_duty       = 0.20f,
            .overheat_c      = 55.0f,
            .sample_period_s = 1.0f,
        },
    },
};

const size_t imu_heater_presets_n =
    sizeof(imu_heater_presets) / sizeof(imu_heater_presets[0]);

int imu_heater_init(imu_heater_t *h, const imu_heater_cfg_t *cfg)
{
    if (h == NULL || cfg == NULL) {
        return -1;
    }
    if (cfg->target_temp_c <= 0.0f || cfg->hyst_c <= 0.0f) {
        return -2;                      /* 目标与滞回必须为正数 */
    }
    if (cfg->overheat_c <= cfg->target_temp_c + cfg->hyst_c) {
        return -3;                      /* 过热阈值必须高于正常停机带 */
    }
    h->cfg       = *cfg;
    if (h->cfg.stage != NULL && h->cfg.stage_n == 0) {
        return -4;                      /* 给了表指针却没给长度 */
    }
    h->state     = IMU_HEATER_OFF;
    h->duty      = 0.0f;
    h->heating_s = 0.0f;
    return 0;
}

int imu_heater_enable(imu_heater_t *h)
{
    if (h == NULL) {
        return -1;
    }
    if (h->state == IMU_HEATER_OFF) {
        h->state = IMU_HEATER_IDLE;     /* 从 IDLE 起步，达标不加热 */
    }
    return 0;
}

int imu_heater_disable(imu_heater_t *h)
{
    if (h == NULL) {
        return -1;
    }
    h->state = IMU_HEATER_OFF;
    h->duty  = 0.0f;
    return 0;
}

/** 按温差查阶段表：delta ≥ 表项 delta_c 即取该项 duty；低于最小档 → hold_duty */
static float stage_duty(const imu_heater_cfg_t *cfg, float delta)
{
    if (cfg->stage == NULL || cfg->stage_n == 0) {
        return cfg->max_duty;           /* 恒流档 */
    }
    for (size_t i = 0; i < cfg->stage_n; i++) {
        if (delta >= cfg->stage[i].delta_c) {
            float d = cfg->stage[i].duty;
            return (d > cfg->max_duty) ? cfg->max_duty : d;
        }
    }
    float d = cfg->hold_duty;
    return (d > cfg->max_duty) ? cfg->max_duty : d;
}

float imu_heater_update(imu_heater_t *h, float temp_c)
{
    if (h == NULL) {
        return 0.0f;
    }
    if (h->state == IMU_HEATER_OFF) {
        h->duty = 0.0f;             /* 失能恒 0 */
        return 0.0f;
    }

    imu_heater_cfg_t *cfg = &h->cfg;

    /* 过热保护优先：任何态温度撞上限即断 */
    if (temp_c >= cfg->overheat_c) {
        h->state = IMU_HEATER_OVERHEAT;
        h->duty  = 0.0f;
        return 0.0f;
    }
    /* OVERHEAT 复位条件：跌回安全带上沿（target+hyst）之下，且当周期仍断 */
    if (h->state == IMU_HEATER_OVERHEAT) {
        if (temp_c < cfg->target_temp_c + cfg->hyst_c) {
            h->state = IMU_HEATER_IDLE;
        }
        h->duty = 0.0f;
        return 0.0f;
    }

    switch (h->state) {
    case IMU_HEATER_IDLE:
        if (temp_c <= cfg->target_temp_c - cfg->hyst_c) {
            h->state = IMU_HEATER_WARMING;
            /* 迁移当周期即出开度（不空转一拍） */
            const float delta = cfg->target_temp_c - temp_c;
            h->duty = stage_duty(cfg, delta);
        } else {
            h->duty = 0.0f;         /* 达标省电 */
        }
        break;

    case IMU_HEATER_WARMING:
        if (temp_c >= cfg->target_temp_c) {
            h->state = IMU_HEATER_HOLD; /* 爬过目标 → 恒温微调 */
            h->duty  = cfg->hold_duty;
        } else {
            const float delta = cfg->target_temp_c - temp_c;
            h->duty = stage_duty(cfg, delta);
        }
        break;

    case IMU_HEATER_HOLD:
        if (temp_c >= cfg->target_temp_c + cfg->hyst_c) {
            h->state = IMU_HEATER_IDLE; /* 高温 → 停止省电 */
            h->duty  = 0.0f;
        } else if (temp_c <= cfg->target_temp_c - cfg->hyst_c) {
            h->state = IMU_HEATER_WARMING;  /* 掉出低温 → 重新升温（当周期即开度） */
            const float delta = cfg->target_temp_c - temp_c;
            h->duty = stage_duty(cfg, delta);
        } else {
            h->duty = cfg->hold_duty;   /* 恒温窗内微调保持 */
        }
        break;

    case IMU_HEATER_OVERHEAT:
    case IMU_HEATER_OFF:
    default:
        h->duty = 0.0f;
        break;
    }

    /* 时长累计：仅在加热档且 duty>0（失能/达标/过热不算） */
    if (h->duty > 0.0f && cfg->sample_period_s > 0.0f) {
        h->heating_s += cfg->sample_period_s;
    }
    return h->duty;
}

const imu_heater_cfg_preset_t *imu_heater_cfg_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < imu_heater_presets_n; i++) {
        if (imu_heater_presets[i].name != NULL &&
            strcmp_impl(name, imu_heater_presets[i].name) == 0) {
            return &imu_heater_presets[i];
        }
    }
    return NULL;
}