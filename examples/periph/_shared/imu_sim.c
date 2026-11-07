/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_sim.c
 * @brief   固件层 IMU 合成数据源实现（静止 / 正弦摇摆 + 零偏与温漂注入）
 *
 * @note    全自研（原创律）：模型推导见 imu_sim.h 文件头；无第三方依赖，
 *          仅 C 标准库 <string.h> <math.h>（sin/cos/isfinite）。
 * @note    确定性：同配置同拍号 → 逐位相同输出（无噪声、无隐藏状态）。
 */

#include <stddef.h>
#include <string.h>
#include <math.h>
#include "imu_sim.h"

/* ---------------------------------------------------------------- 预设表 */

const imu_sim_cfg_preset_t imu_sim_cfg_presets[] = {
    /* exp2 静止调零：注入已知零偏 + 与 cal.exp2.drift 默认系数同值的温漂
     * （故意同值——SIM 验收判据 = 温补整抵消后，0x02 帧去偏块回到真值）。
     * 自热斜坡 0.1 °C/s：1 分钟流送 ≈ +6 °C，漂移效果肉眼可验。 */
    { "sim.exp2.static", {
        .mode = IMU_SIM_MODE_STATIC,
        .att = IMU_SIM_ATT_Z_UP,
        .sway_axis = IMU_SIM_AXIS_Y,     /* STATIC 模式未用，填合法默认 */
        .sway_amp_deg = 0.0f,
        .sway_period_s = 1.0f,           /* 同上，占位合法值 */
        .sample_rate_hz = 100.0f,        /* 与 cal.exp2.bias 的 5 s 窗口匹配 */
        .temp_start_c = 25.0f,
        .temp_ramp_c_per_s = 0.1f,
        .temp_ref_c = 25.0f,
        .bias = { .gx = 1.0f, .gy = -0.5f, .gz = 2.5f,
                  .ax = 0.02f, .ay = -0.03f, .az = 0.015f },
        .gyro_drift  = { 0.05f, 0.05f, 0.05f },
        .accel_drift = { 0.0005f, 0.0005f, 0.0005f },
    } },
    /* exp2 正弦摇摆：调零鲁棒性 / 波形目检（重力绕 Y 摆 ±15°，4 s 周期） */
    { "sim.exp2.sway", {
        .mode = IMU_SIM_MODE_SWAY,
        .att = IMU_SIM_ATT_Z_UP,
        .sway_axis = IMU_SIM_AXIS_Y,
        .sway_amp_deg = 15.0f,
        .sway_period_s = 4.0f,
        .sample_rate_hz = 100.0f,
        .temp_start_c = 25.0f,
        .temp_ramp_c_per_s = 0.05f,
        .temp_ref_c = 25.0f,
        .bias = { .gx = 1.0f, .gy = -0.5f, .gz = 2.5f,
                  .ax = 0.02f, .ay = -0.03f, .az = 0.015f },
        .gyro_drift  = { 0.05f, 0.05f, 0.05f },
        .accel_drift = { 0.0005f, 0.0005f, 0.0005f },
    } },
    /* exp3 欧拉角姿态链：绕 Y 纯摆 ±20°（= pitch 摆动）、4 s 周期、
     * **零误差注入**（无零偏/无温漂/恒温）——exp3 验证的是姿态表示法
     * （欧拉 vs 四元数）而非标定鲁棒性（那是 exp2 的靶），
     * 验收判据 = mahony 欧拉输出 pitch 峰值与摆幅一致（±10%）。 */
    { "sim.exp3.sway", {
        .mode = IMU_SIM_MODE_SWAY,
        .att = IMU_SIM_ATT_Z_UP,
        .sway_axis = IMU_SIM_AXIS_Y,
        .sway_amp_deg = 20.0f,
        .sway_period_s = 4.0f,
        .sample_rate_hz = 100.0f,
        .temp_start_c = 25.0f,
        .temp_ramp_c_per_s = 0.0f,
        .temp_ref_c = 25.0f,
        .bias = { .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
                  .ax = 0.0f, .ay = 0.0f, .az = 0.0f },
        .gyro_drift  = { 0.0f, 0.0f, 0.0f },
        .accel_drift = { 0.0f, 0.0f, 0.0f },
    } },
};

const int imu_sim_cfg_preset_count =
    (int)(sizeof(imu_sim_cfg_presets) / sizeof(imu_sim_cfg_presets[0]));

const imu_sim_cfg_t *imu_sim_cfg_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < imu_sim_cfg_preset_count; i++) {
        if (strcmp(name, imu_sim_cfg_presets[i].name) == 0)
            return &imu_sim_cfg_presets[i].cfg;
    }
    return NULL;
}

/* ---------------------------------------------------------------- 内部模型 */

/** 姿态重力单位向量（静止真值，g；模长恒 1） */
static void sim_att_gravity(imu_sim_att_t att, float v[3])
{
    switch (att) {
    case IMU_SIM_ATT_Z_UP:    v[0] = 0.0f; v[1] = 0.0f; v[2] =  1.0f; break;
    case IMU_SIM_ATT_Z_DOWN:  v[0] = 0.0f; v[1] = 0.0f; v[2] = -1.0f; break;
    case IMU_SIM_ATT_X_UP:    v[0] =  1.0f; v[1] = 0.0f; v[2] = 0.0f; break;
    case IMU_SIM_ATT_X_DOWN:  v[0] = -1.0f; v[1] = 0.0f; v[2] = 0.0f; break;
    case IMU_SIM_ATT_Y_UP:    v[0] = 0.0f; v[1] =  1.0f; v[2] = 0.0f; break;
    case IMU_SIM_ATT_Y_DOWN:  v[0] = 0.0f; v[1] = -1.0f; v[2] = 0.0f; break;
    default:                  v[0] = 0.0f; v[1] = 0.0f; v[2] = 0.0f; break;
    }
}

/**
 * 地球系向量 → 体轴系（体绕轴 +rad 转动后的体上读数，被动旋转）。
 * 与陀螺 +dps 同号自洽：体绕 +Y 转 θ，重力读数向 −X 侧倾。
 */
static void sim_rotate_passive(float v[3], imu_sim_axis_t axis, float rad)
{
    const float c = cosf(rad), s = sinf(rad);
    const float x = v[0], y = v[1], z = v[2];
    switch (axis) {
    case IMU_SIM_AXIS_X:
        v[0] = x;
        v[1] =  y * c + z * s;
        v[2] = -y * s + z * c;
        break;
    case IMU_SIM_AXIS_Y:
        v[0] = x * c - z * s;
        v[1] = y;
        v[2] = x * s + z * c;
        break;
    case IMU_SIM_AXIS_Z:
        v[0] = x * c + y * s;
        v[1] = -x * s + y * c;
        v[2] = z;
        break;
    default:
        break;
    }
}

/** 拍号 → 温度，°C（自热线性斜坡；t = tick/rate 秒） */
static float sim_temp_at(const imu_sim_cfg_t *c, uint32_t tick)
{
    const double t = (double)tick / (double)c->sample_rate_hz;
    return (float)((double)c->temp_start_c +
                   (double)c->temp_ramp_c_per_s * t);
}

/** 拍号 → 无误差真值（重力投影 / 摇摆角速度；单位 g / dps） */
static void sim_truth_at(const imu_sim_cfg_t *c, uint32_t tick,
                         imu_sample_t *truth)
{
    float g[3];
    sim_att_gravity(c->att, g);
    float w[3] = { 0.0f, 0.0f, 0.0f };

    if (c->mode == IMU_SIM_MODE_SWAY) {
        const double t = (double)tick / (double)c->sample_rate_hz;
        const double phase = IMU_SIM_TWO_PI * t / (double)c->sway_period_s;
        const double th_deg = (double)c->sway_amp_deg * sin(phase);
        sim_rotate_passive(g, c->sway_axis,
                           (float)(th_deg * IMU_SIM_DEG2RAD));
        /* ω = dθ/dt（度制直接求导：deg/s，无需弧度换算） */
        w[c->sway_axis] = (float)((double)c->sway_amp_deg *
                                  (IMU_SIM_TWO_PI / (double)c->sway_period_s) *
                                  cos(phase));
    }

    truth->ax = g[0]; truth->ay = g[1]; truth->az = g[2];
    truth->gx = w[0]; truth->gy = w[1]; truth->gz = w[2];
}

/* ---------------------------------------------------------------- 接口 */

int imu_sim_init(imu_sim_t *sim, const imu_sim_cfg_t *cfg)
{
    if (sim == NULL || cfg == NULL) return -1;

    /* 枚举界检查（C 枚举不保证传值合法，显式挡） */
    if (cfg->mode != IMU_SIM_MODE_STATIC && cfg->mode != IMU_SIM_MODE_SWAY)
        return -1;
    if (cfg->att < IMU_SIM_ATT_Z_UP || cfg->att > IMU_SIM_ATT_Y_DOWN)
        return -1;
    if (cfg->sway_axis < IMU_SIM_AXIS_X || cfg->sway_axis > IMU_SIM_AXIS_Z)
        return -1;

    /* 数值域检查（NaN 经 isfinite 兼拒） */
    if (!isfinite(cfg->sample_rate_hz) || cfg->sample_rate_hz <= 0.0f)
        return -1;
    if (cfg->mode == IMU_SIM_MODE_SWAY) {
        if (!isfinite(cfg->sway_period_s) || cfg->sway_period_s <= 0.0f)
            return -1;
        if (!isfinite(cfg->sway_amp_deg) || cfg->sway_amp_deg < 0.0f)
            return -1;
    }
    if (!isfinite(cfg->temp_start_c) ||
        !isfinite(cfg->temp_ramp_c_per_s) ||
        !isfinite(cfg->temp_ref_c))
        return -1;

    if (!isfinite(cfg->bias.gx) || !isfinite(cfg->bias.gy) ||
        !isfinite(cfg->bias.gz) ||
        !isfinite(cfg->bias.ax) || !isfinite(cfg->bias.ay) ||
        !isfinite(cfg->bias.az))
        return -1;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(cfg->gyro_drift[i])) return -1;
        if (!isfinite(cfg->accel_drift[i])) return -1;
    }

    sim->cfg = *cfg;
    sim->inited = 1;
    sim->tick = 0;
    return 0;
}

int imu_sim_read(imu_sim_t *sim, imu_sample_t *out)
{
    if (sim == NULL || out == NULL) return -1;
    if (!sim->inited) return -1;

    const imu_sim_cfg_t *c = &sim->cfg;

    /* 真值（重力投影 / 摇摆角速度） */
    sim_truth_at(c, sim->tick, out);

    /* 注入误差：bias + drift · (T − temp_ref)（六轴同形） */
    const float dt = sim_temp_at(c, sim->tick) - c->temp_ref_c;
    out->gx += c->bias.gx + c->gyro_drift[0] * dt;
    out->gy += c->bias.gy + c->gyro_drift[1] * dt;
    out->gz += c->bias.gz + c->gyro_drift[2] * dt;
    out->ax += c->bias.ax + c->accel_drift[0] * dt;
    out->ay += c->bias.ay + c->accel_drift[1] * dt;
    out->az += c->bias.az + c->accel_drift[2] * dt;

    sim->tick++;   /* 本拍计算完毕才推进（先 temp 后 read 即同拍配对） */
    return 0;
}

int imu_sim_read_temp(imu_sim_t *sim, float *celsius)
{
    if (sim == NULL || celsius == NULL) return -1;
    if (!sim->inited) return -1;

    *celsius = sim_temp_at(&sim->cfg, sim->tick);   /* 不推进拍号 */
    return 0;
}
