/* SPDX-License-Identifier: MIT */
/**
 * @file    traj_scen.c
 * @brief   三段轨迹标记场景实现（模型与段索引单源：固件/host 自检共用）
 *
 * @note    段索引用整数拍数运算（tick / ticks_per_seg）：无浮点边界毛刺，
 *          「三段各占循环 1/3 拍数」由构造保证（每段恰 ticks_per_seg 拍），
 *          testing/exp5_sim_main.c 以逐拍计数独立复核。
 */

#include <stdio.h>
#include <string.h>
#include "traj_scen.h"

/* 预设命名空间：traj.<实验>.<场景>；追加预设只改本表 */
const traj_scen_cfg_preset_t traj_scen_cfg_presets[] = {
    /* 实验五默认：200Hz × 三段各 1.0s（循环 3.0s = 600 拍，段 200 拍整），
       a = 0.5 m/s² → 段末速度 0.5 m/s、每循环位移 1.0 m（教学量级，
       比力 |ax| ≈ 0.051g——mahony 姿态被线加速度牵偏 ≤ atan(0.051)≈2.9°，
       属真实链路固有误差形态，Host 重建器按静止检测/ZUPT 处理） */
    { "traj.exp5.seg3", {
        .sample_rate_hz = 200.0f,
        .seg_len_s      = 1.0f,
        .accel_mps2     = 0.5f,
    }},
};

const int traj_scen_cfg_preset_count =
    sizeof(traj_scen_cfg_presets) / sizeof(traj_scen_cfg_presets[0]);

const traj_scen_cfg_t *traj_scen_cfg_find(const char *name)
{
    for (int i = 0; i < traj_scen_cfg_preset_count; i++)
        if (name && strcmp(traj_scen_cfg_presets[i].name, name) == 0)
            return &traj_scen_cfg_presets[i].cfg;
    return 0;
}

static int finitef(float v)
{
    /* C99 无 isfinite 宏保证时自实现：NaN 与 ±Inf 都过不了这两关 */
    return v == v && v > -3.4e38f && v < 3.4e38f;
}

int traj_scen_init(traj_scen_t *sc, const traj_scen_cfg_t *cfg)
{
    if (!sc || !cfg) return -1;
    if (!finitef(cfg->sample_rate_hz) || cfg->sample_rate_hz <= 0.0f) return -1;
    if (!finitef(cfg->seg_len_s)      || cfg->seg_len_s      <= 0.0f) return -1;
    if (!finitef(cfg->accel_mps2)     || cfg->accel_mps2     <= 0.0f) return -1;

    const uint32_t tps = (uint32_t)(cfg->sample_rate_hz * cfg->seg_len_s + 0.5f);
    if (tps < 1u) return -1;   /* 段不足 1 拍：段索引无意义 */

    sc->cfg           = *cfg;
    sc->ticks_per_seg = tps;
    sc->tick          = 0;
    sc->inited        = 1;
    return 0;
}

int traj_scen_read(traj_scen_t *sc, imu_sample_t *out,
                   traj_seg_t *seg_out, uint32_t *cycle_out)
{
    if (!sc || !out || !sc->inited) return -1;

    const uint32_t gidx  = sc->tick / sc->ticks_per_seg;      /* 全局段号 */
    const unsigned seg   = (unsigned)(gidx % TRAJ_SEG_COUNT);
    const uint32_t cycle = gidx / TRAJ_SEG_COUNT;

    /* 线加速度（m/s²，世界系 +X）：加速 +a / 匀速 0 / 减速 −a */
    const float a_world =
        (seg == TRAJ_SEG_ACCEL)  ?  sc->cfg.accel_mps2 :
        (seg == TRAJ_SEG_CRUISE) ?  0.0f :
                                    -sc->cfg.accel_mps2;

    /* 水平姿态机体比力（g）：f_b = (a_world/g, 0, +1)；无旋转 → 角速度全零 */
    out->ax = a_world / TRAJ_SCEN_GRAVITY_MPS2;
    out->ay = 0.0f;
    out->az = 1.0f;
    out->gx = 0.0f;
    out->gy = 0.0f;
    out->gz = 0.0f;

    if (seg_out)   *seg_out   = (traj_seg_t)seg;
    if (cycle_out) *cycle_out = cycle;

    sc->tick++;    /* 拍号按已产出计数（uint32 饱和：200Hz 下约 248 天） */
    return 0;
}

int traj_scen_marker_line(char *buf, size_t cap, uint32_t cycle, unsigned seg)
{
    static const char *names[TRAJ_SEG_COUNT] = { "ACCEL", "CRUISE", "DECEL" };
    if (!buf || cap == 0 || seg >= TRAJ_SEG_COUNT) return 0;
    const int n = snprintf(buf, cap, "SEG %lu %u %s",
                           (unsigned long)cycle, seg, names[seg]);
    return (n > 0 && (size_t)n < cap) ? n : 0;
}
