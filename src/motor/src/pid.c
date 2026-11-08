/* SPDX-License-Identifier: MIT */
/**
 * @file    pid.c
 * @brief   通用 PID 实现（back-calculation 抗饱和）
 *
 * 计算结构提炼自旧式实现 旧式实现 host/backend/control/speed_control.py 的
 * PIController.step（:134-150），按审查报告以【代码】为准重组；该文件 :13 docstring
 * 的抗饱和符号表述与代码相反，本处全部注释为按代码语义重写的正确方向，未照搬。
 */
#include "mtr/pid.h"

#include <math.h>     /* isfinite（编译期内建宏，无库调用） */
#include <string.h>   /* strcmp */

int pid_init(pid_t *pid, const pid_cfg_t *cfg)
{
    if (pid == NULL || cfg == NULL) {
        return -1;
    }
    if (!isfinite(cfg->kp) || !isfinite(cfg->ki) || !isfinite(cfg->kd) ||
        !isfinite(cfg->out_min) || !isfinite(cfg->out_max) ||
        !isfinite(cfg->kb)) {
        return -1;
    }
    if (!(cfg->out_min < cfg->out_max) || cfg->kb < 0.0f) {
        return -1;
    }

    pid->cfg = *cfg;          /* 配置深拷：此后改原 cfg 不影响本实例 */
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->have_prev = 0;
    pid->ready = 1;
    pid->out = 0.0f;
    pid->out_unsat = 0.0f;
    return 0;
}

int pid_reset(pid_t *pid)
{
    if (pid == NULL) {
        return -1;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->have_prev = 0;
    pid->out = 0.0f;
    pid->out_unsat = 0.0f;
    return 0;                 /* 配置与 ready 标志保留：实例仍可用 */
}

int pid_step(pid_t *pid, float error, float dt)
{
    float p, d, u, u_unsat;

    if (pid == NULL || pid->ready != 1 ||
        !isfinite(error) || !isfinite(dt) || dt <= 0.0f) {
        return -1;
    }

    /* P 项 */
    p = pid->cfg.kp * error;

    /* D 项：首拍无上拍误差 → 恒 0（防阶跃差分冲击） */
    d = 0.0f;
    if (pid->cfg.kd != 0.0f && pid->have_prev != 0) {
        d = pid->cfg.kd * (error - pid->prev_error) / dt;
    }

    /* 未限幅输出：积分取本拍【之前】的累积值（与旧式实现已验证代码同构） */
    u_unsat = p + pid->integral + d;

    /* 限幅：不饱和时 u == u_unsat，下方回灌项自然为 0 */
    u = u_unsat;
    if (u > pid->cfg.out_max) {
        u = pid->cfg.out_max;
    } else if (u < pid->cfg.out_min) {
        u = pid->cfg.out_min;
    }

    /* 积分推进 + back-calculation 回灌：
       正向饱和（u_unsat > out_max）⇒ (u − u_unsat) < 0 ⇒ 积分被抑制；
       负向饱和（u_unsat < out_min）⇒ (u − u_unsat) > 0 ⇒ 积分被抑制（镜像）。 */
    pid->integral += pid->cfg.ki * error * dt
                   + pid->cfg.kb * (u - u_unsat) * dt;

    pid->prev_error = error;
    pid->have_prev = 1;
    pid->out = u;
    pid->out_unsat = u_unsat;
    return 0;
}

/* ---- 预设注册表（追加只改本文件） ---- */

const pid_cfg_preset_t pid_cfg_presets[] = {
    /* 速度环 PI 底版：数值提炼自旧式实现 PIParams 默认值（speed_control.py:109-113）。
       量纲：误差 counts/s（编码器），输出 duty %（±100 = 满占空）。
       kd=0：速度环一般不需要 D（旧式实现同款理由）；kb 旧式实现默认 1（其注云常取 ki/kp 量级）。 */
    { "pid.vel_pi.default",
      { .kp = 2.0f, .ki = 20.0f, .kd = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f, .kb = 1.0f } },
    /* 无量纲底版：归一化误差/输出（仿真与单测用） */
    { "pid.pi.normalized",
      { .kp = 1.0f, .ki = 1.0f, .kd = 0.0f,
        .out_min = -1.0f, .out_max = 1.0f, .kb = 1.0f } },
};

const int pid_cfg_preset_count =
    (int)(sizeof(pid_cfg_presets) / sizeof(pid_cfg_presets[0]));

const pid_cfg_t *pid_cfg_find(const char *name)
{
    int i;

    if (name == NULL || name[0] == '\0') {
        return NULL;
    }
    for (i = 0; i < pid_cfg_preset_count; i++) {
        if (strcmp(pid_cfg_presets[i].name, name) == 0) {
            return &pid_cfg_presets[i].cfg;
        }
    }
    return NULL;
}
