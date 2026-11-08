/* SPDX-License-Identifier: MIT */
/**
 * @file    motor_scen_table.c
 * @brief   电机实验场景表数据（契约见 motor_scen_table.h；改实验只改本文件）
 *
 * @note    四专项（T-112 任务卡口径，步值即验收断言的期望值源）+ DC 收满：
 *          1) pwm    占空阶梯 0/25/50/75/100%，每档 2s（1000 拍）；
 *          2) servo  角度五点 0/45/90/135/180°（升后降，每点 2s）；
 *          3) enc    闭环阶跃 30→90→30→90 rpm（每段 4s，两升沿可统计）；
 *          4) nested 同一定时器双电机（20kHz ch0/ch1）+ 独立定时器一电机
 *                    （50Hz 舵机）——A 阶梯步进、B 恒 0.5、C 慢角度摆动；
 *          5) dc     H 桥正反转切换：+50% 持续 2s → −50% 持续 2s 循环——
 *                    验收断言：方向位翻转（signed duty 符号随步翻转，正/反
 *                    段各 ≥2 次循环）+ duty 对称（正反段幅值均 0.5）+
 *                    逐拍跟随（开环直通，桥臂反对称 = 直通零容忍的间接判据）。
 * @note    hold_ticks 全部以 MOTOR_DEMO_POLL_US=2ms 计（时基单源）；
 *          plant 参数沿用演示表 slot2 取值（160 rpm/duty、τ=20ms——
 *          motor_demo_table.c 注释里的整定依据在此同源复用）。
 * @note    场景 5 独立运行（start 全量重置语义）——20kHz 定时器无 demo 争用，
 *          DC 走库建议载波（motor.dc.generic 底版的 timer_preset = pwm.motor.20k，
 *          桥对 ch0/1）；demo 第 4 槽的 50Hz override 是演示共存约束下的 wiring
 *          选择（见 motor_demo_table.c 表头注），与本场景互不相干。
 */

#include "motor_scen_table.h"
#include "motor_demo_table.h"   /* MOTOR_DEMO_POLL_US（拍周期单源） */

/* ── 场景 1：PWM 占空阶梯扫描 ── */
static const char *const scen1_timers[] = { "pwm.motor.20k" };
static const float scen1_steps[] = { 0.00f, 0.25f, 0.50f, 0.75f, 1.00f };
static const motor_scen_slot_t scen1_slots[] = {
    { "motor.pwm.generic", 0, 4, -1, -1, -1,
      0.0f, 0.0f,
      scen1_steps, 5, 1000 },   /* 5 档 × 2s = 10s 一循环 */
};

/* ── 场景 2：SG90 角度五点标定（升后降：0..180..0，每点双窗可重复测量） ── */
static const char *const scen2_timers[] = { "pwm.servo.50" };
static const float scen2_steps[] = { 0.0f, 45.0f, 90.0f, 135.0f, 180.0f,
                                     135.0f, 90.0f, 45.0f };
static const motor_scen_slot_t scen2_slots[] = {
    { "motor.servo.sg90", 0, 5, -1, -1, -1,
      0.0f, 0.0f,
      scen2_steps, 8, 1000 },   /* 8 点 × 2s = 16s 一循环 */
};

/* ── 场景 3：ENC 闭环阶跃调参（30→90 两升沿/循环；每段 4s 留足收敛+统计窗） ── */
static const char *const scen3_timers[] = { "pwm.motor.20k" };
static const float scen3_steps[] = { 30.0f, 90.0f, 30.0f, 90.0f };
static const motor_scen_slot_t scen3_slots[] = {
    { "motor.enc.n20", 0, 6, 7, 8, 9,
      160.0f, 0.02f,
      scen3_steps, 4, 2000 },   /* 4 段 × 4s = 16s 一循环 */
};

/* ── 场景 4：嵌套定时器（同定时器双电机 + 独立定时器一电机） ── */
static const char *const scen4_timers[] = { "pwm.motor.20k", "pwm.servo.50" };
static const float scen4a_steps[] = { 0.00f, 0.25f, 0.50f, 0.75f, 1.00f };
static const float scen4b_steps[] = { 0.50f };
static const float scen4c_steps[] = { 0.0f, 90.0f, 180.0f, 90.0f };
static const motor_scen_slot_t scen4_slots[] = {
    /* A：与 B 共享 20kHz 定时器（ch0），阶梯步进——隔离断言的「动方」 */
    { "motor.pwm.generic", 0, 4, -1, -1, -1,
      0.0f, 0.0f,
      scen4a_steps, 5, 1000 },
    /* B：同 20kHz 定时器 ch1（≠A 的 ch0），恒占空 0.5——隔离断言的「静方」 */
    { "motor.pwm.generic", 1, 10, -1, -1, -1,
      0.0f, 0.0f,
      scen4b_steps, 1, 1000 },
    /* C：独立 50Hz 定时器（SG90），慢角度摆动——频率并存的可观测面 */
    { "motor.servo.sg90", 0, 5, -1, -1, -1,
      0.0f, 0.0f,
      scen4c_steps, 4, 1250 },  /* 4 点 × 2.5s = 10s 一循环 */
};

/* ── 场景 5：DC H 桥正反转切换（+50% 持续 → −50%，方向位翻转 + duty 对称） ── */
static const char *const scen5_timers[] = { "pwm.motor.20k" };
static const float scen5_steps[] = { 0.50f, -0.50f };
static const motor_scen_slot_t scen5_slots[] = {
    { "motor.dc.generic", 0, 11, 12, -1, -1,
      0.0f, 0.0f,
      scen5_steps, 2, 1000 },   /* 2 步 × 2s = 4s 一循环（26s 采集 ≈ 6.5 循环） */
};

const motor_scen_def_t motor_scen_defs[] = {
    { 1, "pwm",    "duty staircase 0/25/50/75/100% (2s/step)",
      scen1_timers, 1, scen1_slots, 1 },
    { 2, "servo",  "servo 5-point calibration 0/45/90/135/180 deg (2s/point)",
      scen2_timers, 1, scen2_slots, 1 },
    { 3, "enc",    "closed-loop step 30->90 rpm (4s/segment, 2 rises/cycle)",
      scen3_timers, 1, scen3_slots, 1 },
    { 4, "nested", "shared-timer A/B (20kHz ch0/ch1) + independent C (50Hz servo)",
      scen4_timers, 2, scen4_slots, 3 },
    { 5, "dc",     "H-bridge direction flip +50%/-50% signed duty (2s/step)",
      scen5_timers, 1, scen5_slots, 1 },
};

const int motor_scen_def_count =
    (int)(sizeof(motor_scen_defs) / sizeof(motor_scen_defs[0]));
