/* SPDX-License-Identifier: MIT */
#ifndef EXP_MOTOR_PIDSCAN_SCEN_H
#define EXP_MOTOR_PIDSCAN_SCEN_H

/**
 * @file    exp_motor_pidscan_scen.h
 * @brief   PID 参数网格扫描实验运行时（kp×ki 3×3=9 组注册表驱动，每组 ENC
 *          闭环阶跃响应 + 三指标记录 + 对比表汇总；表驱动，零写死）
 *
 * @note    实验形态（调参实验 = 阶段维补齐）：对同一台 N20 带编码器电机
 *          （同一 wiring、同一合成 plant）轮转 9 组速度环 PI 参数——每组
 *          重建电机实例（vel_pid 网格值覆盖底版 motor.enc.n20），跑
 *          30→90 rpm 阶跃（低段前导 + 高段统计窗），记录三指标：
 *          收敛时间 settle_s / 超调% overshoot_pct / 稳态误差% ss_err_pct。
 *          9 组跑完算最优组（score 三指标折算加权和最小），TEXT 汇总帧
 *          逐行输出对比表（最优组 '*' 标注）。
 * @note    装配序：pwm_hal_register → 定时器（pwm.motor.20k）→ plant 注册
 *          （常驻：enc 引脚对全组不变，控制环与反馈环在纯 RAM 闭环）→
 *          每组 group_begin（destroy 旧电机 + create 新电机[网格 PID] +
 *          motor_hal_attach 挂编码器 HAL）→ 逐拍 tick → group_end
 *          （destroy 电机 + 指标定案）。plant 常驻的理由：sim plant 按
 *          enc 引脚对分派、与电机实例解耦；motor_destroy 清零桥双通道
 *          （motor.c:189-197），组间 plant 从零速重启（无跨组残留耦合）。
 * @note    遥测 actual = 编码器测量 rpm（plant 一阶滞后 160 rpm/duty、
 *          τ=20ms，同 exp_motor_enc 源）；duty = 速度环 PID 输出占空的
 *          sim 存储回读。参数零写死：网格/步表/保持拍数/wiring/plant 增益
 *          全在 exp_motor_pidscan_scen.c 的表里；电机底版 = 库预设
 *          motor.enc.n20（cpr/限幅单源在预设）。
 * @note    0x05 帧 slot 域复用：本工程单电机轮转，slot 字段 = 扫描组号
 *          （0..8，帧注释「槽序固定 0..3」是四槽综合演示的语义；单实例
 *          工程按 payload 长度自适配先例见 exp_motor_enc）；phase 字段 =
 *          组内步号（0=低段 30rpm，1=高段 90rpm）。Host 查看器按本契约解。
 */

#include <stdint.h>
#include <stddef.h>
#include "mtr/motor.h"

/** 控制周期，us（拍时基单源：hold_ticks 与 plant dt_s 以此计 = 固件周期） */
#define EXP_MOTOR_PIDSCAN_POLL_US 2000

/** 网格数（3×3 kp×ki；网格本体在 scen.c 的注册表，此处只供数组定长） */
#define EXP_MOTOR_PIDSCAN_GROUPS 9

/** 段收敛判据：稳态误差上限 %（任务卡：终值 ±3%） */
#define EXP_MOTOR_PIDSCAN_SS_TOL_PCT 3.0f

/** 高段目标 rpm（统计窗 = 升沿 + 稳态；网格全体共用一目标）——值单源住 scen.c 表 */

/**
 * @brief   单组调参结果（三指标 + 评分；group_end 定案，get_result 取回）
 */
typedef struct {
    float settle_s;        /**< 收敛时间 s（高段末次出带到段末；不收敛 = -1 哨兵） */
    float overshoot_pct;   /**< 超调 %（高段峰值超出目标的量 / 阶跃幅度×100；无超调 = 0） */
    float ss_err_pct;      /**< 稳态误差 %（高段末半段 |均值−目标|/目标×100） */
    float score;           /**< 评分 = settle_s + ov/100 + se/100（越小越优；不收敛 = -1） */
    int   ok;              /**< 1 = 收敛（末拍在 ±3% 带内且稳态误差 ≤3%）；0 = 不收敛 */
} exp_pid_result_t;

/**
 * @brief   单拍遥测（0x05 帧 16B 记录的 C 侧载体，字段序 = proto/frame.h
 *          PROTO_TYPE_MOTOR 注释——布局单源在彼）
 */
typedef struct {
    float    setpoint;   /**< 当前步目标（ENC_CLOSED 域 = rpm） */
    float    duty;       /**< 速度环输出占空 = sim 存储回读（回读失败 -1 哨兵） */
    float    actual;     /**< 实际值 = 编码器测量 rpm（合成 plant） */
    int32_t  enc_count;  /**< 编码器累计计数（plant 累积，跨组连续） */
    uint32_t fault;      /**< 活状态故障位图（位义见 mtr/motor_types.h） */
    int      phase;      /**< 组内步号（0=低段，1=高段；0x05 记录 phase 字段） */
    int      kind;       /**< motor_kind_t（帧 payload 用） */
} exp_motor_tele_t;

/**
 * @brief   装配实验（sim PWM/编码器 HAL + 定时器 + 常驻 plant；init 一次）
 * @retval  0    装配成功（group_begin/tick/group_end 即可用）
 * @retval  -1   装配失败（细节已 printf 到 stderr；失败原子回滚不留半挂）
 */
int exp_motor_pidscan_scen_init(void);

/**
 * @brief   一组扫描的总拍数（步表 hold 之和，单源 = scen.c 表；低段+高段）
 * @return  拍数（未 init = 0）——固件 main 与 host 自检的组循环长度
 */
uint32_t exp_motor_pidscan_group_ticks(void);

/**
 * @brief   全量反装配（destroy 活电机 + plant 注销 + 定时器；幂等）
 * @note    固件常驻不需要；host 自检收尾/异常清场用。
 */
void exp_motor_pidscan_scen_destroy(void);

/**
 * @brief   开始一组扫描（组间轮转：销毁上一组电机 + 按网格建新电机）
 * @param   g  组号（0..EXP_MOTOR_PIDSCAN_GROUPS-1；越界 = -1 拒绝）
 * @retval  0    组装配成功（tick 从低段第 0 拍起步）
 * @retval  -1   参数非法或电机创建失败（上一组已销毁，状态干净）
 */
int exp_motor_pidscan_group_begin(int g);

/**
 * @brief   推进一拍（组内步进发生器 + set/poll/get_state + 存储回读 + 指标累积）
 * @param   now_us  单调时基，us（调用方提供；固件侧逐拍累加 POLL_US）
 * @param   out     出参：本拍遥测（非 NULL；组未装配 = fault 哨兵行）
 */
void exp_motor_pidscan_scen_tick(uint32_t now_us, exp_motor_tele_t *out);

/**
 * @brief   结束当前组（销毁电机 + 三指标定案）
 * @param   g     组号（须与 group_begin 的一致；不一致 = -1 拒绝）
 * @param   out   出参：组结果（非 NULL；结果同时存内部表，get_result 可再取）
 * @retval  0    定案成功；-1 = 参数非法或无活组
 */
int exp_motor_pidscan_group_end(int g, exp_pid_result_t *out);

/**
 * @brief   取组结果（group_end 之后有效；未跑组 = 全 -1 哨兵 + ok=0）
 */
void exp_motor_pidscan_get_result(int g, exp_pid_result_t *out);

/**
 * @brief   最优组号（score 最小者；并列取组号小——可复现）
 * @retval  0..8  最优组；-1 = 无任何收敛组（网格全体不收敛，如实上报）
 */
int exp_motor_pidscan_best_group(void);

/**
 * @brief   组行对比表文本（TEXT 0x7F 帧 payload；≤ cap-1 字符 + NUL）
 * @param   g          组号（0..8）
 * @param   buf        输出缓冲（非 NULL）
 * @param   cap        缓冲容量（建议 ≥64；含最优 '*' 标注的行长上限）
 * @param   mark_best  1 = 最优组行尾加 " *"；0 = 从不标注（流式进度行用——
 *                     网格未跑全时 best 只见部分结果，携带 '*' 会让整表
 *                     出现多个星，违反唯一标注契约）
 * @return  行长（不含 NUL）；0 = 参数非法
 * @note    行格式（单源契约，host 自检/上位机按此解析）：
 *          "G<k> kp<kp> ki<ki> st<settle>s ov<ov>%% se<se>%%[ *| !]"
 *          例 "G6 kp0.015 ki1.00 st0.042s ov9.7% se0.00% *"（* = 最优组，
 *          仅轮末全表调用时恰 1 个；不收敛组尾标 !）。
 */
size_t exp_motor_pidscan_summary_line(int g, char *buf, size_t cap, int mark_best);

/**
 * @brief   对比表表头行（TEXT 帧 payload；"g kp ki settle_s ov%% se%%"）
 * @return  行长（不含 NUL）；0 = 参数非法
 */
size_t exp_motor_pidscan_header_line(char *buf, size_t cap);

/**
 * @brief   遥测打包成 0x05 帧 payload（1×16B；slot = 当前组号）
 * @note    固件帧输出与 host 自检字节级断言共用同一布局契约（proto/frame.h）。
 */
void exp_motor_pidscan_scen_pack(const exp_motor_tele_t *tele, uint8_t payload[16]);

#endif /* EXP_MOTOR_PIDSCAN_SCEN_H */
