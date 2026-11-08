/* SPDX-License-Identifier: MIT */
#ifndef MTR_SCURVE_H
#define MTR_SCURVE_H

#include <stdint.h>

/**
 * @file    scurve.h
 * @brief   S 梯度（七相位恒定 jerk）轨迹发生器——motor_poll 的设定值整形源
 *
 * @note    **资产定性（如实）**：提炼自旧式实现冻结资产，仅单测覆盖，无实机验证。
 *          提炼源两件（均为旧式实现自有代码）：
 *          - `旧式实现路径`
 *            （表回放执行器：冻结资产、无运行时使用者、仅单测——本库继承其
 *            「纯状态机零硬件依赖、可 host 单测」形态与实例游标语义）；
 *          - `旧式实现路径`
 *            （七段解析闭式 _accel_segs/_mirror_to_decel/_plan_move_analytic
 *            ——本库闭式数学的提炼源）。
 *          旧式实现设计为「MCU 不算 S 曲线、只查 host 下发间隔表」；本库按新任务
 *          要求反转为 MCU 侧闭式自算（仍零硬件访问，motor_poll 直接消费）。
 *
 * @note    按审查报告修雷（相对提炼源）：
 *          1. 三角峰值旧式实现用 80 次二分逼近（motion_planner.py `_plan_move_analytic`）
 *             → 本库改解析闭式解，零迭代、零迭代次数魔数；
 *          2. 旧式实现 host 采样存在「dt 整除总时长时末点重复」雷（poly_profile.py
 *             先 n=int(T/dt)+1 采满再 append 末点）→ 本库 t 单调推进 + done 锁存
 *             + 终点钉死，末点只发一次，done 后恒返回终值；
 *          3. 旧式实现 algo cfg 零防御 → 本库全入口参数校验，且每次计划构建后做
 *             自检（段间 v/a 连续 / 位移闭合 / 终态静止 / |a|、|j| 界内），
 *             任一失败拒装表。
 *
 * @note    七相位模型（v(t) 折线圆角化；Biagiotti & Melchiorri 经典 7 段）：
 *          jerk+ / 匀加 / jerk− / 匀速 / jerk− / 匀减 / jerk+。
 *          短行程自动退化（构造时裁掉零时长相位）：距离不足到 vmax → 三角
 *          （无匀速段）；更短 → 4 相（匀加/匀减也无）；vmax < amax²/jerk 时
 *          恒为无匀加段形态。恰达 d_full 的行程 → 匀速段为零（6 相）。
 *
 * @note    单位与方向契约：u 为调用方域单位（步数 / 角度 / rpm……全库不自设
 *          量纲）；vmax [u/s]、amax [u/s²]、jerk [u/s³]、dt [s]；target 与
 *          step 返回值为位置域绝对设定值 [u]，起点固定 0，target 可为负
 *          （反向运动，相位序不变、v/a 取镜像符号）。
 * @note    POSIX 风格：snake_case；int 返回 0 = 成功 / -1 = 失败；指针显式
 *          校验（float 返回面的 NULL 防御见 scurve_step 注）；实例结构体化
 *          （栈/静态/数组皆可，N 个随意），库内零全局可变态（const 预设表除外）。
 * @note    线程模型：非线程安全。同一实例须在同一执行上下文（motor_poll
 *          任务域）串行调用；跨上下文共享由调用方加锁。
 */

/** @brief 相位（含终态；零时长相位被裁掉，短行程自动退化见文件注） */
typedef enum {
    SCURVE_PHASE_JERK_UP1 = 0, /**< 相位1 加速侧 jerk+：a 从 0 爬到 +amax */
    SCURVE_PHASE_ACC,          /**< 相位2 匀加速：a = +amax */
    SCURVE_PHASE_JERK_DN1,     /**< 相位3 加速侧 jerk−：a 从 +amax 回 0（v 到峰值） */
    SCURVE_PHASE_CRUISE,       /**< 相位4 匀速：v = vmax，a = 0 */
    SCURVE_PHASE_JERK_DN2,     /**< 相位5 减速侧 jerk−：a 从 0 压到 −amax */
    SCURVE_PHASE_DEC,          /**< 相位6 匀减速：a = −amax */
    SCURVE_PHASE_JERK_UP2,     /**< 相位7 减速侧 jerk+：a 从 −amax 回 0（v 归零） */
    SCURVE_PHASE_DONE          /**< 终态：done 后 step 恒返回终值 */
} scurve_phase_t;

/** @brief 轨迹动力学配置（一切可调参数走此表；代码内零写死） */
typedef struct {
    float vmax; /**< 峰值速度上限 [u/s]，须 > 0 且有限 */
    float amax; /**< 峰值加速度上限 [u/s²]，须 > 0 且有限 */
    float jerk; /**< 加加速度幅值 [u/s³]（段内恒定 jerk，带符号使用），须 > 0 且有限 */
    float dt;   /**< 步进周期 [s]：一次 scurve_step 推进一个 dt，须 > 0 且有限 */
} scurve_cfg_t;

/** @brief 段表容量：保守停稳段 ≤3 + 七相位主计划 ≤7（retarget 最大拼装） */
#define SCURVE_SEGS_MAX 10

/**
 * @brief   恒定 jerk 解析段：v(t)=v0+a0·t+j·t²/2，s(t)=v0·t+a0·t²/2+j·t³/6
 * @note    为 scurve_t 内嵌存储而公开；字段对调用方只读（改段 = 重建计划）。
 */
typedef struct {
    scurve_phase_t phase; /**< 该段相位名（遥测/测试断言用） */
    double T;             /**< 段时长 [s] */
    double v0;            /**< 段初速度 [u/s]（绝对域，带符号） */
    double a0;            /**< 段初加速度 [u/s²]（绝对域，带符号） */
    double j;             /**< 段内恒定 jerk [u/s³]（带符号） */
} scurve_seg_t;

/** @brief 采样态查询输出（scurve_get） */
typedef struct {
    float pos;            /**< 位置设定值 [u]（绝对，init 起点为 0） */
    float vel;            /**< 速度 [u/s]（带符号，可作速度环前馈） */
    float acc;            /**< 加速度 [u/s²]（带符号） */
    scurve_phase_t phase; /**< 当前相位 */
} scurve_state_t;

/**
 * @brief   S 梯度实例（值语义：栈/静态/数组皆可，N 个随意；零全局态）
 * @note    pid_t 同款 ready 语义：零初始化（BSS/静态）实例 valid=0，
 *          全接口安全拒绝；成功 init 后 valid=1。
 */
typedef struct {
    scurve_cfg_t cfg;                  /**< 配置（init 深拷，此后改原 cfg 不影响实例） */
    scurve_seg_t seg[SCURVE_SEGS_MAX]; /**< 当前活动段表 */
    int    n_seg;                      /**< 段数 */
    double t_total;                    /**< 当前计划总时长 [s] */
    double p_end;                      /**< 计划终点位置 [u]（钉死在 target：收敛锚） */
    double t;                          /**< 计划内时间 [s]（每 step +dt，到 t_total 锁存） */
    int    idx;                        /**< 游标：当前段下标 */
    double t_acc;                      /**< 游标：当前段起点的计划内时间 [s] */
    double p_acc;                      /**< 游标：当前段起点的绝对位置 [u] */
    double p, v, a;                    /**< 上次求值采样态（绝对域） */
    scurve_phase_t phase;              /**< 当前相位 */
    uint8_t done;                      /**< 1 = 终态（step 恒返回终值） */
    uint8_t valid;                     /**< 1 = 已成功 init（零初始化实例全接口拒） */
} scurve_t;

/**
 * @brief   初始化：静止 0 位 → target 的完整 S 梯度计划（闭式一次性构建）
 * @param   s       实例（出参；失败也被置为「不动安全态」）
 * @param   cfg     动力学配置（非 NULL；四字段全 > 0 且有限）
 * @param   target  目标位置 [u]（有限；可为负 = 反向；0 = 零行程合法，立即 done）
 * @retval  0    计划构建并通过自检
 * @retval  -1   参数非法或构建自检失败；实例置安全态（done=1、step 恒 0）
 * @note    重调 init = 复位重走（旧式实现 scurve_exec_rewind 的同构替代）。
 */
int scurve_init(scurve_t *s, const scurve_cfg_t *cfg, float target);

/**
 * @brief   推进一个 cfg.dt 并返回推进后的位置设定值
 * @param   s  实例
 * @return  当前位置设定值 [u]（绝对域）
 * @note    末点语义（防旧式实现「整除末点重复」雷）：t 单调 +dt，到 t_total 当拍
 *          置 done 并返回钉死的终点（只发这一次）；此后每拍恒返回同一终值。
 * @note    防御：s == NULL 或未 init（零初始化）返回 0.0f——float 返回面无
 *          错误码通道，安全值 = 原点不动；需要错误码的面走 init/retarget/get。
 */
float scurve_step(scurve_t *s);

/**
 * @brief   计划是否已走完
 * @retval  1 = 已终态（含未 init / s == NULL 的防御判定：视为终态不推进）
 * @retval  0 = 运动中
 */
int scurve_done(const scurve_t *s);

/**
 * @brief   中途改目标重规划（保守「先停稳再走」语义）
 * @param   s       运行中实例（采样态 (p, v, a) 即重规划起点）
 * @param   target  新目标位置 [u]（有限，可为负 / 可等于当前位）
 * @retval  0    新计划构建并通过自检，已原子替换旧计划
 * @retval  -1   参数非法（NULL/未 init/非有限 target）或自检失败；
 *               此时旧计划原样保留（validate-then-commit，运动不被中断）
 * @note    语义：从当前 (p,v,a) 先闭式刹停（jerk−/匀减/jerk+ 三形态，含
 *          过制动回收），再从停稳点向新目标走完整七相位计划。全程 p/v/a
 *          连续、|a| ≤ amax、|j| ≤ jerk，终态精确静止于新目标。
 *          非时间最优（任意态时间最优重规划是 Ruckig 域，明确不在本库）；
 *          过制动回收相的短暂反向速度上界为 a0²/(2·jerk)（动力学决定，
 *          不受 vmax 约束——这是「刹车踩过头必须回退一点」的物理代价）。
 */
int scurve_retarget(scurve_t *s, float target);

/** @brief 查询当前相位（NULL/未 init → SCURVE_PHASE_DONE 防御值） */
scurve_phase_t scurve_phase(const scurve_t *s);

/**
 * @brief   查询采样态（上次 init/retarget/step 求值的 (p, v, a, phase)）
 * @retval  0    已写入 *out
 * @retval  -1   s/out 为 NULL 或未 init
 */
int scurve_get(const scurve_t *s, scurve_state_t *out);

/* ---- 配置预设注册表（与 pwm_timer/motor 同构：命名挂载，零写死） ---- */

/** @brief 命名预设项（底版：dt 须匹配调用方 poll 周期） */
typedef struct {
    const char *name;     /**< 预设名（如 "scurve.rpm.gentle"） */
    scurve_cfg_t cfg;     /**< 配置底版（单位域见名：rpm / step） */
} scurve_cfg_preset_t;

/** @brief 预设表（定义在 src/scurve.c，追加只改那一个文件） */
extern const scurve_cfg_preset_t scurve_cfg_presets[];
extern const int scurve_cfg_preset_count;

/**
 * @brief   按名查预设（返回静态存储的底版指针）
 * @return  匹配项；NULL = 无此预设或 name 为 NULL
 * @note    底版直接可喂 scurve_init；按需改 dt 后再用（cfg 值语义拷贝）。
 */
const scurve_cfg_t *scurve_cfg_find(const char *name);

#endif /* MTR_SCURVE_H */
