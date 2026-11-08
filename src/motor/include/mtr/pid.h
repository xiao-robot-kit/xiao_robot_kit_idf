/* SPDX-License-Identifier: MIT */
#ifndef MTR_PID_H
#define MTR_PID_H

/**
 * @file    pid.h
 * @brief   通用 PID 控制器（back-calculation 抗饱和）——被电机 ENC_CLOSED 速度环消费
 *
 * @note    单位约定：误差与输出的量纲映射由调用方定义，本库不做任何物理假设。
 *          例（速度环）：error = 目标速度 − 实测速度（counts/s），输出 = 占空 %，
 *          则 kp 量纲 = %/(counts/s)，ki 量纲 = %/(counts/s·s)。
 * @note    POSIX 风格：返回 0 = 成功 / -1 = 参数非法；实例结构体化——pid_t 自带
 *          配置与全部运行态，零全局可变态、零静态槽位（要 N 个环就定义 N 个实例）。
 * @note    非线程安全：同一实例的 pid_step 须在同一执行上下文（控制环任务/回调）
 *          串行调用；跨上下文共享由调用方加锁。
 * @note    命名提示：与 POSIX <sys/types.h> 的进程号 pid_t 同名。本组件翻译单元不
 *          引入 sys/types.h；消费方若须两者并用，请勿在同一名字空间混用。
 */

/** @brief PID 配置（参数全来自业务层/预设表，库代码内零写死） */
typedef struct {
    float kp;             /**< 比例增益（输出量纲/误差量纲） */
    float ki;             /**< 积分增益（输出量纲/误差量纲·s） */
    float kd;             /**< 微分增益（输出量纲/误差量纲·s；速度环通常取 0） */
    float out_min;        /**< 输出下限（须 < out_max） */
    float out_max;        /**< 输出上限 */
    float kb;             /**< 抗饱和回灌系数（back-calculation，≥ 0；0 = 关闭抗饱和） */
} pid_cfg_t;

/**
 * @brief   PID 实例（配置 + 运行态；pid_init 深拷配置，此后改原 cfg 不影响实例）
 * @note    cfg 字段为实例私有，运行期外部修改属未定义行为（须改参请重建实例）。
 */
typedef struct {
    pid_cfg_t cfg;        /**< 配置（init 时拷入，运行期只读） */
    float     integral;   /**< 积分项状态（输出量纲；抗饱和在此状态上回灌） */
    float     prev_error; /**< 上拍误差（D 项差分用；have_prev=0 时无效） */
    int       have_prev;  /**< 上拍误差是否有效（首拍 D 项恒为 0） */
    int       ready;      /**< 1 = 已 pid_init 成功；零初始化实例为 0，step 拒绝 */
    float     out;        /**< 上拍输出（已限幅，控制量） */
    float     out_unsat;  /**< 上拍未限幅输出（诊断/测试用） */
} pid_t;

/**
 * @brief   初始化实例（配置深拷 + 状态清零）
 * @param   pid  实例（非 NULL）
 * @param   cfg  配置（非 NULL）
 * @return  0 = 成功；-1 = 参数非法（pid/cfg 为 NULL、增益或限幅非有限值、
 *          out_min >= out_max、kb < 0）。失败时实例内容不变。
 */
int pid_init(pid_t *pid, const pid_cfg_t *cfg);

/**
 * @brief   清运行态（积分/上拍误差/输出），保留配置——换目标/重新起转用
 * @return  0 = 成功；-1 = pid 为 NULL
 */
int pid_reset(pid_t *pid);

/**
 * @brief   推进一拍（P + I + D + 限幅 + back-calculation 抗饱和）
 * @param   pid    实例（非 NULL 且已 init 成功）
 * @param   error  本拍误差（= 设定值 − 测量值；正误差 → 正输出方向）
 * @param   dt     步长 s（> 0 且有限；与控制环周期一致）
 * @return  0 = 成功，控制量已写入 pid->out；-1 = 参数非法（状态不变）
 *
 * 每拍公式（与旧式实现 旧式实现 speed_control.py:134-150 的已验证代码同构）：
 *
 *     p        = kp·error
 *     d        = kd·(error − prev_error)/dt        （首拍 d = 0）
 *     u_unsat  = p + integral + d                  （integral 取本拍【之前】的累积值）
 *     u        = clamp(u_unsat, out_min, out_max)
 *     integral += ki·error·dt + kb·(u − u_unsat)·dt
 *
 * 抗饱和方向（本库正确性核心；旧式实现 :13 docstring 的符号表述与代码相反，勿抄）：
 *     正向饱和 u_unsat > out_max ⇒ (u − u_unsat) < 0 ⇒ 回灌为负，积分被【抑制】；
 *     负向饱和 u_unsat < out_min ⇒ (u − u_unsat) > 0 ⇒ 回灌为正，积分被【抑制】；
 *     回灌项恒把积分拉离饱和区（退饱和更快、反向超调更小）。kb = 0 时退化为普通 PID。
 */
int pid_step(pid_t *pid, float error, float dt);

/* ---- 预设注册表（与 motor 组件同构：命名挂载，零写死） ---- */

/** @brief PID 命名预设项 */
typedef struct {
    const char *name;   /**< 预设名（如 "pid.vel_pi.default"） */
    pid_cfg_t   cfg;    /**< 配置底版（可直接 pid_init） */
} pid_cfg_preset_t;

/** @brief 预设表（定义在 src/pid.c；追加只改那一个文件） */
extern const pid_cfg_preset_t pid_cfg_presets[];

/** @brief 预设数量（= 表长） */
extern const int pid_cfg_preset_count;

/**
 * @brief   按名查预设
 * @return  匹配项的配置底版指针（静态存储）；NULL = 名为 NULL/空串/无此预设
 */
const pid_cfg_t *pid_cfg_find(const char *name);

#endif /* MTR_PID_H */
