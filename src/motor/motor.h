/* SPDX-License-Identifier: MIT */
#ifndef MTR_MOTOR_H
#define MTR_MOTOR_H

#include "motor_types.h"
#include "pwm_timer.h"

/**
 * @file    motor.h
 * @brief   实例化电机驱动库接口（业务层 create 一个实例 = 一台电机，随意 N 个）
 *
 * @note    参数零写死（与 IMU 库同构）：一切经 motor_cfg_t 挂载——配置可整结构体
 *          手填，或从命名预设表取底版再改；驱动库内零 magic number。
 * @note    POSIX 风格：pthread_create 型（不透明句柄 + 属性结构体）；
 *          实例自带全部状态（PID 积分 / S 梯度相位 / 编码器累计），无全局态。
 */

/** @brief 电机实例（不透明句柄，实现定义于 src） */
typedef struct motor motor_t;

/** @brief 速度环 PID 配置（参数全来自业务层/预设表） */
typedef struct {
    float kp, ki, kd;       /**< 增益 */
    float out_min, out_max; /**< 输出限幅（积分饱和防护在此界内实现） */
} motor_pid_cfg_t;

/**
 * @brief   一台电机的完整配置（挂载进实例的「表」）
 *
 * @note    timer+channel 决定输出物理通道（嵌套关联见 pwm_timer.h）；
 *          编码器字段仅 MOTOR_ENC_CLOSED 使用；舵机 pulse_min/max_us
 *          仅 MOTOR_SERVO 使用。
 */
typedef struct {
    motor_kind_t     kind;          /**< 电机种类 */
    pwm_timer_t     *timer;         /**< ★ 关联定时器句柄（不可为 NULL） */
    uint8_t          channel;       /**< 该定时器上的通道号（须 < timer 的 channels） */
    int              pin_a;         /**< PWM 输出引脚（DC 桥正向/单 PWM 输出） */
    int              pin_b;         /**< DC 桥反向引脚（非 DC 类可填 -1 不用） */
    /* 编码器（ENC_CLOSED） */
    int              enc_a;         /**< 编码器 A 相引脚 */
    int              enc_b;         /**< 编码器 B 相引脚 */
    int32_t          cpr;           /**< 每转计数（counts per revolution） */
    /* 舵机（SERVO） */
    uint16_t         pulse_min_us;  /**< 最小角度脉宽，us（如 500） */
    uint16_t         pulse_max_us;  /**< 最大角度脉宽，us（如 2500） */
    float            angle_min;     /**< 脉宽 min 对应角度，deg */
    float            angle_max;     /**< 脉宽 max 对应角度，deg */
    /* 闭环（ENC_CLOSED） */
    motor_pid_cfg_t  vel_pid;       /**< 速度环参数 */
    /* 通用保护 */
    float            duty_max;      /**< 占空比上限 0.0~1.0 */
} motor_cfg_t;

/**
 * @brief   创建电机实例（配置挂载：cfg 深拷进实例，之后改 cfg 不影响已建实例）
 * @param   cfg  完整配置（timer 须已创建且 channel 在范围内）
 * @return  实例句柄；NULL = 配置非法（timer=NULL / channel 越界 / duty_max 越界）
 */
motor_t *motor_create(const motor_cfg_t *cfg);

/**
 * @brief   销毁电机实例（释放通道；不动定时器——定时器由业务层自管）
 * @return  0 = 成功；-1 = 参数 NULL
 */
int motor_destroy(motor_t *m);

/**
 * @brief   设定目标
 * @param   target  速度 rpm（PWM_OPEN/DC/ENC_CLOSED）或角度 deg（SERVO）
 * @return  0 = 成功；-1 = 参数非法
 */
int motor_set_target(motor_t *m, float target);

/**
 * @brief   节拍推进（S 梯度轨迹 + PID 闭环 + 输出到 PWM 通道）
 * @param   now_us  当前时刻，us（单调时基，调用方提供）
 * @return  0 = 成功；-1 = 参数或时基回退非法
 * @note    由业务层在固定周期回调/任务里驱动；多实例各自 poll，互不干扰。
 */
int motor_poll(motor_t *m, uint32_t now_us);

/**
 * @brief   查询运行状态（actual 在 ENC_CLOSED 为编码器测量，其余为上次设定）
 * @return  0 = 成功；-1 = 参数 NULL
 */
int motor_get_state(const motor_t *m, motor_state_t *out);

/* ---- 电机预设注册表（与 IMU 同构：命名挂载，零写死） ---- */

/** @brief 电机命名预设项（timer_preset 指名创建后传入 cfg） */
typedef struct {
    const char   *name;         /**< 预设名（如 "motor.dc.n20" / "motor.servo.sg90"） */
    motor_cfg_t   cfg;          /**< 配置底版（timer 字段由业务层创建后填入） */
    const char   *timer_preset; /**< 建议的定时器预设名（如 "pwm.motor.20k"） */
} motor_cfg_preset_t;

/** @brief 预设表（定义在 motor_cfg_presets.c，追加只改那一个文件） */
extern const motor_cfg_preset_t motor_cfg_presets[];
extern const int                motor_cfg_preset_count;

/**
 * @brief   按名查预设（返回配置底版指针，静态存储）
 * @return  匹配项；NULL = 无此预设
 * @note    底版的 timer=NULL——业务层须先 pwm_timer_create_preset(项.timer_preset)
 *          并把句柄填进 cfg 副本再 create（三行样板见预设注册表的底版约定）。
 */
const motor_cfg_t *motor_cfg_find(const char *name);

#endif /* MTR_MOTOR_H */
