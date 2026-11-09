/* SPDX-License-Identifier: MIT */
#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stdint.h>
#include "imu/imu_types.h"

/**
 * @file    imu_sim.h
 * @brief   固件层 IMU 合成数据源（exp2 无传感器全链路；纯算法，无平台依赖）
 *
 * @note    定位：给「调零 / 温补」提供**确定性合成真值**——
 *          静止或正弦摇摆的真值 + 可注入的合成误差（零偏 / 温漂）。
 *          标定/补偿算法吃它产出的样本，收敛结果可与注入真值精确比对。
 * @note    单位契约（与 imu/imu_types.h 一致，本模块零换算）：
 *          加速度 g、角速度 dps、温度 °C、时间秒、采样率 Hz。
 * @note    确定性：无噪声注入（同配置同拍号 → 逐位相同输出），
 *          断言可精确到浮点容差；需要噪声场景时后续以参数化注入扩展。
 * @note    运动学简化（诚实声明）：摇摆 = 重力投影绕轴旋转 + 同轴角速度，
 *          **不含**切向 / 向心加速度项（本源服务于零偏与温漂验证，
 *          非动力学仿真）；因此任意模式下加速度模长恒 = 1g。
 */

/** @brief 圆周率相关物理常量（单位换算，非可调参数） */
#define IMU_SIM_TWO_PI   6.283185307179586   /* 2π（相位 rad） */
#define IMU_SIM_DEG2RAD  0.017453292519943295 /* 度 → rad */

/** @brief 合成运动模式 */
typedef enum {
    IMU_SIM_MODE_STATIC = 0,   /**< 静止：加速度 = 姿态重力向量，角速度 = 0 */
    IMU_SIM_MODE_SWAY    = 1,  /**< 正弦摇摆：重力绕轴摆动 + 同轴正弦角速度 */
} imu_sim_mode_t;

/** @brief 姿态（静止时重力在体轴系的方向；决定标定的重力轴与符号） */
typedef enum {
    IMU_SIM_ATT_Z_UP   = 0,    /**< 水平放置：az = +1g（屏朝上，exp 默认） */
    IMU_SIM_ATT_Z_DOWN = 1,    /**< 翻转：az = −1g */
    IMU_SIM_ATT_X_UP   = 2,    /**< 侧立：ax = +1g */
    IMU_SIM_ATT_X_DOWN = 3,    /**< 侧立：ax = −1g */
    IMU_SIM_ATT_Y_UP   = 4,    /**< 侧立：ay = +1g */
    IMU_SIM_ATT_Y_DOWN = 5,    /**< 侧立：ay = −1g */
} imu_sim_att_t;

/** @brief 摇摆轴（= 角速度所在轴；枚举值即 gx/gy/gz 的轴下标 0/1/2） */
typedef enum {
    IMU_SIM_AXIS_X = 0,
    IMU_SIM_AXIS_Y = 1,
    IMU_SIM_AXIS_Z = 2,
} imu_sim_axis_t;

/**
 * @brief   合成源配置（参数零写死：可调项全部经此注册）
 *
 * @note    摇摆模型（t = 拍号 / sample_rate_hz，秒）：
 *          摆角 θ(t) = sway_amp_deg · sin(2π·t / sway_period_s)（度）
 *          角速度 ω(t) = sway_amp_deg · (2π / sway_period_s) · cos(2π·t / sway_period_s)（dps）
 *          重力投影 = 姿态向量绕 sway_axis 旋转 θ(t)（模长恒 1g）
 * @note    温度模型（芯片自热线性斜坡）：
 *          T(t) = temp_start_c + temp_ramp_c_per_s · t（°C）
 * @note    误差注入模型（标定/温补的验证靶）：
 *          合成零偏 bias(T) = bias + drift · (T − temp_ref_c)
 *          输出样本 = 真值(t) + bias(T(t))；恒定项与温漂项单位同 imu_bias_t。
 */
typedef struct {
    imu_sim_mode_t mode;           /**< 运动模式 */
    imu_sim_att_t  att;            /**< 姿态（静止重力方向） */
    imu_sim_axis_t sway_axis;      /**< 摇摆轴（仅 SWAY 模式使用） */
    float sway_amp_deg;            /**< 摆幅，度（仅 SWAY；0 合法=不摆） */
    float sway_period_s;           /**< 摆动周期，秒（仅 SWAY；须 > 0） */
    float sample_rate_hz;          /**< 采样率，Hz（时间基：每读一拍 = 1/rate 秒） */
    float temp_start_c;            /**< 起始温度，°C */
    float temp_ramp_c_per_s;       /**< 温度斜率，°C/s（自热；0 = 恒温） */
    float temp_ref_c;              /**< 温漂参考温度，°C（T=此值时漂移项为 0） */
    imu_bias_t bias;               /**< 恒定零偏真值（gx..gz dps；ax..az g） */
    float gyro_drift[3];           /**< 陀螺温漂真值，dps/°C（X/Y/Z） */
    float accel_drift[3];          /**< 加速度温漂真值，g/°C（X/Y/Z） */
} imu_sim_cfg_t;

/** @brief 命名预设（注册表项；表定义在 imu_sim.c，追加预设只改那一个文件） */
typedef struct {
    const char    *name;   /**< 预设名（点分命名空间如 "sim.exp2.static"） */
    imu_sim_cfg_t  cfg;    /**< 参数 */
} imu_sim_cfg_preset_t;

/** @brief 预设表与项数（定义在 imu_sim.c） */
extern const imu_sim_cfg_preset_t imu_sim_cfg_presets[];
extern const int                  imu_sim_cfg_preset_count;

/**
 * @brief   按名查预设
 *
 * @param[in] name  预设名
 *
 * @return  匹配项的 cfg 指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const imu_sim_cfg_t *imu_sim_cfg_find(const char *name);

/**
 * @brief   合成源实例（状态全部实例内，无全局可变态）
 */
typedef struct {
    imu_sim_cfg_t cfg;       /**< init 时拷贝的配置 */
    uint8_t       inited;    /**< 1 = 已 init（防未初始化即读） */
    uint32_t      tick;      /**< 已产出采样拍号（时间 t = tick / sample_rate_hz；
                                  uint32 饱和不回绕：100 Hz 下约 497 天） */
} imu_sim_t;

/**
 * @brief   初始化实例（绑定配置并复位拍号）
 *
 * @param[out] sim  实例
 * @param[in]  cfg  配置（拷贝入实例，调用方后续改原结构不影响已初始化实例）
 *
 * @retval  0  成功
 * @retval -1  sim/cfg 为 NULL；或参数非法：
 *             mode/att/sway_axis 越界；sample_rate_hz ≤ 0 或非有限；
 *             SWAY 模式下 sway_period_s ≤ 0 或非有限、sway_amp_deg < 0 或非有限；
 *             temp_start_c / temp_ramp_c_per_s / temp_ref_c 任一非有限；
 *             bias 六分量 / drift 六分量任一非有限。
 */
int imu_sim_init(imu_sim_t *sim, const imu_sim_cfg_t *cfg);

/**
 * @brief   读一拍六轴样本（真值 + 注入误差），并推进拍号
 *
 * @param[in,out] sim  实例
 * @param[out]     out  样本（单位：ax/ay/az = g，gx/gy/gz = dps）。
 *                      含姿态重力投影 / 摇摆角速度 + bias + drift·(T−temp_ref)
 *
 * @retval  0  已写入并推进一拍
 * @retval -1  sim/out 为 NULL，或实例未 init
 *
 * @note    拍号从 0 起：首次 read 计算的是 t = 0 的样本（静止模式下即纯真值）。
 */
int imu_sim_read(imu_sim_t *sim, imu_sample_t *out);

/**
 * @brief   读当前拍的温度（不推进拍号）
 *
 * @param[in]  sim      实例
 * @param[out] celsius  温度，°C（T = temp_start_c + ramp · 拍号/rate）
 *
 * @retval  0  已写入
 * @retval -1  sim/celsius 为 NULL，或实例未 init
 *
 * @note    同拍配对用法：**先本函数后 imu_sim_read**，两者按同一拍号计算
 *          （本函数不推进、read 计算后推进）——温度与样本严格同时刻。
 */
int imu_sim_read_temp(imu_sim_t *sim, float *celsius);

#endif /* IMU_SIM_H */
