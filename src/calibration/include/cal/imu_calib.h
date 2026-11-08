/* SPDX-License-Identifier: MIT */
#ifndef CAL_IMU_CALIB_H
#define CAL_IMU_CALIB_H

#include <stdint.h>
#include "imu/imu_types.h"

/**
 * @file    imu_calib.h
 * @brief   IMU 静止零偏标定（均值法 + 重力轴检测；纯算法，无平台依赖）
 *
 * @note    提炼自旧式实现自有实现 旧式实现
 *          `embedded/src/core/calibration/imu_calib.c:55-124`（重组，非照搬）。
 * @note    原理：静止采集 N 样本 → 零偏 = mean(样本) − expected：
 *          - 重力轴（三轴 |mean| 最大者，**用原始值判定**）：expected = sign × 1g
 *          - 其余两轴：expected = 0
 * @note    单位契约（与 imu/imu_types.h 一致，本组件零换算）：
 *          加速度 g、角速度 dps；零偏与采样同单位。
 *          温度不归本文件（温漂归 cal/temp_comp.h）。
 * @note    重力轴必须用**原始值**判定（旧式实现实机教训，原 imu_calib.h:36-40）：
 *          重力方向是物理事实，与当前补偿量无关；若用去偏后的值判符号，
 *          补偿量与真值偏差 > 1g 时符号判反 → 校准发散。
 * @note    **补偿量是累加残差，不是覆盖**（原型自动补偿法，
 *          旧式实现 `runtime/app_stream.c:96-116`）：
 *          调用方喂样时先减**当前**零偏，本轮算出的是**残差 delta**；
 *          落库用 `imu_calib_accumulate`（bias_new = bias_old + delta），
 *          而不是 `imu_calib_result` 覆盖。多次校准因此逐步收敛，单次噪声
 *          只修正对应量、不会把已有的准确值抹掉。
 *          数学（以加速度水平轴为例，expected=0）：
 *          ```
 *          corrected = raw − B          // B = 当前存储的零偏
 *          delta     = mean(corrected)
 *          B_new     = B + delta        // 累加
 *          ```
 *          若真实零偏 4、当前 B = 5：corrected 均值 = −1 → B_new = 4。
 */

/** @brief 重力轴理论幅值，单位 g（物理常量：轴单位即 g，重力恰为 1；非可调参数） */
#define IMU_CALIB_GRAVITY_G   1.0f

/**
 * @brief   零偏标定配置（参数零写死：可调项全部经此注册）
 */
typedef struct {
    int32_t target_n;   /**< 目标样本数（采集时长 = target_n / 实际采样率，秒） */
} imu_calib_cfg_t;

/** @brief 命名预设（注册表项；表定义在 src/imu_calib.c，追加预设只改那一个文件） */
typedef struct {
    const char      *name;   /**< 预设名（协议可寻址，点分命名空间如 "cal.exp2.bias"） */
    imu_calib_cfg_t  cfg;    /**< 参数 */
} imu_calib_cfg_preset_t;

/** @brief 预设表与项数（定义在 src/imu_calib.c） */
extern const imu_calib_cfg_preset_t imu_calib_cfg_presets[];
extern const int                    imu_calib_cfg_preset_count;

/**
 * @brief   按名查预设
 *
 * @param[in] name  预设名
 *
 * @return  匹配项的 cfg 指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const imu_calib_cfg_t *imu_calib_cfg_find(const char *name);

/** @brief 采集状态 */
typedef enum {
    IMU_CALIB_IDLE = 0,       /**< 空闲（未开始，或本轮已完成） */
    IMU_CALIB_COLLECTING = 1, /**< 采集中 */
} imu_calib_state_t;

/**
 * @brief   零偏标定实例（状态全部实例内，无全局可变态）
 *
 * @note    累加用 double：N 个 float 求和用 float 会累积舍入误差
 *          （沿旧式实现结构体成员选型，原 imu_calib.h:18-19）。
 */
typedef struct {
    imu_calib_cfg_t   cfg;                        /**< init 时拷贝的配置 */
    imu_calib_state_t state;                      /**< 采集状态 */
    uint32_t          n;                          /**< 已采集样本数 */
    double            gx_sum, gy_sum, gz_sum;     /**< 求均值用累加（dps） */
    double            ax_sum, ay_sum, az_sum;     /**< 求均值用累加（g） */
    double            ax_raw_sum, ay_raw_sum, az_raw_sum; /**< 原始值累加（g；仅重力轴判定） */
    int               grav_axis;                  /**< 重力轴 0=X 1=Y 2=Z；-1 = 未检测 */
    int               grav_sign;                  /**< 重力方向符号 +1 / -1 */
} imu_calib_t;

/**
 * @brief   初始化实例（绑定配置并复位全部状态）
 *
 * @param[out] c    实例
 * @param[in]  cfg  配置（拷贝入实例，调用方后续改原结构不影响已初始化实例）
 *
 * @retval  0  成功
 * @retval -1  c/cfg 为 NULL，或 target_n < 1
 */
int imu_calib_init(imu_calib_t *c, const imu_calib_cfg_t *cfg);

/**
 * @brief   开始采集
 *
 * @param[in,out] c  实例
 *
 * @retval  0  成功（含"已在采集中"的幂等路径）
 * @retval -1  c 为 NULL
 *
 * @note    IDLE → COLLECTING：**清全部累加（九个 double）、n、重力轴检测字段**
 *          并转 COLLECTING。
 *          旧式实现雷已修【high】：旧 start 只清 6 个求均值累加与 n，漏清
 *          ax_raw_sum 等 raw 三轴（旧式实现 imu_calib.c:36-42，
 *          其头注却声称"会清零累加"——注释与实现失实），第二轮标定的
 *          重力轴判定被上轮残留污染。本版 start 清全部状态。
 * @note    已在 COLLECTING：幂等返回 0 且**不清进度**（重发命令不重启采集，
 *          沿旧式实现语义，原 imu_calib.c:37）。
 */
int imu_calib_start(imu_calib_t *c);

/**
 * @brief   喂入一个样本（采集推进）
 *
 * @param[in,out] c    实例
 * @param[in]     s    参与求均值的样本（单位：ax/ay/az = g，gx/gy/gz = dps）。
 *                     可喂**原始值**（结果即绝对零偏），也可喂**已减当前零偏的
 *                     残差值**（结果为增量，调用方累加进现有零偏——旧式实现
 *                     app_stream.c:103-111 的自动补偿法）。两种用法公式同形。
 * @param[in]     raw  同一时刻的**未去偏原始值**，仅用于重力轴判定（见文件头）；
 *                     NULL 合法：退化为用 s 判定（无已知零偏时二者本就相同）
 *
 * @retval  0  样本已接受（凑满与否都算成功；完成与否查 imu_calib_ready）
 * @retval -1  c/s 为 NULL，或未处于采集状态（调用序错误）
 *
 * @note    凑满 target_n 时：内部完成重力轴检测（原始均值三轴取 |max|）、
 *          转回 IDLE。检测用的均值以 target_n 为除数。
 */
int imu_calib_feed(imu_calib_t *c, const imu_sample_t *s, const imu_sample_t *raw);

/**
 * @brief   查询本轮标定是否完成且结果可取
 *
 * @param[in] c  实例
 *
 * @retval  1  已完成（可取结果）
 * @retval  0  未完成 / 未开始 / 正在采集
 * @retval -1  c 为 NULL
 */
int imu_calib_ready(const imu_calib_t *c);

/**
 * @brief   读取零偏标定结果
 *
 * @param[in]  c     实例
 * @param[out] bias  输出零偏（gx/gy/gz 单位 dps；ax/ay/az 单位 g；
 *                   gravity 轴分量已减 sign × 1g，即 bias = measured − expected）
 *
 * @retval  0  已写入结果
 * @retval -1  c/bias 为 NULL，或未就绪（imu_calib_ready != 1）
 *
 * @note    未就绪**显式拒绝**（返回 -1）：不返回"部分样本均值"——
 *          部分均值易被误读为"标定成功但零偏很小"。旧式实现以零填充
 *          处理同问题（原 imu_calib.c:106-111），本版以错误码强化契约。
 * @note    语义是**残差**（喂样时已减当前零偏）→ 落库请用
 *          `imu_calib_accumulate`；本函数是"只看本轮修正量"的读出口。
 */
int imu_calib_result(const imu_calib_t *c, imu_bias_t *bias);

/**
 * @brief   把本轮残差**累加**进现有零偏（自动补偿法落库；单源，禁止调用方手抄）
 *
 * @param[in]     c          实例
 * @param[in,out] bias_inout 当前零偏（入=旧值，出=旧值 + 本轮残差）
 *
 * @retval  0  已累加
 * @retval -1  c/bias_inout 为 NULL，或未就绪（imu_calib_ready != 1）
 *
 * @note    定义式 `bias_inout += imu_calib_result(c)`；累加在 float 上做
 *          （零偏本身是 float 量，原型 `app_stream.c:106-108` 同式）。
 * @note    **前置条件**：本轮喂样用的是 `raw − bias_inout`（见 imu_calib_feed
 *          的 s 参数），否则累加会重复计入已有零偏。
 */
int imu_calib_accumulate(const imu_calib_t *c, imu_bias_t *bias_inout);

#endif /* CAL_IMU_CALIB_H */
