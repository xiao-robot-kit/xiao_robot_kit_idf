/* SPDX-License-Identifier: MIT */
/**
 * @file    imu_heater.h
 * @brief   IMU 恒温加热控制组件（纯算法，零平台依赖）
 *
 * @note    物理背景：XIAO Robot Board 板上 U18 LSM6DS3 附近有加热电阻（D12 PWM
 *          → Q1 NMOS 驱动），职责＝升/稳 IMU 周围温度以配合温漂补偿
 *          （components/calibration 的 temp_comp）。本例程为改温场景强制的
 *          预加热手段。
 *
 * @note    与任务书 P-10 imu_heater_thermo 的对应：P-10 是物理层
 *          （ADC 读热敏 / PWM 输出 / 过温保护），本组件是**控制层**纯算法：
 *          给当前芯片温度，返回 PWM 开度，三档 API 齐备 ——
 *          使能/失能、阈值触发启停、恒温滞回微调、阶段减缓加热。
 *
 * @note    「温度」一律 °C，取 imu_types.h 的 imu_sample_temp_t.temp 语义
 *          （驱动侧已过换算，本组件只消费物理温度不接触寄存器）。
 */

#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 加热状态机状态 */
typedef enum {
    IMU_HEATER_OFF = 0,   /**< 失能：开度恒 0（省电档，禁用即断热） */
    IMU_HEATER_IDLE,      /**< 使能但温度已达标（滞回上沿之上），不加热 */
    IMU_HEATER_WARMING,   /**< 加热中：温差按阶段表逐档减缓 */
    IMU_HEATER_HOLD,      /**< 恒温窗内：最低档微调保持（补偿热损） */
    IMU_HEATER_OVERHEAT,  /**< 超限：温度过高强制关闭输出（保护） */
} imu_heater_state_t;

/** @brief 阶段减缓表项：温差 ≥ delta_c 时开度 duty（表按 delta_c 降序） */
typedef struct {
    float delta_c;  /**< 温差阈值 °C（目标温度 − 当前温度；越大越冷） */
    float duty;     /**< 该档开度 0~1 */
} imu_heater_stage_t;

/** @brief 参数（一律走配置结构体，代码内禁零写死 magic number） */
typedef struct {
    float target_temp_c;   /**< 恒温目标 °C（加热目标；配合 temp_comp 温补使用时设为温标参考温度附近） */
    float hyst_c;          /**< 启停滞回 °C：temp ≤ target−hyst 触发加热；temp ≥ target+hyst 停止 */
    const imu_heater_stage_t *stage;  /**< 阶段递减表（降序）；NULL = 恒流全开 */
    size_t stage_n;        /**< 表项数 */
    float max_duty;        /**< 开度上限 0~1（电气/热功耗护栏） */
    float hold_duty;       /**< 恒温窗内微调保持开度 0~1（补偿热损，最小档） */
    float overheat_c;      /**< 过热保护温度 °C：temp ≥ 此值 → OVERHEAT 强制 0 开度 */
    float sample_period_s; /**< 调用周期 s（仅用于 heating_s 时长累计；≤0 不累计） */
} imu_heater_cfg_t;

/** @brief 运行实例 */
typedef struct {
    imu_heater_cfg_t  cfg;
    imu_heater_state_t state;
    float             duty;      /**< 本周期输出开度 0~1 */
    float             heating_s; /**< 累计加热时长 s（WARMING/HOLD 且 duty>0 时按 sample_period_s 累计） */
} imu_heater_t;

/** @brief 命名预设（注册表项；表定义在 src/imu_heater.c，追加预设只改那一个文件） */
typedef struct {
    const char        *name;
    imu_heater_cfg_t   cfg;
} imu_heater_cfg_preset_t;

/** @brief 预设表与项数（定义在 src/imu_heater.c） */
extern const imu_heater_cfg_preset_t imu_heater_presets[];
extern const size_t                  imu_heater_presets_n;

int imu_heater_init(imu_heater_t *h, const imu_heater_cfg_t *cfg);
int imu_heater_enable(imu_heater_t *h);
int imu_heater_disable(imu_heater_t *h);

/**
 * @brief 推进状态机一步（周期调用；样本周期由调用方节奏决定）
 * @param temp_c 当前 IMU 芯片温度 °C
 * @return 本周期输出开度 0~1（= h->duty）
 *
 * @note  状态机（滞回恒温）：
 *          IDLE（达标零开度）→[temp ≤ target−hyst]→ WARMING（按阶段表逐档）
 *          WARMING →[temp ≥ target]→ HOLD（hold_duty 恒温微调）
 *          HOLD  →[temp ≥ target+hyst]→ IDLE（停止省电）／[temp ≤ target−hyst]→ WARMING（重升温）
 *          任意 →[temp ≥ overheat_c]→ OVERHEAT（0 开度保护）→[回降 target+hyst 内]→ IDLE
 */
float imu_heater_update(imu_heater_t *h, float temp_c);

/** @brief 查询预设（找不到返回 NULL） */
const imu_heater_cfg_preset_t *imu_heater_cfg_find(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* IMU_HEATER_H */