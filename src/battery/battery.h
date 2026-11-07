/* SPDX-License-Identifier: MIT */
/**
 * @file    battery.h
 * @brief   电池电压 / 剩余电量（SOC）计算组件（纯算法，零平台依赖）
 *
 * @note    XIAO Robot Board：D16 = 电池电压采样引脚 → 板上分压电阻 100K:30K
 *          （电池 VBAT → 分压 → ESP32-S3 ADC，量程适配 4.2V 满电）。
 *          本组件负责：分压换算（ADC→VBAT）+ 电压→SOC（可配多段折线）+
 *          剩余毫安时换算；ADC 采样 / 滤波 / GPIO 由例程固件组织。
 *          任务卡 P-04 battery_voltage 的换算底座。
 *
 * @note    SOC 语义（用户需求要点）：
 *          - 电池「容量」≠ 可用能量：低电压处还有残余电荷但设备已停止工作，
 *            因此 SOC=0% 锚定在「放电截止电压 v_cutoff」，而不是 0V；
 *          - 「满电」锚定在「充电截止电压 v_full」，高于它一律 100%；
 *          - 两锚点之间按多段折线（OCV 曲线经验值）插值，默认 5 点典型
 *            锂聚合物/18650 曲线，也可自定义曲线点表。
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief SOC 折线关键点（电压 → 百分比，须单调升序） */
typedef struct {
    float volt_c;  /**< 电池电压 V */
    float soc_pct; /**< 剩余电量 %（0~100） */
} battery_curve_pt_t;

/** @brief 参数（全部可调，代码内禁零写死 magic number） */
typedef struct {
    /* --- 硬件换算 --- */
    float r_high_kohm;   /**< 分压上臂电阻 kΩ（板上 100） */
    float r_low_kohm;    /**< 分压下臂电阻 kΩ（板上 30） */
    /* --- 电池特性 --- */
    float v_full;        /**< 充电截止电压 V（满电锚点；高于此 = 100%） */
    float v_cutoff;      /**< 放电截止电压 V（SOC=0 锚点；低于此仍有残余电荷但设备已不工作） */
    float capacity_mah;  /**< 电池容量 mAh（可调参数，用户需求） */
    /* --- SOC 折线（NULL + n=0 = 使用内置默认 5 点典型曲线） --- */
    const battery_curve_pt_t *curve;
    size_t curve_n;
    /* --- 低电告警 --- */
    float v_alarm;       /**< 低电告警电压 V（可选告警阈值；>0 才启用） */
} battery_cfg_t;

/** @brief 运行实例（只缓存换算系数，保证线程/中断安全无状态） */
typedef struct {
    battery_cfg_t cfg;
    float         divider_ratio;   /**< 缓存：分压比 = (r_hi+r_lo)/r_lo */
    float         v_last;          /**< 最近一次换算的电池电压 V */
    float         soc_last;        /**< 最近一次 SOC % */
    float         mah_last;        /**< 最近一次剩余 mAh */
} battery_t;

/** @brief 命名预设（注册表项；表定义在 src/battery.c，追加预设只改那一个文件） */
typedef struct {
    const char   *name;
    battery_cfg_t cfg;
} battery_cfg_preset_t;

extern const battery_cfg_preset_t battery_presets[];
extern const size_t              battery_presets_n;

int battery_init(battery_t *b, const battery_cfg_t *cfg);

/**
 * @brief 由 ADC 原始值换算全部三量（VBAT / SOC% / 剩余 mAh）并缓存
 * @param adc_raw     ADC 原始读数（0~2^bits-1）
 * @param adc_vref_v  ADC 参考电压 V（ESP32-S3 内部 3.0~3.3 由例程喂入）
 * @param adc_bits    ADC 分辨率（ESP32-S3 12 位）
 * @return 0 成功；<0 参数错
 */
int battery_update_adc(battery_t *b, uint32_t adc_raw, float adc_vref_v, uint8_t adc_bits);

/** @brief 直接由电池电压换算（换算两量 + 缓存；用于外部已算好 VBAT 的场景） */
int battery_update_volt(battery_t *b, float v_bat);

/** @brief 由 ADC 原始值换算电池电压 V（不更新缓存，只当纯函数） */
float battery_voltage_from_adc(const battery_t *b, uint32_t adc_raw, float adc_vref_v, uint8_t adc_bits);

/** @brief 电压 → 剩余电量 %（0~100，按折线插值；纯函数） */
float battery_soc_from_volt(const battery_t *b, float v_bat);

/** @brief 剩余电量 % → 剩余 mAh（容量 × SOC/100；纯函数） */
float battery_mah_from_soc(const battery_t *b, float soc_pct);

/** @brief 低电告警判断（v_alarm ≤ 0 时恒 0） */
int battery_is_alarm(const battery_t *b);

/** @brief 查询预设（找不到返回 NULL） */
const battery_cfg_preset_t *battery_cfg_find(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */