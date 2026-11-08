/* SPDX-License-Identifier: MIT */
#ifndef ZERO_CAL_H
#define ZERO_CAL_H

#include <stddef.h>
#include <stdint.h>
#include "imu/imu_types.h"
#include "cal/imu_calib.h"

/**
 * @file    zero_cal.h
 * @brief   零点校准（零偏标定）口径：N 帧静止均值 → 去偏 → 0x02/TEXT 输出契约
 *
 * @note    同源同口径：算法与 exp2（examples/imu/exp2_calib/main/exp2_main.c
 *          的 `calibrate_bias()`）逐段同构——预设 "cal.exp2.bias"（N=500）、
 *          imu_calib 均值法 + 重力轴判定（bias = mean − expected，重力轴减
 *          sign·1g、其余轴减 0）、喂样时 raw 传 NULL（无已知零偏时用样本自身
 *          判重力轴，重力方向是物理事实，不由补偿量决定）。本模块只复用该
 *          口径 + 输出格式单源，不引入新算法、不改重力约定。
 * @note    触发纪律：只走**显式触发**——开机标定一次 + 运行期收到 `CAL` 行
 *          命令重标定；无隐式回落、无自动重标定。
 * @note    本文件在 exp3/exp4/exp5 三工程**逐字同源**（单 app 阶段零组件框架，
 *          同 att_cfg_presets.c 模式）：改一处必须三处同步。
 */

/** 0x02 IMU_CAL 帧负载：19×float32 LE（raw6 | deb6 | bias6 | temp），布局单源
 *  = proto/frame.h PROTO_TYPE_IMU_CAL 注释（零偏块为采样序 ax..gz，
 *  与 imu_bias_t 的字段序 gx..gz,ax..az 不同——打包侧按采样序重排） */
#define ZERO_CAL_FRAME_FLOATS 19
#define ZERO_CAL_FRAME_BYTES  (ZERO_CAL_FRAME_FLOATS * 4)

/** TEXT 行缓冲上限（"OK calib done bias …" 满幅 96 字符量级；取 128 留余量） */
#define ZERO_CAL_TEXT_CAP 128

/** 标定进度行间隔（帧）：每满这么多帧发一行 TEXT——标定窗内不静默 */
#define ZERO_CAL_PROGRESS_EVERY 100

/**
 * @brief 标定会话（状态全部实例内，无全局可变态；重标定即新会话）
 *
 * @note target_n 从预设拷贝：进度行分母不依赖调用方另行保存配置。
 */
typedef struct {
    imu_calib_t cal;       /**< 标定算法实例（累加/重力轴判定状态） */
    uint32_t    target_n;  /**< 目标样本数（= 预设 target_n） */
} zero_cal_t;

/**
 * @brief   开启一次标定会话（初值 + 开始采集；清全部累加）
 * @retval  0  已进入采集中
 * @retval -1  z/cfg 为 NULL 或 cfg->target_n < 1
 */
int zero_cal_begin(zero_cal_t *z, const imu_calib_cfg_t *cfg);

/**
 * @brief   喂入一帧**未去偏原始样本**（本会话按 exp2 口径：无已知零偏）
 * @retval  0  已接受（凑满与否都算成功，完成查 zero_cal_done）
 * @retval -1  z 为 NULL 或未处于采集状态
 */
int zero_cal_feed(zero_cal_t *z, const imu_sample_t *raw);

/**
 * @brief   本轮是否已采满
 * @retval  1 已完成（可取结果）／0 未完成／-1 z 为 NULL
 */
int zero_cal_done(const zero_cal_t *z);

/**
 * @brief   读进度（供进度行文本使用；实测值，不做判据）
 * @param[out] n       已采集帧数
 * @param[out] target  目标帧数
 */
void zero_cal_progress(const zero_cal_t *z, uint32_t *n, uint32_t *target);

/**
 * @brief   取本轮零偏（重力轴已按 exp2 口径减 sign·1g）
 * @retval  0 已写入；-1 z/bias 为 NULL 或未就绪（显式拒绝部分均值）
 */
int zero_cal_result(const zero_cal_t *z, imu_bias_t *bias);

/**
 * @brief   去偏：deb = raw − bias（六轴同式；重力轴因 bias 已减 1g，
 *          deb 的该轴回到 ±1g 真值——与 exp2 直出通道同口径）
 */
void zero_cal_apply(const imu_bias_t *bias, const imu_sample_t *raw,
                    imu_sample_t *deb);

/**
 * @brief   0x02 帧打包：19×float32 LE（raw6|deb6|bias6|temp），同 exp2 pack_cal
 * @param[out] out  ZERO_CAL_FRAME_BYTES 字节
 */
void zero_cal_pack_frame(const imu_sample_t *raw, const imu_sample_t *deb,
                         const imu_bias_t *bias, float temp, uint8_t *out);

/**
 * @brief   TEXT 行格式（固件发送与 host 自检断言共用同一单源）
 * @return  写入字符数（不含 NUL）；0 = 缓冲不足
 *
 * @note    固定格式行（上位机按行解析，禁改字段序）：
 *          - start   ：`OK calib start N=500`
 *          - progress：`OK calib progress <n>/<N>`
 *          - done    ：`OK calib done bias ax=%+.4f ay=%+.4f az=%+.4f gx=%+.4f
 *                       gy=%+.4f gz=%+.4f temp=%.1f`
 */
int zero_cal_text_start(char *buf, size_t cap, uint32_t n);
int zero_cal_text_progress(char *buf, size_t cap, uint32_t n, uint32_t target);
int zero_cal_text_done(char *buf, size_t cap, const imu_bias_t *bias, float temp);

#endif /* ZERO_CAL_H */
