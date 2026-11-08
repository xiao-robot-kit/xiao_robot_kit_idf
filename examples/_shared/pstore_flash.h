/* SPDX-License-Identifier: MIT */
/**
 * @file    pstore_flash.h（port/esp_idf）
 * @brief   pstore 的裸 flash 分区后端（esp_partition 直写，无 NVS 中间层）
 *
 * @note    本项目第二个 pstore 介质后端，与 pstore_nvs 并列：
 *          - pstore_nvs：键值 blob，NVS 自带磨损均衡/掉电回滚/加密，接口经 nvs_*；
 *          - pstore_flash：独占一个自定义 data 分区，esp_partition_read/write 直写，
 *            擦除按扇区（分区对齐），**不提供磨损均衡**——期望寿命取决于分区擦写次数，
 *            适合演示/教学（T-006）与"介质行为可见"的实验场景。
 * @note    契约：读整个分区前 capacity 字节；写一整帧；erase 掉整个分区。
 *          断电窗口（erase-write 之间 / write 中途掉电）→ 介质无有效帧或半帧
 *          → pstore_load 报 ERR_MAGIC / ERR_CHECKSUM → 调用方走默认值路径
 *          （可接受降级，与组件 README「断电语义」一致）。
 * @note    空白介质语义：出厂 flash 全 0xFF → 首载即 ERR_MAGIC（正路径，
 *          非后端故障）；无需像 NVS 那样做 NOT_FOUND→0xFF 归一。
 */

#ifndef PORT_ESP_IDF_PSTORE_FLASH_H
#define PORT_ESP_IDF_PSTORE_FLASH_H

#include <stddef.h>
#include <stdint.h>
#include "esp_partition.h"
#include "pstore.h"

/** @brief flash 分区后端实例（ctx = 本结构） */
typedef struct {
    const esp_partition_t *part;   /**< data 分区（label "store"） */
} pstore_flash_t;

/**
 * @brief 绑定 flash 分区后端
 *
 * @param[out] be     待填充的后端操作表（read/write/erase + ctx）
 * @param[out] flash  后端实例
 * @param[in]  part   分区句柄（须 data 型；分区大小 ≥ capacity 由调用方保证）
 *
 * @retval 0 成功；-1 参数非法（NULL）
 * @note   esp_partition 是线程安全的，无需加锁；调用链由 pstore 同步驱动。
 */
int pstore_flash_backend_init(pstore_backend_t *be, pstore_flash_t *flash,
                              const esp_partition_t *part);

/**
 * @brief 按分区表 label 查找并绑定（便捷入口）
 *
 * @param[out] be     后端操作表
 * @param[out] flash  后端实例
 * @param[in]  label  分区 label（如 "store"）；NULL = 取第一个 data 分区
 *
 * @retval 0 成功；-1 参数非法；-2 分区未找到
 */
int pstore_flash_open(pstore_backend_t *be, pstore_flash_t *flash,
                      const char *label);

#endif /* PORT_ESP_IDF_PSTORE_FLASH_H */