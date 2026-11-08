/* SPDX-License-Identifier: MIT */
#ifndef ESP_PSTORE_NVS_H
#define ESP_PSTORE_NVS_H

#include <stddef.h>
#include <stdint.h>
#include "nvs.h"
#include "pstore/pstore.h"

/**
 * @file    pstore_nvs.h（port/esp_idf）
 * @brief   参数持久化的**介质后端**：pstore 帧落在 ESP-IDF NVS 的一个 blob 里
 *
 * @note    分层（依赖单向，MEM-4）：
 *          ```
 *          实验 app → cal_cmd（会话） → pstore（帧/CRC/两阶段原子） → 本后端 → NVS 分区 → flash
 *          ```
 *          pstore 核心全程不 include 任何平台头；平台细节全部收在本文件与 .c 里
 *          （本目录是固件侧唯一允许出现 esp_xxx 平台头的地方之一）。
 *
 * @note    **为什么用 NVS 而不是裸分区**（重组决策，替代原型直写 flash 的做法）：
 *          1. 掉电语义更强：NVS 的写入是「先写新页、再更新索引」，掉电回滚到**旧值**
 *             （而裸分区 erase→write 窗口内断电 → 介质无有效帧）。pstore 的
 *             "断电 → 走默认值" 路径因此在本后端下退化为罕见的极端情况；
 *          2. 磨损均衡/坏块管理由 NVS 提供 —— 正是 xiao 组件库 `pstore` README
 *             里登记为"port 层升级路径"的那部分能力，核心组件因此无须改动；
 *          3. 不需要新分区表：直接用现有 `nvs` 分区（app 与 NVS 各自独立，
 *             NVS 的键名空间天然隔离别的模块数据）。
 *          代价：单 blob 上限受 NVS 分区大小约束（本用例帧 ~200B，边距充裕）；
 *          写入比裸 flash 慢（NVS 页管理），但校准参数只在命令完成时写一次，不在采样环内。
 *
 * @note    **与 pstore 契约的映射**：
 *          - `read`  = `nvs_get_blob`；键不存在（空白介质/首次上电）→ 填 0xFF 并返回成功
 *                      （= pstore RAM 后端的"空白介质"语义 → 核心报 `PSTORE_ERR_MAGIC`
 *                      走默认值路径）。读到的实际长度小于请求长度时，尾部同样填 0xFF。
 *          - `write` = `nvs_set_blob` + `nvs_commit`（**同步提交**：返回值 0 即已落盘）
 *          - `erase` = `nvs_erase_key`（键不存在视为成功：擦除幂等）
 *          NVS 自身没有"字节可覆盖写"概念，`erase` 在语义上是"丢掉整个值"，
 *          与 pstore "先 erase 后 write 一帧"的调用序天然吻合。
 *
 * @note    线程安全：NVS API 自身可重入；pstore 非线程安全（核心契约），
 *          本后端不额外加锁——调用方须保证同一 `pstore_t` 不被多任务并发使用。
 */

/**
 * @brief NVS 后端实例（一个 blob = 一份 pstore 介质）
 *
 * @note  `key` 指向调用方持有的字符串（生命周期须覆盖使用期）。
 */
typedef struct {
    nvs_handle_t h;      /**< 已打开的 NVS 命名空间句柄 */
    const char  *key;    /**< blob 键名（一份介质一个键） */
} pstore_nvs_t;

/**
 * @brief   打开 NVS 命名空间并填充 pstore 后端操作表
 *
 * @param[out] be       后端操作表（成功时填 read/write/erase/ctx）
 * @param[out] nvs      后端实例
 * @param[in]  ns_name  NVS 命名空间名（≤ 15 字符，如 "calstore"）
 * @param[in]  key      blob 键名（≤ 15 字符，如 "cal.frame"）
 *
 * @retval  0   成功（后端可用）
 * @retval -1   参数非法
 * @retval -2   `nvs_flash_init` 失败（重试擦除后仍失败 → 分区不可用）
 * @retval -3   `nvs_open` 失败
 *
 * @note    `nvs_flash_init` 在本函数内完成（含首批上电的 `NO_FREE_PAGES` /
 *          `NEW_VERSION_FOUND` → `nvs_flash_erase` + 重试的常规路径），
 *          调用方不必自行初始化 NVS。
 * @note    失败时 `be` 保持调用前内容、`nvs.h` 无效——调用方应降级为"无持久化运行"
 *          （功能全在，只是每次开机重新标定），**不要**因此中止实验固件。
 */
int pstore_nvs_backend_init(pstore_backend_t *be, pstore_nvs_t *nvs,
                            const char *ns_name, const char *key);

/**
 * @brief 关闭后端（`nvs_close`；不删数据）
 */
void pstore_nvs_backend_deinit(pstore_backend_t *be, pstore_nvs_t *nvs);

#endif /* ESP_PSTORE_NVS_H */
