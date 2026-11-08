/* SPDX-License-Identifier: MIT */
/**
 * @file    pstore_nvs.c（port/esp_idf）
 * @brief   pstore 的 NVS 介质后端实现（read/write/erase → nvs_get_blob/set_blob/erase_key）
 *
 * @note    契约与设计决策全在 pstore_nvs.h；本文件只做 API 映射与错误归一。
 * @note    空白介质的表达：键不存在 → 输出缓冲填 0xFF 并**返回成功**。
 *          不能返回失败：pstore 会把后端失败报成 `PSTORE_ERR_BACKEND`（介质坏了），
 *          而"首次上电没有存档"是正路径，必须走 `PSTORE_ERR_MAGIC`（→ 默认值）。
 */

#include <string.h>
#include "nvs_flash.h"
#include "pstore_nvs.h"

/* NVS 命名空间与键名上限（ESP-IDF 契约：NVS_KEY_NAME_MAX_SIZE = 16 含结尾 0） */
#define PSTORE_NVS_NAME_MAX 15u

/* ---------------------------------------------------------------- 后端操作 */

/**
 * @brief 读整个介质
 *
 * @note  `nvs_get_blob` 的 `len` 入参是缓冲容量、出参是实际长度；
 *        实际长度小于请求长度时尾部补 0xFF（等价"未写过的字节"），
 *        这样 pstore 看到的永远是一块定长介质。
 */
static int nvs_read_op(void *ctx, void *buf, size_t len)
{
    pstore_nvs_t *n = (pstore_nvs_t *)ctx;
    if (n == NULL || buf == NULL || len == 0) return -1;

    size_t sz = len;
    const esp_err_t err = nvs_get_blob(n->h, n->key, buf, &sz);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        memset(buf, 0xFF, len);   /* 空白介质：首载 → PSTORE_ERR_MAGIC → 默认值路径 */
        return 0;
    }
    if (err != ESP_OK) return -1;
    if (sz > len) return -1;      /* 存档比本固件的介质容量大：无法安全读入 */
    if (sz < len) memset((uint8_t *)buf + sz, 0xFF, len - sz);
    return 0;
}

/**
 * @brief 写一帧（先 erase 后 write 的调用序由 pstore 保证）
 *
 * @note  **同步提交**：`nvs_commit` 返回成功才报成功——pstore 的
 *        `PSTORE_OK` 语义必须等于"已经落盘"，否则上层日志会撒谎。
 */
static int nvs_write_op(void *ctx, const void *buf, size_t len)
{
    pstore_nvs_t *n = (pstore_nvs_t *)ctx;
    if (n == NULL || buf == NULL || len == 0) return -1;
    if (nvs_set_blob(n->h, n->key, buf, len) != ESP_OK) return -1;
    return (nvs_commit(n->h) == ESP_OK) ? 0 : -1;
}

/** @brief 擦除介质（键不存在 = 已经是空介质，幂等成功） */
static int nvs_erase_op(void *ctx, size_t len)
{
    (void)len;   /* NVS 以"键"为粒度，无按字节擦除概念 */
    pstore_nvs_t *n = (pstore_nvs_t *)ctx;
    if (n == NULL) return -1;
    const esp_err_t err = nvs_erase_key(n->h, n->key);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return -1;
    return (nvs_commit(n->h) == ESP_OK) ? 0 : -1;
}

/* ---------------------------------------------------------------- 生命周期 */

int pstore_nvs_backend_init(pstore_backend_t *be, pstore_nvs_t *nvs,
                            const char *ns_name, const char *key)
{
    if (be == NULL || nvs == NULL || ns_name == NULL || key == NULL) return -1;
    if (ns_name[0] == '\0' || strlen(ns_name) > PSTORE_NVS_NAME_MAX) return -1;
    if (key[0] == '\0' || strlen(key) > PSTORE_NVS_NAME_MAX) return -1;

    /* 常规首次上电路径：无空闲页 / 旧版本 → 擦一次再初始化（NVS 官方推荐做法） */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) return -2;
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return -2;

    if (nvs_open(ns_name, NVS_READWRITE, &nvs->h) != ESP_OK) return -3;
    nvs->key = key;

    be->read  = nvs_read_op;
    be->write = nvs_write_op;
    be->erase = nvs_erase_op;
    be->ctx   = nvs;
    return 0;
}

void pstore_nvs_backend_deinit(pstore_backend_t *be, pstore_nvs_t *nvs)
{
    if (nvs != NULL) {
        nvs_close(nvs->h);
        nvs->key = NULL;
    }
    if (be != NULL) {
        be->read = NULL;
        be->write = NULL;
        be->erase = NULL;
        be->ctx = NULL;
    }
}
