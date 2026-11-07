/* SPDX-License-Identifier: MIT */
/**
 * @file    pstore_flash.c（port/esp_idf）
 * @brief   pstore 的裸 flash 分区后端实现（esp_partition 直写）
 *
 * @note    实现分三面：
 *          1) read  — esp_partition_read（分区地址 0 起，编址偏移零的"存储区"就是分区头）；
 *          2) write — esp_partition_write（pstore 保证先 erase 后 write 的调用序）；
 *          3) erase — esp_partition_erase_range 整个分区（分区尺寸对齐扇区）。
 * @note    与 pstore_nvs 的差异在 README/flash 头注释：无磨损均衡、掉电窗口直接可见。
 */

#include "pstore_flash.h"

/* ---------------------------------------------------------------- 后端操作 */

static int flash_read_op(void *ctx, void *buf, size_t len)
{
    pstore_flash_t *f = (pstore_flash_t *)ctx;
    if (f == NULL || f->part == NULL || buf == NULL || len == 0) return -1;
    if (len > f->part->size) return -1;               /* 分区比请求容量小：无法安全读 */
    return (esp_partition_read(f->part, 0, buf, len) == ESP_OK) ? 0 : -1;
}

static int flash_write_op(void *ctx, const void *buf, size_t len)
{
    pstore_flash_t *f = (pstore_flash_t *)ctx;
    if (f == NULL || f->part == NULL || buf == NULL || len == 0) return -1;
    if (len > f->part->size) return -1;
    return (esp_partition_write(f->part, 0, buf, len) == ESP_OK) ? 0 : -1;
}

static int flash_erase_op(void *ctx, size_t len)
{
    (void)len;   /* 擦整个分区（分区大小对齐扇区，erase_range 要求偏移/长度对齐 4KB） */
    pstore_flash_t *f = (pstore_flash_t *)ctx;
    if (f == NULL || f->part == NULL) return -1;
    const esp_err_t err = esp_partition_erase_range(f->part, 0, f->part->size);
    return (err == ESP_OK) ? 0 : -1;
}

/* ---------------------------------------------------------------- 生命周期 */

int pstore_flash_backend_init(pstore_backend_t *be, pstore_flash_t *flash,
                              const esp_partition_t *part)
{
    if (be == NULL || flash == NULL || part == NULL) return -1;
    flash->part = part;
    be->read  = flash_read_op;
    be->write = flash_write_op;
    be->erase = flash_erase_op;
    be->ctx   = flash;
    return 0;
}

int pstore_flash_open(pstore_backend_t *be, pstore_flash_t *flash,
                      const char *label)
{
    if (be == NULL || flash == NULL) return -1;
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, label);
    if (part == NULL) return -2;
    return pstore_flash_backend_init(be, flash, part);
}