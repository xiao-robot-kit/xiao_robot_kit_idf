/* SPDX-License-Identifier: MIT */
/**
 * @file    store_demo.c
 * @brief   pstore 独立例程会话实现（T-006/T-018）
 *
 * @note    被测对象既在固件（真板/QEMU）又在 host-SIM（testing/check_sim.sh）：
 *          本文件不含任何平台头，编译时只依赖 pstore.h。
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "store_demo.h"
#include "proto/frame.h"

/* CMD 帧最小长度（不引 cal_cmd 的内部常量；proto/frame.h 有 PROTO_CMD_HEAD_LEN=2 */

/* ───────────────────────── 小工具 ───────────────────────── */

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static float rd_f32le(const uint8_t *p)
{
    float f;
    memcpy(&f, p, sizeof(f));
    return f;
}

static void wr_f32le(uint8_t *p, float f)
{
    memcpy(p, &f, sizeof(f));
}

/* ───────────────────────── 生命周期 ───────────────────────── */

store_demo_t *store_demo_create(const store_demo_cfg_t *cfg)
{
    if (cfg == NULL || cfg->backend == NULL || cfg->capacity < 32) return NULL;

    pstore_cfg_t pc;
    memset(&pc, 0, sizeof(pc));
    pc.backend   = cfg->backend;
    pc.capacity  = cfg->capacity;
    pc.max_items = 8;
    pc.magic     = STORE_DEMO_MAGIC;

    pstore_t *ps = pstore_create(&pc);
    if (ps == NULL) return NULL;

    store_demo_t *sd = (store_demo_t *)calloc(1, sizeof(*sd));
    if (sd == NULL) {
        pstore_destroy(ps);
        return NULL;
    }
    sd->ps = ps;
    sd->capacity_kb = (cfg->capacity + 1023u) / 1024u;

    /* 注册失败（FULL/DUP/ARG）即报废：注册面是静态契约，不该失败 */
    if (pstore_register(ps, STORE_KEY_BOOT,  &sd->boot_count, sizeof(sd->boot_count)) != PSTORE_OK ||
        pstore_register(ps, STORE_KEY_GAIN,  &sd->gain,       sizeof(sd->gain))       != PSTORE_OK ||
        pstore_register(ps, STORE_KEY_FLAGS, &sd->flags,      sizeof(sd->flags))      != PSTORE_OK ||
        pstore_register(ps, STORE_KEY_NOTE,  &sd->note,       sizeof(sd->note))       != PSTORE_OK) {
        pstore_destroy(ps);
        free(sd);
        return NULL;
    }

    /* 会话镜像放在（动态分配的）store_demo_t 里，pstore 也持有同一地址 ——
       两段生命周期不同（destroy 顺序：先 destroy pstore 再 free sd） */
    sd->load_state = 0;   /* 默认：fresh */
    sd->dirty      = 0;
    sd->file_len   = 0;
    return sd;
}

void store_demo_destroy(store_demo_t *sd)
{
    if (sd == NULL) return;
    /* pstore 内部分配释放；条目地址失效后不再使用 */
    /* （ps 持有内部注册表 + stage 缓冲，先释放它，再释放会话结构体） */
    pstore_destroy(sd->ps);
    free(sd);
}

int store_demo_boot(store_demo_t *sd)
{
    if (sd == NULL) return -1;

    /* 首次默认值（与 load 前一致；load 成功后会被覆盖） */
    sd->boot_count = 0;
    sd->gain       = 0.0f;
    sd->flags      = 0;

    int rc = pstore_load(sd->ps);
    if (rc == PSTORE_OK) {
        sd->load_state = 1;            /* restored */
    } else if (rc == PSTORE_ERR_MAGIC) {
        sd->load_state = 0;            /* fresh（空白介质 → 默认值） */
    } else {
        sd->load_state = rc;           /* 其他错误（CRC/版本/后端/结构）：负数原样保留 */
    }

    sd->boot_count += 1;               /* 断电续跑：每次开机 +1 */

    /* 尾拍语义：无论 fresh/restored/err 都落盘（fresh 也把 1 写上，
       否则下次仍是空白介质 → 永远 fresh，看不到"续跑"证据） */
    pstore_mark_dirty(sd->ps);
    int s = pstore_save(sd->ps);
    sd->dirty = pstore_is_dirty(sd->ps);
    sd->file_len = pstore_frame_len(sd->ps);
    if (s != PSTORE_OK) return -2;     /* 落盘失败（容量/后端），状态已如实 */
    return 0;
}

/* ───────────────────────── 命令处理 ───────────────────────── */

static int ack_status(uint8_t mod, uint8_t func, uint8_t status,
                      const uint8_t *data, uint8_t dlen,
                      store_demo_tx_fn tx)
{
    uint8_t p[PROTO_PAYLOAD_MAX];
    if (dlen > PROTO_PAYLOAD_MAX - PROTO_ACK_HEAD_LEN) return -1;
    p[0] = mod;
    p[1] = func;
    p[2] = status;
    if (dlen > 0) memcpy(p + PROTO_ACK_HEAD_LEN, data, dlen);
    return tx(PROTO_TYPE_ACK, p, (uint8_t)(PROTO_ACK_HEAD_LEN + dlen));
}

static int store_handle_get(store_demo_t *sd, store_demo_tx_fn tx)
{
    uint8_t d[22];
    wr_u32le(d + 0,  sd->boot_count);
    wr_f32le(d + 4,  sd->gain);
    d[8]  = sd->flags;
    memcpy(d + 9, sd->note, STORE_NOTE_LEN);
    d[17] = (uint8_t)sd->load_state;
    d[18] = (uint8_t)(sd->file_len & 0xFF);
    d[19] = (uint8_t)((sd->file_len >> 8) & 0xFF);
    d[20] = sd->dirty;
    d[21] = (uint8_t)(sd->capacity_kb);
    return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_GET, PROTO_ACK_OK, d, sizeof(d), tx);
}

static int store_handle_status(store_demo_t *sd, store_demo_tx_fn tx)
{
    uint8_t d[6];
    d[0] = (uint8_t)sd->load_state;
    d[1] = (uint8_t)(sd->file_len & 0xFF);
    d[2] = (uint8_t)((sd->file_len >> 8) & 0xFF);
    d[3] = sd->dirty;
    d[4] = 4;                          /* 注册条目数（固定契约） */
    d[5] = (uint8_t)(sd->capacity_kb);
    return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_STATUS, PROTO_ACK_OK, d, sizeof(d), tx);
}

static int store_handle_set(store_demo_t *sd, const uint8_t *params, uint8_t plen,
                            store_demo_tx_fn tx)
{
    /* params = [count][(klen<32 key dlen data)×]；SET 完整体校验后才落盘 */
    if (plen < 1) {
        return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET, PROTO_ACK_BAD_ARG, NULL, 0, tx);
    }
    uint8_t count = params[0];
    size_t off = 1;
    for (uint8_t i = 0; i < count; i++) {
        if (off + 2 > plen) return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                                              PROTO_ACK_BAD_ARG, NULL, 0, tx);
        uint8_t klen = params[off++];
        if (klen >= 32 || off + klen > plen) {
            return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                              PROTO_ACK_BAD_ARG, NULL, 0, tx);
        }
        if (off + klen + 2 > plen) {
            return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                              PROTO_ACK_BAD_ARG, NULL, 0, tx);
        }
        uint8_t dlen = params[off + klen];
        if (off + klen + 1 + dlen > plen) {
            return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                              PROTO_ACK_BAD_ARG, NULL, 0, tx);
        }
        const uint8_t *key = params + off;
        const uint8_t *val = params + off + klen + 1;
        uint8_t rklen = (uint8_t)strlen(STORE_KEY_BOOT);
        if (klen == rklen && memcmp(key, STORE_KEY_BOOT, klen) == 0) {
            if (dlen != sizeof(sd->boot_count)) {
                return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                                  PROTO_ACK_BAD_ARG, NULL, 0, tx);
            }
            sd->boot_count = rd_u32le(val);
        } else if (klen == strlen(STORE_KEY_GAIN) && memcmp(key, STORE_KEY_GAIN, klen) == 0) {
            if (dlen != sizeof(sd->gain)) {
                return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                                  PROTO_ACK_BAD_ARG, NULL, 0, tx);
            }
            sd->gain = rd_f32le(val);
        } else if (klen == strlen(STORE_KEY_FLAGS) && memcmp(key, STORE_KEY_FLAGS, klen) == 0) {
            if (dlen != sizeof(sd->flags)) {
                return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                                  PROTO_ACK_BAD_ARG, NULL, 0, tx);
            }
            sd->flags = *val;
        } else if (klen == strlen(STORE_KEY_NOTE) && memcmp(key, STORE_KEY_NOTE, klen) == 0) {
            if (dlen != STORE_NOTE_LEN) {
                return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                                  PROTO_ACK_BAD_ARG, NULL, 0, tx);
            }
            memcpy(sd->note, val, STORE_NOTE_LEN);
        } else {
            return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                              PROTO_ACK_BAD_ARG, NULL, 0, tx);
        }
        off += klen + 1 + dlen;
    }
    pstore_mark_dirty(sd->ps);
    int s = pstore_save(sd->ps);
    sd->dirty = pstore_is_dirty(sd->ps);
    sd->file_len = pstore_frame_len(sd->ps);
    if (s != PSTORE_OK) {
        return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET,
                          PROTO_ACK_EXEC_FAIL, NULL, 0, tx);
    }
    return ack_status(PROTO_MOD_STORE, PROTO_STORE_FUNC_SET, PROTO_ACK_OK, NULL, 0, tx);
}

static int store_handle_erase(store_demo_t *sd, store_demo_tx_fn tx)
{
    /* 本版不提供 ERASE（见头文件注记）：pstore 库缺介质级擦除 API。
       仅保留签名以对应协议表；实际由 handle 的 UNKNOWN_FUNC 路径覆盖。 */
    (void)sd;
    (void)tx;
    return -1;
}

int store_demo_handle(store_demo_t *sd, const uint8_t *payload, uint8_t len,
                      store_demo_tx_fn tx)
{
    if (sd == NULL || tx == NULL || payload == NULL || len < PROTO_CMD_HEAD_LEN) return -1;
    if (payload[0] != PROTO_MOD_STORE) return -1;   /* 不是本模块的命令 */
    switch (payload[1]) {
    case PROTO_STORE_FUNC_GET:
        return store_handle_get(sd, tx);
    case PROTO_STORE_FUNC_STATUS:
        return store_handle_status(sd, tx);
    case PROTO_STORE_FUNC_SET:
        return store_handle_set(sd, payload + 2, (uint8_t)(len - 2), tx);
    case PROTO_STORE_FUNC_ERASE:
        /* 本版不提供 ERASE（pstore 库缺介质级擦除 API）→ 诚实回 UNKNOWN_FUNC */
        return ack_status(PROTO_MOD_STORE, payload[1], PROTO_ACK_UNKNOWN_FUNC, NULL, 0, tx);
    default:
        return ack_status(PROTO_MOD_STORE, payload[1], PROTO_ACK_UNKNOWN_FUNC, NULL, 0, tx);
    }
}

int store_demo_rx(store_demo_t *sd, const uint8_t *buf, size_t n,
                  store_demo_tx_fn tx)
{
    if (sd == NULL || buf == NULL || tx == NULL) return -1;
    for (size_t i = 0; i < n; i++) {
        const uint8_t b = buf[i];
        if (sd->rx_len + 1u > (uint16_t)sizeof(sd->rx)) {  /* 溢出护栏（不可达） */
            sd->rx_len = 0;
            continue;
        }
        sd->rx[sd->rx_len++] = b;

        for (;;) {  /* 一次可弹多帧（粘包） */
            if (sd->rx_len < 4) break;                     /* 头部不足 */
            if (sd->rx[0] != PROTO_SOF) {                  /* 假 SOF｜重同步 */
                memmove(sd->rx, sd->rx + 1, sd->rx_len - 1);
                sd->rx_len--;
                continue;
            }
            const uint8_t payload_len = sd->rx[1];
            if (payload_len > PROTO_PAYLOAD_MAX) {         /* 超限 LEN｜重同步 */
                memmove(sd->rx, sd->rx + 1, sd->rx_len - 1);
                sd->rx_len--;
                continue;
            }
            const uint16_t total = (uint16_t)(4u + payload_len + 1u);
            if (sd->rx_len < total) break;                 /* 等更多字节 */
            const uint8_t crc = proto_crc8(sd->rx + 1, (size_t)(3u + payload_len));
            if (crc != sd->rx[4 + payload_len]) {          /* CRC 错｜重同步 */
                memmove(sd->rx, sd->rx + 1, sd->rx_len - 1);
                sd->rx_len--;
                continue;
            }
            /* 完整合法帧 → 分发（非 CMD / MOD 不匹配 → handle 内部吞掉） */
            if (sd->rx[2] == PROTO_TYPE_CMD) {
                store_demo_handle(sd, sd->rx + 4, payload_len, tx);
            }
            memmove(sd->rx, sd->rx + total, sd->rx_len - total);
            sd->rx_len = (uint16_t)(sd->rx_len - total);
            break;
        }
    }
    return 0;
}

/* ───────────────────────── 日志 ───────────────────────── */

int store_demo_text(const store_demo_t *sd, char *buf, int cap)
{
    if (sd == NULL || buf == NULL || cap <= 0) return 0;
    char note[STORE_NOTE_LEN + 1];
    memcpy(note, sd->note, STORE_NOTE_LEN);
    note[STORE_NOTE_LEN] = '\0';
    /* 逐字节 ASCII 转义（note 是原始字节，可能非可打印） */
    int n = snprintf(buf, cap,
                     "store boot=%u gain=%.3f flags=0x%02X note=%s "
                     "load=%d len=%u dirty=%u",
                     (unsigned)sd->boot_count, (double)sd->gain, sd->flags,
                     note,
                     sd->load_state, (unsigned)sd->file_len, sd->dirty);
    return (n < 0) ? 0 : n;
}