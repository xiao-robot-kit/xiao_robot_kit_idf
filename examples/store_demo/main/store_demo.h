/* SPDX-License-Identifier: MIT */
/**
 * @file    store_demo.h
 * @brief   pstore 独立例程（T-006/T-018）：演示参数持久化全链路
 *
 * @note    本模块是**纯会话逻辑**（不包含任何平台 IO）：pstore 条目注册、
 *          boot_count 断电续跑、命令帧处理（MOD_STORE）——与 exp2 的
 *          cal_cmd 同构但独立于 IMU 算法链，作为 pstore 库的"裸演示"。
 *
 * @note    依赖单向：本模块 → pstore.h（vendor）→ 后端（平台注入）。
 *          后端由 app 层选择：真板/QEMU = esp_partition flash 后端；
 *          host-SIM 自检 = RAM 后端（testing/check_sim.sh）。
 */

#ifndef STORE_DEMO_H
#define STORE_DEMO_H

#include <stdint.h>
#include <stddef.h>
#include "pstore/pstore.h"
#include "proto/frame.h"   /* PROTO_FRAME_MAX / PROTO_MOD_STORE / PROTO_ACK_* */

/* 存储区.STORE 应用身份魔数（避开 0xFFFFFFFF；load 用它区分空白/异种介质） */
#define STORE_DEMO_MAGIC     0x53545231u   /* "STR1" */

/* 条目键 */
#define STORE_KEY_BOOT       "boot"
#define STORE_KEY_GAIN       "gain"
#define STORE_KEY_FLAGS      "flags"
#define STORE_KEY_NOTE       "note"

/* 注意：note 定长 8，无 NUL 约定（与协议 ACK data 布局一致） */
#define STORE_NOTE_LEN       8

/* 会话状态（load 结果 + 镜像条目值） */
typedef struct {
    /* —— 内部实例（不透明，destroy 时先销毁） —— */
    pstore_t *ps;             /**< pstore 实例（create 时分配） */
    uint32_t  capacity_kb;    /**< 存储区容量（KB，STATUS 上报用） */

    /* —— 镜像条目（与 pstore 注册地址一致，load 直接写入） —— */
    uint32_t boot_count;      /**< 断电续跑计数器：每次开机 +1 落盘 */
    float    gain;            /**< 示例 float 参数 */
    uint8_t  flags;           /**< 示例位集参数 */
    uint8_t  note[STORE_NOTE_LEN];   /**< 示例字符串参数 */

    /* —— 会话（load 后定格，供 /STATUS 与日志） —— */
    int      load_state;      /**< 0=fresh 首载默认 / 1=restored 恢复存档 / <0 pstore 错误码 */
    uint32_t file_len;        /**< 最近存档帧长（pstore_frame_len） */
    uint8_t  dirty;           /**< 1=有未落盘变更 */

    /* —— RX 组帧缓冲（字节流 → 帧；容错契约见 store_demo_rx） —— */
    uint8_t  rx[PROTO_FRAME_MAX];
    uint16_t rx_len;
} store_demo_t;

/* ---- 后端注入（app 层填好 backend + 存储区容量） ---- */
typedef struct {
    const pstore_backend_t *backend;
    uint32_t                capacity;   /* 存储区字节数（create 传入） */
} store_demo_cfg_t;

typedef int (*store_demo_tx_fn)(uint8_t type, const uint8_t *payload,
                                uint8_t len);

/**
 * @brief 创建会话（分配 pstore 实例 + 注册条目；不读介质）
 * @param cfg  backend/capacity 必填
 * @return 实例；NULL=参数非法或内存不足
 */
store_demo_t *store_demo_create(const store_demo_cfg_t *cfg);

/**
 * @brief 上电加载 + 断电续跑
 *
 * 语义（被 host-SIM 断言锁定）：
 *  - 空白介质（PSTORE_ERR_MAGIC）→ 条目保持默认值，load_state=0（fresh）
 *  - 存档健康（PSTORE_OK）→ 条目被回填，load_state=1（restored）
 *  - 其他错误（CRC/后端/版本/结构）→ 条目保持默认值，load_state=错误码原值（负数）
 *  - 无论哪种，boot_count += 1 并落盘（最后一次 save 失败仍如实报 load_state）
 *
 * @return 0 成功；-1 参数非法；-2 落盘失败（load 本身成功但 save 失败）
 */
int store_demo_boot(store_demo_t *sd);

/**
 * @brief 处理一帧 CMD（payload = [MOD 0x20][FUNC][params...]），按需回 ACK
 *
 * 支持的 FUNC（PROTO_STORE_FUNC_*）：
 *  - GET(0x01)     ACK data = 22B 布局见 proto/frame.h
 *  - SET(0x02)     改写条目并落盘；params=[count][(klen<32 key dlen data)×]
 *  - STATUS(0x04)  ACK data = 6B 布局见 proto/frame.h
 *  - **RESET（复位归零）**：入参 count=1、[boot,4,0x00000000]，即 SET 的 boot=0
 *    落盘（note/其他参数可一并改）——**本版不提供 ERASE**（pstore 库缺
 *    `pstore_media_erase` API，vendor 只读消费不可在本仓补；介质级清空已登记
 *    为下一步，见 README）。
 * 其他 FUNC → ACK UNKNOWN_FUNC（0xFF）；MOD 不匹配或参数错 → 不回 ACK（-1）。
 *
 * @return 0 已处理；-1 非本模块（MOD 不匹配或参数非法）
 */
int store_demo_handle(store_demo_t *sd, const uint8_t *payload, uint8_t len,
                      store_demo_tx_fn tx);

/**
 * @brief 字节流入口（组帧 + 分发；等价 cal_cmd_rx 但只服务本模块帧）
 *
 * @param buf 收到的一批字节（可含多帧/半帧/非 CMD 帧）
 * @retval  0  正常
 * @retval -1  参数非法
 *
 * @note  容错契约与 IMU 解码器一致：假 SOF / 超限 LEN / CRC 错 → 丢一字节重同步。
 *        非 CMD 帧（如回环的数据帧）静默丢弃。
 */
int store_demo_rx(store_demo_t *sd, const uint8_t *buf, size_t n,
                  store_demo_tx_fn tx);

/** @brief 渲染一行 TEXT 状态（调试/日志用；写入 buf，返回长度） */
int store_demo_text(const store_demo_t *sd, char *buf, int cap);

/** @brief 销毁会话（不触碰介质） */
void store_demo_destroy(store_demo_t *sd);

#endif /* STORE_DEMO_H */