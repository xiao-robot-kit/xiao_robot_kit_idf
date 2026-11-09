/* SPDX-License-Identifier: MIT */
#ifndef PSTORE_PSTORE_H
#define PSTORE_PSTORE_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file    pstore.h
 * @brief   参数持久化组件：条目注册表 + 统一帧序列化 + CRC32 + 两阶段原子加载
 *
 * @note    提炼自旧式实现自有实现 旧式实现：
 *          - `embedded/src/shared/pstore/pstore.c`（序列化 / CRC / 条目注册——重组）
 *          - `embedded/examples/_shared/pstore/pstore.c`（flash 后端思路——平台件分层）
 *          重组非照搬：句柄实例化 + 参数零写死 + 后端带 ctx，帧格式自 v1 起算
 *          （与旧式实现格式不兼容，那是封存树的资产，按 INT-5 单向蒸馏）。
 *
 * @note    解决的问题：多个模块各自往持久化介质写会互相覆盖、各自管格式。
 *          本组件把所有参数条目收进一张注册表，一次性序列化进一个帧，
 *          集中做校验与脏检查落盘。
 *
 * @note    序列化帧格式（v1，主机字节序，字段经 memcpy 原序落盘）：
 *          ```
 *          offset  0: magic    u32   （应用身份，cfg 传入）
 *          offset  4: version  u32   （帧格式版本，库常量 PSTORE_FORMAT_VERSION）
 *          offset  8: count    u32   （帧内条目数）
 *          offset 12: data_len u32   （数据区长度，不含 header）
 *          offset 16: crc32    u32   （覆盖 offset 8..16 的 count+data_len 两字段 + 数据区）
 *          offset 20: item × count: klen(2) key(klen) dlen(2) data(dlen)
 *          ```
 *          @note CRC 覆盖 count 与 data_len（旧式实现雷已修【medium→雷3】：
 *          旧 header CRC 只盖数据区，count 无保护——见 load 注记）。
 *          @note magic/version 是身份字段，不进 CRC：不匹配按
 *          ERR_MAGIC/ERR_VERSION 上报（首次上电/格式演进的正路径），不按损坏处理。
 *
 * @note    两阶段原子加载（旧式实现雷已修【medium→雷1】）：
 *          旧 `pstore_load` 注释称「失败不修改 ram」，但解析中途遇 CORRUPT 返回时，
 *          之前已匹配的条目已被 memcpy 覆盖——注释与实现失实
 *          （旧式实现 embedded/src/shared/pstore/pstore.c:234-269）。
 *          本版 load 严格两阶段：**阶段一全量读入内部缓冲、逐条边界校验并
 *          生成应用计划，任一步失败立即返回、ram 零字节改动；阶段二才把
 *          计划一次性 memcpy 应用**。加载失败 = 帧不可用 = 调用方走默认值路径。
 *
 * @note    旧式实现雷已修【low→雷2】：旧 PSTORE_ERR_OVERFLOW 无产生路径、
 *          `pstore_clear_fault` 全仓零调用——本版错误码每个都有产生路径且
 *          被单测覆盖，无死接口（容量超限 = ERR_NOFIT，真实可测）。
 */

/** @brief 帧格式版本（v1 自本组件起算；变更帧布局必须 bump 并处理旧帧 ERR_VERSION 路径） */
#define PSTORE_FORMAT_VERSION       1u

/** @brief 帧头字节数（5 × u32：magic/version/count/data_len/crc） */
#define PSTORE_FRAME_HEADER_BYTES   20u

/**
 * @brief 帧头（对外公开以便编译期断言尺寸；字段按上表顺序 memcpy 落盘，主机字节序）
 */
typedef struct {
    uint32_t magic;     /**< 应用身份（cfg.magic） */
    uint32_t version;   /**< 帧格式版本（PSTORE_FORMAT_VERSION） */
    uint32_t count;     /**< 帧内条目数（在 CRC 覆盖范围内） */
    uint32_t data_len;  /**< 数据区长度，字节（在 CRC 覆盖范围内） */
    uint32_t crc;       /**< CRC32（覆盖 count+data_len 两字段与数据区） */
} pstore_frame_hdr_t;

/* 【编译期】帧头尺寸必须与对外宏一致（防结构体改动后帧布局静默漂移） */
_Static_assert(sizeof(pstore_frame_hdr_t) == PSTORE_FRAME_HEADER_BYTES,
               "pstore_frame_hdr_t size changed — update PSTORE_FRAME_HEADER_BYTES");

/**
 * @brief 加载/保存错误码
 *
 * @note 每个码都有真实产生路径且被单测覆盖（雷2 修复：无死码）。
 */
typedef enum {
    PSTORE_OK            = 0,   /**< 成功 */
    PSTORE_ERR_ARG       = -1,  /**< 参数非法（NULL / len==0 / key 空或超 65535） */
    PSTORE_ERR_FULL      = -2,  /**< 注册表满（max_items 用尽） */
    PSTORE_ERR_DUP       = -3,  /**< 重复注册同 key */
    PSTORE_ERR_NOTFOUND  = -4,  /**< unregister 未找到该 key */
    PSTORE_ERR_NOFIT     = -5,  /**< 序列化帧超出 capacity（真实溢出路径，测试覆盖） */
    PSTORE_ERR_BACKEND   = -6,  /**< 后端 read/write/erase 失败 */
    PSTORE_ERR_MAGIC     = -7,  /**< magic 不匹配（空白介质/异种数据 → 走默认值路径） */
    PSTORE_ERR_VERSION   = -8,  /**< 帧版本不匹配（旧固件数据/格式演进） */
    PSTORE_ERR_CHECKSUM  = -9,  /**< CRC32 校验失败（含 count/data_len 篡改） */
    PSTORE_ERR_CORRUPT   = -10, /**< 帧结构损坏（越界/解析末尾不对齐/条目数与 count 不符） */
} pstore_err_t;

/* ───────────────────────── 存储后端契约 ───────────────────────── */

/**
 * @brief 存储后端操作表（RAM 后端内置；flash 后端由 port 层实现并填充）
 *
 * @note ctx 为实现方上下文（RAM 后端传 pstore_ram_t*；flash 后端传分区/页描述），
 *       由本组件在每次调用时原样回传。
 * @note 擦写临界区（关中断/锁）由后端实现自理：后端自己知道是否需要、
 *       需要多粗的粒度（重组决策：旧式实现把 critical 表传进核心层，
 *       分层上属于后端职责）。
 */
typedef struct {
    /**
     * @brief 读取整个存储区
     * @param ctx   实现上下文
     * @param buf   接收缓冲（容量 ≥ len，由核心层保证 = capacity）
     * @param len   字节数（核心层恒传 capacity）
     * @retval 0 成功；非 0 失败（核心层上报 ERR_BACKEND）
     */
    int (*read)(void *ctx, void *buf, size_t len);

    /**
     * @brief 写入帧（核心层先 erase 后 write）
     * @param ctx   实现上下文
     * @param buf   帧数据
     * @param len   帧长（≤ capacity）
     * @retval 0 成功；非 0 失败
     * @note  flash 语义要求先擦后写（核心层保证调用序）；EEPROM/RAM 可直接写。
     */
    int (*write)(void *ctx, const void *buf, size_t len);

    /**
     * @brief 擦除存储区
     * @param ctx  实现上下文
     * @param len  擦除字节数（核心层恒传 capacity）
     * @retval 0 成功；非 0 失败
     * @note  EEPROM/RAM 后端可为幂等空操作之外的任意安全实现（本组件 RAM 后端置 0xFF）。
     */
    int (*erase)(void *ctx, size_t len);

    void *ctx;   /**< 实现上下文（回传给上述三个函数） */
} pstore_backend_t;

/* ───────────────────────── 内置 RAM 后端 ───────────────────────── */

/**
 * @brief RAM 后端实例（测试 / 桌面模拟用；介质 = 调用方提供的字节数组）
 *
 * @note 计数器（reads/writes/erases）供脏检查门禁断言「不脏不写」。
 */
typedef struct {
    uint8_t *mem;      /**< 存储区（调用方提供，生命周期覆盖使用期） */
    uint32_t size;     /**< 存储区字节数 */
    uint32_t reads;    /**< read 调用计数 */
    uint32_t writes;   /**< write 调用计数（脏检查断言用） */
    uint32_t erases;   /**< erase 调用计数 */
} pstore_ram_t;

/**
 * @brief   绑定 RAM 后端（一次调用完成初始化）
 *
 * @param[out] be    待填充的后端操作表
 * @param[out] ram   RAM 后端实例（计数清零）
 * @param[in]  mem   存储区数组（容量 ≥ size）
 * @param[in]  size  存储区字节数
 *
 * @note  介质初始化为全 0xFF（模拟未写 flash；首载即 ERR_MAGIC → 默认值路径）。
 * @note  「断电保持」模拟：调用方 memcpy(mem) 即得快照；重启语义 = 重建实例 + load。
 */
void pstore_ram_backend_init(pstore_backend_t *be, pstore_ram_t *ram,
                             uint8_t *mem, uint32_t size);

/* ───────────────────────── 实例配置与句柄 ───────────────────────── */

/**
 * @brief   持久化实例配置（参数零写死：容量/条目上限一切经此传入）
 */
typedef struct {
    const pstore_backend_t *backend;   /**< 存储后端（必填，见上方契约） */
    uint32_t                capacity;  /**< 存储区字节数（≥ PSTORE_FRAME_HEADER_BYTES；帧长上限） */
    uint32_t                max_items;  /**< 条目注册表容量（注册数上限） */
    uint32_t                magic;      /**< 应用身份魔数（建议避开 0xFFFFFFFF 以区分空白介质） */
} pstore_cfg_t;

/** @brief 实例句柄（内容私有，定义在 src/pstore.c） */
typedef struct pstore pstore_t;

/* ───────────────────────── 实例生命周期 ───────────────────────── */

/**
 * @brief   创建实例（校验配置并分配内部缓冲/注册表，不访问介质）
 *
 * @param[in] cfg  配置（backend 非 NULL；capacity ≥ PSTORE_FRAME_HEADER_BYTES；
 *                 max_items ≥ 1）
 *
 * @return  实例句柄；NULL = 参数非法或内存不足
 *
 * @note    分配三块：stage 缓冲（capacity 字节，序列化+加载暂存）、
 *          应用计划表（max_items 项）、注册表（max_items 项）。
 *          加载路径无动态分配（两阶段原子性不依赖堆可用性）。
 */
pstore_t *pstore_create(const pstore_cfg_t *cfg);

/**
 * @brief   销毁实例（释放内部缓冲；不触碰介质与已注册条目的 ram）
 */
void pstore_destroy(pstore_t *ps);

/* ───────────────────────── 条目注册表 ───────────────────────── */

/**
 * @brief   注册一个参数条目
 *
 * @param[in] ps   实例
 * @param[in] key  键名（NUL 结尾，长度 1..65535；字符串生命周期须覆盖实例使用期）
 * @param[in] ram  RAM 目标地址（本组件只做拷贝，不持有所有权）
 * @param[in] len  数据字节数（1..65535，帧格式上限）
 *
 * @retval  PSTORE_OK            成功
 * @retval  PSTORE_ERR_ARG       ps/key/ram 为 NULL、len==0、key 空或超长
 * @retval  PSTORE_ERR_FULL      注册表满（max_items 用尽）
 * @retval  PSTORE_ERR_DUP       key 已注册
 *
 * @note    注册顺序 = 序列化顺序（重组决策：旧式实现链表头插导致序反转，
 *          数组注册表按序写入，帧内容与注册序一致、可直接对读）。
 */
int pstore_register(pstore_t *ps, const char *key, void *ram, size_t len);

/**
 * @brief   注销一个参数条目（注册表压缩保序；条目下次 save 起不再入帧）
 *
 * @param[in] ps   实例
 * @param[in] key  键名
 *
 * @retval  PSTORE_OK             成功（并置脏：帧需重写以收缩）
 * @retval  PSTORE_ERR_ARG        ps/key 为 NULL
 * @retval  PSTORE_ERR_NOTFOUND   无此 key
 *
 * @note    load 遇帧内有、注册表无的条目会跳过（前向兼容：固件删参数不废旧帧）。
 */
int pstore_unregister(pstore_t *ps, const char *key);

/* ───────────────────────── 落盘与加载 ───────────────────────── */

/**
 * @brief   标记数据已变更（请求落盘）
 */
void pstore_mark_dirty(pstore_t *ps);

/**
 * @brief   当前是否有未落盘变更
 *
 * @retval  1 有（下次 save 会写介质）；0 无；-1 ps 为 NULL
 */
int pstore_is_dirty(const pstore_t *ps);

/**
 * @brief   落盘（脏检查：不脏零后端调用；脏则 序列化→erase→write 一帧）
 *
 * @param[in] ps  实例
 *
 * @retval  PSTORE_OK          成功（或本就不脏，直接返回）
 * @retval  PSTORE_ERR_ARG     ps 为 NULL
 * @retval  PSTORE_ERR_NOFIT   帧超容量（脏标记保留，删条目后可重试）
 * @retval  PSTORE_ERR_BACKEND erase/write 失败（脏标记保留）
 *
 * @note    同步单发（重组决策：参数帧小、写入频率低，旧式实现的异步状态机
 *          为保采样循环不阻塞——那是 app 层调度问题，port 可自行包线程/延迟落盘，
 *          核心保持纯函数式同步语义）。
 * @note    断电窗口：erase→write 非原子，断电落其间 → 介质无有效帧 →
 *          下次 load 走 ERR_* → 调用方恢复默认值（可接受降级，磨损均衡
 *          双缓冲是 port 层升级路径，见组件 README）。
 */
int pstore_save(pstore_t *ps);

/**
 * @brief   上电加载（两阶段原子：先全量校验成计划，后一次性应用）
 *
 * @param[in] ps  实例
 *
 * @retval  PSTORE_OK           成功（ram 已按帧内容更新，脏标记清除）
 * @retval  PSTORE_ERR_ARG      ps 为 NULL
 * @retval  PSTORE_ERR_BACKEND  后端读失败
 * @retval  PSTORE_ERR_MAGIC    magic 不匹配（空白/异种）
 * @retval  PSTORE_ERR_VERSION  帧版本不匹配
 * @retval  PSTORE_ERR_CHECKSUM CRC 失败（数据区或 count/data_len 被改）
 * @retval  PSTORE_ERR_CORRUPT  帧结构损坏（条目越界/解析末尾与 data_len 不对齐/
 *                              解析条目数与 count 不符/匹配计划超注册表容量）
 *
 * @note    【原子性承诺】任一失败路径下，所有已注册条目的 ram **零字节改动**
 *          （雷1 修复：解析在内部 stage 缓冲完成，应用计划全量生成后才 memcpy）。
 * @note    匹配规则：key 逐字节相同 **且** dlen == 注册 len 才应用；
 *          帧内未注册条目跳过；同 key 在帧内出现多条（合法写入器不产生，
 *          仅损坏帧可达）时按帧序后者胜——确定性语义，文档化处理。
 * @note    阶段一结构校验与 CRC 双层覆盖 count（雷3 修复）：count 篡改先撞
 *          CRC（ERR_CHECKSUM）；若 CRC 被一并伪造成一致（测试专用构造），
 *          解析对齐断言（条目数==count、末尾==data_len）兜底拒绝（ERR_CORRUPT）。
 */
int pstore_load(pstore_t *ps);

/* ───────────────────────── 查询与工具 ───────────────────────── */

/**
 * @brief   计算当前注册表序列化后的帧长（不写缓冲、不访问介质）
 *
 * @param[in] ps  实例
 *
 * @return  帧字节数（header + 全部条目）；ps 为 NULL 返回 0
 *
 * @note    启动期容量门禁用：calc_len > capacity ⇒ 注册面超限，
 *          save 将返回 ERR_NOFIT。
 */
uint32_t pstore_calc_len(const pstore_t *ps);

/**
 * @brief   最近一次成功 save/load 的帧长（存储占用查询）
 *
 * @return  帧字节数；从未成功保存/加载返回 0；ps 为 NULL 返回 0
 */
uint32_t pstore_frame_len(const pstore_t *ps);

/**
 * @brief   当前注册条目数
 *
 * @return  条目数；ps 为 NULL 返回 0
 */
uint32_t pstore_entry_count(const pstore_t *ps);

/**
 * @brief   CRC32（标准 CRC-32/ISO-HDLC：反射多项式 0xEDB88320，
 *          初值 0xFFFFFFFF，结果取反；自研逐位实现，无查表）
 *
 * @param[in] data  数据
 * @param[in] len   字节数
 *
 * @return  CRC32 值
 *
 * @note    公开为帧校验原语：单测用它构造「CRC 一致但结构损坏」的防回归帧；
 *          port/工具可用它离线验证帧完整性。标准自检向量见单测
 *          （"123456789" → 0xCBF43926）。
 */
uint32_t pstore_crc32(const void *data, size_t len);

/**
 * @brief   错误码 → 可读名（上电日志用）
 *
 * @param[in] err  错误码（pstore_err_t 或其整型值）
 *
 * @return  静态字符串；未知码返回 "unknown"
 */
const char *pstore_err_name(int err);

#endif /* PSTORE_PSTORE_H */
