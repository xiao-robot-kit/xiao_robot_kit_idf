/* SPDX-License-Identifier: MIT */
/**
 * @file    pstore.c
 * @brief   参数持久化组件实现：注册表序列化 + CRC32 + 两阶段原子加载 + 内置 RAM 后端
 *
 * @note    设计要点（完整契约见 pstore.h 文件头）：
 *          - 两阶段原子 load：阶段一只写内部 stage/plan，阶段二才碰 ram；
 *          - 无死接口：错误码全部有产生路径；
 *          - count/data_len 入 CRC + 解析对齐断言双层防护。
 * @note    非线程安全：单线程核心；后端若被多线程访问，由后端实现自理加锁
 *          （见 pstore.h 后端契约注记）。
 */

#include <stdlib.h>
#include <string.h>
#include "pstore.h"

/* ───────────────────────── 内部类型 ───────────────────────── */

/** @brief 注册表条目（klen/len 缓存，帧格式上限 65535） */
typedef struct {
    const char *key;   /**< 键名（NUL 结尾，调用方持有） */
    void       *ram;   /**< RAM 目标地址（调用方持有，本组件只拷贝） */
    uint16_t    klen;  /**< 键名长度（不含结尾 0） */
    uint16_t    len;   /**< 数据字节数 */
} entry_t;

/** @brief 应用计划项（阶段一产出：哪个注册条目、数据在 stage 的哪个偏移） */
typedef struct {
    uint32_t entry;   /**< 注册表下标 */
    uint32_t off;     /**< stage 内该条目 data 起始偏移 */
} plan_t;

/** @brief 实例（内容私有，契约见 pstore.h） */
struct pstore {
    pstore_cfg_t cfg;      /**< 创建时拷入的配置（backend 指针归调用方持有） */
    uint8_t   *stage;      /**< 序列化/解析暂存缓冲（capacity 字节） */
    plan_t    *plan;       /**< 两阶段加载的应用计划（max_items 项） */
    entry_t   *entries;    /**< 条目注册表（max_items 项，注册序即序列化序） */
    uint32_t   n;          /**< 已注册条目数 */
    uint32_t   frame_len;  /**< 最近一次成功 save/load 的帧长 */
    int        dirty;      /**< 未落盘变更标志 */
};

/* ───────────────────────── CRC32 ───────────────────────── */

/**
 * @brief   CRC32 增量喂入（标准 CRC-32/ISO-HDLC：反射多项式 0xEDB88320）
 *
 * @param[in]     crc  当前状态（首段喂初值 0xFFFFFFFF）
 * @param[in]     d    数据
 * @param[in]     n    字节数
 *
 * @return  更新后的状态（末段需再取反才是最终 CRC）
 *
 * @note    逐位实现（无查表）：参数帧小，省表换通用性。
 */
static uint32_t crc32_feed(uint32_t crc, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int k = 0; k < 8; k++)
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
    return crc;
}

/* 对外一步式封装（契约见 pstore.h） */
uint32_t pstore_crc32(const void *data, size_t len) {
    return ~crc32_feed(0xFFFFFFFFu, (const uint8_t *)data, len);
}

/**
 * @brief   帧校验和：CRC32 覆盖 header 的 count+data_len 两字段（offset 8..16）
 *          与数据区（offset 20 .. 20+data_len）
 *
 * @note    count/data_len 入覆盖范围：篡改任一先撞 ERR_CHECKSUM；
 *          解析对齐断言（见 pstore_load）再兜一层。
 */
static uint32_t frame_crc(const uint8_t *stage, uint32_t data_len) {
    uint32_t c = crc32_feed(0xFFFFFFFFu, stage + 8, 8);
    c = crc32_feed(c, stage + PSTORE_FRAME_HEADER_BYTES, data_len);
    return ~c;
}

/* ───────────────────────── 实例生命周期 ───────────────────────── */

/* 契约见 pstore.h */
pstore_t *pstore_create(const pstore_cfg_t *cfg) {
    if (cfg == NULL || cfg->backend == NULL) return NULL;
    if (cfg->capacity < PSTORE_FRAME_HEADER_BYTES) return NULL;
    if (cfg->max_items < 1) return NULL;

    pstore_t *ps = calloc(1, sizeof(*ps));
    if (ps == NULL) return NULL;
    ps->cfg = *cfg;

    ps->stage   = malloc(cfg->capacity);
    ps->plan    = malloc((size_t)cfg->max_items * sizeof(plan_t));
    ps->entries = malloc((size_t)cfg->max_items * sizeof(entry_t));
    if (ps->stage == NULL || ps->plan == NULL || ps->entries == NULL) {
        pstore_destroy(ps);   /* destroy 自理半初始化态（各指针 NULL 安全） */
        return NULL;
    }
    return ps;
}

/* 契约见 pstore.h（半初始化态安全：三个指针可能为 NULL，free(NULL) 合法） */
void pstore_destroy(pstore_t *ps) {
    if (ps == NULL) return;
    free(ps->stage);
    free(ps->plan);
    free(ps->entries);
    free(ps);
}

/* ───────────────────────── 条目注册表 ───────────────────────── */

/**
 * @brief   按 key 查注册表下标
 *
 * @return  下标；-1 = 无此 key（ps/key 已由调用方判空）
 */
static int32_t find_entry(const pstore_t *ps, const char *key) {
    for (uint32_t i = 0; i < ps->n; i++)
        if (strcmp(ps->entries[i].key, key) == 0) return (int32_t)i;
    return -1;
}

/* 契约见 pstore.h */
int pstore_register(pstore_t *ps, const char *key, void *ram, size_t len) {
    if (ps == NULL || key == NULL || ram == NULL) return PSTORE_ERR_ARG;
    size_t klen = strlen(key);
    if (len == 0 || len > 0xFFFFu) return PSTORE_ERR_ARG;
    if (klen == 0 || klen > 0xFFFFu) return PSTORE_ERR_ARG;
    if (find_entry(ps, key) >= 0) return PSTORE_ERR_DUP;
    if (ps->n == ps->cfg.max_items) return PSTORE_ERR_FULL;

    entry_t *e = &ps->entries[ps->n++];
    e->key  = key;
    e->ram  = ram;
    e->klen = (uint16_t)klen;
    e->len  = (uint16_t)len;
    return PSTORE_OK;
}

/* 契约见 pstore.h */
int pstore_unregister(pstore_t *ps, const char *key) {
    if (ps == NULL || key == NULL) return PSTORE_ERR_ARG;
    int32_t i = find_entry(ps, key);
    if (i < 0) return PSTORE_ERR_NOTFOUND;

    memmove(&ps->entries[i], &ps->entries[i + 1],
            (size_t)(ps->n - (uint32_t)i - 1) * sizeof(entry_t));
    ps->n--;
    ps->dirty = 1;   /* 帧需收缩重写 */
    return PSTORE_OK;
}

/* ───────────────────────── 落盘 ───────────────────────── */

/**
 * @brief   序列化整张注册表到 stage
 *
 * @param[in]  ps        实例
 * @param[out] out_len   帧长（header + 全部条目）
 *
 * @retval  PSTORE_OK        成功
 * @retval  PSTORE_ERR_NOFIT 任一条目放不下（容量超限的真实溢出路径）
 *
 * @note    溢出检查先于写入该条目（`need > capacity - off`，减法形式无溢出）；
 *          失败时 stage 内容无效，但 save 不会带着它碰后端（NOFIT 提前返回）。
 * @note    头部先写 magic..data_len（offset 0..16），算完 CRC 再写 offset 16..20
 *          ——CRC 覆盖范围（count/data_len/数据区）本身不含 CRC 字段，顺序无关。
 */
static int build_frame(pstore_t *ps, uint32_t *out_len) {
    uint32_t off = PSTORE_FRAME_HEADER_BYTES;
    for (uint32_t i = 0; i < ps->n; i++) {
        const entry_t *e = &ps->entries[i];
        uint32_t need = 2u + e->klen + 2u + e->len;
        if (need > ps->cfg.capacity - off) return PSTORE_ERR_NOFIT;

        memcpy(ps->stage + off, &e->klen, 2); off += 2;
        memcpy(ps->stage + off, e->key, e->klen); off += e->klen;
        memcpy(ps->stage + off, &e->len, 2); off += 2;
        memcpy(ps->stage + off, e->ram, e->len); off += e->len;
    }

    pstore_frame_hdr_t h;
    h.magic    = ps->cfg.magic;
    h.version  = PSTORE_FORMAT_VERSION;
    h.count    = ps->n;
    h.data_len = off - PSTORE_FRAME_HEADER_BYTES;
    memcpy(ps->stage, &h, PSTORE_FRAME_HEADER_BYTES - 4);   /* magic..data_len */
    h.crc = frame_crc(ps->stage, h.data_len);
    memcpy(ps->stage + 16, &h.crc, 4);

    *out_len = off;
    return PSTORE_OK;
}

/* 契约见 pstore.h */
int pstore_save(pstore_t *ps) {
    if (ps == NULL) return PSTORE_ERR_ARG;
    if (!ps->dirty) return PSTORE_OK;   /* 脏检查：不脏零后端调用 */

    uint32_t len = 0;
    int rc = build_frame(ps, &len);
    if (rc != PSTORE_OK) return rc;   /* NOFIT：脏保留，删条目后可重试 */

    const pstore_backend_t *be = ps->cfg.backend;
    if (be->erase(be->ctx, ps->cfg.capacity) != 0) return PSTORE_ERR_BACKEND;
    if (be->write(be->ctx, ps->stage, len) != 0)   return PSTORE_ERR_BACKEND;
    /* 注意：erase 成功而 write 失败 ⇒ 介质已无有效帧（断电窗口语义，pstore.h 注记） */

    ps->frame_len = len;
    ps->dirty = 0;
    return PSTORE_OK;
}

/* ───────────────────────── 加载（两阶段原子） ───────────────────────── */

/* 契约见 pstore.h（原子性/匹配规则/双层 count 防护的完整承诺在头文件） */
int pstore_load(pstore_t *ps) {
    if (ps == NULL) return PSTORE_ERR_ARG;
    const pstore_backend_t *be = ps->cfg.backend;
    if (be->read(be->ctx, ps->stage, ps->cfg.capacity) != 0)
        return PSTORE_ERR_BACKEND;

    /* ═══ 阶段一：全量校验 + 生成应用计划（此阶段对 ram 零写入——原子性核心） ═══ */
    pstore_frame_hdr_t h;
    memcpy(&h, ps->stage, sizeof(h));
    if (h.magic != ps->cfg.magic)           return PSTORE_ERR_MAGIC;
    if (h.version != PSTORE_FORMAT_VERSION) return PSTORE_ERR_VERSION;
    if (h.data_len > ps->cfg.capacity - PSTORE_FRAME_HEADER_BYTES)
        return PSTORE_ERR_CORRUPT;
    if (h.crc != frame_crc(ps->stage, h.data_len))
        return PSTORE_ERR_CHECKSUM;

    const uint32_t limit = PSTORE_FRAME_HEADER_BYTES + h.data_len;   /* 数据区末尾 */
    uint32_t off = PSTORE_FRAME_HEADER_BYTES;
    uint32_t planned = 0;

    for (uint32_t i = 0; i < h.count; i++) {
        /* 每步先界后读（`limit - off` 形式：off ≤ limit 不变量下无下溢/无溢出） */
        if (limit - off < 2u) return PSTORE_ERR_CORRUPT;
        uint16_t klen;
        memcpy(&klen, ps->stage + off, 2); off += 2;
        if (klen == 0 || limit - off < klen) return PSTORE_ERR_CORRUPT;
        const uint8_t *key = ps->stage + off; off += klen;
        if (limit - off < 2u) return PSTORE_ERR_CORRUPT;
        uint16_t dlen;
        memcpy(&dlen, ps->stage + off, 2); off += 2;
        if (limit - off < dlen) return PSTORE_ERR_CORRUPT;

        for (uint32_t j = 0; j < ps->n; j++) {
            const entry_t *e = &ps->entries[j];
            if (e->len == dlen && e->klen == klen &&
                memcmp(e->key, key, klen) == 0) {
                /* 帧内可匹配计划数 ≤ 注册表容量（合法写入器只产注册过的 key，
                 * 超出 ⇒ 帧内重复 key ⇒ 损坏，拒绝而非溢出计划表） */
                if (planned == ps->cfg.max_items) return PSTORE_ERR_CORRUPT;
                ps->plan[planned].entry = j;
                ps->plan[planned].off   = off;
                planned++;
                break;   /* 注册表 key 唯一（register 查重），至多一个 j 匹配 */
            }
        }
        off += dlen;
    }

    /* 对齐断言（兜底层）：count 条目恰好吃满数据区——CRC 被伪造一致时
     * （测试专用构造），count/data_len 与实际条目流不符在此判 ERR_CORRUPT */
    if (off != limit) return PSTORE_ERR_CORRUPT;

    /* ═══ 阶段二：一次性应用（此前 ram 零改动；从这行起才允许碰 ram） ═══ */
    for (uint32_t k = 0; k < planned; k++) {
        const plan_t *p = &ps->plan[k];
        memcpy(ps->entries[p->entry].ram, ps->stage + p->off,
               ps->entries[p->entry].len);
    }

    ps->frame_len = limit;
    ps->dirty = 0;   /* load 成功 = 介质与 ram 同步 */
    return PSTORE_OK;
}

/* ───────────────────────── 查询与杂项 ───────────────────────── */

void pstore_mark_dirty(pstore_t *ps) {
    if (ps != NULL) ps->dirty = 1;
}

int pstore_is_dirty(const pstore_t *ps) {
    if (ps == NULL) return -1;
    return ps->dirty ? 1 : 0;
}

uint32_t pstore_calc_len(const pstore_t *ps) {
    if (ps == NULL) return 0;
    uint32_t len = PSTORE_FRAME_HEADER_BYTES;
    for (uint32_t i = 0; i < ps->n; i++)
        len += 2u + ps->entries[i].klen + 2u + ps->entries[i].len;
    return len;
}

uint32_t pstore_frame_len(const pstore_t *ps) {
    return (ps != NULL) ? ps->frame_len : 0;
}

uint32_t pstore_entry_count(const pstore_t *ps) {
    return (ps != NULL) ? ps->n : 0;
}

const char *pstore_err_name(int err) {
    switch (err) {
    case PSTORE_OK:           return "ok";
    case PSTORE_ERR_ARG:      return "invalid argument";
    case PSTORE_ERR_FULL:     return "registry full (max_items)";
    case PSTORE_ERR_DUP:      return "duplicate key";
    case PSTORE_ERR_NOTFOUND: return "key not registered";
    case PSTORE_ERR_NOFIT:    return "frame exceeds capacity";
    case PSTORE_ERR_BACKEND:  return "backend read/write/erase failed";
    case PSTORE_ERR_MAGIC:    return "magic mismatch (blank/foreign medium)";
    case PSTORE_ERR_VERSION:  return "frame version mismatch";
    case PSTORE_ERR_CHECKSUM: return "crc32 mismatch";
    case PSTORE_ERR_CORRUPT:  return "corrupt frame structure";
    default:                  return "unknown";
    }
}

/* ───────────────────────── 内置 RAM 后端 ───────────────────────── */

/** @brief RAM 后端读（契约见 pstore.h；越界 = 调用方容量配置失配） */
static int ram_read(void *ctx, void *buf, size_t len) {
    pstore_ram_t *r = (pstore_ram_t *)ctx;
    if (len > r->size) return -1;
    memcpy(buf, r->mem, len);
    r->reads++;
    return 0;
}

/** @brief RAM 后端写（字节可写，erase 非必要但幂等兼容 flash 调用序） */
static int ram_write(void *ctx, const void *buf, size_t len) {
    pstore_ram_t *r = (pstore_ram_t *)ctx;
    if (len > r->size) return -1;
    memcpy(r->mem, buf, len);
    r->writes++;
    return 0;
}

/** @brief RAM 后端擦（模拟 flash 擦后态：全 0xFF） */
static int ram_erase(void *ctx, size_t len) {
    pstore_ram_t *r = (pstore_ram_t *)ctx;
    if (len > r->size) return -1;
    memset(r->mem, 0xFF, len);
    r->erases++;
    return 0;
}

/* 契约见 pstore.h */
void pstore_ram_backend_init(pstore_backend_t *be, pstore_ram_t *ram,
                             uint8_t *mem, uint32_t size) {
    if (be == NULL || ram == NULL || mem == NULL) return;
    ram->mem = mem;
    ram->size = size;
    ram->reads = 0;
    ram->writes = 0;
    ram->erases = 0;
    memset(mem, 0xFF, size);   /* 空白介质语义：首载 ERR_MAGIC → 默认值路径 */
    be->read  = ram_read;
    be->write = ram_write;
    be->erase = ram_erase;
    be->ctx   = ram;
}
