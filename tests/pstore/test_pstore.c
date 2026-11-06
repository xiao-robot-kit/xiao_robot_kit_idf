/* SPDX-License-Identifier: MIT */
/**
 * @file    test_pstore.c
 * @brief   参数持久化组件无硬件单测（RAM 后端介质 + 两阶段原子性防回归 + 断电保持模拟）
 *
 * @note    自研极简断言（不引第三方测试框架），风格对齐 test_imu.c。
 * @note    验证范围：
 *          ① CRC32 标准向量 / ② create 与注册面参数校验（含
 *          FULL/DUP/NOTFOUND/NOFIT 的真实路径）/ ③ 序列化→load 往返字节一致 /
 *          ④【防回归】坏 CRC、坏 count、CORRUPT 中途（第 i 条目越界）、
 *          伪一致 count、伪一致 data_len 五种失败下 ram 零字节改动 /
 *          ⑤ count 篡改被 CRC 捕获 + CRC 伪造一致时对齐断言兜底 /
 *          ⑥ 增删条目后重写 / ⑦ 脏检查写计数（不脏零后端调用）/
 *          ⑧ 断电保持模拟 1000 循环零失真。
 *          不覆盖：真实 flash 介质行为与 QEMU 级断电重启（留后续）。
 * @note    「CRC 一致但结构损坏」帧的构造法：篡改帧字段后，用公开的
 *          pstore_crc32 对（count+data_len 头两字段 || 数据区）重算 CRC 回填
 *          ——这是到达 pstore_load 解析阶段的唯一测试途径（CRC 先于解析），
 *          亦是两阶段原子性防回归的关键构造。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pstore.h"

/* ---------- 极简断言（对齐 test_imu.c） ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)

/* ---------- 测试参数表（可调参数集中注册，代码内零写死） ---------- */
typedef struct {
    uint32_t capacity;        /**< 介质容量，字节（create 传入语义） */
    uint32_t max_items;       /**< 注册表上限 */
    uint32_t magic;           /**< 应用魔数 */
    uint32_t power_cycles;    /**< 断电保持循环数（1000） */
    uint32_t lcg_seed;        /**< 确定性 LCG 种子（跨平台可复现） */
    uint32_t tiny_capacity;   /**< NOFIT 用小容量（装不下任何条目） */
    uint32_t tiny_max_items;  /**< FULL 用小注册表 */
    uint32_t corrupt_item;    /**< 「CORRUPT 中途」构造的越界条目下标（0 基） */
    uint32_t scramble_byte;   /**< 断电后 RAM 垃圾填充字节 */
    uint32_t big_len;         /**< 超上限的 len（> 0xFFFF） */
    uint32_t big_klen;        /**< 超上限的 key 长度（> 0xFFFF） */
} pst_test_cfg_t;

static const pst_test_cfg_t T = {
    .capacity        = 128,
    .max_items       = 8,
    .magic           = 0x58535243u,   /* "XSRC"：XIAO Storage Resource Contract */
    .power_cycles    = 1000,
    .lcg_seed        = 0x20261105u,   /* 确定性种子 */
    .tiny_capacity   = 24,            /* 仅容 header+1 个微条目，容不下 4 条目帧 */
    .tiny_max_items  = 2,
    .corrupt_item    = 2,             /* 第 0/1 条已可匹配 → 旧式非原子实现此刻已污染 ram */
    .scramble_byte   = 0xC3,
    .big_len         = 0x10000u,
    .big_klen        = 0x10001u,
};

/* 帧加工缓冲上限（capacity + 头两字段拼接余量；测试夹具尺寸，非被测参数） */
#define PST_TEST_SCRATCH 160u

/* ---------- 被测条目夹具（四类型宽度：u32 / float / u8[6] / u16） ---------- */
static const char *K_KP  = "pid.kp";      /* 6 字节键 */
static const char *K_VEL = "vel.max";     /* 7 字节键 */
static const char *K_SER = "id.serial";   /* 9 字节键 */
static const char *K_MID = "pwm.mid";     /* 7 字节键 */

typedef struct {
    float    kp;
    uint32_t vel;
    uint8_t  ser[6];
    uint16_t mid;
} params_t;

/** 注册全套四条目（写入方/读取方实例共用的注册面） */
static int reg_all(pstore_t *ps, params_t *p) {
    int rc = pstore_register(ps, K_KP,  &p->kp,  sizeof(p->kp));
    if (rc) return rc;
    rc = pstore_register(ps, K_VEL, &p->vel, sizeof(p->vel));
    if (rc) return rc;
    rc = pstore_register(ps, K_SER, &p->ser, sizeof(p->ser));
    if (rc) return rc;
    return pstore_register(ps, K_MID, &p->mid, sizeof(p->mid));
}

/* ---------- 确定性 LCG（Numerical Recipes 常数；禁 rand() 保证可复现） ---------- */
static uint32_t lcg(uint32_t *s) {
    *s = 1664525u * *s + 1013904223u;
    return *s;
}

/** 用 LCG 填充一段 RAM（断电保持循环的参数生成） */
static void lcg_fill(uint32_t *s, void *dst, size_t n) {
    uint8_t *p = (uint8_t *)dst;
    while (n >= 4) { uint32_t v = lcg(s); memcpy(p, &v, 4); p += 4; n -= 4; }
    if (n) { uint32_t v = lcg(s); memcpy(p, &v, n); }
}

/* ---------- 帧加工工具（测试专用：篡改 + CRC 重算） ---------- */

/** 读取帧内 data_len 字段 */
static uint32_t frame_data_len(const uint8_t *frame) {
    uint32_t v;
    memcpy(&v, frame + 12, 4);
    return v;
}

/**
 * @brief   重算并回填帧 CRC（「CRC 一致但结构损坏」构造的必要步骤）
 *
 * @note    覆盖契约与库内一致：count+data_len 头两字段（offset 8..16）
 *          拼接数据区（offset 20..20+data_len）后一步 CRC。
 */
static void fix_crc(uint8_t *frame) {
    uint32_t data_len = frame_data_len(frame);
    uint8_t scratch[PST_TEST_SCRATCH];
    if (8u + data_len > PST_TEST_SCRATCH) return;   /* 夹具保护：理论不可达 */
    memcpy(scratch, frame + 8, 8);
    memcpy(scratch + 8, frame + 20, data_len);
    uint32_t crc = pstore_crc32(scratch, 8u + data_len);
    memcpy(frame + 16, &crc, 4);
}

/**
 * @brief   定位第 item_index 条目 dlen 字段的帧内偏移（自走帧结构，不依赖注册面）
 *
 * @return  偏移；item_index 越界返回 0
 */
static uint32_t item_dlen_off(const uint8_t *frame, uint32_t item_index) {
    uint32_t count = 0, off = PSTORE_FRAME_HEADER_BYTES;
    memcpy(&count, frame + 8, 4);
    for (uint32_t i = 0; i < count; i++) {
        uint16_t klen = 0, dlen = 0;
        memcpy(&klen, frame + off, 2);
        if (i == item_index) return off + 2u + klen;
        memcpy(&dlen, frame + off + 2u + klen, 2);
        off += 2u + klen + 2u + dlen;
    }
    return 0;
}

/* ---------- 坏后端（read 恒失败，测 ERR_BACKEND 路径） ---------- */
static int fail_read(void *ctx, void *buf, size_t len) {
    (void)ctx; (void)buf; (void)len;
    return -1;
}
static int ok_write(void *ctx, const void *buf, size_t len) {
    (void)ctx; (void)buf; (void)len;
    return 0;
}
static int ok_erase(void *ctx, size_t len) {
    (void)ctx; (void)len;
    return 0;
}

/* ---------- 防回归夹具：哨兵填充与零改动校验 ---------- */

/** 四条目 ram 全填哨兵字节（失败注入前的初始态） */
static void fill_sentinel(params_t *p) {
    memset(&p->kp,  0xA5, sizeof(p->kp));
    memset(&p->vel, 0xA5, sizeof(p->vel));
    memset(p->ser,  0xA5, sizeof(p->ser));
    memset(&p->mid, 0xA5, sizeof(p->mid));
}

/** 断言四条目 ram 仍是完整哨兵（两阶段原子：失败路径零字节改动） */
static void check_sentinel(const params_t *p, const char *tag) {
    static const uint8_t a5_kp[4]  = {0xA5,0xA5,0xA5,0xA5};
    static const uint8_t a5_vel[4] = {0xA5,0xA5,0xA5,0xA5};
    static const uint8_t a5_ser[6] = {0xA5,0xA5,0xA5,0xA5,0xA5,0xA5};
    static const uint8_t a5_mid[2] = {0xA5,0xA5};
    CHECK(memcmp(&p->kp,  a5_kp,  sizeof(a5_kp))  == 0, tag);
    CHECK(memcmp(&p->vel, a5_vel, sizeof(a5_vel)) == 0, tag);
    CHECK(memcmp(p->ser,  a5_ser, sizeof(a5_ser)) == 0, tag);
    CHECK(memcmp(&p->mid, a5_mid, sizeof(a5_mid)) == 0, tag);
}

int main(void) {
    uint8_t mem[T.capacity];      /* 介质（RAM 后端） */
    uint8_t snap[T.capacity];     /* 断电快照 */
    pstore_backend_t be;
    pstore_ram_t ram;
    pstore_cfg_t cfg = { 0 };

    /* ═══ 1. CRC32 标准自检向量（CRC-32/ISO-HDLC："123456789" → 0xCBF43926） ═══ */
    {
        CHECK(pstore_crc32("123456789", 9) == 0xCBF43926u,
              "case1 crc32 check vector");
        CHECK(pstore_crc32("", 0) == 0x0u, "case1 crc32 empty input");
    }

    /* ═══ 2. create 参数校验（容量/条目上限经 cfg 传入，零写死验证） ═══ */
    {
        pstore_ram_backend_init(&be, &ram, mem, T.capacity);
        cfg.backend   = &be;
        cfg.capacity  = T.capacity;
        cfg.max_items = T.max_items;
        cfg.magic     = T.magic;

        pstore_cfg_t bad = cfg;
        bad.backend = NULL;
        CHECK(pstore_create(&bad) == NULL, "case2 create rejects NULL backend");
        bad = cfg; bad.capacity = PSTORE_FRAME_HEADER_BYTES - 1;
        CHECK(pstore_create(&bad) == NULL, "case2 create rejects capacity < header");
        bad = cfg; bad.max_items = 0;
        CHECK(pstore_create(&bad) == NULL, "case2 create rejects max_items 0");
        CHECK(pstore_create(NULL) == NULL, "case2 create rejects NULL cfg");
        pstore_t *ps = pstore_create(&cfg);
        CHECK(ps != NULL, "case2 create accepts valid cfg");
        pstore_destroy(ps);
        pstore_destroy(NULL);   /* 幂等路径 */
    }

    /* ═══ 3. 注册面校验（ARG/FULL/DUP/NOTFOUND 的真实路径） ═══ */
    {
        pstore_t *ps = pstore_create(&(pstore_cfg_t){
            .backend = &be, .capacity = T.capacity,
            .max_items = T.tiny_max_items, .magic = T.magic });
        CHECK(ps != NULL, "case3 tiny registry create");
        params_t p;
        CHECK(pstore_register(NULL, K_KP, &p, 4) == PSTORE_ERR_ARG,
              "case3 register rejects NULL ps");
        CHECK(pstore_register(ps, NULL, &p, 4) == PSTORE_ERR_ARG,
              "case3 register rejects NULL key");
        CHECK(pstore_register(ps, K_KP, NULL, 4) == PSTORE_ERR_ARG,
              "case3 register rejects NULL ram");
        CHECK(pstore_register(ps, K_KP, &p, 0) == PSTORE_ERR_ARG,
              "case3 register rejects len 0");
        CHECK(pstore_register(ps, K_KP, &p, T.big_len) == PSTORE_ERR_ARG,
              "case3 register rejects len > 0xFFFF");
        CHECK(pstore_register(ps, "", &p, 4) == PSTORE_ERR_ARG,
              "case3 register rejects empty key");

        char *bigkey = malloc(T.big_klen + 1);   /* 超长 key：堆上构造，测后即释 */
        CHECK(bigkey != NULL, "case3 bigkey alloc");
        if (bigkey) {
            memset(bigkey, 'k', T.big_klen);
            bigkey[T.big_klen] = '\0';
            CHECK(pstore_register(ps, bigkey, &p, 4) == PSTORE_ERR_ARG,
                  "case3 register rejects key > 0xFFFF");
            free(bigkey);
        }

        CHECK(pstore_register(ps, K_KP, &p, 4) == PSTORE_OK, "case3 register ok");
        CHECK(pstore_register(ps, K_KP, &p, 4) == PSTORE_ERR_DUP,
              "case3 register rejects duplicate key");
        CHECK(pstore_register(ps, K_VEL, &p, 4) == PSTORE_OK, "case3 register 2nd");
        CHECK(pstore_register(ps, K_SER, &p, 6) == PSTORE_ERR_FULL,
              "case3 register rejects full registry");
        CHECK(pstore_entry_count(ps) == 2, "case3 entry count 2");
        CHECK(pstore_unregister(ps, "no.such") == PSTORE_ERR_NOTFOUND,
              "case3 unregister unknown key");
        CHECK(pstore_unregister(ps, K_VEL) == PSTORE_OK, "case3 unregister ok");
        CHECK(pstore_entry_count(ps) == 1, "case3 entry count back to 1");
        CHECK(pstore_unregister(NULL, K_KP) == PSTORE_ERR_ARG,
              "case3 unregister rejects NULL ps");
        pstore_destroy(ps);
    }

    /* ═══ 4. 序列化→load 往返一致（空白介质→写→新实例读，字节级） ═══ */
    {
        params_t author, reader;
        pstore_t *w = NULL, *r = NULL;
        uint32_t seed = T.lcg_seed;

        lcg_fill(&seed, &author, sizeof(author));
        fill_sentinel(&reader);
        pstore_ram_backend_init(&be, &ram, mem, T.capacity);   /* 空白介质 */

        cfg.backend = &be; cfg.capacity = T.capacity;
        cfg.max_items = T.max_items; cfg.magic = T.magic;

        w = pstore_create(&cfg);
        CHECK(w != NULL && reg_all(w, &author) == PSTORE_OK, "case4 writer setup");
        r = pstore_create(&cfg);
        CHECK(r != NULL && reg_all(r, &reader) == PSTORE_OK, "case4 reader setup");

        /* 空白介质：首载 ERR_MAGIC 且 ram 保持哨兵（默认值路径不动已有值） */
        CHECK(pstore_load(r) == PSTORE_ERR_MAGIC, "case4 blank medium -> ERR_MAGIC");
        check_sentinel(&reader, "case4 blank load keeps sentinel ram");

        /* 落盘 → 新实例加载 → 逐条目字节一致 */
        pstore_mark_dirty(w);
        CHECK(pstore_save(w) == PSTORE_OK, "case4 save ok");
        CHECK(pstore_load(r) == PSTORE_OK, "case4 load ok");
        CHECK(memcmp(&reader.kp,  &author.kp,  sizeof(author.kp))  == 0, "case4 kp roundtrip");
        CHECK(memcmp(&reader.vel, &author.vel, sizeof(author.vel)) == 0, "case4 vel roundtrip");
        CHECK(memcmp(reader.ser,  author.ser,  sizeof(author.ser)) == 0, "case4 ser roundtrip");
        CHECK(memcmp(&reader.mid, &author.mid, sizeof(author.mid)) == 0, "case4 mid roundtrip");

        /* 帧长不变式：落盘帧长 == 注册面序列化长（存储占用可预算） */
        CHECK(pstore_frame_len(w) == pstore_calc_len(w), "case4 frame_len == calc_len");
        CHECK(pstore_frame_len(w) > PSTORE_FRAME_HEADER_BYTES, "case4 frame has items");
        CHECK(pstore_is_dirty(w) == 0, "case4 save clears dirty");
        CHECK(pstore_is_dirty(r) == 0, "case4 load clears dirty");

        /* 版本字段篡改 → ERR_VERSION（身份字段不进 CRC，改即身份错） */
        mem[4] ^= 0xFF;
        fill_sentinel(&reader);
        CHECK(pstore_load(r) == PSTORE_ERR_VERSION, "case4 tampered version -> ERR_VERSION");
        check_sentinel(&reader, "case4 version fail keeps sentinel ram");
        mem[4] ^= 0xFF;   /* 还原，供后续用例 */
        pstore_destroy(w);
        pstore_destroy(r);
    }

    /* ═══ 5.【防回归】五种失败注入下 ram 零字节改动 ═══
     * 公共构造：写好合法帧 → 哨兵 ram → 篡改帧（必要时 fix_crc）→ load 必须报错
     * 且哨兵完好。防回归目标：解析中途 CORRUPT 返回时，前序条目不得被 memcpy。 */
    {
        params_t saved, probe;
        pstore_t *w = NULL, *r = NULL;

        lcg_fill(&(uint32_t){T.lcg_seed}, &saved, sizeof(saved));
        pstore_ram_backend_init(&be, &ram, mem, T.capacity);
        cfg.backend = &be; cfg.capacity = T.capacity;
        cfg.max_items = T.max_items; cfg.magic = T.magic;

        w = pstore_create(&cfg);
        r = pstore_create(&cfg);
        CHECK(w != NULL && r != NULL, "case5 setup");
        CHECK(reg_all(w, &saved) == PSTORE_OK, "case5 writer registry");
        CHECK(reg_all(r, &probe) == PSTORE_OK, "case5 reader registry");
        pstore_mark_dirty(w);
        CHECK(pstore_save(w) == PSTORE_OK, "case5 base frame saved");

        /* 5a. 坏 CRC：数据区翻一个字节（不动头）→ ERR_CHECKSUM */
        {
            uint8_t keep[T.capacity];
            memcpy(keep, mem, T.capacity);
            mem[frame_data_len(mem) / 2u + PSTORE_FRAME_HEADER_BYTES] ^= 0x40u;
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CHECKSUM, "case5a bad data crc -> ERR_CHECKSUM");
            check_sentinel(&probe, "case5a ram zero-mutation");
            memcpy(mem, keep, T.capacity);   /* 还原帧 */
        }

        /* 5b. 坏 count：仅改 count 字段（不修 CRC）→ 被 CRC 捕获 → ERR_CHECKSUM
         *     （第一层：count 已入 CRC 覆盖范围） */
        {
            uint8_t keep[T.capacity];
            uint32_t c;
            memcpy(keep, mem, T.capacity);
            memcpy(&c, mem + 8, 4);
            c += 1;
            memcpy(mem + 8, &c, 4);
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CHECKSUM, "case5b bad count -> ERR_CHECKSUM");
            check_sentinel(&probe, "case5b ram zero-mutation");
            memcpy(mem, keep, T.capacity);
        }

        /* 5c. CORRUPT 中途：第 corrupt_item 条目 dlen 改 0xFFFF + CRC 伪造成一致
         *     → 解析推进到该条目才撞边界 → ERR_CORRUPT；此刻前 corrupt_item 条
         *     已可匹配——非原子实现此刻已污染 ram，本版必须零改动（两阶段原子核心） */
        {
            uint8_t keep[T.capacity];
            uint32_t dlen_off = item_dlen_off(mem, T.corrupt_item);
            uint16_t huge = 0xFFFF;
            CHECK(dlen_off != 0, "case5c dlen field located");
            memcpy(keep, mem, T.capacity);
            memcpy(mem + dlen_off, &huge, 2);
            fix_crc(mem);   /* CRC 一致但结构损坏：抵达解析阶段的唯一途径 */
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CORRUPT, "case5c mid-frame overrun -> ERR_CORRUPT");
            check_sentinel(&probe, "case5c ram zero-mutation (two-phase atomic)");
            memcpy(mem, keep, T.capacity);
        }

        /* 5d. 伪一致 count：count+1 且 CRC 修一致 → 解析多吃一条越界 → ERR_CORRUPT
         *     （兜底层：CRC 被伪造一致时对齐断言拒绝） */
        {
            uint8_t keep[T.capacity];
            uint32_t c;
            memcpy(keep, mem, T.capacity);
            memcpy(&c, mem + 8, 4);
            c += 1;
            memcpy(mem + 8, &c, 4);
            fix_crc(mem);
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CORRUPT, "case5d forged-consistent count -> ERR_CORRUPT");
            check_sentinel(&probe, "case5d ram zero-mutation");
            memcpy(mem, keep, T.capacity);
        }

        /* 5e. 伪一致 data_len：放大 1 且 CRC 修一致 → 条目解析完 off < limit
         *     → 对齐断言拒绝 → ERR_CORRUPT（解析末尾 != 数据区末尾的唯一命中路径） */
        {
            uint8_t keep[T.capacity];
            uint32_t dl;
            memcpy(keep, mem, T.capacity);
            memcpy(&dl, mem + 12, 4);
            dl += 1;
            memcpy(mem + 12, &dl, 4);
            fix_crc(mem);
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CORRUPT, "case5e forged-consistent data_len -> ERR_CORRUPT");
            check_sentinel(&probe, "case5e ram zero-mutation");
            memcpy(mem, keep, T.capacity);
        }

        /* 5f. 后端读失败 → ERR_BACKEND（ram 同样零改动） */
        {
            pstore_backend_t bad_be = { .read = fail_read, .write = ok_write,
                                        .erase = ok_erase, .ctx = NULL };
            pstore_cfg_t bad_cfg = { .backend = &bad_be, .capacity = T.capacity,
                                     .max_items = T.max_items, .magic = T.magic };
            pstore_t *br = pstore_create(&bad_cfg);
            CHECK(br != NULL, "case5f bad-backend instance");
            fill_sentinel(&probe);
            CHECK(pstore_load(br) == PSTORE_ERR_BACKEND, "case5f backend read fail -> ERR_BACKEND");
            check_sentinel(&probe, "case5f ram zero-mutation");
            pstore_destroy(br);
        }

        /* 5g. 帧超容量（data_len 越界）→ ERR_CORRUPT（容量上限判据路径） */
        {
            uint8_t keep[T.capacity];
            uint32_t dl = T.capacity;   /* 必然 > capacity - 20 */
            memcpy(keep, mem, T.capacity);
            memcpy(mem + 12, &dl, 4);
            fix_crc(mem);
            fill_sentinel(&probe);
            CHECK(pstore_load(r) == PSTORE_ERR_CORRUPT, "case5g data_len beyond capacity -> ERR_CORRUPT");
            check_sentinel(&probe, "case5g ram zero-mutation");
            memcpy(mem, keep, T.capacity);
        }

        /* 收尾健康检查：还原后的帧仍可正常加载（介质未被测试构造永久破坏） */
        fill_sentinel(&probe);
        CHECK(pstore_load(r) == PSTORE_OK, "case5 frame intact after restore");
        CHECK(memcmp(&probe.vel, &saved.vel, sizeof(saved.vel)) == 0,
              "case5 healthy reload restores data");
        pstore_destroy(w);
        pstore_destroy(r);
    }

    /* ═══ 6. 多条目增删后重写 ═══ */
    {
        params_t p, fresh;
        pstore_t *ps = NULL, *r = NULL;
        lcg_fill(&(uint32_t){T.lcg_seed ^ 1u}, &p, sizeof(p));

        pstore_ram_backend_init(&be, &ram, mem, T.capacity);
        cfg.backend = &be; cfg.capacity = T.capacity;
        cfg.max_items = T.max_items; cfg.magic = T.magic;
        ps = pstore_create(&cfg);
        CHECK(ps != NULL, "case6 create");

        /* 三条目落盘 */
        CHECK(pstore_register(ps, K_KP,  &p.kp,  sizeof(p.kp))  == PSTORE_OK, "case6 reg kp");
        CHECK(pstore_register(ps, K_VEL, &p.vel, sizeof(p.vel)) == PSTORE_OK, "case6 reg vel");
        CHECK(pstore_register(ps, K_MID, &p.mid, sizeof(p.mid)) == PSTORE_OK, "case6 reg mid");
        pstore_mark_dirty(ps);
        CHECK(pstore_save(ps) == PSTORE_OK, "case6 save 3 entries");
        CHECK(pstore_entry_count(ps) == 3, "case6 count 3");

        /* 增：第四条目（ser）→ 重写 → 四条目帧 */
        uint32_t fl3 = pstore_frame_len(ps);
        CHECK(pstore_register(ps, K_SER, p.ser, sizeof(p.ser)) == PSTORE_OK, "case6 add ser");
        CHECK(pstore_is_dirty(ps) == 0, "case6 register alone stays clean");
        pstore_mark_dirty(ps);
        CHECK(pstore_save(ps) == PSTORE_OK, "case6 save 4 entries");
        CHECK(pstore_frame_len(ps) == pstore_calc_len(ps), "case6 frame == calc (4)");
        CHECK(pstore_frame_len(ps) > fl3, "case6 frame grew with 4th entry");
        uint32_t fl4 = pstore_frame_len(ps);

        /* 新实例全量加载：四条目齐 */
        r = pstore_create(&cfg);
        fill_sentinel(&fresh);
        CHECK(r != NULL && reg_all(r, &fresh) == PSTORE_OK, "case6 reader 4 entries");
        CHECK(pstore_load(r) == PSTORE_OK, "case6 load 4");
        CHECK(memcmp(&fresh, &p, sizeof(p)) == 0, "case6 all 4 roundtrip");

        /* 删：注销 vel → 自动置脏 → 重写 → 帧收缩为三条目 */
        CHECK(pstore_unregister(ps, K_VEL) == PSTORE_OK, "case6 unregister vel");
        CHECK(pstore_is_dirty(ps) == 1, "case6 unregister marks dirty");
        CHECK(pstore_entry_count(ps) == 3, "case6 count back to 3");
        CHECK(pstore_save(ps) == PSTORE_OK, "case6 save after shrink");
        CHECK(pstore_frame_len(ps) == pstore_calc_len(ps), "case6 frame == calc (3)");
        CHECK(pstore_frame_len(ps) < fl4, "case6 frame shrank after removal");

        /* 旧注册面实例加载收缩帧：三条目还原、已删条目不再被介质覆盖 */
        {
            params_t stale;
            pstore_t *rs = pstore_create(&cfg);
            CHECK(rs != NULL, "case6 stale reader create");
            CHECK(pstore_register(rs, K_KP,  &stale.kp,  sizeof(stale.kp))  == PSTORE_OK &&
                  pstore_register(rs, K_VEL, &stale.vel, sizeof(stale.vel)) == PSTORE_OK &&
                  pstore_register(rs, K_MID, &stale.mid, sizeof(stale.mid)) == PSTORE_OK,
                  "case6 stale reader registry (vel kept)");
            fill_sentinel(&stale);
            CHECK(pstore_load(rs) == PSTORE_OK, "case6 stale load ok");
            CHECK(memcmp(&stale.kp, &p.kp, sizeof(p.kp)) == 0, "case6 stale kp applied");
            CHECK(memcmp(&stale.mid, &p.mid, sizeof(p.mid)) == 0, "case6 stale mid applied");
            CHECK(stale.vel == 0xA5A5A5A5u, "case6 stale vel not in frame -> untouched");
            pstore_destroy(rs);
        }
        pstore_destroy(ps);
        pstore_destroy(r);
    }

    /* ═══ 7. 脏检查写计数（不脏零后端调用；NOFIT 不碰后端） ═══ */
    {
        params_t p;
        pstore_t *ps = NULL;
        lcg_fill(&(uint32_t){T.lcg_seed ^ 2u}, &p, sizeof(p));

        pstore_ram_backend_init(&be, &ram, mem, T.capacity);
        cfg.backend = &be; cfg.capacity = T.capacity;
        cfg.max_items = T.max_items; cfg.magic = T.magic;
        ps = pstore_create(&cfg);
        CHECK(ps != NULL && reg_all(ps, &p) == PSTORE_OK, "case7 setup");

        CHECK(pstore_save(ps) == PSTORE_OK, "case7 clean save is no-op ok");
        CHECK(ram.writes == 0 && ram.erases == 0, "case7 clean save: zero backend writes");

        pstore_mark_dirty(ps);
        CHECK(pstore_save(ps) == PSTORE_OK, "case7 dirty save ok");
        CHECK(ram.writes == 1, "case7 dirty save wrote once");
        CHECK(ram.erases == 1, "case7 dirty save erased once");

        CHECK(pstore_save(ps) == PSTORE_OK, "case7 second clean save");
        CHECK(ram.writes == 1 && ram.erases == 1, "case7 still one write (dirty check)");

        CHECK(pstore_load(ps) == PSTORE_OK, "case7 load after save");
        CHECK(pstore_save(ps) == PSTORE_OK, "case7 save after load");
        CHECK(ram.writes == 1 && ram.erases == 1, "case7 load keeps store clean");

        /* NOFIT：小容量装不下帧 → 返回错误且不碰后端，脏保留可重试 */
        {
            uint8_t tiny_mem[T.tiny_capacity];
            pstore_backend_t tbe;
            pstore_ram_t tram;
            pstore_t *tp = NULL;
            pstore_ram_backend_init(&tbe, &tram, tiny_mem, T.tiny_capacity);
            tp = pstore_create(&(pstore_cfg_t){ .backend = &tbe,
                .capacity = T.tiny_capacity, .max_items = T.max_items, .magic = T.magic });
            CHECK(tp != NULL && reg_all(tp, &p) == PSTORE_OK, "case7 tiny setup");
            CHECK(pstore_calc_len(tp) > T.tiny_capacity, "case7 calc_len exceeds tiny capacity");
            pstore_mark_dirty(tp);
            CHECK(pstore_save(tp) == PSTORE_ERR_NOFIT, "case7 oversized frame -> ERR_NOFIT");
            CHECK(tram.writes == 0 && tram.erases == 0, "case7 NOFIT touches no backend");
            CHECK(pstore_is_dirty(tp) == 1, "case7 NOFIT keeps dirty for retry");
            CHECK(pstore_unregister(tp, K_SER) == PSTORE_OK, "case7 drop one entry");
            CHECK(pstore_save(tp) == PSTORE_ERR_NOFIT, "case7 still nofit after one drop");
            pstore_destroy(tp);
        }
        pstore_destroy(ps);
    }

    /* ═══ 8. 断电保持模拟：写→后端快照→改 ram→load(快照)→还原，
     *        1000 循环零失真。断电语义 = 实例销毁重建（pstore 态全失）+
     *        ram 打垃圾 + 介质仅存快照字节。 ═══ */
    {
        params_t live, exp;
        pstore_t *ps = NULL;
        uint32_t seed = T.lcg_seed ^ 3u;
        uint32_t zero_distortion = 0;

        pstore_ram_backend_init(&be, &ram, mem, T.capacity);
        cfg.backend = &be; cfg.capacity = T.capacity;
        cfg.max_items = T.max_items; cfg.magic = T.magic;

        for (uint32_t i = 0; i < T.power_cycles; i++) {
            /* 写：本拍参数落盘 */
            lcg_fill(&seed, &live, sizeof(live));
            memcpy(&exp, &live, sizeof(exp));   /* 期望值定格 */
            ps = pstore_create(&cfg);
            if (ps == NULL || reg_all(ps, &live) != PSTORE_OK) {
                CHECK(0, "case8 instance setup");
                if (ps) pstore_destroy(ps);
                break;
            }
            pstore_mark_dirty(ps);
            if (pstore_save(ps) != PSTORE_OK) {
                CHECK(0, "case8 save");
                pstore_destroy(ps);
                break;
            }
            pstore_destroy(ps);

            /* 后端快照：介质字节定格为断电前状态 */
            memcpy(snap, mem, T.capacity);

            /* 断电：RAM 打垃圾；介质先破坏再回放快照（证 load 读的是快照字节） */
            memset(&live, T.scramble_byte, sizeof(live));
            memset(mem, 0xFF, T.capacity);
            memcpy(mem, snap, T.capacity);

            /* 重启：全新实例 + 全新注册面（ram 仍是垃圾）→ load */
            ps = pstore_create(&cfg);
            if (ps == NULL || reg_all(ps, &live) != PSTORE_OK) {
                CHECK(0, "case8 reboot instance setup");
                if (ps) pstore_destroy(ps);
                break;
            }
            if (pstore_load(ps) != PSTORE_OK) {
                CHECK(0, "case8 reboot load");
                pstore_destroy(ps);
                break;
            }

            /* 数据还原断言：四条目逐字节与期望一致 */
            if (memcmp(&live.kp,  &exp.kp,  sizeof(exp.kp))  == 0 &&
                memcmp(&live.vel, &exp.vel, sizeof(exp.vel)) == 0 &&
                memcmp(live.ser,  exp.ser,  sizeof(exp.ser)) == 0 &&
                memcmp(&live.mid, &exp.mid, sizeof(exp.mid)) == 0 &&
                pstore_frame_len(ps) == pstore_calc_len(ps))
                zero_distortion++;
            else
                CHECK(0, "case8 power-cycle data distortion");
            pstore_destroy(ps);
        }
        CHECK(zero_distortion == T.power_cycles,
              "case8 1000 power cycles zero distortion");
        CHECK(ram.writes == T.power_cycles, "case8 exactly one write per cycle");
        CHECK(ram.reads == T.power_cycles, "case8 exactly one read per cycle");

        /* 尾拍：断电发生在「改 ram 未 save」之后 → load 还原上一拍快照（最新已落盘态） */
        {
            memset(&live, T.scramble_byte, sizeof(live));
            ps = pstore_create(&cfg);
            CHECK(ps != NULL && reg_all(ps, &live) == PSTORE_OK, "case8 tail setup");
            CHECK(pstore_load(ps) == PSTORE_OK, "case8 tail load");
            CHECK(memcmp(&live, &exp, sizeof(exp)) == 0,
                  "case8 unsaved ram changes lost, last snapshot restored");
            pstore_destroy(ps);
        }
    }

    /* ═══ 9. 错误码表完备（err_name 无未知漏网） ═══ */
    {
        for (int e = PSTORE_OK; e >= PSTORE_ERR_CORRUPT; e--) {
            const char *n = pstore_err_name(e);
            CHECK(n != NULL && strcmp(n, "unknown") != 0, "case9 err_name mapped");
        }
        CHECK(strcmp(pstore_err_name(77), "unknown") == 0, "case9 unknown code");
        CHECK(strcmp(pstore_err_name(PSTORE_OK), "ok") == 0, "case9 ok name");
    }

    printf("== test_pstore: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
