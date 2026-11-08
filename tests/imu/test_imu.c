/* SPDX-License-Identifier: MIT */
/**
 * @file    test_imu.c
 * @brief   IMU 驱动库包无硬件单测（mock I2C 总线 + 内存寄存器阵列）
 *
 * @note    自研极简断言（原创律：不引第三方测试框架）。
 * @note    验证范围：查表正确性 / 寄存器写序列 / 换算 / 生命周期转移。
 *          不覆盖：真实总线时序（需实机，实验一在板验证）。
 */

#include <stdio.h>
#include <string.h>
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"

/* ---------- 极简断言 ---------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else      { g_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)
#define CHECK_EQ(a, b, msg) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a == _b) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s: %ld != %ld (line %d)\n", msg, _a, _b, __LINE__); } \
} while (0)
static int feq(float a, float b) { float d = a - b; return d < 0.000001f && d > -0.000001f; }

/* ---------- mock I2C：内存寄存器阵列 ---------- */
#define MOCK_ADDR 0x6A
static uint8_t  regs[256];            /* 器件寄存器影像 */
static uint8_t  wr_log[64];           /* 写寄存器地址序列 */
static int      wr_len;
static int      fail_next;            /* 注入接下来 N 次总线操作失败 */

static int mock_write(uint8_t addr, uint8_t reg, const uint8_t *tx, size_t n, void *ctx)
{
    (void)ctx;
    if (fail_next > 0) { fail_next--; return -1; }
    if (addr != MOCK_ADDR || reg + n > 256) return -1;
    for (size_t i = 0; i < n; i++) regs[(reg + i) & 0xFF] = tx[i];
    if (wr_len < 64) wr_log[wr_len++] = reg;
    return 0;
}
static int mock_read(uint8_t addr, uint8_t reg, uint8_t *rx, size_t n, void *ctx)
{
    (void)ctx;
    if (fail_next > 0) { fail_next--; return -1; }
    if (addr != MOCK_ADDR) return -1;
    for (size_t i = 0; i < n; i++) rx[i] = regs[(reg + i) & 0xFF];
    return 0;
}
static const imu_i2c_bus_t mock_bus = { mock_write, mock_read, 0 };

static void mock_reset(void)
{
    memset(regs, 0, sizeof(regs));
    wr_len = 0; fail_next = 0;
    regs[0x0F] = 0x6A;   /* WHO_AM_I */
}
/* 写一个 16 位小端数据到寄存器影像 */
static void mock_put16(uint8_t reg, int16_t v)
{
    regs[reg] = (uint8_t)(v & 0xFF);
    regs[reg + 1] = (uint8_t)((uint16_t)v >> 8);
}

int main(void)
{
    lsm6ds3_t imu;

    /* ---- 1. probe：器件应答正确 ---- */
    mock_reset();
    lsm6ds3_init(&imu, &mock_bus);
    CHECK_EQ(lsm6ds3_probe(&imu), 0, "probe ok");
    CHECK_EQ(imu.state, LSM6DS3_STATE_PROBED, "state PROBED");

    /* ---- 2. probe：ID 不符 ---- */
    mock_reset(); regs[0x0F] = 0x00;
    lsm6ds3_init(&imu, &mock_bus);
    CHECK_EQ(lsm6ds3_probe(&imu), -2, "probe wrong id");

    /* ---- 3. probe：总线失败 ---- */
    mock_reset(); fail_next = 1;
    lsm6ds3_init(&imu, &mock_bus);
    CHECK_EQ(lsm6ds3_probe(&imu), -1, "probe bus fail");

    /* ---- 4. configure：写序列与灵敏度查表 ---- */
    mock_reset();
    lsm6ds3_init(&imu, &mock_bus);
    const imu_cfg_t *c = imu_cfg_find("imu.raw.104");
    CHECK(c != 0, "find preset imu.raw.104");
    CHECK_EQ(lsm6ds3_configure(&imu, c), 0, "configure ok");
    /* 104Hz=0x4, ±2g=0x0 → CTRL1_XL = 0x40；±250dps=0x0 → CTRL2_G = 0x40 */
    CHECK_EQ(regs[0x10], 0x40, "CTRL1_XL value");
    CHECK_EQ(regs[0x11], 0x40, "CTRL2_G value");
    CHECK_EQ(regs[0x12], 0x44, "CTRL3_C BDU+IFINC");
    CHECK_EQ(wr_log[0], 0x10, "write order 1");
    CHECK_EQ(wr_log[1], 0x11, "write order 2");
    CHECK_EQ(wr_log[2], 0x12, "write order 3");
    CHECK(feq(imu.accel_sens, 0.000061f), "accel sens 2g");
    CHECK(feq(imu.gyro_sens, 0.00875f), "gyro sens 250dps");
    CHECK_EQ(imu.state, LSM6DS3_STATE_READY, "state READY");
    /* cfg 同步 */
    CHECK_EQ(imu.cfg.accel_odr_hz, 104, "cfg synced");

    /* ---- 5. configure：表外档位拒绝且不写硬件 ---- */
    mock_reset();
    lsm6ds3_init(&imu, &mock_bus);
    imu_cfg_t bad = *imu_cfg_find("imu.raw.104");
    bad.accel_odr_hz = 777;   /* 查表外 */
    CHECK_EQ(lsm6ds3_configure(&imu, &bad), -2, "configure reject out-of-table");
    CHECK_EQ(wr_len, 0, "no hw write on reject");

    /* ---- 6. read：已知 LSB → 物理量 ---- */
    mock_reset();
    lsm6ds3_init(&imu, &mock_bus);
    lsm6ds3_configure(&imu, imu_cfg_find("imu.raw.104"));
    mock_put16(0x28, 1000);  /* ax = 1000 LSB × 0.000061 = 0.061 g */
    mock_put16(0x22, -2000); /* gx = -2000 LSB × 0.00875 = -17.5 dps */
    imu_sample_t s;
    CHECK_EQ(lsm6ds3_read(&imu, &s), 0, "read ok");
    CHECK(feq(s.ax, 0.061f), "ax conversion");
    CHECK(feq(s.gx, -17.5f), "gx conversion");

    /* ---- 7. read_temp：raw=256 → 26°C ---- */
    mock_reset(); mock_put16(0x20, 256);
    lsm6ds3_init(&imu, &mock_bus);
    float t;
    CHECK_EQ(lsm6ds3_read_temp(&imu, &t), 0, "temp read ok");
    CHECK(feq(t, 26.0f), "temp 256LSB -> 26C");

    /* ---- 8. FAULT 转移与恢复 ---- */
    mock_reset();
    lsm6ds3_init(&imu, &mock_bus);
    lsm6ds3_configure(&imu, imu_cfg_find("imu.raw.104"));
    fail_next = 10;  /* 连续 10 次失败 */
    imu_sample_t s2;
    for (int i = 0; i < 10; i++) lsm6ds3_read(&imu, &s2);
    CHECK_EQ(imu.state, LSM6DS3_STATE_FAULT, "state FAULT after 10 fails");
    fail_next = 0;
    CHECK_EQ(lsm6ds3_read(&imu, &s2), 0, "read recovers");
    CHECK_EQ(imu.state, LSM6DS3_STATE_READY, "state back READY");

    /* ---- 9. 预设查表：命中/未命中 ---- */
    CHECK(imu_cfg_find("imu.attitude.208") != 0, "find attitude preset");
    CHECK(imu_cfg_find("no.such.preset") == 0, "find miss");
    CHECK_EQ(imu_cfg_preset_count, 3, "preset count");

    printf("== test_imu: %d pass, %d fail ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
