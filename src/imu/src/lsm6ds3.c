/* SPDX-License-Identifier: MIT */
/**
 * @file    lsm6ds3.c
 * @brief   LSM6DS3TR-C 驱动实现（查表驱动：语义参数 → 寄存器编码 → 灵敏度）
 *
 * @note    本文件是全库唯一的器件知识存放处：换传感器 = 换查表与寄存器序列，
 *          契约面（include/）不动。
 * @note    提炼自本项目旧式实现 旧式实现 的实机验证实现，按新契约重组。
 */

#include "imu/lsm6ds3.h"

/* ---------- 器件寄存器地址（唯一存放处） ---------- */
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10    /* 加速度 ODR[3:0] | FS_XL[1:0] */
#define REG_CTRL2_G     0x11    /* 陀螺 ODR[3:0] | FS_G[1:0] */
#define REG_CTRL3_C     0x12    /* BDU + IF_INC */
#define CTRL3_BDU_IFINC 0x44
#define REG_OUT_TEMP_L  0x20    /* 温度 2 字节，小端 */
#define REG_OUTX_L_G    0x22    /* 陀螺 6 字节起 */
#define REG_OUTX_L_XL   0x28    /* 加速度 6 字节起 */
#define WHO_AM_I_VAL    0x6A

/** 连续失败多少次读后转 FAULT（实测偶发 NACK 容忍度） */
#define FAULT_THRESHOLD 10

/* ---------- 查表：语义值 → 编码 + 灵敏度（数据手册值） ---------- */

typedef struct { uint16_t hz;   uint8_t  code; } odr_entry_t;
typedef struct { uint16_t fs;   uint8_t  code; float sens; } fs_entry_t;

/* ODR 档位（CTRL1_XL / CTRL2_G 共用编码） */
static const odr_entry_t k_odr[] = {
    { 52,  0x3 }, { 104, 0x4 }, { 208, 0x5 }, { 416, 0x6 }, { 833, 0x7 },
};

/* 加速度量程：注意 FS_XL 编码非顺序（00=±2g, 01=±16g, 10=±4g, 11=±8g） */
static const fs_entry_t k_accel_fs[] = {
    {   2, 0x0, 0.000061f },
    {  16, 0x1, 0.000488f },
    {   4, 0x2, 0.000122f },
    {   8, 0x3, 0.000244f },
};

/* 陀螺量程：编码顺序（±125 特殊档不进主表，动态配置实验再议） */
static const fs_entry_t k_gyro_fs[] = {
    {  250, 0x0, 0.00875f },
    {  500, 0x1, 0.01750f },
    { 1000, 0x2, 0.03500f },
    { 2000, 0x3, 0.07000f },
};

static const odr_entry_t *odr_lookup(uint16_t hz)
{
    for (unsigned i = 0; i < sizeof(k_odr) / sizeof(k_odr[0]); i++)
        if (k_odr[i].hz == hz) return &k_odr[i];
    return 0;
}

static const fs_entry_t *fs_lookup(const fs_entry_t *tab, unsigned n, uint16_t fs)
{
    for (unsigned i = 0; i < n; i++)
        if (tab[i].fs == fs) return &tab[i];
    return 0;
}

/* ---------- 生命周期 ---------- */

void lsm6ds3_init(lsm6ds3_t *imu, const imu_i2c_bus_t *bus)
{
    imu->bus        = bus;
    imu->state      = LSM6DS3_STATE_UNINIT;
    imu->accel_sens = 0.0f;
    imu->gyro_sens  = 0.0f;
    imu->fault_cnt  = 0;
}

int lsm6ds3_probe(lsm6ds3_t *imu)
{
    if (!imu->bus) return -1;
    uint8_t id = 0;
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_WHO_AM_I, &id, 1, imu->bus->ctx) != 0)
        return -1;
    if (id != WHO_AM_I_VAL)
        return -2;
    imu->state = LSM6DS3_STATE_PROBED;
    return 0;
}

int lsm6ds3_configure(lsm6ds3_t *imu, const imu_cfg_t *cfg)
{
    if (!imu->bus) return -1;

    const odr_entry_t *a_odr = odr_lookup(cfg->accel_odr_hz);
    const odr_entry_t *g_odr = odr_lookup(cfg->gyro_odr_hz);
    const fs_entry_t  *a_fs  = fs_lookup(k_accel_fs, sizeof(k_accel_fs) / sizeof(k_accel_fs[0]), cfg->accel_fs_g);
    const fs_entry_t  *g_fs  = fs_lookup(k_gyro_fs,  sizeof(k_gyro_fs)  / sizeof(k_gyro_fs[0]),  cfg->gyro_fs_dps);
    if (!a_odr || !g_odr || !a_fs || !g_fs)
        return -2;  /* 配置含不支持的档位（表外值，拒绝写硬件） */

    uint8_t ctrl1 = (uint8_t)((a_odr->code << 4) | (a_fs->code << 2));
    uint8_t ctrl2 = (uint8_t)((g_odr->code << 4) | (g_fs->code << 2));

    if (imu->bus->write(LSM6DS3_I2C_ADDR, REG_CTRL1_XL, &ctrl1, 1, imu->bus->ctx) != 0) return -1;
    if (imu->bus->write(LSM6DS3_I2C_ADDR, REG_CTRL2_G,  &ctrl2, 1, imu->bus->ctx) != 0) return -1;
    if (imu->bus->write(LSM6DS3_I2C_ADDR, REG_CTRL3_C, &(uint8_t){CTRL3_BDU_IFINC}, 1, imu->bus->ctx) != 0) return -1;

    imu->cfg        = *cfg;
    imu->accel_sens = a_fs->sens;
    imu->gyro_sens  = g_fs->sens;
    imu->state      = LSM6DS3_STATE_READY;
    imu->fault_cnt  = 0;
    return 0;
}

/* ---------- 数据通道 ---------- */

/* 总线失败计数 → FAULT 转移；成功清零 */
static int bus_fail(lsm6ds3_t *imu)
{
    if (++imu->fault_cnt >= FAULT_THRESHOLD)
        imu->state = LSM6DS3_STATE_FAULT;
    return -1;
}

static void bus_ok(lsm6ds3_t *imu)
{
    imu->fault_cnt = 0;
    if (imu->state == LSM6DS3_STATE_FAULT)
        imu->state = LSM6DS3_STATE_READY;  /* 恢复采样即回 READY */
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int lsm6ds3_read_raw(lsm6ds3_t *imu, lsm6ds3_raw_t *out)
{
    uint8_t g[6], a[6];
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_OUTX_L_G,  g, 6, imu->bus->ctx) != 0)
        return bus_fail(imu);
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_OUTX_L_XL, a, 6, imu->bus->ctx) != 0)
        return bus_fail(imu);
    out->gx = le16(&g[0]); out->gy = le16(&g[2]); out->gz = le16(&g[4]);
    out->ax = le16(&a[0]); out->ay = le16(&a[2]); out->az = le16(&a[4]);
    bus_ok(imu);
    return 0;
}

int lsm6ds3_read(lsm6ds3_t *imu, imu_sample_t *out)
{
    lsm6ds3_raw_t raw;
    if (lsm6ds3_read_raw(imu, &raw) != 0)
        return -1;
    out->ax = (float)raw.ax * imu->accel_sens;
    out->ay = (float)raw.ay * imu->accel_sens;
    out->az = (float)raw.az * imu->accel_sens;
    out->gx = (float)raw.gx * imu->gyro_sens;
    out->gy = (float)raw.gy * imu->gyro_sens;
    out->gz = (float)raw.gz * imu->gyro_sens;
    return 0;
}

int lsm6ds3_read_temp(lsm6ds3_t *imu, float *celsius)
{
    uint8_t t[2];
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_OUT_TEMP_L, t, 2, imu->bus->ctx) != 0)
        return bus_fail(imu);
    *celsius = 25.0f + (float)le16(t) / 256.0f;
    bus_ok(imu);
    return 0;
}

int lsm6ds3_get_ctrl(lsm6ds3_t *imu, uint8_t *ctrl1_xl, uint8_t *ctrl2_g)
{
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_CTRL1_XL, ctrl1_xl, 1, imu->bus->ctx) != 0)
        return -1;
    if (imu->bus->read(LSM6DS3_I2C_ADDR, REG_CTRL2_G,  ctrl2_g,  1, imu->bus->ctx) != 0)
        return -1;
    return 0;
}
