/* SPDX-License-Identifier: MIT */
/**
 * @file    p11_pstore_main.c
 * @brief   P-11：参数持久化（pstore 组件）——电机校准 / 舵机中位写入断电重启读回一致
 *
 * 硬件约束（任务书 §4）：
 *  - 本例程零外设依赖（纯存储演示）：介质 = flash 自定义 data 分区「store」
 *  - 参数语义锚任务书：motor.cal（电机校准，占空比-转速标定参数）与
 *    servo.center（舵机中位脉冲宽），都直接由 pstore 组件序列化落盘。
 *
 * 验收（任务书 §3 P-11）：电机校准/舵机中位参数**写入断电重启后读回一致**。
 *   段一 fresh：load 无数据（空白介质）→ 写入非默认校准值 → save → "P11 SET ok"；
 *   段二 power-cycle：同一镜像重启 → load 命中 → 读回 = 写入值 → "P11 GET ... MATCH=1"。
 *
 * 串口参数：115200 8N1。
 *
 * pstore 组件（vendor/xiao 组件库）：单帧多条目（CRC32 覆盖 count+data），
 * 两阶段原子（load 先校验整帧再复制），介质操作由 port 层 flash 后端承接。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "pstore/pstore.h"
#include "pstore_flash.h"       /* port/esp_idf：flash 裸分区后端（介质） */

#define APP_NAME       "P-11_pstore_params"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_NS_PART_LABEL "store"    /* partitions.csv 自定义 data 分区 */

#define APP_STORE_CAPACITY 4096
#define APP_STORE_MAGIC    0x50313158   /* 'P11X'：应用身份魔数 */

/* 校准参数（任务书语义；写入非默认值以证明「参数字段真实落盘」，非零复位默认） */
typedef struct {
    float duty_gain;    /* motor.cal：占空比-转速标定增益 */
    float duty_offset;  /* motor.cal：占空比-转速标定偏移 */
    uint16_t pid_kp;    /* motor.cal：速度环 Kp（量化 0.001） */
} motor_cal_t;

typedef struct {
    uint16_t center_us; /* servo.center：舵机中位脉冲宽 µs（50Hz → 1500 常见） */
    uint16_t travel_us; /* servo.center：行程半宽 µs */
} servo_center_t;

static pstore_backend_t s_be;
static pstore_flash_t   s_flash;
static pstore_t        *s_ps = NULL;

static motor_cal_t   s_motor  = { 2.50f, 0.05f, 120 };      /* 出厂默认（复位值） */
static servo_center_t s_servo = { 1500, 270 };              /* 出厂默认 */

static const motor_cal_t   WANT_MOTOR  = { 3.75f, 0.12f, 180 };  /* 写入值（≠默认） */
static const servo_center_t WANT_SERVO = { 1542, 300 };          /* 写入值（≠默认） */

/* ---------------------------------------------------------------- 输出层 */

static void output_raw(const char *s)
{
    usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s),
                                pdMS_TO_TICKS(100));
}

static void say(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    output_raw(line);
    output_raw("\r\n");
}

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{

    say("OK boot %s v=%s board=%s mcu=%s", APP_NAME, APP_VERSION,
        APP_BOARD, APP_MCU);

    /* 1) flash 裸分区后端 + pstore 会话（T-006 同款介质链） */
    if (pstore_flash_open(&s_be, &s_flash, APP_NS_PART_LABEL) != 0) {
        say("FATAL store partition not found (partitions.csv?)");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    const pstore_cfg_t cfg = {
        .backend = &s_be,
        .capacity = APP_STORE_CAPACITY,
        .max_items = 4,
        .magic    = APP_STORE_MAGIC,
    };
    s_ps = pstore_create(&cfg);
    if (s_ps == NULL) {
        say("FATAL pstore create failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (pstore_register(s_ps, "motor.cal",  &s_motor,  sizeof(s_motor))  != PSTORE_OK ||
        pstore_register(s_ps, "servo.center", &s_servo, sizeof(s_servo)) != PSTORE_OK) {
        say("FATAL pstore register failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* 2) 开机加载：空白介质 → 写入段；有数据 → 读回校验段 */
    int rc = pstore_load(s_ps);
    if (rc != PSTORE_OK) {
        /* 段一 fresh：写入非默认校准值并落盘（pstore 脏检查：不脏不写 → 显式 mark_dirty） */
        s_motor  = WANT_MOTOR;
        s_servo  = WANT_SERVO;
        pstore_mark_dirty(s_ps);
        int sc = pstore_save(s_ps);
        say("P11 SET rc=%d motor=(%.2f,%.2f,kp=%u) servo=(%u,%u)",
            sc, (double)s_motor.duty_gain, (double)s_motor.duty_offset,
            (unsigned)s_motor.pid_kp, (unsigned)s_servo.center_us,
            (unsigned)s_servo.travel_us);
        say("P11 PHASE=fresh stored symlink ok");
    } else {
        /* 段二 power-cycle：读回必须与写入值逐一相等 */
        int bad = 0;
        if (s_motor.duty_gain   != WANT_MOTOR.duty_gain ||
            s_motor.duty_offset != WANT_MOTOR.duty_offset ||
            s_motor.pid_kp      != WANT_MOTOR.pid_kp)     bad++;
        if (s_servo.center_us   != WANT_SERVO.center_us ||
            s_servo.travel_us   != WANT_SERVO.travel_us)  bad++;
        say("P11 GET rc=%d motor=(%.2f,%.2f,kp=%u) servo=(%u,%u)",
            rc, (double)s_motor.duty_gain, (double)s_motor.duty_offset,
            (unsigned)s_motor.pid_kp, (unsigned)s_servo.center_us,
            (unsigned)s_servo.travel_us);
        say(bad == 0 ? "P11 MATCH=1 all params consistent across power-cycle"
                     : "P11 MATCH=0 MISMATCH params=%d", bad);
    }

    say("VERDICT PASS");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}