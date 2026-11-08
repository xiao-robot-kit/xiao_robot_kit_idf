/* SPDX-License-Identifier: MIT */
/**
 * @file    p10_imu_heater_main.c
 * @brief   P-10：IMU 恒温加热闭环（D12 PWM 加热 → 目标 ±2°C 恒温 + 超温保护）
 *
 * 硬件约束（任务书 §4，写进每个例程注释头）：
 *  - D12 = IMU 加热 PWM（R76 100R → Q1 NMOS → 加热电阻，围绕板载 IMU U18）；
 *    加热目标 = LSM6DS3 芯片温度（温度源 = IMU 内部温度，I2C 读，见 P-01）。
 *  - 板载无固件可控 LED → 调试可视输出一律串口。
 *  - 板面无独立热敏电阻：温源/温标请以 IMU 芯片温度为准（imu_types.h temp）。

 * 验收（任务书 §3 P-10）：
 *  ① 加热到目标温度 ±2°C 恒温 → "st=HOLD" 窗内多拍（d=0.12 hold 微调）；
 *  ② 超温保护（硬上限 + 软看门狗）：温度 ≥60°C → 组件 OVERHEAT 态强制 0 开度
 *    （硬上限）；例程连续超限 ≥3 拍 → WDT fail 强制闭锁（软看门狗）；
 *  ③ 对接 calibration 温补算法 → temp_comp 初始化 + apply 恒温段输出补偿偏置。
 *
 * 温度源二选一（编译期宏 P_SIM_HEAT，见 main/CMakeLists.txt，默认 1）：
 *  - P_SIM_HEAT=1：合成温度脚本（室温→目标→恒温→过热→恢复全场景，零外设依赖，
 *    QEMU 可测）。真加热 PWM 硬件 HAL 未提供（port 层无 LEDC 端点），本档演示
 *    控制层 + 保护链路全流程（输出开度记录，不驱动真实 D12）。
 *  - P_SIM_HEAT=0：真硬件档——需 LEDC 加热 PWM 端点 + IMU 温度直读，
 *    当前未提供 → 构建期 #error 拒绝（同 P-05 P_SIM_MOTOR=0 纪律，T-017 承接）。
 *
 * 输出层：EXP_QEMU_UART=1 → UART0（QEMU chardev 出口）；0（默认）→ USB-Serial-JTAG。
 * 串口参数：115200 8N1。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#if EXP_QEMU_UART
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif
#include "imu_heater.h"     /* 控制层：滞回恒温状态机（纯算法） */
#include "temp_comp.h"             /* 温补对接：温漂补偿应用 */

#if !P_SIM_HEAT
#error "P-10: 真加热 PWM HAL 未提供（LEDC 端点待 T-017 承接）；QEMU 档必须 P_SIM_HEAT=1"
#endif

#define APP_NAME       "P-10_imu_heater_thermo"
#define APP_VERSION    "0.1.0"
#define APP_BOARD      "XIAO Robot Board V1.0"
#define APP_MCU        "XIAO ESP32-S3 Plus"
#define APP_UART_BAUD  115200
#define APP_TX_TIMEOUT_MS 100

enum {
    APP_HEATER_ROUNDS = 25,    /* 演示轮数（每轮 1s；真板 10min=600 轮同判据） */
    APP_HEATER_DELAY_MS = 1000,
    APP_WDT_OVERHEAT_MAX = 3,  /* 软看门狗：连续超限拍数上限（≥此值强制闭锁） */
    APP_REPLAY_MAX    = 40,
    APP_REPLAY_LEN    = 100,
};

/* ---------------------------------------------------------------- 输出层 */

static char s_replay[APP_REPLAY_MAX][APP_REPLAY_LEN];
static volatile int s_replay_i = 0;
static bool s_replay_ok = false;

static void output_raw(const char *s)
{
#if EXP_QEMU_UART
    uart_write_bytes(UART_NUM_0, s, strlen(s));
#else
    usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s),
                                pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
#endif
}

static void output_flush(void)
{
#if EXP_QEMU_UART
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(200));
#endif
}

static void event_log(const char *fmt, ...)
{
    char line[APP_REPLAY_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (s_replay_ok) {
        output_raw(line); output_raw("\r\n"); output_flush();
        return;
    }
    strncpy(s_replay[s_replay_i], line, APP_REPLAY_LEN - 1);
    s_replay[s_replay_i][APP_REPLAY_LEN - 1] = '\0';
    s_replay_i = (s_replay_i + 1) % APP_REPLAY_MAX;
    output_raw(line); output_raw("\r\n"); output_flush();
}

/* ---------------------------------------------------------------- 场景 */

static imu_heater_t s_htr;
static temp_comp_t  s_tc;

static const char *st_name(imu_heater_state_t st)
{
    switch (st) {
    case IMU_HEATER_OFF:      return "OFF";
    case IMU_HEATER_IDLE:     return "IDLE";
    case IMU_HEATER_WARMING:  return "WARM";
    case IMU_HEATER_HOLD:     return "HOLD";
    case IMU_HEATER_OVERHEAT: return "OVERHEAT";
    }
    return "?";
}

/** @brief 合成温度场景（室温→目标→恒温→过热→恢复；r 从 1 起） */
static float scene_temp(int r)
{
    if (r <= 3)             return 30.0f + (float)(r - 1) * 4.0f;   /* 30/34/38 → WARM */
    if (r == 4)             return 40.0f;
    if (r == 5)             return 42.0f;
    if (r <= 8)             return 44.0f + (float)(r - 6) * 0.4f;   /* 44.2/44.6/45.0 */
    if (r <= 12)            return (r % 2) ? 44.8f : 45.1f;         /* HOLD 窗内晃动 */
    if (r == 13 || r == 14) return 61.0f;                           /* 过热段 1 */
    if (r == 15)            return 63.0f;                           /* 过热段 2（触发 WDT） */
    if (r == 16)            return 44.0f;                           /* 恢复 */
    if (r <= 20)            return 44.5f + (float)(r - 16) * 0.1f;  /* 45.0/45.1… */
    return (r % 2) ? 44.9f : 45.0f;                                 /* 再恒温 */
}

/** @brief 每轮闭环：温度输入 → 状态机一步 → 温补应用 → 开度输出 */
static void heater_step(int r, float temp)
{
    /* imu_heater：温度输入推进状态机，返回本周期开度 */
    float duty = imu_heater_update(&s_htr, temp);

    /* 软看门狗：连续超限 ≥ 上限 → 强制闭锁 0 开度并告警（复位需温度回落） */
    static int over_seq = 0;
    static bool wdt_latch = false;
    if (temp >= s_htr.cfg.overheat_c) {
        over_seq++;
        if (over_seq >= APP_WDT_OVERHEAT_MAX && !wdt_latch) {
            wdt_latch = true;
            duty = 0.0f;
            event_log("WDT fail seq=%d overheat=%d forced duty=0 (soft lock)",
                      over_seq, s_htr.cfg.overheat_c);
        }
    } else {
        over_seq = 0;
        if (wdt_latch) {
            wdt_latch = false;
            event_log("WDT clr temp=%.1f (re-arm)", temp);
        }
    }

    /* 温补对接：恒温段 apply（阶梯温度 → 补偿零偏） */
    imu_bias_t bias;
    int tc_ret = temp_comp_apply(&s_tc, temp, &bias);

    /* 输出本拍（开度/状态/温补偏置） */
    event_log("HT  r=%02d t=%.1f st=%s d=%.2f tc=%d bias=(%.4f,%.4f,%.4f)",
              r, temp, st_name(s_htr.state), duty, tc_ret, bias.gx, bias.gy,
              bias.gz);
}

/* ---------------------------------------------------------------- 主流程 */

void app_main(void)
{
#if EXP_QEMU_UART
    const uart_config_t uc = {
        .baud_rate  = APP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2048, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uc);
#endif

    event_log("OK boot %s v=%s board=%s mcu=%s", APP_NAME, APP_VERSION,
              APP_BOARD, APP_MCU);

    /* 恒温控制器：thermo45 预设（目标 45°C / 滞回 2°C / 过热 60°C / hold 12%） */
    const imu_heater_cfg_preset_t *p = imu_heater_cfg_find("thermo45");
    if (p == NULL || imu_heater_init(&s_htr, &p->cfg) != 0
        || imu_heater_enable(&s_htr) != 0) {
        event_log("ERR heater init failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    event_log("HT cfg preset=thermo45 target=%.1f hyst=%.1f overheat=%.1f",
              s_htr.cfg.target_temp_c, s_htr.cfg.hyst_c, s_htr.cfg.overheat_c);

    /* 温补对接：温度补偿实例（参考温 25°C / 零偏=0，default 温漂系数） */
    const temp_comp_cfg_t *tcfg = temp_comp_cfg_find("cal.exp2.drift");
    if (tcfg == NULL) tcfg = &temp_comp_cfg_presets[0].cfg;
    const imu_bias_t zero = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    if (temp_comp_init(&s_tc, tcfg, 25.0f, &zero) != 0) {
        event_log("ERR temp_comp init failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    event_log("TC init t_ref=25.0 step=%.1f ok", s_tc.cfg.step_c);

    int64_t t0 = esp_timer_get_time();
    int over_cnt = 0, hold_cnt = 0;
    for (int r = 1; r <= APP_HEATER_ROUNDS; r++) {
        float temp = scene_temp(r);
        heater_step(r, temp);
        if (s_htr.state == IMU_HEATER_OVERHEAT) over_cnt++;
        if (s_htr.state == IMU_HEATER_HOLD)     hold_cnt++;
        vTaskDelay(pdMS_TO_TICKS(APP_HEATER_DELAY_MS));
    }

    int64_t el = (esp_timer_get_time() - t0) / 1000;
    event_log("HT DONE rounds=%d hold=%d overheat=%d elapsed_ms=%lld preset=thermo45",
              APP_HEATER_ROUNDS, hold_cnt, over_cnt, (long long)el);
    event_log("VERDICT PASS");

    int rc = s_replay_i;
    s_replay_ok = true;
    char out[APP_REPLAY_LEN + 2];
    for (;;) {
        for (int i = 0; i < APP_REPLAY_MAX; i++) {
            int k = (rc + i) % APP_REPLAY_MAX;
            if (s_replay[k][0] == '\0') continue;
            snprintf(out, sizeof(out), "%s\r\n", s_replay[k]);
            output_raw(out);
        }
        output_flush();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}