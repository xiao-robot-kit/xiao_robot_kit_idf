/* SPDX-License-Identifier: MIT */
/**
 * @file    exp2_main.c
 * @brief   实验二：调零 + 温补 + 裸数据直出（读 N 帧标零偏 → 去偏 → 温补 → 0x02 帧直出）
 *
 * @note    数据源二选一（编译期宏 EXP_SIM，main/CMakeLists.txt 定义）：
 *          - EXP_SIM=1：testing/imu_sim 固件层合成源（跳过 I2C probe，
 *            无传感器跑通全链路；注入零偏/温漂真值，供效果验收）
 *          - EXP_SIM=0：LSM6DS3TR-C(0x6A, I2C@GPIO5/6) 真传感器（复用 port_i2c）
 * @note    链路：源(六轴+温度) → imu_calib（N 帧均值标零偏，重力轴自动判）
 *          → temp_comp（bias = bias_ref + k·(T−t_ref)，阶梯式重算）
 *          → 去偏样本 → proto/frame TYPE_IMU_CAL(0x02) → 帧输出层（二选一，编译期）：
 *          - EXP_QEMU_UART=0（默认）：USB-Serial-JTAG（硬件上位机链路，同 exp1）
 *          - EXP_QEMU_UART=1：UART0 直写——QEMU esp32s3 的 USB-Serial-JTAG 设备
 *            无 chardev 出口（验收波实测三 -serial 槽 0 字节），而 UART0 = QEMU
 *            第 1 个 -serial 槽有出口（ROM banner 实证）；EXP_SIM 联调走此路。
 * @note    下行命令通道（REQ-003，算法全在 MCU）：同一条串口上收
 *          TYPE_CMD(0x7D) 帧 → `cal_cmd` 组帧/校验/分发 → TYPE_ACK(0x7E) 应答：
 *          `CALIBRATE(0x01)` 异步（采完才回 ACK）/ `GET_BIAS(0x02)` /
 *          `TEMP_CALIB(0x03)` 异步 / `SET_FLAGS(0x09)` / `GET_STATUS(0x0A)` /
 *          `GET_DRIFT(0x0D)` / `GET_TC_STATE(0x10)`。
 *          明文行 `CAL`（上一轮上位机旧通道）保留为 CALIBRATE 的兼容别名。
 *          上位机只发命令/读数据，**不做任何标定数学**。
 * @note    参数零写死：标定帧数 = 预设 "cal.exp2.bias"、温标系数 = 预设
 *          "cal.exp2.drift"（xiao 组件库 calibration 组件注册表）；
 *          数据源 = 预设 "sim.exp2.static" / "imu.calib.104"。
 * @note    **参数持久化（T-014）**：零偏 + 锚点温度 + 温漂系数 + 功能位 + 两个 valid 位
 *          落进 NVS（介质后端 `examples/_shared/pstore_nvs`；帧/CRC32/两阶段原子由
 *          xiao 组件库 `pstore` 负责）。开机两种路径：
 *          - 空介质/坏帧/schema 不符 → 跑一次开机静止标定窗 → 落盘（首次上电路径）；
 *          - 有健康存档 → **直接恢复并跳过标定窗**（零偏/系数/开关与上次一致）。
 *          运行时 CALIBRATE / TEMP_CALIB / SET_FLAGS 完成即自动落盘（各回一行 TEXT 证据）。
 *          NVS 不可用不致命 → 降级为"每次开机重新标定"（功能全在）。
 * @note    锚点契约（temp_comp.h）：t_ref 取零偏标定窗内的**平均温度**
 *          （线性模型下均值零偏对应均值温度，锚点不错位）。
 * @note    验收：
 *          - SIM：0x02 帧补偿后块 ≈ 合成真值 (0,0,+1g,0,0,0)——注入温漂
 *            与 cal.exp2.drift 默认系数同值，温补应整抵消；
 *          - 硬件（EXP_SIM=0）：上位机收 0x02 帧，静止时补偿后 az ≈ ±1g、
 *            陀螺各轴 ≈ 0，温升过程零偏块随温度阶梯走。
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if EXP_QEMU_UART
#include "driver/uart.h"           /* 帧出口 UART0（QEMU -serial 第 1 槽）；宏见 main/CMakeLists.txt */
#else
#include "driver/usb_serial_jtag.h"
#endif
#include "imu/imu_types.h"
#include "cal/imu_calib.h"
#include "cal/temp_comp.h"
#include "cal_cmd.h"
#include "proto/frame.h"
#include "pstore/pstore.h"
#include "pstore_nvs.h"       /* port/esp_idf：pstore 的 NVS 介质后端 */
#if EXP_SIM
#include "imu_sim.h"
#else
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "port_i2c.h"         /* port/esp_idf */
#endif

/* 应用管路常数（具名防 magic number；非传感器参数，不入配置表） */
enum {
    APP_TX_TIMEOUT_MS  = 100,   /* 串口帧写超时，ms（USB-Serial-JTAG 路径） */
    APP_UART_BAUD      = 115200,/* QEMU UART0 帧出口波特率（与 QEMU chardev 后端一致） */
    APP_UART_RX_BUF    = 256,   /* UART0 驱动 RX 缓冲（install 约束须 >FIFO_LEN=128；
                                   CMD 帧下行也走此口，见 service_rx） */
    APP_FAIL_FATAL     = 200,   /* 标定期连续读失败上限（超限判源死） */
    APP_LOG_EVERY      = 100,   /* 流送期限流报错取模（次） */
    APP_PROBE_RETRY_MS = 1000,  /* I2C probe 重试间隔，ms（沿 exp1） */
    APP_STORE_CAPACITY = 512,   /* 校准存档区字节数（帧 ~200B，留同量级余量；NVS blob 上限内） */
};

/* NVS 命名（一份介质一个 blob；命名空间/键名 ≤ 15 字符，见 pstore_nvs.h） */
#define APP_STORE_NS    "calstore"
#define APP_STORE_KEY   "cal.frame"

/* 0x02 帧负载布局常数（与 proto/frame.h 的 TYPE_IMU_CAL 规格注释同源） */
#define EXP2_CAL_FLOATS  19   /* 原始 6 + 补偿后 6 + 零偏 6 + 温度 1 */
#define EXP2_CAL_BYTES   (EXP2_CAL_FLOATS * 4)

/** 秒 → 毫秒换算（单位换算常量，非可调参数） */
#define EXP2_MS_PER_S    1000.0f

static uint8_t s_seq = 0;

/** 标定命令会话（状态实例内；算法实例也在里面） */
static cal_cmd_t s_cmd;

/* 持久化介质（NVS 后端；pstore 实例由 cal_cmd 持有，见 cal_cmd_store_open） */
static pstore_backend_t s_be;
static pstore_nvs_t     s_nvs;

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = proto_frame_encode(type, s_seq++, payload, len, buf, sizeof(buf));
    if (n)
#if EXP_QEMU_UART
        /* tx 环形缓冲 0 = 直写 FIFO 模式，驱动内部阻塞排空（uart.c:1668 起） */
        uart_write_bytes(UART_NUM_0, buf, n);
#else
        usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
#endif
}

static void send_text(const char *msg)
{
    size_t n = strlen(msg);
    if (n > PROTO_PAYLOAD_MAX) n = PROTO_PAYLOAD_MAX;   /* 帧 payload 上限内截断（不静默丢整行） */
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)n);
}

/** 帧出口回调（cal_cmd 用）：与数据帧共用同一条链路口 */
static void tx_send(void *io, uint8_t type, const uint8_t *payload, uint8_t len)
{
    (void)io;
    send_frame(type, payload, len);
}

/**
 * @brief   收一拍下行字节并交给命令会话组帧/分发（非阻塞）
 *
 * @note    帧与明文行共用同一串口（UART0 / USB-Serial-JTAG），解析分区在 cal_cmd 内。
 * @note    timeout=0：主循环节奏不被串口拖住（RX 由驱动环形缓冲暂存）。
 */
static void service_rx(void)
{
    uint8_t buf[64];
#if EXP_QEMU_UART
    const int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), 0);
#else
    const int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
#endif
    if (n > 0) cal_cmd_rx(&s_cmd, buf, (size_t)n, tx_send, NULL);
}

/* float → 毫单位整数（文本帧用；截断取整，仅展示不做判据） */
static int f_to_milli(float v)
{
    return (int)(v * 1000.0f);
}

/* 0x02 帧打包：19×float LE（零偏块按采样序重排，见 frame.h 规格） */
static void pack_cal(const imu_sample_t *raw, const imu_sample_t *deb,
                     const imu_bias_t *bias, float temp, uint8_t *out)
{
    const float v[EXP2_CAL_FLOATS] = {
        raw->ax, raw->ay, raw->az, raw->gx, raw->gy, raw->gz,
        deb->ax, deb->ay, deb->az, deb->gx, deb->gy, deb->gz,
        bias->ax, bias->ay, bias->az, bias->gx, bias->gy, bias->gz,
        temp,
    };
    memcpy(out, v, EXP2_CAL_BYTES);   /* Xtensa 小端，与协议字节序一致（同 exp1） */
}

/* ---------------------------------------------------------------- 数据源 */

#if EXP_SIM
static imu_sim_t s_src;       /* 合成源实例（状态实例内，见 imu_sim.h） */
#else
static lsm6ds3_t s_src;       /* 传感器驱动实例 */
#endif
static uint32_t s_period_ms;  /* 推流周期，ms（源配置派生） */

/**
 * @brief   初始化数据源并推导推流周期
 * @retval  0  就绪
 * @retval -1  初始化失败（总线/器件/配置非法）
 * @retval -2  预设名未命中（注册表缺项，工程装配错误）
 */
static int source_init(void)
{
#if EXP_SIM
    const imu_sim_cfg_t *sc = imu_sim_cfg_find("sim.exp2.static");
    if (!sc) return -2;
    if (imu_sim_init(&s_src, sc) != 0) return -1;
    s_period_ms = (uint32_t)(EXP2_MS_PER_S / sc->sample_rate_hz + 0.5f);  /* ms/拍 */

    char hi[96];
    snprintf(hi, sizeof(hi),
             "OK sim: static Z_UP %uHz temp %dmC ramp %+dmC/s bias_g %+dmdps",
             (unsigned)sc->sample_rate_hz, f_to_milli(sc->temp_start_c),
             f_to_milli(sc->temp_ramp_c_per_s), f_to_milli(sc->bias.gx));
    send_text(hi);
    return 0;
#else
    const imu_cfg_t *ic = imu_cfg_find("imu.calib.104");
    if (!ic) return -2;
    if (esp_port_i2c_init(ic->i2c_speed_hz) != 0) return -1;

    lsm6ds3_init(&s_src, &esp_port_i2c_bus);
    for (;;) {
        int r = lsm6ds3_probe(&s_src);
        if (r == 0) break;
        send_text(r == -2 ? "ERR who_am_i mismatch" : "ERR i2c no ack, retry 1s");
        vTaskDelay(pdMS_TO_TICKS(APP_PROBE_RETRY_MS));
    }
    send_text("OK probe: LSM6DS3TR-C @0x6A");
    if (lsm6ds3_configure(&s_src, ic) != 0) return -1;
    s_period_ms = ic->sample_period_ms;
    return 0;
#endif
}

/**
 * @brief   读一拍：六轴样本 + 同拍温度
 * @param[out] s     样本（g / dps，未去偏原始值）
 * @param[out] temp  温度，°C（SIM：先读温度后读样本 = 严格同拍，见 imu_sim.h）
 * @retval  0  两者已写入
 * @retval -1  读失败（总线 / 实例态）
 */
static int source_read(imu_sample_t *s, float *temp)
{
#if EXP_SIM
    if (imu_sim_read_temp(&s_src, temp) != 0) return -1;
    return imu_sim_read(&s_src, s);
#else
    if (lsm6ds3_read(&s_src, s) != 0) return -1;
    return lsm6ds3_read_temp(&s_src, temp);
#endif
}

/* ---------------------------------------------------------------- 主流程 */

/**
 * @brief   零偏标定阶段：读 N 帧求零偏，同时取窗口平均温度做温补锚点
 * @param[in]  cc     标定配置（预设 cal.exp2.bias）
 * @param[out] bias   零偏（dps / g）
 * @param[out] t_ref  标定窗平均温度，°C
 * @retval  0  完成
 * @retval -1  失败（连续读失败超上限 / 标定器拒绝）
 */
static int calibrate_bias(const imu_calib_cfg_t *cc, imu_bias_t *bias,
                          float *t_ref)
{
    imu_calib_t cal;
    if (imu_calib_init(&cal, cc) != 0) return -1;
    if (imu_calib_start(&cal) != 0) return -1;

    double t_sum = 0.0;
    uint32_t t_n = 0;
    uint32_t fails = 0;
    while (imu_calib_ready(&cal) != 1) {
        imu_sample_t s;
        float temp;
        if (source_read(&s, &temp) != 0) {
            if (++fails >= APP_FAIL_FATAL) return -1;   /* 源死，标定无意义 */
            vTaskDelay(pdMS_TO_TICKS(s_period_ms));
            continue;
        }
        fails = 0;
        /* 首轮标定无已知零偏：raw 传 NULL，重力轴判定退化为用 s 本身 */
        if (imu_calib_feed(&cal, &s, NULL) != 0) return -1;
        t_sum += (double)temp;
        t_n++;
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }
    if (t_n == 0) return -1;                /* 不可达护栏（ready ⇒ 已喂满 N） */
    if (imu_calib_result(&cal, bias) != 0) return -1;
    *t_ref = (float)(t_sum / (double)t_n);  /* 线性模型锚点 = 均值温度 */
    return 0;
}

void app_main(void)
{
#if EXP_QEMU_UART
    /* QEMU 帧出口初始化：UART0 115200 8N1 + 驱动安装（write_bytes 前置必要条件，
       uart.c:1704；tx 环形缓冲 0 = 直写 FIFO）。console 仍在 USB-Serial-JTAG，
       UART0 仅承载协议帧，ROM 早期横幅与其共用槽位由帧解码器 SOF+CRC 容错。 */
    const uart_config_t uc = {
        .baud_rate  = APP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &uc);
    uart_driver_install(UART_NUM_0, APP_UART_RX_BUF, 0, 0, NULL, 0);
#else
    usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&uc);
#endif

    /* 预设装配（参数零写死：全部来自注册表） */
    const imu_calib_cfg_t *cal_cfg = imu_calib_cfg_find("cal.exp2.bias");
    if (!cal_cfg) { send_text("FATAL preset cal.exp2.bias not found"); return; }
    const temp_comp_cfg_t *tc_cfg = temp_comp_cfg_find("cal.exp2.drift");
    if (!tc_cfg) { send_text("FATAL preset cal.exp2.drift not found"); return; }

    int r = source_init();
    if (r != 0) {
        send_text(r == -2 ? "FATAL source preset not found"
                          : "FATAL source init failed");
        return;
    }

    /* 持久化装配（失败不致命：无存档只是每次开机重新标定，功能全在） */
    const int store_ok =
        (pstore_nvs_backend_init(&s_be, &s_nvs, APP_STORE_NS, APP_STORE_KEY) == 0);
    if (!store_ok) send_text("WARN nvs store unavailable -> no persistence");

    /* 会话前置：先读一拍拿初始温度（零偏占位 0；未标定 → 温补双调不成立）。
       为什么要这一步：cal_cmd_init 需要锚点温度，而锚点只能从数据源得。 */
    imu_sample_t s0;
    float t0 = 0.0f;
    for (uint32_t fails = 0; ; ) {
        if (source_read(&s0, &t0) == 0) break;
        if (++fails >= APP_FAIL_FATAL) {
            send_text("FATAL source dead before calib");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }
    const imu_bias_t zero_bias = { 0 };
    if (cal_cmd_init(&s_cmd, cal_cfg, tc_cfg, t0, &zero_bias, 0) != 0) {
        send_text("FATAL cal cmd init failed");
        return;
    }

    /* 开机恢复：有健康存档 → 直接恢复（跳过标定窗）；否则跑一次标定窗再落盘。
       restored == 0 且 bias_valid == 1 才算"已标定过"（存档可能只是没标定的空壳）。 */
    int restored = PSTORE_ERR_ARG;   /* 未接入存储时按"无存档"处理 */
    if (store_ok) {
        const cal_cmd_store_cfg_t scol = {
            .backend  = &s_be,
            .capacity = APP_STORE_CAPACITY,
            .magic    = CAL_CMD_STORE_MAGIC,
            .schema   = CAL_CMD_STORE_SCHEMA,
        };
        restored = cal_cmd_store_open(&s_cmd, &scol);
        char hi[96];
        if (restored == 0) {
            snprintf(hi, sizeof(hi),
                     "OK store restore bias_valid=%d tc_valid=%d t_ref=%dmC flags=0x%04X",
                     s_cmd.bias_valid, s_cmd.tc_valid,
                     f_to_milli(s_cmd.comp.t_ref), (unsigned)s_cmd.sys_flags);
            send_text(hi);
            snprintf(hi, sizeof(hi),
                     "OK store bias g[%d,%d,%d]mdps a[%d,%d,%d]mg",
                     f_to_milli(s_cmd.bias.gx), f_to_milli(s_cmd.bias.gy),
                     f_to_milli(s_cmd.bias.gz), f_to_milli(s_cmd.bias.ax),
                     f_to_milli(s_cmd.bias.ay), f_to_milli(s_cmd.bias.az));
            send_text(hi);
        } else {
            snprintf(hi, sizeof(hi), "WARN store load: %s -> boot calibration",
                     cal_cmd_store_err_name(restored));
            send_text(hi);
        }
    }

    if (restored != 0 || s_cmd.bias_valid != 1) {
        /* 阶段一：读 N 帧标零偏（采集时长 = target_n / 采样率，秒） */
        imu_bias_t bias;
        float t_ref;
        if (calibrate_bias(cal_cfg, &bias, &t_ref) != 0) {
            send_text("FATAL bias calibration failed");
            return;
        }
        {
            char hi[96];
            snprintf(hi, sizeof(hi),
                     "OK calib %d frames: bias g[%d,%d,%d]mdps a[%d,%d,%d]mg t_ref %dmC",
                     (int)cal_cfg->target_n,
                     f_to_milli(bias.gx), f_to_milli(bias.gy), f_to_milli(bias.gz),
                     f_to_milli(bias.ax), f_to_milli(bias.ay), f_to_milli(bias.az),
                     f_to_milli(t_ref));
            send_text(hi);
        }

        /* 开机标定窗 = 一次已完成的 CALIBRATE（无命令触发故不回 ACK） */
        if (cal_cmd_seed_calibrated(&s_cmd, &bias, t_ref) != 0) {
            send_text("FATAL cal cmd seed failed");
            return;
        }

        /* 首次落盘（boot-cal 无命令触发，因此由 app 显式触发一次；运行时三个突变点自动写） */
        if (store_ok) {
            const int rc = cal_cmd_store_save(&s_cmd);
            char hi[96];
            if (rc == PSTORE_OK) {
                snprintf(hi, sizeof(hi), "OK store saved boot-cal len=%u",
                         (unsigned)pstore_frame_len(s_cmd.ps));
            } else {
                snprintf(hi, sizeof(hi), "ERROR store save failed (boot-cal): %s",
                         cal_cmd_store_err_name(rc));
            }
            send_text(hi);
        }
    }

    /* 阶段四：去偏 + 温补 + 直出；同时服务下行命令（调零/温漂/读值） */
    send_text("OK stream: TYPE_IMU_CAL 0x02 76B; CMD 0x7D MOD_IMU 0x01-0x10");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));

        service_rx();   /* 下行 CMD / 明文 CAL：标定期间也不停（否则无法中断/重发） */

        imu_sample_t raw;
        float temp;
        if (source_read(&raw, &temp) != 0) {
            static uint32_t err = 0;
            if (++err % APP_LOG_EVERY == 1)   /* 限流报错，不淹总线 */
                send_text("WARN read fail (counter running)");
            continue;
        }
        /* 标定采集 / 温标采集 / 温补应用（步进均在会话内，含双判 + 采集期冻结） */
        if (cal_cmd_step(&s_cmd, &raw, temp, tx_send, NULL) != 0) {
            static uint32_t err = 0;
            if (++err % APP_LOG_EVERY == 1)
                send_text("WARN cal cmd step failed");
            continue;
        }

        const imu_bias_t *bias_now = cal_cmd_bias(&s_cmd);
        const imu_sample_t deb = {
            .ax = raw.ax - bias_now->ax,
            .ay = raw.ay - bias_now->ay,
            .az = raw.az - bias_now->az,
            .gx = raw.gx - bias_now->gx,
            .gy = raw.gy - bias_now->gy,
            .gz = raw.gz - bias_now->gz,
        };
        uint8_t p[EXP2_CAL_BYTES];
        pack_cal(&raw, &deb, bias_now, temp, p);
        send_frame(PROTO_TYPE_IMU_CAL, p, (uint8_t)sizeof(p));
    }
}
