/* SPDX-License-Identifier: MIT */
/**
 * @file    exp5_main.c
 * @brief   实验五：3D 轨迹——固件侧 = 轨迹实验数据源（QUAT+IMU 双流 + 段标记）
 *
 * @note    职责边界（本实验与 exp4 的本质差异）：轨迹重建算法在 Host
 *          （xiao_robot_kit_host/backend/trajectory.py，上位机见
 *          xiao_robot_kit_host/examples/imu/exp5_trajectory）——固件侧只提供
 *          「轨迹实验数据流」：每拍两帧（QUAT 先发，Host 桥按 hold-latest
 *          四元数配对消费，backend/bridge.py 口径）：
 *            0x04 QUAT 帧：mahony 姿态估计（渲染/重建姿态单源）
 *            0x01 IMU 帧 ：原始六轴比力（Host 去重力→双重积分重建位移）
 *          exp4 只发 0x04，Host 真串口轨迹链路缺加速度（app.py exp5 注
 *          「轨迹需待双流固件」）——本固件即补此缺口的真源双流。
 * @note    零点校准（继承 exp2，见 main/zero_cal.h）：**只走显式触发**，两种入口同源——
 *          ① 开机：读预设 "cal.exp2.bias"（N=500 帧）跑一次**阻塞**静止标定窗，
 *             完成后发固定格式 `OK calib done bias …` + **3 帧 0x02 IMU_CAL**；
 *          ② 运行期：**UART0** 上的 CMD 帧（0x7D，`cal_cmd` 七命令）与明文 `CAL`
 *             行命令走**同一实现**（`examples/imu/common/cal_cmd.c`，与 exp2 单源）；
 *             标定采集**非阻塞**（边流边采），完成后补发 3 帧 0x02。
 * @note    合成源无温度通道（traj_scen 纯运动学）：SIM 档温度恒为 `APP_SIM_TEMP_C`，
 *          因此 `TEMP_CALIB` 在本工程 SIM 档必判无效（跨度=0 → tc_valid=0，语义诚实）；
 *          温漂标定与温补请用 exp2 或真板。
 * @note    去偏范围（协议不改）：**姿态通道**（0x04 前的 mahony 输入）用去偏
 *          样本；**0x01 保持原始比力语义**（proto spec 0x01 = 6×float 原始
 *          采样，重建侧另有自己的零偏学习 backend/trajectory.py）——本次零偏
 *          经 3 帧 0x02 的 bias 块下发，重建侧需要时自行减除，固件不替上位机
 *          改帧语义。
 * @note    段标记场景（EXP_SIM=1，QEMU/无板演示）：合成三段运动
 *          （直线加速/匀速/减速，各占循环 1/3 时长，模型见 traj_scen.h），
 *          段边界发 TEXT 帧标记行 "SEG <cycle> <seg> <NAME>"（轨迹页对齐
 *          段边界用）。**不走 0x06 SCEN**——该帧型为电机实验专用布局
 *          （protocol-spec.md），复用即契约冲突；TEXT 行约定是「新增说明」。
 *          真源（EXP_SIM=0）无段真值：自由运动双流直发，无标记。
 * @note    帧输出层二选一（编译期宏 EXP_QEMU_UART，main/CMakeLists.txt）：
 *          - EXP_QEMU_UART=0（默认）：USB-Serial-JTAG（硬件上位机链路）
 *          - EXP_QEMU_UART=1：UART0 直写——QEMU esp32s3 的 USB-Serial-JTAG
 *            设备无 chardev 出口（验收波实测三 -serial 槽 0 字节），UART0 =
 *            QEMU 第 1 个 -serial 槽有出口（ROM banner 实证）。
 * @note    参数零写死：Mahony kp/ki/dt 走预设表 att.exp5.mahony（本组件）；
 *          采样节拍/量程走 imu.attitude.208（vendor xiao 组件库 驱动库包）；
 *          合成场景参数走 traj.exp5.seg3（traj_scen 组件）；
 *          标定帧数走 cal.exp2.bias（vendor calibration 包）。
 * @note    EXP_SIM=1（CMake）切合成源（traj_scen 三段场景，非 imu_sim 静止源
 *          ——imu_sim 无线加速度项，见其头注诚实声明）。**标定窗**另用 imu_sim
 *          静止实例：三段轨迹场景开机即加速、无静止段，若在运动窗上标定，
 *          运动分量会被吸进零偏并污染后续轨迹积分 → 标定窗按「静止」采样
 *          （等效开机静置），运动场景拍号不受影响（标定后才开始推进）。
 * @note    验收（上板）：上位机收 0x04+0x01 双流；轨迹页位移曲线与真实
 *          移动一致（静止漂移指标见 Host tests/test_trajectory.py）。
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"           /* UART0：QEMU 帧出口 + 全模式 CAL 命令通道 */
#if !EXP_QEMU_UART
#include "driver/usb_serial_jtag.h"
#endif
#include "imu/imu_cfg.h"
#include "att/mahony.h"
#include "proto/frame.h"
#include "cal/imu_calib.h"
#include "att_cfg.h"
#include "zero_cal.h"          /* 开机标定窗口径（同 exp2 calibrate_bias） */
#include "cal_cmd.h"           /* CMD/ACK 命令会话（examples/imu/common 单源，同 exp2） */
#include "pstore/pstore.h"
#include "pstore_nvs.h"        /* port/esp_idf：pstore 的 NVS 介质后端 */
#if defined(EXP_USE_SIM)
#include "traj_scen.h"      /* 三段轨迹标记场景（合成源，本组件） */
#include "imu_sim.h"        /* 标定窗静止实例（testing/imu_sim） */
#else
#include "imu/lsm6ds3.h"
#include "port_i2c.h"      /* port/esp_idf */
#endif

/* 应用管路常数（具名防 magic number；非滤波/传感器/场景参数，不入配置表） */
enum {
    APP_TX_TIMEOUT_MS  = 100,   /* USB-Serial-JTAG 帧写超时，ms */
    APP_UART_BAUD      = 115200,/* QEMU UART0 帧出口波特率（与 QEMU chardev 后端一致） */
    APP_UART_RX_BUF    = 256,   /* UART0 驱动 RX 缓冲（install 约束须 >FIFO_LEN=128；
                                   CAL 行命令从这路进） */
    APP_MARKER_BUF     = 32,    /* 段标记行缓冲（"SEG <cycle> <seg> <NAME>" 最长 22+NUL） */
    APP_CAL_FAIL_FATAL = 200,   /* 标定窗连续读失败上限（超限判源死，同 exp2） */
    APP_CAL_FRAMES     = 3,     /* 每次标定完成后连发的 0x02 帧数（契约：仅 3 帧） */
    APP_QUAT_BYTES     = 16,    /* 0x04 帧 payload = 4×float32 LE（w,x,y,z） */
    APP_SAMPLE_BYTES   = 24,    /* 0x01 帧 payload = 6×float32 LE（ax..gz） */
    APP_STORE_CAPACITY = 512,   /* 校准存档区字节数（帧 ~207B，留同量级余量） */
};

/* NVS 命名（一份介质一个 blob；命名空间/键名 ≤ 15 字符，见 pstore_nvs.h） */
#define APP_STORE_NS    "calstore5"
#define APP_STORE_KEY   "cal.frame"

/**
 * @brief 合成源档的"假温度"（°C）
 *
 * @note  exp5 的合成源是 traj_scen（纯运动学三段，**无温度通道**）。用固定室温
 *        而不是造一条假升温曲线：温度跨度恒为 0 → 温漂拟合必判无效（tc_valid=0），
 *        与「温漂标定需真实温度变化」的原型语义一致；想验证温补请用 exp2 或真板。
 */
#define APP_SIM_TEMP_C  25.0f

static uint8_t s_seq = 0;

/* 命令会话（持有生效零偏 + 温补实例 + 持久化实例）；CMD 与明文 CAL 共用此入口 */
static cal_cmd_t s_cmd;
static const imu_calib_cfg_t *s_cal_cfg;   /* 预设 cal.exp2.bias（N=500） */
static const temp_comp_cfg_t *s_tc_cfg;    /* 预设 cal.exp2.drift */

/* 持久化介质（NVS 后端） */
static pstore_backend_t s_be;
static pstore_nvs_t     s_nvs;

#if defined(EXP_USE_SIM)
static imu_sim_t s_cal_sim;  /* 标定窗源：静止实例（分段场景无静止段） */
#else
static lsm6ds3_t s_imu;      /* 传感器驱动实例（标定窗与流送共用） */
#endif

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
    if (n > PROTO_PAYLOAD_MAX) n = PROTO_PAYLOAD_MAX;   /* 帧 payload 上限内截断（行格式见 zero_cal.h） */
    send_frame(PROTO_TYPE_TEXT, (const uint8_t *)msg, (uint8_t)n);
}

/* quat → payload 小端 16B（qw,qx,qy,qz；规格见 proto/frame.h TYPE_QUAT 注释） */
static void pack_quat(float w, float x, float y, float z, uint8_t *p)
{
    const float v[4] = { w, x, y, z };
    memcpy(p, v, APP_QUAT_BYTES);   /* Xtensa 小端，与协议字节序一致 */
}

/* imu_sample → payload 小端 24B（ax,ay,az,gx,gy,gz；0x01 契约，同 exp1） */
static void pack_sample(const imu_sample_t *s, uint8_t *p)
{
    const float v[6] = { s->ax, s->ay, s->az, s->gx, s->gy, s->gz };
    memcpy(p, v, APP_SAMPLE_BYTES);   /* Xtensa 小端，与协议字节序一致 */
}

/* ---------------------------------------------------------------- 数据源 */

/**
 * @brief 标定窗读一拍：原始六轴 + 同拍温度
 *
 * @note  SIM：先温度后样本 = 严格同拍（imu_sim.h 契约）；真源：先样本后温度
 *        （同 exp2 source_read 口径）。
 * @retval 0 已写入；-1 读失败（源死 → 由调用方计数判 FATAL）
 */
static int calib_read(imu_sample_t *raw, float *temp)
{
#if defined(EXP_USE_SIM)
    if (imu_sim_read_temp(&s_cal_sim, temp) != 0) return -1;
    return imu_sim_read(&s_cal_sim, raw);
#else
    if (lsm6ds3_read(&s_imu, raw) != 0) return -1;
    return lsm6ds3_read_temp(&s_imu, temp);
#endif
}

/* ---------------------------------------------------------------- 命令通道 */

/**
 * @brief 取同拍温度（°C）
 *
 * @note  SIM：traj_scen 无温度通道 → 固定 `APP_SIM_TEMP_C`（温标跨度=0 → 无效，
 *        语义诚实，不造假升温）；真源：芯片温度（先样本后温度，同 exp2 口径）。
 */
static int source_read_temp(float *temp)
{
#if defined(EXP_USE_SIM)
    *temp = APP_SIM_TEMP_C;
    return 0;
#else
    return lsm6ds3_read_temp(&s_imu, temp);
#endif
}

/* ---------------------------------------------------------------- 命令通道 */

/** 帧出口回调（cal_cmd 用）：CMD/ACK/TEXT 与数据帧共用同一条链路口 */
static void tx_send(void *io, uint8_t type, const uint8_t *payload, uint8_t len)
{
    (void)io;
    send_frame(type, payload, len);
}

/**
 * @brief 轮询 UART0 字节并交给命令会话组帧/分发（非阻塞）
 *
 * @note  帧与明文行命令共用同一入口（解析分区在 cal_cmd 内）：因此 `CAL\n`
 *        （旧上位机通道）与 CMD `CALIBRATE(0x01)` 走**同一实现**。
 * @note  timeout=0：主循环节拍不被串口拖住（RX 由驱动环形缓冲暂存）。
 */
static void service_rx(void)
{
    uint8_t buf[64];
    const int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), 0);
    if (n > 0) cal_cmd_rx(&s_cmd, buf, (size_t)n, tx_send, NULL);
}

/**
 * @brief 发 3 帧 0x02 IMU_CAL（上位机契约：每次零偏标定完成后连发 3 帧）
 *
 * @note  3 帧同内容（用标定后第一拍样本），仅供上位机对照展示；
 *        持久化恢复（跳过标定窗）不发——没发生标定就谎报会误导用户。
 */
static void emit_cal_frames(const imu_sample_t *raw, float temp)
{
    const imu_bias_t *b = cal_cmd_bias(&s_cmd);
    imu_sample_t deb;
    zero_cal_apply(b, raw, &deb);
    uint8_t p[ZERO_CAL_FRAME_BYTES];
    zero_cal_pack_frame(raw, &deb, b, temp, p);
    for (int i = 0; i < APP_CAL_FRAMES; i++)
        send_frame(PROTO_TYPE_IMU_CAL, p, sizeof(p));
}

/* ---------------------------------------------------------------- 校准 */

/**
 * @brief 跑一次零点标定窗：TEXT 进度 → 固定格式完成行 → 3 帧 0x02
 *
 * @param[in]  period_ms  采样节拍（标定窗按同一节拍采 N 帧）
 * @param[out] bias_out   标定出的零偏（dps / g）
 * @param[out] tref_out   标定窗**平均温度**（温补锚点，°C）
 *
 * @retval 0  标定完成
 * @retval -1 失败（源死 / 算法拒绝）
 *
 * @note  只用于**开机**标定窗（阻塞）：没有零偏就无法出姿态流。
 *        运行期 `CALIBRATE` 走 cal_cmd 的**非阻塞**采集（边流边采），不再调本函数。
 * @note  SIM 档锚点温度恒为 `APP_SIM_TEMP_C`（traj_scen 无温度通道），
 *        但零偏标定仍有意义（角速度零偏与温度无关地存在）；温漂拟合才是无意义的。
 * @note  窗内收到的 CAL 在结束时清掉（uart_flush_input）：不排队、不连环触发。
 */
static int run_calibration(uint16_t period_ms, imu_bias_t *bias_out, float *tref_out)
{
    char line[ZERO_CAL_TEXT_CAP];
    zero_cal_t cal;

    if (zero_cal_begin(&cal, s_cal_cfg) != 0) {
        send_text("FATAL calib init failed");
        return -1;
    }
    zero_cal_text_start(line, sizeof(line), s_cal_cfg->target_n);
    send_text(line);

    imu_sample_t last = { 0 };
    float last_temp = 0.0f;
    double t_sum = 0.0;
    uint32_t t_n = 0;
    uint32_t next_report = ZERO_CAL_PROGRESS_EVERY;
    uint32_t fails = 0;

    while (zero_cal_done(&cal) != 1) {
        imu_sample_t raw;
        float temp;
        if (calib_read(&raw, &temp) != 0) {
            if (++fails >= APP_CAL_FAIL_FATAL) {
                send_text("FATAL calib read fail");
                return -1;
            }
            vTaskDelay(pdMS_TO_TICKS(period_ms));
            continue;
        }
        fails = 0;
        zero_cal_feed(&cal, &raw);
        last = raw;
        last_temp = temp;
        t_sum += (double)temp;
        t_n++;

        uint32_t n = 0, target = 0;
        zero_cal_progress(&cal, &n, &target);
        if (n >= next_report && n < target) {     /* 进度行：标定窗内不静默 */
            zero_cal_text_progress(line, sizeof(line), n, target);
            send_text(line);
            next_report = n + ZERO_CAL_PROGRESS_EVERY;
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    if (t_n == 0) return -1;                       /* 不可达护栏（done ⇒ 已喂满 N） */
    if (zero_cal_result(&cal, bias_out) != 0) {
        send_text("FATAL calib result failed");
        return -1;
    }
    *tref_out = (float)(t_sum / (double)t_n);      /* 锚点 = 窗均温 */

    zero_cal_text_done(line, sizeof(line), bias_out, last_temp);
    send_text(line);

    imu_sample_t deb;
    zero_cal_apply(bias_out, &last, &deb);
    uint8_t p[ZERO_CAL_FRAME_BYTES];
    zero_cal_pack_frame(&last, &deb, bias_out, last_temp, p);
    for (int i = 0; i < APP_CAL_FRAMES; i++)
        send_frame(PROTO_TYPE_IMU_CAL, p, sizeof(p));

    uart_flush_input(UART_NUM_0);   /* 窗内 CAL 不排队（防连按连环标定） */
    return 0;
}

void app_main(void)
{
    /* 帧出口初始化 */
#if !EXP_QEMU_UART
    usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&uc);
#endif
    /* UART0：QEMU 模式下是帧出口；硬件模式下只收命令（console 在 USB-JTAG，
       故 UART0 空闲可用）。 */
    const uart_config_t uart_cfg = {
        .baud_rate  = APP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, APP_UART_RX_BUF, 0, 0, NULL, 0);

    const att_cfg_t *att = att_cfg_find("att.exp5.mahony");   /* 表注册，零写死 */
    if (!att) { send_text("FATAL preset att.exp5.mahony not found"); return; }
    s_cal_cfg = imu_calib_cfg_find("cal.exp2.bias");          /* 与 exp2 同一预设 */
    if (!s_cal_cfg) { send_text("FATAL preset cal.exp2.bias not found"); return; }

    mahony_t mf;
    mahony_init(&mf, att->kp, att->ki, att->dt);
    uint16_t period_ms;     /* 推流节拍：真源走 imu 预设，合成源走场景采样率 */

#if defined(EXP_USE_SIM)
    /* 三段轨迹标记场景（traj.exp5.seg3：200Hz，三段各 1s，a=0.5m/s²） */
    const traj_scen_cfg_t *scen_cfg = traj_scen_cfg_find("traj.exp5.seg3");
    if (!scen_cfg) { send_text("FATAL preset traj.exp5.seg3 not found"); return; }
    traj_scen_t scen;
    if (traj_scen_init(&scen, scen_cfg) != 0) {
        send_text("FATAL traj_scen_init rejected cfg"); return;
    }

    /* 标定窗静止实例：imu_sim 静止预设派生（时基/室温取预设），注入项清零
       ——轨迹场景本身无零偏注入，标定窗须同误差水平，否则标定值会污染
       后续轨迹积分 */
    const imu_sim_cfg_t *cal_base = imu_sim_cfg_find("sim.exp2.static");
    if (!cal_base) { send_text("FATAL preset sim.exp2.static not found"); return; }
    imu_sim_cfg_t cal_cfg = *cal_base;
    cal_cfg.temp_ramp_c_per_s = 0.0f;                     /* 标定窗恒温（只上报） */
    memset(&cal_cfg.bias, 0, sizeof(cal_cfg.bias));
    memset(cal_cfg.gyro_drift, 0, sizeof(cal_cfg.gyro_drift));
    memset(cal_cfg.accel_drift, 0, sizeof(cal_cfg.accel_drift));
    if (imu_sim_init(&s_cal_sim, &cal_cfg) != 0) {
        send_text("FATAL imu_sim_init rejected calib cfg"); return;
    }

    /* 合成源自带时间基：滤波 dt 与节拍都取场景采样率（与表内 att dt 解耦） */
    const float sim_dt = 1.0f / scen_cfg->sample_rate_hz;
    mahony_set_dt(&mf, sim_dt);
    period_ms = (uint16_t)(1000.0f * sim_dt + 0.5f);
    send_text("OK sim source active (EXP_SIM, traj.exp5.seg3: 3-seg motion)");
#else
    const imu_cfg_t *cfg = imu_cfg_find("imu.attitude.208");
    if (!cfg) { send_text("FATAL preset imu.attitude.208 not found"); return; }

    if (esp_port_i2c_init(cfg->i2c_speed_hz) != 0) {
        send_text("FATAL i2c bus init failed"); return;
    }

    lsm6ds3_init(&s_imu, &esp_port_i2c_bus);

    for (;;) {
        int r = lsm6ds3_probe(&s_imu);
        if (r == 0) break;
        send_text(r == -2 ? "ERR who_am_i mismatch" : "ERR i2c no ack, retry 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    send_text("OK probe: LSM6DS3TR-C @0x6A");

    if (lsm6ds3_configure(&s_imu, cfg) != 0) {
        send_text("FATAL configure failed"); return;
    }

    char hi[80];
    snprintf(hi, sizeof(hi), "OK dual stream: %uHz +-%ug +-%udps period %ums",
             (unsigned)cfg->accel_odr_hz, (unsigned)cfg->accel_fs_g,
             (unsigned)cfg->gyro_fs_dps, (unsigned)cfg->sample_period_ms);
    send_text(hi);

    period_ms = cfg->sample_period_ms;
    /* 滤波 dt 以实际节拍（imu 预设）为准：表值只是初始值，不一致则校正并提示 */
    const float actual_dt = (float)cfg->sample_period_ms / 1000.0f;
    if (fabsf(actual_dt - att->dt) > 1e-6f) {
        mahony_set_dt(&mf, actual_dt);
        send_text("WARN att dt != imu period, filter follows imu period");
    }
#endif

    /* 预设：温标/温补配置（命令会话需要；与 exp2 同一 preset） */
    s_tc_cfg = temp_comp_cfg_find("cal.exp2.drift");
    if (!s_tc_cfg) { send_text("FATAL preset cal.exp2.drift not found"); return; }

    /* 持久化装配（失败不致命：无存档只是每次开机重新标定） */
    const int store_ok =
        (pstore_nvs_backend_init(&s_be, &s_nvs, APP_STORE_NS, APP_STORE_KEY) == 0);
    if (!store_ok) send_text("WARN nvs store unavailable -> no persistence");

    /* 命令会话：先读一拍拿锚点温度（未标定 → 零偏占位 0，温补双判不成立） */
    {
        float t0 = 0.0f;
        if (source_read_temp(&t0) != 0) {
            send_text("FATAL source dead before calib");
            return;
        }
        const imu_bias_t zero_bias = { 0 };
        if (cal_cmd_init(&s_cmd, s_cal_cfg, s_tc_cfg, t0, &zero_bias, 0) != 0) {
            send_text("FATAL cal cmd init failed");
            return;
        }
    }

    /* 有健康存档 → 直接恢复（跳过标定窗）；否则跑标定窗再落盘 */
    int restored = PSTORE_ERR_ARG;
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
                     (int)(s_cmd.comp.t_ref * 1000.0f), (unsigned)s_cmd.sys_flags);
            send_text(hi);
        } else {
            snprintf(hi, sizeof(hi), "WARN store load: %s -> boot calibration",
                     cal_cmd_store_err_name(restored));
            send_text(hi);
        }
    }

    if (restored != 0 || s_cmd.bias_valid != 1) {
        imu_bias_t bias;
        float t_ref = 0.0f;
        if (run_calibration(period_ms, &bias, &t_ref) != 0) {
            send_text("FATAL boot calibration failed");
            return;
        }
        if (cal_cmd_seed_calibrated(&s_cmd, &bias, t_ref) != 0) {
            send_text("FATAL cal cmd seed failed");
            return;
        }
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

#if defined(EXP_USE_SIM)
    traj_seg_t prev_seg = TRAJ_SEG_COUNT;   /* 初值 = 越界值：第 0 拍必发首段标记 */
#endif

    for (;;) {
        /* 下行命令：CMD 帧与明文 `CAL` 同一入口（标定采集期也不停，否则无法重发） */
        service_rx();

        vTaskDelay(pdMS_TO_TICKS(period_ms));
        imu_sample_t s;
        int r;
#if defined(EXP_USE_SIM)
        traj_seg_t seg;
        uint32_t cycle;
        r = traj_scen_read(&scen, &s, &seg, &cycle);
        if (r == 0 && seg != prev_seg) {
            /* 段边界标记行（TEXT 帧单源格式，见 traj_scen.h 契约注） */
            char marker[APP_MARKER_BUF];
            if (traj_scen_marker_line(marker, sizeof(marker), cycle, seg) > 0) {
                send_text(marker);
                prev_seg = seg;
            }
        }
#else
        r = lsm6ds3_read(&s_imu, &s);
#endif
        if (r != 0) {
            static uint8_t err = 0;
            if (++err % 50 == 1)   /* 限流报错，不淹总线 */
                send_text("WARN read fail (counter running)");
            continue;
        }

        /* 同拍温度（SIM 恒室温）+ 命令会话步进（标定采集/温标/温补） */
        float temp = APP_SIM_TEMP_C;
        if (source_read_temp(&temp) != 0) {
            static uint8_t err = 0;
            if (++err % 50 == 1)
                send_text("WARN temp read fail (counter running)");
            continue;
        }
        const int was_collecting = cal_cmd_collecting(&s_cmd);
        if (cal_cmd_step(&s_cmd, &s, temp, tx_send, NULL) != 0) {
            static uint8_t err = 0;
            if (++err % 50 == 1)
                send_text("WARN cal cmd step failed");
            continue;
        }
        /* 零偏标定刚完成 → 按上位机契约补 3 帧 0x02（raw|deb|bias|temp） */
        if (was_collecting && cal_cmd_collecting(&s_cmd) == 0 && s_cmd.bias_valid)
            emit_cal_frames(&s, temp);

        imu_sample_t deb;
        zero_cal_apply(cal_cmd_bias(&s_cmd), &s, &deb);     /* 姿态通道去偏（口径同 exp2） */
        mahony_update(&mf, &deb);              /* g/dps 进；加速度无效帧内部跳过 */

        float w, x, y, z;
        mahony_get_quat(&mf, &w, &x, &y, &z);
        uint8_t pq[APP_QUAT_BYTES];
        pack_quat(w, x, y, z, pq);
        send_frame(PROTO_TYPE_QUAT, pq, sizeof(pq));       /* 姿态帧先发（配对口径） */

        uint8_t pi[APP_SAMPLE_BYTES];
        pack_sample(&s, pi);                               /* 0x01 仍发原始比力（协议语义不变） */
        send_frame(PROTO_TYPE_IMU_SAMPLE, pi, sizeof(pi)); /* 比力帧后发（重建原料） */
    }
}
