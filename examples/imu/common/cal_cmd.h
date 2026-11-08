/* SPDX-License-Identifier: MIT */
#ifndef CAL_CMD_H
#define CAL_CMD_H

#include <stddef.h>
#include <stdint.h>
#include "imu/imu_types.h"
#include "cal/imu_calib.h"
#include "cal/temp_comp.h"
#include "proto/frame.h"
#include "pstore/pstore.h"   /* 持久化：本模块可选接入（ps == NULL = 不持久化） */

/**
 * @file    cal_cmd.h
 * @brief   IMU 标定命令会话：CMD 组帧 → 校验 → 分发（CALIBRATE/GET_BIAS/TEMP_CALIB/
 *          SET_FLAGS/GET_STATUS/GET_DRIFT/GET_TC_STATE）+ 采样推进
 *
 * @note    分工（与原型 旧式实现 同构）：
 *          - 算法全在 `cal/imu_calib` + `cal/temp_comp`（xiao 组件库 单源）；
 *          - 本模块只做**会话状态**（当前生效零偏 / 标定标志 / 功能位）+ 协议编解码，
 *            不自带数学；
 *          - 传输层（UART0 / USB-Serial-JTAG）与数据源由调用方注入：送帧走
 *            `cal_cmd_send_fn` 回调，采样走 `cal_cmd_step` 入参——因此本模块在
 *            host 上可被 gcc 直接单测（`testing/check_sim.sh`）。
 * @note    语义锚（与原型逐条对应，实现处有行号注释）：
 *          - 零偏**累加残差**：喂样 = raw − 当前 bias，落库 `imu_calib_accumulate`
 *            （bias += delta），见 `cal_cmd_step`；
 *          - `CALIBRATE`/`TEMP_CALIB` **异步**：启动时不回 ACK，采集完毕才回
 *            （ACK 上下文用 `ack_mod/ack_func` 记住发起命令，防被并发命令覆盖）；
 *          - `SET_FLAGS` 只写功能位，**不碰** tc_valid（标定与开关分离）；
 *          - 温补双判 = `temp_comp_should_apply(TEMPCOMP_EN, tc_valid)`，
 *            关闭时 `temp_comp_disable` 把零偏恢复 `bias_ref`；
 *          - 零偏标定采集期间**冻结温补**（否则基准每秒被改写，残差不准）。
 * @note    兼容：明文行命令 `CAL`（上位机旧通道，`bridge.py` 的 `CAL\n`）
 *          与 CMD `CALIBRATE(0x01)` 等价——同一 `cmd_calibrate` 分支。
 * @note    与原型的两处**有意差异**（均不改语义，只改节奏/接口）：
 *          ① 温补每拍评估（原型 1 s 节拍）——本工程每拍已读温度（0x02 帧带温度），
 *             阶梯判据 `step_c` 决定实际重算时机，行为等价且更及时；
 *          ② 标定完成后的 TEXT 行沿用本仓 `zero_cal` 的固定格式（上位机按行高亮
 *             "OK calib"），字段序与原型 `[calib] done: …` 不同。
 */

/** 明文行命令缓冲上限（"CAL\r\n" 量级；超长行整体丢弃，不截断执行） */
#define CAL_CMD_LINE_CAP       16

/** 温漂标定的采样率假设（秒 → 样本数换算，与原型 `seconds * 100` 同口径） */
#define CAL_CMD_RATE_HZ        100u

/**
 * @brief 「从未应用过温补」的可观测哨兵（GET_TC_STATE 的 last_applied 用）
 *
 * @note  组件内以 `temp_comp_t.applied` 标志表达"是否已应用"；本哨兵只服务
 *        对外快照，值与原型 `TEMP_COMP_NOT_APPLIED` 一致（上位机判据可比）。
 */
#define CAL_CMD_TC_NOT_APPLIED (-999.0f)

/** @brief 帧出口：把一帧（type + payload）交给传输层（固件=UART0；自检=收集器） */
typedef void (*cal_cmd_send_fn)(void *io, uint8_t type, const uint8_t *payload, uint8_t len);

/* ───────────────────── 持久化（T-014：pstore 组合层） ───────────────────── */

/**
 * @brief 存储记录语义版本（**payload 布局**版本；帧格式版本归 pstore 自己管）
 *
 * @note  与 pstore 的 `PSTORE_FORMAT_VERSION` 是两层：前者管帧怎么装，本值管
 *        "这 8 个字段各是什么含义/宽度"。任一改动（加字段/换单位/换语义）必须 bump，
 *        旧记录会被明确拒绝（`CAL_CMD_STORE_ERR_SCHEMA`）而不是被误读成新语义。
 */
#define CAL_CMD_STORE_SCHEMA       1u

/** @brief 注册条目数（bias / t_ref / gyro_drift / accel_drift / sys_flags / 两 valid / schema） */
#define CAL_CMD_STORE_MAX_ITEMS    8u

/**
 * @brief 应用身份魔数（pstore 帧头 magic）
 *
 * @note  "CAL1"：校准参数存档的介质身份。介质上是异种数据（别的模块/别人的分区）
 *        时 pstore 报 `PSTORE_ERR_MAGIC`，走默认值路径而不是硬套。
 */
#define CAL_CMD_STORE_MAGIC        0x43414C31u

/** @brief 存储记录 schema 与本固件不符（升级/回滚的正路径，不是损坏） */
#define CAL_CMD_STORE_ERR_SCHEMA       (-20)

/** @brief 未接入存储就调用``保存``（显式拒绝：不静默假装成功） */
#define CAL_CMD_STORE_ERR_UNATTACHED   (-21)

/**
 * @brief 存储记录（**介质镜像**，与生效状态分离）
 *
 * @note  为什么分离：pstore 的 `load` 是两阶段原子（失败零改动），但它的"写入目标"
 *        就是注册时给的 RAM 地址。若直接注册会话字段，load 失败虽不写，却让
 *        "介质记录"与"当前生效状态"混为一块，schema 不符时无法在"已读入但不敢用"
 *        的中间态上做判断。本结构做介质镜像：load 只改本结构，验证通过后才
 *        （`cal_cmd_store_open` 内）一次性搬到会话字段。
 */
typedef struct {
    imu_bias_t bias;        /**< 生效零偏（含温补后的当前值，dps / g） */
    float      t_ref;       /**< 温补锚点温度（= 零偏标定窗均温），°C */
    float      gyro_drift[3];   /**< 陀螺温漂系数，dps/°C（tc_valid=0 时全 0） */
    float      accel_drift[3];  /**< 加速度温漂系数，g/°C（同上） */
    uint16_t   sys_flags;   /**< SYS_FLAGS（bit0 = TEMPCOMP_EN） */
    uint8_t    bias_valid;  /**< 零偏是否已标定 */
    uint8_t    tc_valid;    /**< 温漂是否已标定（≠ 是否启用） */
    uint32_t   schema;      /**< 记录语义版本（CAL_CMD_STORE_SCHEMA） */
} cal_cmd_store_rec_t;

/**
 * @brief 存储装配配置（参数零写死：容量/魔数/schema 全部由调用方传入）
 */
typedef struct {
    const pstore_backend_t *backend;   /**< 介质后端（flash/NVS/RAM；必填） */
    uint32_t capacity;                 /**< 存储区字节数（≥ 帧长；≥ PSTORE_FRAME_HEADER_BYTES） */
    uint32_t magic;                    /**< 应用身份（通常 CAL_CMD_STORE_MAGIC） */
    uint32_t schema;                   /**< 本固件期望的记录语义版本（通常 CAL_CMD_STORE_SCHEMA） */
} cal_cmd_store_cfg_t;

/**
 * @brief 标定命令会话（状态全部实例内，无全局可变态）
 *
 * @note  累加/拟合实例内嵌（`cal`/`tcal`/`comp`）：一个会话同一时刻只跑一次
 *        零偏标定与一次温标（重发命令幂等，见各 cmd_* 分支）。
 */
typedef struct {
    const imu_calib_cfg_t *bcal_cfg;        /**< 零偏标定预设（cal.exp2.bias） */
    const temp_comp_cfg_t *tcfg;            /**< 温标/温补预设（cal.exp2.drift） */

    imu_calib_t   cal;                      /**< 零偏标定实例（残差累加状态） */
    temp_calib_t  tcal;                     /**< 温标实例（最小二乘累加器） */
    temp_comp_t   comp;                     /**< 温补实例（阶梯 + 锚点） */

    imu_bias_t    bias;                     /**< **当前生效**零偏（含温补；GET_BIAS/0x02 帧的来源） */
    float         gyro_drift[3];            /**< 温漂系数 dps/°C（拟合值；0 = 未标定） */
    float         accel_drift[3];           /**< 温漂系数 g/°C（拟合值；0 = 未标定） */
    float         temp_now;                 /**< 最近一拍温度，°C（GET_TC_STATE 的 t_now） */
    int           bias_valid;               /**< 零偏是否已标定（GET_STATUS[8]） */
    int           tc_valid;                 /**< 温漂是否**已标定**（GET_STATUS[9]；≠ 是否启用） */
    uint16_t      sys_flags;                /**< SYS_FLAGS（bit0 = TEMPCOMP_EN，见原型 serial.h） */

    uint8_t       ack_mod;                  /**< 异步 ACK 上下文：模块号（发起命令时记） */
    uint8_t       ack_func;                 /**< 异步 ACK 上下文：功能码 */
    int           ack_pending;              /**< 1 = 有异步命令在采集中（完成时回 ACK） */

    uint8_t       rx[PROTO_FRAME_MAX];      /**< 帧组包缓冲（SOF..CRC8） */
    uint16_t      rx_len;                   /**< 已缓存字节数 */
    char          line[CAL_CMD_LINE_CAP];   /**< 明文行命令缓冲（CAL 兼容通道） */
    uint8_t       line_len;                 /**< 行缓冲已用字节数 */

    pstore_t             *ps;               /**< 持久化实例；NULL = 未接入（自检/无分区场景） */
    cal_cmd_store_cfg_t   store_cfg;        /**< 装配配置副本（backend 归调用方持有） */
    cal_cmd_store_rec_t   rec;              /**< 介质镜像（load 写入这里，验证后才搬到会话） */
} cal_cmd_t;

/**
 * @brief   初始化会话（锚点 + 初始零偏 + 功能位默认值）
 *
 * @param[out] c         会话
 * @param[in]  bcal_cfg  零偏标定预设
 * @param[in]  tcfg      温标/温补预设
 * @param[in]  t_ref     参考温度（= 零偏标定窗的平均温度），°C
 * @param[in]  bias_ref  参考温度下的零偏（dps / g）
 * @param[in]  sys_flags 初始 SYS_FLAGS（通常 0：温补默认关，标定后才由 SET_FLAGS 打开）
 *
 * @retval  0  成功
 * @retval -1  任一指针为 NULL，或 imu_calib_init / temp_comp_init 拒绝参数
 *
 * @note    温漂系数初值取预设默认值、`tc_valid = 0`：温补双判的第二项未满足，
 *          故默认状态下 `temp_comp_apply` 不会被调用（等价原型"未标定不补偿"）。
 */
int cal_cmd_init(cal_cmd_t *c, const imu_calib_cfg_t *bcal_cfg,
                 const temp_comp_cfg_t *tcfg, float t_ref,
                 const imu_bias_t *bias_ref, uint16_t sys_flags);

/**
 * @brief   喂入收到的原始字节（帧与明文行命令同一入口）
 *
 * @param[in,out] c    会话
 * @param[in]     buf  字节（可为 NULL 当 n==0）
 * @param[in]     n    字节数
 * @param[in]     send 帧出口（NULL = 丢弃回执，仅本机自检场景）
 * @param[in,out] io   传给 send 的上下文
 *
 * @note    容错契约与上位机解码器同口径：坏 CRC / 超限 LEN / 假 SOF → 丢一字节
 *          重同步，**合法帧全收、坏帧全弃**。
 * @note    明文行以 `\n` 结束（`\r` 忽略）；仅识别 `CAL`（= CALIBRATE），
 *          其余行静默忽略（不产生噪声回执）。
 */
void cal_cmd_rx(cal_cmd_t *c, const uint8_t *buf, size_t n,
                cal_cmd_send_fn send, void *io);

/**
 * @brief   推进一拍：标定采集 / 温标采集 / 温补应用（每采到一帧样本调一次）
 *
 * @param[in,out] c    会话
 * @param[in]     raw  **未去偏**原始样本（g / dps）；重力轴判定用它的原始值
 * @param[in]     temp 同拍温度，°C
 * @param[in]     send 帧出口（异步 ACK / TEXT 走它）
 * @param[in,out] io   传给 send 的上下文
 *
 * @retval  0  已推进（无错误）
 * @retval -1  c/raw 为 NULL，或采集器拒绝样本（调用序错误）
 *
 * @note    零偏标定期把 `raw − bias` 喂给 `imu_calib_feed`（残差法），raw 另传；
 *          凑满时 `imu_calib_accumulate` 累加进 `bias` 并同步温补锚点
 *          （`bias_ref = 新 bias`、`t_ref = 当前温度`）。
 * @note    温补应用受双判约束且在零偏标定期冻结（见文件头）。
 */
int cal_cmd_step(cal_cmd_t *c, const imu_sample_t *raw, float temp,
                 cal_cmd_send_fn send, void *io);

/**
 * @brief   当前生效零偏（0x02 帧 / deb 计算用；含温补后的值）
 *
 * @return  指向会话内零偏的只读指针（c 为 NULL 时返回 NULL）
 */
const imu_bias_t *cal_cmd_bias(const cal_cmd_t *c);

/**
 * @brief   注入"开机标定窗已完成"的结果（等价一次成功的 CALIBRATE，但不回 ACK）
 *
 * @param[in,out] c      会话
 * @param[in]     bias   标定窗算出的零偏（dps / g）
 * @param[in]     t_ref  标定窗的**平均温度**，°C（温补锚点：线性模型下均值零偏对应均值温度）
 *
 * @retval  0  成功（`bias_valid = 1`，温补锚点已对齐）
 * @retval -1  c/bias 为 NULL，或 t_ref 非有限
 *
 * @note    为什么需要：exp2 开机会先跑一次静止标定窗（无命令触发，自然无 ACK），
 *          其锚点温度是**窗口均值**而不是最后一拍温度——不能用 `cal_cmd_step` 的
 *          标定路径代替，故以本接口显式注入。
 */
int cal_cmd_seed_calibrated(cal_cmd_t *c, const imu_bias_t *bias, float t_ref);

/**
 * @brief   是否处于"标定中"（零偏标定或温标采集，任一在跑都算）
 *
 * @return  1 = 采集进行中（上位机应显示"标定中"）；0 = 空闲
 */
int cal_cmd_collecting(const cal_cmd_t *c);

/* ───────────────────── 持久化接口 ───────────────────── */

/**
 * @brief   接入持久化并尝试恢复上一次的标定状态
 *
 * @param[in,out] c    会话（须已 `cal_cmd_init`）
 * @param[in]     cfg  存储配置（backend 必填；容量/魔数/schema 由调用方给）
 *
 * @retval  0                          已从存储恢复（bias / t_ref / 系数 / flags / 两个 valid 全来自介质）
 * @retval  CAL_CMD_STORE_ERR_SCHEMA   记录 schema 与本固件不符（旧固件拒绝新记录，或反向）
 * @retval  PSTORE_ERR_MAGIC           空白/异种介质（首次上电的正路径）
 * @retval  PSTORE_ERR_CHECKSUM        CRC 失败（介质损坏）
 * @retval  PSTORE_ERR_CORRUPT         帧结构损坏
 * @retval  PSTORE_ERR_BACKEND         介质读失败
 * @retval  PSTORE_ERR_ARG             c/cfg/backend 为 NULL 或容量过小
 *
 * @note    返回值 **0 才是恢复成功**；任何非 0 都表示"会话保持调用前状态"（零字节改动），
 *          调用方应走默认值路径（exp2 即"跑一次开机标定窗再落盘"）。
 * @note    非 0 返回后实例**仍然挂接着**（可直接 `cal_cmd_store_save` 重写存档）——
 *          这是升级迁移路径：旧 schema 记录被拒绝 → 重标定 → 覆盖写入新 schema。
 * @note    可重复调用（先释放旧实例再建新的），便于换后端/换容量做实验。
 */
int cal_cmd_store_open(cal_cmd_t *c, const cal_cmd_store_cfg_t *cfg);

/**
 * @brief   把会话当前状态写进介质（脏检查由 pstore 负责：不脏零后端调用）
 *
 * @retval  PSTORE_OK                   成功（或本就不脏）
 * @retval  CAL_CMD_STORE_ERR_UNATTACHED 未接入存储
 * @retval  PSTORE_ERR_NOFIT            帧超容量（脏标志保留，可重试）
 * @retval  PSTORE_ERR_BACKEND          介质擦/写失败
 * @retval  PSTORE_ERR_ARG              c 为 NULL
 *
 * @note    `finish_bias_calib` / `finish_temp_calib` / `cmd_set_flags` 三处状态改变点
 *          **已内置自动落盘**（并回一行 TEXT 证据）；本接口用于外部补充（开机标定窗注入后）。
 */
int cal_cmd_store_save(cal_cmd_t *c);

/**
 * @brief   断开持久化（销毁实例；不触碰介质与已恢复的会话状态）
 */
void cal_cmd_store_close(cal_cmd_t *c);

/**
 * @brief   是否已接入存储
 * @return  1 = 已接入；0 = 未接入（c 为 NULL 也算未接入）
 */
int cal_cmd_store_attached(const cal_cmd_t *c);

/**
 * @brief   错误码 → 可读名（`cal_cmd_store_open/save` 的返回值；超集含 pstore 全部码）
 */
const char *cal_cmd_store_err_name(int err);

#endif /* CAL_CMD_H */
