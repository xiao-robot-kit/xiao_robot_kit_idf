/* SPDX-License-Identifier: MIT */
/**
 * @file    cal_cmd.c
 * @brief   标定命令会话实现：CMD 组帧/校验/分发 + 每拍采样推进（算法委托 xiao 组件库）
 *
 * @note    算法零重复：零偏/温漂的数学全部在 `cal/imu_calib.h`、`cal/temp_comp.h`
 *          （xiao 组件库 v0.3.0 单源）；本文件只做会话状态与协议编解码。
 * @note    语义锚原型 旧式实现：
 *          - 异步 ACK 上下文（ack_mod/ack_func）← `runtime/app_stream.c:141-144`
 *          - 残差累加（bias += delta）← `runtime/app_stream.c:96-116`
 *          - 温补双判 + 关闭恢复 bias_ref ← `modules/imu/app_imu.c:168-199`
 *          - 七功能码分派 ← `modules/imu/app_imu.c:53-306`
 */

#include <stdio.h>
#include <string.h>
#include "cal_cmd.h"

/* ---------------------------------------------------------------- 局部工具 */

/** 帧出口：编码一帧并交出去（send 为 NULL 时丢弃——自检场景"只看状态"） */
static void emit(cal_cmd_send_fn send, void *io, uint8_t type,
                 const uint8_t *payload, uint8_t len)
{
    if (send != NULL) send(io, type, payload, len);
}

/** TEXT 行（payload = ASCII，无 NUL；超长行截断到 PROTO_PAYLOAD_MAX） */
static void emit_text(cal_cmd_send_fn send, void *io, const char *line)
{
    if (send == NULL || line == NULL) return;
    size_t n = strlen(line);
    if (n > PROTO_PAYLOAD_MAX) n = PROTO_PAYLOAD_MAX;
    emit(send, io, PROTO_TYPE_TEXT, (const uint8_t *)line, (uint8_t)n);
}

/**
 * @brief   发 ACK：payload = `[MOD][FUNC][status][data...]`
 *
 * @param[in] data  数据块（可为 NULL 当 data_len==0；总长上限 PROTO_PAYLOAD_MAX - 3）
 */
static void emit_ack(cal_cmd_send_fn send, void *io, uint8_t mod, uint8_t func,
                     uint8_t status, const uint8_t *data, uint8_t data_len)
{
    uint8_t p[PROTO_PAYLOAD_MAX];
    if (data_len > PROTO_PAYLOAD_MAX - PROTO_ACK_HEAD_LEN)
        data_len = PROTO_PAYLOAD_MAX - PROTO_ACK_HEAD_LEN;
    p[0] = mod;
    p[1] = func;
    p[2] = status;
    if (data != NULL && data_len != 0) memcpy(p + PROTO_ACK_HEAD_LEN, data, data_len);
    emit(send, io, PROTO_TYPE_ACK, p, (uint8_t)(PROTO_ACK_HEAD_LEN + data_len));
}

/* ---------------------------------------------------------------- 持久化 */

/**
 * @brief 介质键名（注册顺序 = 帧内序列化顺序，可直接对读；改动必须 bump schema）
 *
 * @note  前缀 `cal.` 是命名空间：同一个 pstore 实例将来若要装别的模块的键，
 *        靠前缀分区，不靠"谁先注册"。
 */
#define K_BIAS      "cal.bias"
#define K_TREF      "cal.t_ref"
#define K_GDRIFT    "cal.gyro_drift"
#define K_ADRIFT    "cal.accel_drift"
#define K_FLAGS     "cal.sys_flags"
#define K_BVALID    "cal.bias_valid"
#define K_TCVALID   "cal.tc_valid"
#define K_SCHEMA    "cal.schema"

/** 会话 → 介质镜像（写盘前的一步；字段来源单一，不在这里做任何判据） */
static void rec_from_session(cal_cmd_t *c)
{
    c->rec.bias        = c->bias;
    c->rec.t_ref       = c->comp.t_ref;
    memcpy(c->rec.gyro_drift,  c->gyro_drift,  sizeof(c->rec.gyro_drift));
    memcpy(c->rec.accel_drift, c->accel_drift, sizeof(c->rec.accel_drift));
    c->rec.sys_flags   = c->sys_flags;
    c->rec.bias_valid  = (uint8_t)(c->bias_valid ? 1 : 0);
    c->rec.tc_valid    = (uint8_t)(c->tc_valid ? 1 : 0);
    c->rec.schema      = c->store_cfg.schema;
}

/**
 * @brief 介质镜像 → 会话（仅在 ope 成功且 schema 已验证后调用）
 *
 * @retval  0  已恢复
 * @retval -1  temp_comp 拒绝参数（预设非法；属装配错误，不是存储错误）
 *
 * @note  温补实例必须**重建**而不是只拷字段：`comp` 里还有 applied / apply_count /
 *        last_applied_temp 等运行时态，锚点换了不重建会把旧锚点的阶梯状态带着跑。
 * @note  漂移系数只在 `tc_valid=1` 时下发给 `temp_comp`（未标定就保留预设默认系数，
 *        与 `cal_cmd_init` 同口径）；无论 valid 与否，系数本身都恢复到会话字段
 *        （GET_DRIFT 能看到历史上拟合过什么）。
 *        超幅值的坏值由 `temp_comp_set_drift` 单源钳制（防旧存档毁温补）。
 */
static int restore_from_rec(cal_cmd_t *c)
{
    if (temp_comp_init(&c->comp, c->tcfg, c->rec.t_ref, &c->rec.bias) != 0) return -1;
    if (c->rec.tc_valid) {
        if (temp_comp_set_drift(&c->comp, c->rec.gyro_drift,
                                c->rec.accel_drift) != 0) return -1;
    }
    c->bias = c->rec.bias;
    memcpy(c->gyro_drift,  c->rec.gyro_drift,  sizeof(c->gyro_drift));
    memcpy(c->accel_drift, c->rec.accel_drift, sizeof(c->accel_drift));
    c->sys_flags  = c->rec.sys_flags;
    c->bias_valid = c->rec.bias_valid ? 1 : 0;
    c->tc_valid   = c->rec.tc_valid ? 1 : 0;
    c->temp_now   = c->rec.t_ref;
    return 0;
}

/**
 * @brief 状态突变点的自动落盘 + TEXT 证据（未接入存储则静默 no-op）
 *
 * @param why  触发原因（写进 TEXT，便于 QEMU 日志/上位机对账："谁把它写进去的"）
 *
 * @note  失败**不静默**：存储写不进去是用户级事故（下次开机标定白做），必须看得见。
 */
static void store_commit(cal_cmd_t *c, cal_cmd_send_fn send, void *io, const char *why)
{
    if (c->ps == NULL) return;
    const int rc = cal_cmd_store_save(c);
    char line[112];
    if (rc == PSTORE_OK) {
        snprintf(line, sizeof(line),
                 "OK store saved %s bias_valid=%d tc_valid=%d flags=0x%04X len=%u",
                 why, c->bias_valid, c->tc_valid, (unsigned)c->sys_flags,
                 (unsigned)pstore_frame_len(c->ps));
    } else {
        snprintf(line, sizeof(line), "ERROR store save failed (%s): %s",
                 why, cal_cmd_store_err_name(rc));
    }
    emit_text(send, io, line);
}

/* ---------------------------------------------------------------- 命令分支 */

/** 启动零偏标定（CMD CALIBRATE 与明文行 CAL 的**唯一**实现——兼容别名不是第二套逻辑） */
static void cmd_calibrate(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    /* 幂等：采集中重发不重启、不清进度（原型 imu_calib_start 语义） */
    if (imu_calib_start(&c->cal) != 0) {
        emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_CALIBRATE,
                 PROTO_ACK_EXEC_FAIL, NULL, 0);
        return;
    }
    /* **异步**：此刻不回 ACK，等采集完毕（原型 app_imu.c:53-57） */
    c->ack_mod = PROTO_MOD_IMU;
    c->ack_func = PROTO_IMU_FUNC_CALIBRATE;
    c->ack_pending = 1;

    char line[64];
    snprintf(line, sizeof(line), "OK calib start N=%d", (int)c->bcal_cfg->target_n);
    emit_text(send, io, line);
}

/** 启动温漂标定（异步；params[0] = 秒，样本数 = 秒 × CAL_CMD_RATE_HZ） */
static void cmd_temp_calib(cal_cmd_t *c, const uint8_t *params, uint8_t len,
                           cal_cmd_send_fn send, void *io)
{
    uint32_t seconds = (len >= 1) ? (uint32_t)params[0] : 30u;   /* 原型缺省 30 s */
    if (seconds < 1u) {
        emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_TEMP_CALIB,
                 PROTO_ACK_BAD_ARG, NULL, 0);
        return;
    }

    /* 秒 → 样本数（原型 app_imu.c:70-72 的 seconds * 100 同口径） */
    temp_comp_cfg_t cfg = *c->tcfg;
    cfg.calib_target_n = (int32_t)(seconds * CAL_CMD_RATE_HZ);
    if (temp_calib_init(&c->tcal, &cfg) != 0 || temp_calib_start(&c->tcal) != 0) {
        emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_TEMP_CALIB,
                 PROTO_ACK_EXEC_FAIL, NULL, 0);
        return;
    }
    c->ack_mod = PROTO_MOD_IMU;
    c->ack_func = PROTO_IMU_FUNC_TEMP_CALIB;
    c->ack_pending = 1;

    char line[96];
    snprintf(line, sizeof(line),
             "OK tempcalib start %us N=%d need span >= %dmC",
             (unsigned)seconds, (int)cfg.calib_target_n,
             (int)(cfg.calib_min_span_c * 1000.0f));
    emit_text(send, io, line);
    if (!temp_comp_should_apply((c->sys_flags & PROTO_SYSF_TEMPCOMP_EN) ? 1 : 0,
                               c->tc_valid)) {
        emit_text(send, io, "NOTE tempcalib: TEMPCOMP_EN=0 or uncalibrated -> "
                            "not applied after fit (enable bit0 via SET_FLAGS)");
    }
}

/** GET_BIAS：ACK data = 6×float LE（gx,gy,gz,ax,ay,az —— imu_bias_t 字段序，陀螺在前） */
static void cmd_get_bias(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    float v[6] = { c->bias.gx, c->bias.gy, c->bias.gz,
                   c->bias.ax, c->bias.ay, c->bias.az };
    emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_GET_BIAS,
             PROTO_ACK_OK, (const uint8_t *)v, (uint8_t)sizeof(v));
}

/** GET_DRIFT：ACK data = gyro[3] + accel[3]（均为 float LE） */
static void cmd_get_drift(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    float v[6] = { c->gyro_drift[0], c->gyro_drift[1], c->gyro_drift[2],
                   c->accel_drift[0], c->accel_drift[1], c->accel_drift[2] };
    emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_GET_DRIFT,
             PROTO_ACK_OK, (const uint8_t *)v, (uint8_t)sizeof(v));
}

/**
 * @brief SET_FLAGS：写功能位（bit0 = TEMPCOMP_EN）
 *
 * @note  **不碰 tc_valid**（"已标定"与"是否应用"分离，原型 META-008）：
 *        开关打开但未标定 → 打印 WARN 且不生效；关闭且已标定 → 立即恢复 bias_ref。
 */
static void cmd_set_flags(cal_cmd_t *c, const uint8_t *params, uint8_t len,
                          cal_cmd_send_fn send, void *io)
{
    if (len < 2) {   /* 原型约定：len 不足回 0x01（参数错） */
        emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_SET_FLAGS,
                 PROTO_ACK_BAD_ARG, NULL, 0);
        return;
    }
    const uint16_t old_flags = c->sys_flags;
    c->sys_flags = (uint16_t)(params[0] | ((uint16_t)params[1] << 8));

    const int was_on = (old_flags & PROTO_SYSF_TEMPCOMP_EN) != 0;
    const int now_on = (c->sys_flags & PROTO_SYSF_TEMPCOMP_EN) != 0;

    if (temp_comp_should_apply(now_on, c->tc_valid)) {
        if (!was_on) {
            emit_text(send, io, "OK flags TEMPCOMP_EN=1 applied on next tick");
        }
    } else if (now_on && !c->tc_valid) {
        /* 打开但未标定 → 明确告警（不静默：用户以为开了） */
        emit_text(send, io, "WARN TEMPCOMP_EN=1 but tc_valid=0 -> not applied "
                            "(run TEMP_CALIB first)");
    }

    if (was_on && !now_on && c->tc_valid) {
        /* 关闭：立即把零偏恢复 bias_ref（temp_comp_disable 单源） */
        temp_comp_disable(&c->comp, &c->bias);
        char line[64];
        snprintf(line, sizeof(line), "OK flags TEMPCOMP_EN=0 bias restored gx=%+.6f",
                 (double)c->bias.gx);
        emit_text(send, io, line);
    }

    /* 功能位是持久化项：开关状态要跨重启保持（否则每次上电都得手点一遍） */
    store_commit(c, send, io, "flags");

    emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_SET_FLAGS, PROTO_ACK_OK, NULL, 0);
}

/** GET_STATUS：ACK data = 20B（布局单源见 frame.h 的 PROTO_TYPE_CMD 注释段） */
static void cmd_get_status(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    uint8_t d[20];
    memset(d, 0, sizeof(d));
    /* exp2 无推流控制/寄存器回读/姿态参数：这些槽位恒 0（不谎报） */
    d[6] = (uint8_t)(c->sys_flags & 0xFFu);
    d[7] = (uint8_t)(c->sys_flags >> 8);
    d[8] = c->bias_valid ? 1u : 0u;
    d[9] = c->tc_valid ? 1u : 0u;
    emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_GET_STATUS,
             PROTO_ACK_OK, d, (uint8_t)sizeof(d));
}

/** GET_TC_STATE：ACK data = 28B（valid / enabled / t_now / t_ref / last_applied / k_g[3] / count） */
static void cmd_get_tc_state(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    uint8_t d[28];
    memset(d, 0, sizeof(d));
    d[0] = c->tc_valid ? 1u : 0u;
    d[1] = (c->sys_flags & PROTO_SYSF_TEMPCOMP_EN) ? 1u : 0u;

    const float t_now = c->temp_now;
    const float t_ref = c->comp.t_ref;
    const float last = c->comp.applied ? c->comp.last_applied_temp : CAL_CMD_TC_NOT_APPLIED;
    memcpy(d + 2,  &t_now, 4);
    memcpy(d + 6,  &t_ref, 4);
    memcpy(d + 10, &last, 4);
    memcpy(d + 14, c->gyro_drift, 12);
    d[26] = (uint8_t)(c->comp.apply_count & 0xFFu);
    d[27] = (uint8_t)(c->comp.apply_count >> 8);
    emit_ack(send, io, PROTO_MOD_IMU, PROTO_IMU_FUNC_GET_TC_STATE,
             PROTO_ACK_OK, d, (uint8_t)sizeof(d));
}

/** MOD_IMU 二级分发（payload[0]=MOD 已判定；此处 func + params） */
static void dispatch(cal_cmd_t *c, uint8_t mod, uint8_t func,
                     const uint8_t *params, uint8_t len,
                     cal_cmd_send_fn send, void *io)
{
    if (mod != PROTO_MOD_IMU) {          /* 未实现的模块：明确拒绝，不静默 */
        emit_ack(send, io, mod, func, PROTO_ACK_UNKNOWN_FUNC, NULL, 0);
        return;
    }
    switch (func) {
    case PROTO_IMU_FUNC_CALIBRATE:    cmd_calibrate(c, send, io); break;
    case PROTO_IMU_FUNC_GET_BIAS:     cmd_get_bias(c, send, io); break;
    case PROTO_IMU_FUNC_TEMP_CALIB:   cmd_temp_calib(c, params, len, send, io); break;
    case PROTO_IMU_FUNC_SET_FLAGS:    cmd_set_flags(c, params, len, send, io); break;
    case PROTO_IMU_FUNC_GET_STATUS:   cmd_get_status(c, send, io); break;
    case PROTO_IMU_FUNC_GET_DRIFT:    cmd_get_drift(c, send, io); break;
    case PROTO_IMU_FUNC_GET_TC_STATE: cmd_get_tc_state(c, send, io); break;
    default:
        emit_ack(send, io, PROTO_MOD_IMU, func, PROTO_ACK_UNKNOWN_FUNC, NULL, 0);
        break;
    }
}

/* ---------------------------------------------------------------- 收帧 */

/**
 * @brief   尝试从 rx 缓冲弹出一帧并分发
 *
 * @retval  1  弹出一帧（已分发）
 * @retval  0  需要更多字节
 * @retval -1  出现了坏帧（已丢一字节重同步，调用方继续）
 *
 * @note    容错契约同上位机解码器：假 SOF / 超限 LEN / CRC 错 → 丢一字节重同步。
 */
static int rx_try_pop_frame(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    for (;;) {
        if (c->rx_len < 4) return 0;                      /* 头部不足 */
        if (c->rx[0] != PROTO_SOF) {                      /* 假 SOF｜重同步 */
            memmove(c->rx, c->rx + 1, (size_t)(c->rx_len - 1));
            c->rx_len--;
            continue;
        }
        const uint8_t payload_len = c->rx[1];
        if (payload_len > PROTO_PAYLOAD_MAX) {            /* 超限 LEN｜重同步 */
            memmove(c->rx, c->rx + 1, (size_t)(c->rx_len - 1));
            c->rx_len--;
            return -1;
        }
        const uint16_t total = (uint16_t)(4u + payload_len + 1u);
        if (c->rx_len < total) return 0;                  /* 等更多字节 */

        const uint8_t crc = proto_crc8(c->rx + 1, (size_t)(3u + payload_len));
        if (crc != c->rx[4 + payload_len]) {              /* CRC 错｜重同步 */
            memmove(c->rx, c->rx + 1, (size_t)(c->rx_len - 1));
            c->rx_len--;
            return -1;
        }

        const uint8_t type = c->rx[2];
        if (type == PROTO_TYPE_CMD && payload_len >= PROTO_CMD_HEAD_LEN) {
            const uint8_t mod  = c->rx[4];
            const uint8_t func = c->rx[5];
            dispatch(c, mod, func, c->rx + 4 + PROTO_CMD_HEAD_LEN,
                     (uint8_t)(payload_len - PROTO_CMD_HEAD_LEN), send, io);
        }
        /* 非 CMD 帧（例如上位机回环）：静默丢弃——本端不承担数据帧解析 */

        memmove(c->rx, c->rx + total, (size_t)(c->rx_len - total));
        c->rx_len = (uint16_t)(c->rx_len - total);
        return 1;
    }
}

/** 明文行命令：`CAL`(+\r\n) 等价 CALIBRATE；其余行静默忽略 */
static void line_flush(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    /* 取行内**第一个 token**（首个空白/CR 之前），把剩余部分丢弃 */
    char tok[CAL_CMD_LINE_CAP];
    uint8_t n = 0;
    uint8_t i = 0;
    while (i < c->line_len && (c->line[i] == ' ' || c->line[i] == '\t')) i++;  /* 前导空白 */
    for (; i < c->line_len && n < (uint8_t)(sizeof(tok) - 1); i++) {
        const char ch = c->line[i];
        if (ch == '\r' || ch == ' ' || ch == '\t') break;
        tok[n++] = ch;
    }
    tok[n] = '\0';
    c->line_len = 0;
    if (strcmp(tok, "CAL") == 0) {
        cmd_calibrate(c, send, io);   /* 兼容别名 = 同一实现（不是第二套逻辑） */
    }
}

/* ---------------------------------------------------------------- 采集中推进 */

/** 零偏标定收尾：累加残差 → 同步温补锚点 → 回异步 ACK + TEXT */
static void finish_bias_calib(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    imu_bias_t delta = { 0 };
    if (imu_calib_result(&c->cal, &delta) != 0) return;   /* 不可达（ready 已保证） */

    /* 自动补偿法：bias += delta（累加，不是覆盖）——原型 app_stream.c:106-108 */
    if (imu_calib_accumulate(&c->cal, &c->bias) != 0) return;
    c->bias_valid = 1;

    /* 锚点同步：新零偏对应此刻温度（否则下一次温补用旧锚点把刚标的值改回去） */
    temp_comp_init(&c->comp, c->tcfg, c->temp_now, &c->bias);
    if (c->tc_valid) {
        /* 温漂已标定 → 系数沿用拟合值；未标定则保持预设默认系数，
         * 但 tc_valid=0 使双判不成立，温补不生效（原型 E 修正同口径） */
        temp_comp_set_drift(&c->comp, c->gyro_drift, c->accel_drift);
    }

    char line[128];
    snprintf(line, sizeof(line),
             "OK calib done bias ax=%+.4f ay=%+.4f az=%+.4f "
             "gx=%+.4f gy=%+.4f gz=%+.4f temp=%.1f",
             (double)c->bias.ax, (double)c->bias.ay, (double)c->bias.az,
             (double)c->bias.gx, (double)c->bias.gy, (double)c->bias.gz,
             (double)c->temp_now);
    emit_text(send, io, line);

    if (c->ack_pending && c->ack_func == PROTO_IMU_FUNC_CALIBRATE) {
        emit_ack(send, io, c->ack_mod, c->ack_func, PROTO_ACK_OK, NULL, 0);
        c->ack_pending = 0;
    }

    /* 落盘（T-014）：零偏 + 锦点温度是新锚点，必须跨重启活下来 */
    store_commit(c, send, io, "calibrate");
}

/** 温标收尾：读拟合系数 → 覆盖温补系数 → 置 tc_valid → 回异步 ACK + TEXT */
static void finish_temp_calib(cal_cmd_t *c, cal_cmd_send_fn send, void *io)
{
    if (temp_calib_result(&c->tcal, c->gyro_drift, c->accel_drift) != 0) return;
    temp_comp_set_drift(&c->comp, c->gyro_drift, c->accel_drift);

    /* tc_valid = 拟合是否有效（跨度不足/斜率越界 → 0，温补保持关闭）。
     * 原型 poll_temp_calib 同口径；**不碰** TEMPCOMP_EN（开关与标定分离）。 */
    c->tc_valid = (temp_calib_valid(&c->tcal) == 1) ? 1 : 0;
    if (c->tc_valid) {
        /* 锚点值对齐当前零偏（原型 app_stream.c:209 同式：只改 bias_ref，
         * t_ref 保持零偏标定时刻的温度——原型即如此，未在此处改动） */
        c->comp.bias_ref = c->bias;
    }

    char line[128];
    snprintf(line, sizeof(line),
             "OK tempcalib %s k_g=(%.5f %.5f %.5f) dps/C valid=%d en=%d",
             c->tc_valid ? "done" : "REJECTED",
             (double)c->gyro_drift[0], (double)c->gyro_drift[1], (double)c->gyro_drift[2],
             c->tc_valid,
             (c->sys_flags & PROTO_SYSF_TEMPCOMP_EN) ? 1 : 0);
    emit_text(send, io, line);
    if (!c->tc_valid) {
        emit_text(send, io, "NOTE tempcalib rejected: need temperature span, "
                            "or k out of physical range");
    }

    if (c->ack_pending && c->ack_func == PROTO_IMU_FUNC_TEMP_CALIB) {
        /* ACK data = 6×float 拟合 k + tc_valid u8（29B ≤ payload 上限；
         * 上位机对该功能码不解析 data，额外载荷为兼容增量） */
        uint8_t d[25];
        float v[6] = { c->gyro_drift[0], c->gyro_drift[1], c->gyro_drift[2],
                       c->accel_drift[0], c->accel_drift[1], c->accel_drift[2] };
        memcpy(d, v, sizeof(v));
        d[24] = c->tc_valid ? 1u : 0u;
        emit_ack(send, io, c->ack_mod, c->ack_func, PROTO_ACK_OK, d, (uint8_t)sizeof(d));
        c->ack_pending = 0;
    }

    /* 落盘（T-014）：系数 + tc_valid 跨重启保持（否则每次上电温补都不可用） */
    store_commit(c, send, io, "tempcalib");
}

/* ---------------------------------------------------------------- 对外接口 */

int cal_cmd_init(cal_cmd_t *c, const imu_calib_cfg_t *bcal_cfg,
                 const temp_comp_cfg_t *tcfg, float t_ref,
                 const imu_bias_t *bias_ref, uint16_t sys_flags)
{
    if (c == NULL || bcal_cfg == NULL || tcfg == NULL || bias_ref == NULL) return -1;
    memset(c, 0, sizeof(*c));
    c->bcal_cfg = bcal_cfg;
    c->tcfg = tcfg;

    if (imu_calib_init(&c->cal, bcal_cfg) != 0) return -1;
    if (temp_calib_init(&c->tcal, tcfg) != 0) return -1;
    if (temp_comp_init(&c->comp, tcfg, t_ref, bias_ref) != 0) return -1;

    c->bias = *bias_ref;
    c->bias_valid = 0;
    c->tc_valid = 0;
    c->sys_flags = sys_flags;
    c->temp_now = t_ref;
    c->ack_mod = PROTO_MOD_IMU;
    c->ack_func = 0;
    c->ack_pending = 0;
    c->rx_len = 0;
    c->line_len = 0;
    return 0;
}

void cal_cmd_rx(cal_cmd_t *c, const uint8_t *buf, size_t n,
                cal_cmd_send_fn send, void *io)
{
    if (c == NULL || buf == NULL) return;

    for (size_t i = 0; i < n; i++) {
        const uint8_t b = buf[i];
        /* 帧与明文行共用一条字节流：缓冲为空时才可能是行命令的首字节 */
        if (c->rx_len == 0 && b != PROTO_SOF) {
            if (b == '\n') {
                line_flush(c, send, io);
            } else if (c->line_len + 1u < (uint8_t)sizeof(c->line)) {
                c->line[c->line_len++] = (char)b;
            } else {
                c->line_len = 0;   /* 超长行：整体丢弃（不截断执行） */
            }
            continue;
        }
        if (c->rx_len + 1u > (uint16_t)sizeof(c->rx)) {
            /* 缓冲溢出（理论上不会：最长帧 = PROTO_FRAME_MAX）→ 整体丢弃重同步 */
            c->rx_len = 0;
            continue;
        }
        c->rx[c->rx_len++] = b;
        while (rx_try_pop_frame(c, send, io) == 1) { /* 一次可弹多帧（粘包） */ }
    }
}

int cal_cmd_step(cal_cmd_t *c, const imu_sample_t *raw, float temp,
                 cal_cmd_send_fn send, void *io)
{
    if (c == NULL || raw == NULL) return -1;
    c->temp_now = temp;

    /* ---- ① 零偏标定采集：喂 raw − bias（残差），raw 另传（判重力轴） ---- */
    if (c->cal.state == IMU_CALIB_COLLECTING) {
        imu_sample_t resid = {
            .ax = raw->ax - c->bias.ax, .ay = raw->ay - c->bias.ay,
            .az = raw->az - c->bias.az,
            .gx = raw->gx - c->bias.gx, .gy = raw->gy - c->bias.gy,
            .gz = raw->gz - c->bias.gz,
        };
        if (imu_calib_feed(&c->cal, &resid, raw) != 0) return -1;
        if (imu_calib_ready(&c->cal) == 1) {
            finish_bias_calib(c, send, io);
        } else if (c->cal.n % 100u == 0u) {
            char line[48];
            snprintf(line, sizeof(line), "OK calib progress %u/%d",
                     (unsigned)c->cal.n, (int)c->bcal_cfg->target_n);
            emit_text(send, io, line);
        }
    }

    /* ---- ② 温标采集：喂原始值（常值偏移进截距，不影响斜率） ---- */
    if (c->tcal.state == TEMP_CALIB_COLLECTING) {
        if (temp_calib_feed(&c->tcal, temp, raw) != 0) return -1;
        if (temp_calib_ready(&c->tcal) == 1) {
            finish_temp_calib(c, send, io);
        }
    }

    /* ---- ③ 温补：双判（TEMPCOMP_EN && tc_valid）+ 零偏标定期冻结 ---- */
    if (temp_comp_should_apply((c->sys_flags & PROTO_SYSF_TEMPCOMP_EN) ? 1 : 0,
                              c->tc_valid) &&
        c->cal.state != IMU_CALIB_COLLECTING) {
        /* 阶梯判据在组件内：档内保持上次值，跨档才重算（返回 0 也是有效输出） */
        if (temp_comp_apply(&c->comp, temp, &c->bias) != 0) return -1;
    }
    return 0;
}

const imu_bias_t *cal_cmd_bias(const cal_cmd_t *c)
{
    return (c != NULL) ? &c->bias : NULL;
}

int cal_cmd_seed_calibrated(cal_cmd_t *c, const imu_bias_t *bias, float t_ref)
{
    if (c == NULL || bias == NULL) return -1;
    c->bias = *bias;
    c->bias_valid = 1;
    if (temp_comp_init(&c->comp, c->tcfg, t_ref, bias) != 0) return -1;
    if (c->tc_valid) temp_comp_set_drift(&c->comp, c->gyro_drift, c->accel_drift);
    c->temp_now = t_ref;
    return 0;
}

int cal_cmd_collecting(const cal_cmd_t *c)
{
    if (c == NULL) return 0;
    return (c->cal.state == IMU_CALIB_COLLECTING ||
            c->tcal.state == TEMP_CALIB_COLLECTING) ? 1 : 0;
}

/* ---------------------------------------------------------------- 持久化接口 */

int cal_cmd_store_open(cal_cmd_t *c, const cal_cmd_store_cfg_t *cfg)
{
    if (c == NULL || cfg == NULL || cfg->backend == NULL) return PSTORE_ERR_ARG;
    if (cfg->capacity < PSTORE_FRAME_HEADER_BYTES) return PSTORE_ERR_ARG;

    cal_cmd_store_close(c);   /* 重入安全：换后端/换容量实验时不泄漏旧实例 */

    const pstore_cfg_t pc = {
        .backend   = cfg->backend,
        .capacity  = cfg->capacity,
        .max_items = CAL_CMD_STORE_MAX_ITEMS,
        .magic     = cfg->magic,
    };
    pstore_t *ps = pstore_create(&pc);
    if (ps == NULL) return PSTORE_ERR_ARG;

    c->ps = ps;
    c->store_cfg = *cfg;
    memset(&c->rec, 0, sizeof(c->rec));   /* 干净镜像：load 失败时不留旧值 */

    /* 注册面：键名 + 镜像字段 + 宽度（注册序 = 序列化序，可直接对读） */
    int rc = pstore_register(ps, K_BIAS,   &c->rec.bias,        sizeof(c->rec.bias));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_TREF,   &c->rec.t_ref,       sizeof(c->rec.t_ref));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_GDRIFT, &c->rec.gyro_drift,  sizeof(c->rec.gyro_drift));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_ADRIFT, &c->rec.accel_drift, sizeof(c->rec.accel_drift));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_FLAGS,  &c->rec.sys_flags,   sizeof(c->rec.sys_flags));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_BVALID, &c->rec.bias_valid,  sizeof(c->rec.bias_valid));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_TCVALID,&c->rec.tc_valid,    sizeof(c->rec.tc_valid));
    if (rc == PSTORE_OK) rc = pstore_register(ps, K_SCHEMA, &c->rec.schema,      sizeof(c->rec.schema));
    if (rc != PSTORE_OK) {
        cal_cmd_store_close(c);   /* 装配错误：注册表满/重复键 —— 不留半成品实例 */
        return rc;
    }

    /* 两阶段原子 load：失败时 rec 零改动（pstore 承诺），会话本来就没被碰过 */
    rc = pstore_load(ps);
    if (rc != PSTORE_OK) return rc;

    /* schema 闸门："读得进"不等于"看得懂"（旧固件拒新记录，反之亦然） */
    if (c->rec.schema != cfg->schema) return CAL_CMD_STORE_ERR_SCHEMA;

    return (restore_from_rec(c) == 0) ? 0 : PSTORE_ERR_ARG;
}

int cal_cmd_store_save(cal_cmd_t *c)
{
    if (c == NULL) return PSTORE_ERR_ARG;
    if (c->ps == NULL) return CAL_CMD_STORE_ERR_UNATTACHED;
    rec_from_session(c);
    pstore_mark_dirty(c->ps);   /* 状态快照可能与介质不一致（例如开机标定窗注入），无条件置脏 */
    return pstore_save(c->ps);
}

void cal_cmd_store_close(cal_cmd_t *c)
{
    if (c == NULL) return;
    pstore_destroy(c->ps);   /* NULL 安全 */
    c->ps = NULL;
    memset(&c->store_cfg, 0, sizeof(c->store_cfg));
}

int cal_cmd_store_attached(const cal_cmd_t *c)
{
    return (c != NULL && c->ps != NULL) ? 1 : 0;
}

const char *cal_cmd_store_err_name(int err)
{
    switch (err) {
    case CAL_CMD_STORE_ERR_SCHEMA:
        return "record schema mismatch (firmware/record version differ)";
    case CAL_CMD_STORE_ERR_UNATTACHED:
        return "store not attached";
    default:
        return pstore_err_name(err);
    }
}
