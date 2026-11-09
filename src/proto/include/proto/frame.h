/* SPDX-License-Identifier: MIT */
#ifndef PROTO_FRAME_H
#define PROTO_FRAME_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file    frame.h
 * @brief   实验固件 ↔ 上位机 最小串口帧契约（自研，规格单源住 xiao 组件库/契约）
 *
 * @note    帧格式（小端，字节流）：
 *          ┌─────┬─────┬──────┬─────┬──────────┬───────┐
 *          │ SOF │ LEN │ TYPE │ SEQ │ PAYLOAD  │ CRC8  │
 *          │ 0xA5│ 1B  │ 1B   │ 1B  │ LEN 字节 │ 1B    │
 *          └─────┴─────┴──────┴─────┴──────────┴───────┘
 *          CRC8 多项式 0x07、初值 0x00，覆盖 LEN..PAYLOAD（不含 SOF）。
 * @note    SEQ 由发送端自增（0xFF 回绕），上位机用于丢帧统计。
 */

#define PROTO_SOF        0xA5

/* 帧类型（exp1 起步集，后续实验增补走 spec 版本化） */
#define PROTO_TYPE_IMU_SAMPLE 0x01   /**< payload = 6×float LE（ax..gz，g/dps）= 24B */
/**
 * 调零 + 温补帧（exp2）：payload = 19×float LE = 76B，四个块顺序与单位如下
 * （全部小端；**零偏块用采样序 ax,ay,az,gx,gy,gz，与 imu_bias_t 的字段序
 * gx..gz,ax..az 不同**——打包侧按字段重排，解码按本表）：
 *
 *  偏移  0：原始六轴   ax,ay,az（g），gx,gy,gz（dps）  24B（未去偏）
 *  偏移 24：补偿后六轴 ax,ay,az（g），gx,gy,gz（dps）  24B（已减当前零偏）
 *  偏移 48：当前零偏   ax,ay,az（g），gx,gy,gz（dps）  24B（温补后生效值）
 *  偏移 72：芯片温度   temp（°C）                       4B
 */
#define PROTO_TYPE_IMU_CAL    0x02
/**
 * 欧拉角姿态帧（exp3，启用历史保留位 0x03）：payload = roll,pitch,yaw 三个
 * float32 小端 = 12B，单位**弧度**，ZYX 约定（与 mahony_get_angles 输出
 * 同源：roll 绕 X、pitch 绕 Y、yaw 绕 Z）。
 * 布局备选 12B+3×float 角速度(dps)=24B 已评审弃用：角速度 0x01 帧
 * （IMU_SAMPLE）已有单源，双帧型重复携带徒增对齐负担；需要速率的消费方
 * 差分即可——本帧保持最小 12B。
 * 教学定位（exp3↔exp4 对照）：本帧是欧拉通道的教学出口——pitch ±90° 处
 * ZYX 分解万向锁（yaw/roll 融合丢一自由度，spec §5 与上位机页有说明）；
 * 3D/渲染主通道仍是 0x04 四元数帧（无奇异）。
 */
#define PROTO_TYPE_EULER      0x03
/**
 * 四元数姿态帧（exp4）：payload = qw,qx,qy,qz 四个 float32 小端 = 16B。
 * 分量顺序 w 在前（与固件 mahony_get_quat 输出一致）；|q|≈1（归一化保持）。
 * 帧序号走帧头 SEQ 字段（发送端自增、0xFF 回绕），不占 payload。
 * 欧拉角另有教学通道 0x03（exp3，ZYX 弧度）；本帧仍是 3D 主通道，
 * 需要欧拉的消费方亦可从四元数反算（单源解算）。
 */
#define PROTO_TYPE_QUAT       0x04
/**
 * 电机遥测帧（exp_motor，T-102e）：payload = 4 × 16B 记录（槽序固定 0..3，
 * 一控制周期一帧；记录布局全部小端，槽 0/1/2/3 = PWM_OPEN/SERVO/ENC_CLOSED/
 * MOTOR_DC——四类齐。DC 槽为 3→4 槽扩展：记录布局逐字节不变，解码器按
 * payload 长度自适配）：
 *
 *  偏移  0：u8  slot      槽号（0=PWM_OPEN，1=SERVO，2=ENC_CLOSED，3=MOTOR_DC）
 *  偏移  1：u8  kind      motor_kind_t 枚举值（0..3，定义见 mtr/motor_types.h）
 *  偏移  2：f32 setpoint  S 梯度当前设定值 LE（PWM_OPEN: duty 0~1；
 *                         SERVO: 角度 deg；ENC_CLOSED: rpm 目标；
 *                         MOTOR_DC: 带符号 duty −1~+1，符号 = H 桥方向位）
 *  偏移  6：f32 duty      当前输出占空 LE（单通道槽幅值 0~1，SERVO 为脉宽换算
 *                         后占空；MOTOR_DC = 方向桥带符号回读 −1~+1，符号=方向位）
 *  偏移 10：f32 actual    实际值 LE（PWM_OPEN=施加占空；SERVO=施加角度 deg；
 *                         ENC_CLOSED=编码器测量 rpm；MOTOR_DC=带符号施加占空）
 *  偏移 14：u8  phase     该槽 S 梯度相位（scurve_phase_t 0..7，遥测用）
 *  偏移 15：u8  fault     motor_state_t.fault 低 8 位（位义见 mtr/motor_types.h）
 *
 * @note 64B ≤ PAYLOAD_MAX；浮点单位契约与电机库 motor_types.h 同源（rpm/deg/duty）。
 */
#define PROTO_TYPE_MOTOR      0x05
/**
 * 电机实验场景帧（exp_motor，T-112 四专项 + DC 收满）：场景运行时
 * （motor_scen.c）按「SCEN <n>」串口命令激活后逐拍发送；payload = 场景头 +
 * 定时器快照 + 电机记录：
 *
 *  偏移  0：u8  scen_id    场景号（1=pwm 阶梯，2=svc 标定，3=enc 阶跃，
 *                          4=nested，5=dc 正反转切换）
 *  偏移  1：u8  n_tim      定时器快照数（1~2）
 *  偏移  2：n_tim × 6B 定时器记录（idx 递增 0..n_tim-1，全部小端）：
 *            u8 tim_idx | u8 channels | u32 freq_hz（pwm_timer_freq 活实例现取）
 *  偏移  2+6*n_tim：u8 n_rec     电机记录数（1~3，场景表单源）
 *  偏移  3+6*n_tim：n_rec × 16B 电机记录——**布局与 0x05 的 16B 记录逐字节同构**
 *            （slot/kind/setpoint/duty/actual/phase/fault；场景帧内 phase=步进下标
 *            而非 scurve 相位，duty 为 PWM 合成存储回读即物理输出面——DC 槽为
 *            方向桥带符号回读，符号 = 方向位）。
 *
 * @note 总长 = 3 + 6*n_tim + 16*n_rec（最大 63B ≤ PAYLOAD_MAX）；场景号与槽数
 *       单源 = motor 场景表单源（apps 侧）；slot 域按 kind 同 0x05。
 */
#define PROTO_TYPE_SCEN       0x06
/**
 * 命令帧（host → device；exp2 起 IMU 调零/温补的唯一的命令通道）：
 * payload = `[MOD_ID u8][FUNC_ID u8][params...]`，设备以 0x7E ACK 应答
 * （ACK payload = `[MOD_ID][FUNC_ID][status][data...]`，status 语义见下）。
 * 语义锚原型仓 `旧式实现` `plugin/serial/serial.h`（两级命令编码），
 * 即 "CMD 帧 data = [MOD_ID][FUNC_ID][params…]"。
 */
#define PROTO_TYPE_CMD        0x7D

/* 模块 ID（CMD/ACK payload[0]）
 * 只登记本仓已实现的模块；0x02~0xFE 留给后续模块（如电机）扩展。 */
#define PROTO_MOD_IMU        0x01   /**< 六轴惯性（本模块的命令族走 0x7D/0x7E） */
#define PROTO_MOD_STORE      0x20   /**< 参数存储模块（T-006/T-018）：
                                     *   pstore 持久化参数读写/清空 */

/* MOD_IMU 功能码（CMD payload[1]，ACK 回显同编号）
 * 编号与语义均锚原型 FUNC_IMU_*（见 serial.h:48-78）：本表只登记已落地项，
 * 未列出的编号（0x04..0x08/0x0B/0x0E/0x0F）留待后续增量。 */
#define PROTO_IMU_FUNC_CALIBRATE    0x01  /**< 启动零偏标定（**异步**：采集完毕才回 ACK，data 空） */
#define PROTO_IMU_FUNC_GET_BIAS     0x02  /**< 读当前生效零偏：ACK data = 6×float LE（gx,gy,gz,ax,ay,az） */
#define PROTO_IMU_FUNC_TEMP_CALIB   0x03  /**< 温漂标定（**异步**）：params[0]=秒（样本数=秒×100）；ACK data = 6×float 拟合 k + tc_valid u8 */
#define PROTO_IMU_FUNC_SET_FLAGS    0x09  /**< 写功能位：params = u16 LE（bit0 = TEMPCOMP_EN）；data 空 */
#define PROTO_IMU_FUNC_GET_STATUS   0x0A  /**< 状态快照：ACK data = 20B（布局见下） */
#define PROTO_IMU_FUNC_GET_DRIFT    0x0D  /**< 读温漂系数：ACK data = 6×float LE（gyro[3], accel[3]） */
#define PROTO_IMU_FUNC_GET_TC_STATE 0x10  /**< 温补状态快照：ACK data = 28B（布局见下） */

/* MOD_STORE 功能码（CMD payload[1]，ACK 回显同编号）
 * 语义：读写 pstore 持久化条目（与 IMU 的七命令完全隔离，演示第二模块） */
#define PROTO_STORE_FUNC_GET    0x01  /**< 读全体条目：ACK data = 22B（布局见下） */
#define PROTO_STORE_FUNC_SET    0x02  /**< 写条目：params = [count u8][(key_len u8 < 32 key dlen u8 data)×count]，
                                         每次 SET 后自动落盘；ACK data 空 */
#define PROTO_STORE_FUNC_ERASE  0x03  /**< 擦除介质（下次开机走默认值路径）；ACK data 空 */
#define PROTO_STORE_FUNC_STATUS 0x04  /**< 存储状态：ACK data = [state u8][frame_len u16][dirty u8]（布局见下） */

/* MOD_STORE·GET（0x01）ACK data 布局（22B，小端）：
 *   [0:4]   boot_count  u32（断电续跑计数器，每次开机 +1 落盘）
 *   [4:8]   gain        f32（示例 float 参数）
 *   [8]     flags       u8（示例位集参数）
 *   [9:17]  note        u8[8]（示例字符串参数，无 NUL 约定）
 *   [17]    load_state  u8（0=fresh 首载默认值 / 1=restored 恢复存档 / 2..=pstore 错误码取反）
 *   [18:20] frame_len   u16（最近存档帧长）
 *   [20]    dirty       u8
 *   [21]    capacity_kb u8（存储区容量/1024，四舍五入） */

/* MOD_STORE·STATUS（0x04）ACK data 布局（6B）：
 *   [0]     load_state   u8（同上）
 *   [1:3]   frame_len    u16 LE
 *   [3]     dirty        u8
 *   [4]     entries      u8（当前注册条目数）
 *   [5]     capacity_kb  u8 */

/* ACK status 字节（payload[2]）—— 语义锚原型 app_imu.c 的 app_send_ack 约定 */#define PROTO_ACK_OK           0x00  /**< 执行成功 */
#define PROTO_ACK_BAD_ARG      0x01  /**< 参数错（长度/取值域） */
#define PROTO_ACK_EXEC_FAIL    0x02  /**< 执行失败（硬件/状态不允许） */
#define PROTO_ACK_UNKNOWN_FUNC 0xFF  /**< 未知功能码 */

/* SYS_FLAGS 位定义（SET_FLAGS params / GET_STATUS data[6:8]）
 * 本表只登记已落地位，编号与语义锚原型 `plugin/serial/serial.h` 的 SYSF_*：
 * bit0 = 温漂补偿生效（**需已标定**；开关与标定标志分离）。
 * 其余位（调试打印/应用零偏/原始流/上电推流）属推流与寄存器配置，本帧未实现。 */
#define PROTO_SYSF_TEMPCOMP_EN 0x0001

/* GET_STATUS（0x0A）ACK data 布局（20B，小端）—— 与上位机 parse_ack_data 同源：
 *   [0]  stream_active     推流是否开启（exp2 无推流控制，恒 0）
 *   [1]  stream_mask       通道掩码（同上，恒 0）
 *   [2]  rate_hz           上报频率（同上，0）
 *   [3]  ctrl1_xl          加速度控制寄存器（exp2 不回读，0）
 *   [4]  ctrl2_g           陀螺控制寄存器（同上，0）
 *   [5]  i2c_400k          I2C 是否 400 kHz（同上，0）
 *   [6:8] flags            SYS_FLAGS u16 LE（bit0 = TEMPCOMP_EN）
 *   [8]  bias_valid        零偏是否已标定
 *   [9]  tc_valid          温漂是否已标定（≠ 是否启用：启用看 flags bit0）
 *   [10] kp×10（int8，姿态参数；exp2 无姿态链，0）
 *   [11] ki×100（int8，同上，0）
 *   [12] accel_odr / [13] gyro_odr（同上，0）
 *   [14:20] 保留 0
 */

/* GET_TC_STATE（0x10）ACK data 布局（28B，小端）：
 *   [0]     tc_valid      已标定？
 *   [1]     enabled       TEMPCOMP_EN 开关已开？
 *   [2:14]  t_now, t_ref, last_applied（3×float）
 *   [14:26] k_g[3]（3×float，dps/°C）
 *   [26:28] apply_count u16 LE（实际重算次数）
 */
#define PROTO_TYPE_TEXT       0x7F   /**< payload = ASCII 文本（日志/错误行，无 NUL） */
/**
 * ACK 帧：历史形态 payload = 1B 状态码（早期保留位，无实现占用）；
 * **CMD 命令的应答形态** = `[MOD_ID][FUNC_ID][status][data...]`
 * （status 取 PROTO_ACK_*；异步命令 CALIBRATE/TEMP_CALIB 在采集完成时才发）。
 * 两种形态由 payload 长度区分（≥3 字节 = 命令应答），解码器按功能码取 data。
 */
#define PROTO_TYPE_ACK        0x7E

/**
 * CMD/ACK 二级命令寻址字段（payload 前缀，两侧实现必须同序）：
 * - CMD payload：`[MOD_ID][FUNC_ID][params...]`（LEN ≥ 2）
 * - ACK payload：`[MOD_ID][FUNC_ID][status][data...]`（LEN ≥ 3）
 */
#define PROTO_CMD_HEAD_LEN    2
#define PROTO_ACK_HEAD_LEN    3

#define PROTO_PAYLOAD_MAX 96         /**< 单帧 payload 上限（exp2 的 0x02 帧 76B，取 96 留同量级余量） */
/* 整帧字节数上限：SOF(1)+LEN(1)+TYPE(1)+SEQ(1)+PAYLOAD(len)+CRC8(1) = len+5。
 * 曾误写成 (3 + PAYLOAD_MAX + 1)=100，比真实需要少 1 字节：payload 打满 96 时
 * proto_frame_encode 的 out_cap < len+5 判定为真而返回 0，最大帧被静默丢弃。 */
#define PROTO_FRAME_OVERHEAD 5                      /* SOF+LEN+TYPE+SEQ+CRC8 */
#define PROTO_FRAME_MAX   (PROTO_PAYLOAD_MAX + PROTO_FRAME_OVERHEAD)

/** @brief CRC-8（多项式 0x07，初值 0x00；固件与上位机实现必须同结果） */
uint8_t proto_crc8(const uint8_t *data, size_t n);

/**
 * @brief   编码一帧
 * @param   type/seq 帧头字段；payload 可为 NULL（len==0）
 * @return  帧总字节数（含 SOF..CRC8）；0 = 参数越界（len > PROTO_PAYLOAD_MAX）
 */
size_t proto_frame_encode(uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *out, size_t out_cap);

#endif /* PROTO_FRAME_H */
