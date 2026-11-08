# exp_motor_pidscan — PID 参数网格扫描（3×3 = 9 组）

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

同一台 N20 带编码器电机轮转参数网格：`kp × ki` 3×3 = 9 组，每组按网格值覆盖
底版 `motor.enc.n20` 的 `vel_pid` 后跑 30 → 90 rpm 阶跃（低段 1 s 前导 +
高段 2 s 统计窗，500 Hz 控制环），记录三指标：**收敛时间 / 超调 % / 稳态误差 %**。

9 组跑完按 `score = settle + overshoot/100 + ss_err/100` 选最优组，用 TEXT 帧逐行
输出**对比表**（最优组以 `*` 标注）；一轮 27 s，之后循环重扫，便于长期观察一致性。

## 二、帧与参数

- `0x05` MOTOR：逐拍遥测（`setpoint` / `actual` / `duty` / `phase`）。
- `0x7F` TEXT：9 组对比表 + 最优组标注。
- 参数单源：网格与段长 = `main/exp_motor_pidscan_scen.c`；底版 = `motor.enc.n20`。

## 三、变体宏

| 宏 | 1（唯一支持值） | 0 |
|---|---|---|
| `EXP_SIM` | 合成 PWM + 编码器 HAL + 一阶滞后 plant | 构建期 `#error` 显式拒绝 |
| `EXP_QEMU_UART` | 帧出口 = UART0 | 帧出口 = USB-Serial-JTAG（真板档） |

## 构建与运行（本仓）

本工程以库形态消费仓库根目录的组件（`xiao_motor, xiao_proto`），不含任何外部依赖副本：

```bat
run.bat COM5      :: build + flash + serial monitor (real board)
run_qemu.bat      :: build the synthetic-data variant only (no board needed)
```

等价命令行：

```bat
idf.py build
idf.py -B build-sim -DEXP_SIM=1 -DEXP_QEMU_UART=1 build
```

