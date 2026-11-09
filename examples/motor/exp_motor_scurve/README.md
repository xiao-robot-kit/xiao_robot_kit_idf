# exp_motor_scurve — S 曲线七相位轨迹（双槽）

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

双槽各跑一条七相位 S 梯度点对点轨迹（`xiao_motor` 的 scurve 闭式规划）：曲线
**速度**输出作为前馈设定，一条腿走完停驻后 rewind 重扫循环。逐拍出 `0x05` 帧
（`setpoint` = 速度目标 / `duty` = 占空回读 / `actual` = 速度实测 / `phase` = 七相位）。

按相位分段的形状判据：加速段占空单调升、匀速段平稳（波动 < 阈值）、减速段
单调降、到目标收敛（±2 %）。

## 二、帧与参数

- `0x05` MOTOR：双槽 ×16 B 记录（槽 0 = PWM_OPEN，槽 1 = ENC_CLOSED）；布局单源
  `src/proto/include/proto/frame.h`。
- 参数单源：轨迹点 / 时限 / 前馈系数 = `main/exp_motor_scurve_scen.c`；
  scurve 规划参数由 `xiao_motor` 组件提供。

## 三、变体宏

| 宏 | 1（唯一支持值） | 0 |
|---|---|---|
| `EXP_SIM` | 合成 PWM/编码器 HAL | 构建期 `#error` 显式拒绝 |
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

