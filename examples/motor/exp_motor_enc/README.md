# exp_motor_enc — 单编码器电机：速度环阶跃响应

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

单台 N20 带编码器电机（速度环 PID + 合成一阶滞后 plant）做阶跃响应：
30 → 90 rpm 交替、每段 4 s（每循环两条上升沿，可统计重复性），500 Hz 控制环
逐拍出 `0x05` 帧（`actual` = 编码器测量 rpm）。

## 二、帧与参数

- `0x05` MOTOR：单实例 = 1×16 B 记录；布局单源 `src/proto/frame.h`。
- 参数单源：实验表 `main/exp_motor_enc_scen.c`；闭环底版 = `xiao_motor` 预设
  `motor.enc.n20`（cpr = 7200）。
- 合成 HAL：`examples/_shared/motor_hal_sim.c`（编码器测量也由该 HAL 提供）。

## 三、变体宏

| 宏 | 1（唯一支持值） | 0 |
|---|---|---|
| `EXP_SIM` | 合成 PWM + 编码器 HAL | 构建期 `#error` 显式拒绝 |
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

