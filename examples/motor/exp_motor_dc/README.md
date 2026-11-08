# exp_motor_dc — 单 DC 电机：H 桥正反转切换

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

单台直流电机（H 桥）做正反转切换：+50 % 持续 2 s ↔ −50 % 持续 2 s 循环，
500 Hz 控制环逐拍出 `0x05` 遥测帧（`duty` 字段 = 方向桥带符号回读，符号即方向位）。

同系列的单例工程：`../exp_motor_pwm`（开环占空）/ `../exp_motor_servo`（舵机角度）/
`../exp_motor_enc`（编码器闭环）/ `../exp_motor_nested`（嵌套定时器通道隔离）。

## 二、帧与参数

- `0x05` MOTOR：单实例 = 1×16 B 记录，`duty` 为带符号域 −1~+1；布局单源
  `src/proto/frame.h`。
- 参数单源：实验表 `main/exp_motor_dc_scen.c`（步表 / 保持拍数 / wiring）；
  电机与定时器底版来自 `xiao_motor` 预设（`motor.dc.generic` @ `pwm.motor.20k`，
  桥对 ch0/ch1、pin 11/12）。
- 合成 HAL：`examples/_shared/motor_hal_sim.c`（硬件 PWM/编码器 HAL 未随本仓发布）。

## 三、变体宏

| 宏 | 1（唯一支持值） | 0 |
|---|---|---|
| `EXP_SIM` | 合成方向桥 HAL | 构建期 `#error` 显式拒绝（真实电机 HAL 未发布，不静默编「假硬件」固件） |
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

