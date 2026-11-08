# exp_motor_pwm — 单 PWM 开环电机：占空阶梯扫描

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

单台开环 PWM 电机做占空阶梯扫描：0 / 25 / 50 / 75 / 100 % 五档、每档 2 s 循环，
500 Hz 控制环逐拍出 `0x05` 遥测帧。用于观察「占空 → 转速」的单调性与死区。

## 二、帧与参数

- `0x05` MOTOR：单实例 = 1×16 B 记录；布局单源 `src/proto/frame.h`。
- 参数单源：实验表 `main/exp_motor_pwm_scen.c`（阶表 / 保持拍数 / wiring）；
  定时器底版 = `xiao_motor` 预设 `pwm.motor.20k`。

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

