# exp_motor_nested — 嵌套定时器：通道隔离

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

验证两套定时器并存下的通道隔离：A/B 双电机共享 20 kHz 定时器（ch0/ch1，A 做
阶梯步进=动方、B 恒占空 0.5=静方）+ C 舵机独立 50 Hz 定时器（慢角度摆动）。
500 Hz 控制环共拍逐槽出 `0x05` 帧（3×16 B；B 的 `duty` = 存储回读，A 一旦串扰即破）。

## 二、帧与参数

- `0x05` MOTOR：3 槽 ×16 B 记录（槽序 = A / B / C）；布局单源 `src/proto/frame.h`。
- 参数单源：实验表 `main/exp_motor_nested_scen.c`；A/B 走 `pwm.motor.20k`，
  C 走舵机 50 Hz 预设。

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

