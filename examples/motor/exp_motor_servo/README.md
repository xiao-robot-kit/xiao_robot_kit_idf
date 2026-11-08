# exp_motor_servo — 单舵机：五点角度标定

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

单台 SG90 舵机做五点角度标定：0 / 45 / 90 / 135 / 180° 升后降、每点停 2 s
（双窗=上升窗+下降窗，可做迟滞/重复性对照），500 Hz 控制环逐拍出 `0x05` 帧
（`duty` = 脉宽换算后占空的存储回读）。

## 二、帧与参数

- `0x05` MOTOR：单实例 = 1×16 B 记录；布局单源 `src/proto/frame.h`。
- 参数单源：实验表 `main/exp_motor_servo_scen.c`（角度点 / 保持拍数 / wiring）；
  舵机定时器 = 50 Hz 预设（与电机的 20 kHz 定时器相互独立）。

## 三、变体宏

| 宏 | 1（唯一支持值） | 0 |
|---|---|---|
| `EXP_SIM` | 合成舵机 HAL | 构建期 `#error` 显式拒绝 |
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

