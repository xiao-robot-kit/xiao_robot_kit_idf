# exp_motor_qemu — 电机四类演示 + 五场景选择器

> 类目：深度实验（examples/motor）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

四类电机（PWM_OPEN / SERVO / ENC_CLOSED / MOTOR_DC）在同一固件里的演示与实验：
上电即跑 demo（四槽 S 梯度遥测，`0x05` 帧），运行期在 UART0 收 `SCEN <n>` 切换
五场景运行时（`0x06` 帧）——场景 = PWM 占空阶梯 / 舵机五点标定 / 编码器闭环阶跃 /
嵌套定时器 / DC 正反转。

五个单例工程（`../exp_motor_pwm` 等）即本工程各场景的固化版，用于单独观察。

## 二、帧与参数

| 帧 | 内容 |
|---|---|
| `0x05` MOTOR | demo：4 槽 ×16 B（槽 3 = DC 带符号 duty 域） |
| `0x06` SCEN | 场景：按场景槽数 ×16 B |
| `0x7F` TEXT | 场景切换回执与状态行 |

- 演示表单源：`main/motor_demo_table.c` / `main/motor_scen_table.c`；运行时 =
  `main/motor_demo.c` / `main/motor_scen.c`。

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

