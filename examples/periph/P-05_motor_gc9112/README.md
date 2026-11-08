# P-05 motor_gc9112 — 双路 GC9112 电机驱动例程

> 类目：外设例程（periph）· Wave 2 ｜ 任务卡 T-028 ｜ 状态：**构建 + QEMU 断言已收口，真板未验证**（无板，联调项待 T-017）

## 一、覆盖点

| 验收项 | 实现 |
|---|---|
| M1/M2 双路正转/反转/刹车/调速 | MOTOR_DC 桥 target=带符号占空（`+50 → −50 → 0 → ±25/50/75/100` 扫档）；M2 方向反接（sign=−1）演示反向槽 |
| 占空比-转速曲线记录 | 调速扫档后打印 `CURVE Mx duty=.. rpm=.. (model)` 3 点曲线；rpm 为合成模型（duty×3600），真板由 P-06 编码器实测替换 |
| 对接 motor 组件 API | `motor.dc.generic` 预设 + `pwm.motor.20k` 定时器 + `motor_create/set_target/poll/get_state`；桥输出经 `sim_pwm_bridge_duty` 回读（物理输出面，直通零容忍） |
| 输出契约 | 有限序列 + 事件回放：boot + 20 档事件 + CURVE×6 + DONE，完成态每 1s 重放（P-02 模式） |

## 二、端口映射与硬件

- **D1/D2 = M1 GC9112 IN1/IN2**（U14）；**D19/D18 = M2 GC9112 IN1/IN2**（U16）；单 H 桥 × 2，Motor_5V 供电（与 4 路舵机共轨，堵转电流预算见任务书 §5.3）。
- 载波：20kHz（`pwm.motor.20k` 预设）；桥类占 channel/channel+1 成对（M1=ch0/1、M2=ch2/3）。

## 三、双数据源宏

| 宏 | 1 | 0 |
|---|---|---|
| `P_SIM_MOTOR` | 合成 PWM/编码器 HAL（QEMU 可测，**默认**） | 真硬件 LEDC HAL（**未提供，构建期 #error 显式拒绝**——不静默编「假硬件」固件） |
| `EXP_QEMU_UART` | 遥测出口 = UART0（QEMU 档） | 遥测出口 = USB-Serial-JTAG（真板档） |

## 四、验证

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 190,224 B（2026-11-07 实测） |
| QEMU 档构建 | `run_qemu.bat`（`-DP_SIM_MOTOR=1 -DEXP_QEMU_UART=1`，build-qemu） | ✅ bin 190,224 B（2026-11-07 实测） |
| QEMU 文本断言 | 本项目内部 CI 流程（公开仓不含 QEMU runner 与断言脚本） | 本仓仅构建 SIM 变体；`run_qemu.bat` 只做编译验证 |

> 真板验证项（待 T-017）：M1/M2 上电正反转方向与占空线性、刹车电流、堵转温升 ≤ 60s、曲线与 P-06 编码器读数对照。

## 五、组件对接说明（对后续 P-06 的契约）

- P-06 电转闭环：复用本例程装配序（20kHz 定时器 + 桥对），`motor.enc.n20` 预设（cpr=7200）替换 `motor.dc.generic`，挂 `sim_encoder_hal` 即闭环。真板编码器引脚 D3/D14（M1）、D17/D10（M2）见任务书 §4。
## 依赖声明（Dependencies）

### 构建依赖（Build）

| 依赖 | 路径 | 缺失时脚本行为 |
|---|---|---|
| ESP-IDF v6.1 SDK（含 `export.bat`） | `D:\esp\v6.1\esp-idf` | `run.bat`/`run_qemu.bat` 启动检查，缺失打印 `[ERR] ... not found` 并退出 |
| IDF Python 虚拟环境 | `D:\Espressif\python_env\idf6.1_py3.11_env` | 同上 |
| esp32s3 工具链 | 由 `export.bat` 注入 | 激活 IDF 环境失败即退出 |

### 运行依赖（Run）

| 场景 | 依赖 | 缺失时行为 |
|---|---|---|
| 真板（`run.bat COMx`） | XIAO ESP32-S3 + USB-C 数据线（USB-Serial-JTAG），COM 口 | 无 COM 参数时仅构建不烧录（脚本提示） |
| QEMU | bash（Git for Windows / MSYS） | `run_qemu.bat` 检查 `where bash`，缺失提示安装并退出 |

### 硬件依赖（Hardware，真板档）

- GC9112 电机驱动（M1/M2 桥对）+ DC 电机；真硬件 LEDC PWM HAL（未提供时 `P_SIM_MOTOR=0` 构建期 `#error` 显式拒绝）
