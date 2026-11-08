# P-07 servo_pwm — 4 路舵机 PWM 例程（0°/90°/180° 定位 + 电压联动告警）

> 类目：外设例程（periph）· Wave 2 ｜ 任务卡 T-030 ｜ 状态：**构建 + QEMU 断言已收口，真板未验证**（无板，联调项待 T-017）

## 一、覆盖点

| 验收项 | 实现 |
|---|---|
| 4 路独立 50Hz LEDC | 单定时器 `pwm.servo.50`（50Hz/4 通道）× 4 舵机（SERVO 单臂占 1 通道）；引脚 D15/D9/D0/D8 = S1~S4 |
| 0°/90°/180° 定位 | 四槽错相扫角表 `{0 → 90 → 180 → 90 → 0}`（相邻槽起始相位错 0.5s，证明 4 路独立驱动）；每 1s 事件行回读各通道 duty → 换算脉宽 us（50Hz 周期 20ms：us = duty×20000） |
| Motor_5V 共轨电流预算（任务书 §5.3） | 注释头写明：Motor_5V（Q7）同时供 4 舵机 + 2 电机驱动，USB 供电轨功率受限；空载峰流预算约 4×250mA=1A，堵转单舵 0.9A 立超 |
| 电压监测联动告警（任务书 §4.4 硬性） | 每拍采样 D16 电池电压；`v_bat < 6.20V` → 输出 `WARN vbat=xx low (cutoff 6.20V) servo hold` + 各舵机钳 0°（停摆保位，防拉垮 Motor_5V 轨），恢复后打 `OK vbat=xx recovered` |
| 对接 motor 组件 API | `motor.servo.sg90` 预设（500~2500us/0~180°）+ `pwm.servo.50` 定时器 + `motor_create/set_target/poll`；输出经 `sim_pwm_channel_duty` 回读（物理输出面） |
| 输出契约 | 有限序列 + 事件回放：boot + 正常档帧 + WARN/OK 告警事件 + DONE，完成态每 1s 重放（P-02 模式） |

## 二、端口映射与硬件

- **D15 = S1、D9 = S2、D0 = S3、D8 = S4**（4 路舵机口）；电源 Motor_5V（与 M1/M2 电机驱动共轨）。
- 载波：50Hz（`pwm.servo.50` 预设）；SG90 脉宽域 500~2500us ↔ 0~180°，中点 90° = 1500us。
- 告警采样：D16 = 电池电压采样引脚 → 板上分压 100K:30K（Vbat = Vadc×130/30，同 P-04 底座）。

## 三、三数据源宏

| 宏 | 1 | 0 |
|---|---|---|
| `P_SIM_MOTOR` | 合成 PWM HAL（QEMU 可测，**默认**） | 真硬件 LEDC HAL（**未提供，构建期 #error 显式拒绝**——不静默编「假硬件」固件） |
| `EXP_QEMU_UART` | 遥测出口 = UART0（QEMU 档） | 遥测出口 = USB-Serial-JTAG（真板档） |
| `P_ADC_RAW` | 合成电压演示曲线（无电池跑通告警链路，QEMU 可断言 WARN 行，**默认**） | 真 ADC2_CH5 采样 + 一阶滤波（真板档，需 esp_adc） |

## 四、验证

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 195,184 B（2026-11-07 实测） |
| QEMU 档构建 | `run_qemu.bat`（`-DP_SIM_MOTOR=1 -DEXP_QEMU_UART=1 -DP_ADC_RAW=1`，build-qemu） | ✅ bin 195,184 B（2026-11-07 实测） |
| QEMU 文本断言 | 本项目内部 CI 流程（公开仓不含 QEMU runner 与断言脚本） | 本仓仅构建 SIM 变体；`run_qemu.bat` 只做编译验证 |

> 真板验证项（待 T-017）：4 路舵机上电角度定位精度（万用表或示波器测脉宽）、共轨堵转保护生效（WARN 停摆不可自恢复后再摆）、与 P-04 电压读数交叉核对。

## 五、组件对接说明（对后续 P-11 的契约）

- P-11 pstore_params：舵机中位/角度校准参数写入 pstore 后重启读回一致——本倒程的 `motor.servo.sg90` 底版（pulse_min/max、angle_min/max）即校准面，P-11 应把校准值从 pstore 读出覆写 cfg 再 create。
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

- 4 路舵机 S1~S4 = D15/D9/D0/D8（50Hz LEDC，Motor_5V 共轨）；告警采样 D16（电池分压 100K:30K，Vbat=Vadc×130/30，同 P-04）
