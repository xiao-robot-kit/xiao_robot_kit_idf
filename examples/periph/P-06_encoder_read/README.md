# P-06 encoder_read — 编码器读取例程（SPIN 手转 + DRIVE 闭环）

> 类目：外设例程（periph）· Wave 2 ｜ 任务卡 T-029 ｜ 状态：**构建已完成，QEMU 断言回填中**（真板未验证，联调项待 T-017）

## 一、覆盖点

| 验收项 | 实现 |
|---|---|
| 编码器正交信号读取 | `motor.enc.n20` 预设（cpr=7200）+ `sim_encoder_hal` 合成：Phase A 手转 SPIN fwd→stop→rev→stop 各 1.5s，每 0.2s 快照 `ENC COUNT count=%+d`，峰值打 `ENC PIK count=%+d`（rpm 模型换算可见） |
| 闭环控制（DRIVE） | Phase B 目标角度 ±120°：`ENC DRIVE target=+120` → PID 闭环（motor_poll 内部读 encoder 每 10ms 一步，残差 → 转速指令）→ 到位判定 0.2s 快照回读 actual+enc_count |
| 对接 motor 组件 API | `motor.enc.n20` 预设 + `pwm.motor.20k` 定时器 + `motor_create/attach_encoder/register_plant/set_target/poll/get_state`；plant 合成（dt_s=0.01s，编码器相位增量模拟） |
| 输出契约 | 有限序列 + 事件回放（P-02 模式）：boot + 6 相位行 + PIK + DONE，完成态每 1s 全量重放（含 boot——TCP 探针后连接也拿得到） |

## 二、端口映射与硬件

- **D3 = ENC1_A、D14 = ENC1_B（M1 编码器）**；**D17 = ENC2_A、D10 = ENC2_B（M2 编码器）**（任务书 §4）。
- 例程只接 M1 编码器（cpr=7200，折合单圈 7200 脉冲）。
- 手转 = 无电机负担坡手转编码器轴（SPIN 段不驱动电机，仅读取）；DRIVE 段驱动 M1 带编码器闭环。

## 三、双数据源宏

| 宏 | 1 | 0 |
|---|---|---|
| `P_SIM_MOTOR` | 合成 PWM/编码器 HAL（QEMU 可测，**默认**） | 真硬件 LEDC + 真编码器 GPIO 边沿中断 HAL（**未提供，构建期 #error 显式拒绝**） |
| `EXP_QEMU_UART` | 遥测出口 = UART0（QEMU 档） | 遥测出口 = USB-Serial-JTAG（真板档） |

## 四、验证

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 196,272 B（2026-11-07 实测） |
| QEMU 档构建 | `run_qemu.bat`（`-DP_SIM_MOTOR=1 -DEXP_QEMU_UART=1`，build-qemu） | ✅ bin 196,272 B（2026-11-07 实测） |
| QEMU 文本断言 | 本项目内部 CI 流程（公开仓不含 QEMU runner 与断言脚本） | 本仓仅构建 SIM 变体；`run_qemu.bat` 只做编译验证 |

> 真板验证项（待 T-017）：M1 手转方向与计数极性、手转 1 圈 = 7200 计数、DRIVE ±120° 到位误差、与 P-05 曲线 RPM 对照。

## 五、组件对接说明（对后续 P-08~P-13 的契约）

- P-08 i2c_scan_ports：编码器计数外设是 6 路 GPIO 边沿中断（无 I2C），此例程已演示「合成 HAL 挂 motor 组件」；P-08 的 I2C 扫描走 imu 组件同款 `i2c_master` 总线索，编码器不参与。
- 闭环拓扑（P-05→P-06 已打通）：`preset → timer → motor(pwm 桥对) → attach_encoder(sim) → register_plant(plant 合并到 enc HAL) → set_target → poll`——P-07 舵机无编码器，闭环面留空，只有 PWM 输出面。
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

- N20 编码器电机：M1 编码器 D3/D14、M2 D17/D10（任务书 §4）；真硬件 LEDC + 编码器 GPIO 边沿中断 HAL（未提供时 `P_SIM_MOTOR=0` 构建期 `#error` 显式拒绝）
