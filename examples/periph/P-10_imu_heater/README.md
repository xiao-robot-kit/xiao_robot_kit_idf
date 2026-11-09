# P-10 imu_heater_thermo — IMU 恒温加热闭环（D12 PWM, 目标 ±2°C + 超温保护 + 温补对接）

> 类目：外设例程（periph）· Wave 3 ｜ 任务卡 T-027 ｜ 状态：**构建已收口，真板未验证**（无板 + 加热 PWM HAL 未提供，联调项待 T-017）

## 一、覆盖点（任务书 §3 P-10）

| 验收 | 实现 | 输出锚点 |
|---|---|---|
| ① 加热到目标温度 ±2°C 恒温 | imu_heater 状态机（thermo45：target 45 / hyst 2 / overheat 60 / hold 12%） | `HT cfg preset=thermo45` ／ `HT r=10 st=HOLD d=0.12` |
| ② 超温保护（硬上限 + 软看门狗） | 组件 OVERHEAT 态强制 0 开度（硬）；例程连续超限 ≥3 拍 → WDT 强制闭锁（软） | `st=OVERHEAT` ／ `WDT fail seq=3 forced duty=0` |
| ③ 对接 calibration 温补算法 | temp_comp 初始化 + 每拍 apply（阶梯温度 → 补偿零偏） | `TC init t_ref=25.0` ／ `HT ... tc=0 bias=(...)` |

## 二、硬件引脚与物理模型

| 信号 | 引脚 | 说明 |
|---|---|---|
| 加热 PWM（加热电阻，围绕板载 IMU） | D12（GPIO） | R76 100R → Q1 NMOS，板级温控标定用途 |
| 温度源 | IMU 芯片温度 | LSM6DS3 内部温度计（I2C 读，见 P-01）；板面无独立热敏电阻 |

> **HAL 缺口（显式声明）**：真加热 PWM 输出端点（LEDC 单通道）port 层未提供——
> `P_SIM_HEAT=0` 构建期 `#error` 拒绝（同 P-05 P_SIM_MOTOR=0 纪律）。
> SIM 档演示**控制层 + 保护链路**全流程（温度合成 + 开度计算/记录），物理 PWM 驱动待 T-017。

## 三、双数据源宏（编译期，见 main/CMakeLists.txt）

- `P_SIM_HEAT=1`（默认）：合成温度场景（室温 30°C → 目标 45°C → 恒温窗晃动 → 61°C 过热 → 63°C 触发 WDT → 恢复再恒温），实测状态机闭环
- `P_SIM_HEAT=0`：真硬件档——`#error`（加热 PWM HAL + IMU 温度直读未提供）

## 四、验证记录

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 170,336 B（2026-11-08 实测） |

## 五、组件对接（P-10 → xiao_* 组件）

- **控制层**：`xiao_imu_heater`（纯算法状态机，预设 thermo45）；P-10 提供温度输入 + 开度消费 + 保护增强（软看门狗为例程侧策略）
- **温补**：`xiao_calibration` 的 `temp_comp`（应用段）；恒温窗内每拍 `apply` 输出补偿零偏，验证「温度稳定 → 温漂补偿收敛」（温标拟合段由 exp2 承接）
- **真板闭环前提**：LEDC 加热 PWM 端点（D12 单通道）+ 温度直读 —— 均列入 T-017 联调清单
## 依赖声明（Dependencies）

### 构建依赖（Build）

| 依赖 | 路径 | 缺失时脚本行为 |
|---|---|---|
| IDF Python 虚拟环境 | `D:\Espressif\python_env\idf6.1_py3.11_env` | 同上 |
| esp32s3 工具链 | 由 `export.bat` 注入 | 激活 IDF 环境失败即退出 |

### 运行依赖（Run）

| 场景 | 依赖 | 缺失时行为 |
|---|---|---|
| 真板（`run.bat COMx`） | XIAO ESP32-S3 + USB-C 数据线（USB-Serial-JTAG），COM 口 | 无 COM 参数时仅构建不烧录（脚本提示） |

### 硬件依赖（Hardware，真板档）

- D12 加热 PWM + IMU 温度通道（`port/esp_idf` 温度源为唯一事实源）；真硬件加热 PWM HAL（未提供时 `P_SIM_HEAT=0` 构建期 `#error` 显式拒绝）
