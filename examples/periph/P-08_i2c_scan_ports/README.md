# P-08 i2c_scan_ports — I2C 总线扫描 + 地址表管理

> 类目：外设例程（periph）· Wave 3 ｜ 任务卡 T-031 ｜ 状态：**构建 + QEMU 断言已收口，真板未验证**（无板，联调项待 T-017）

## 一、覆盖点（任务书 §3 P-08）

| 验收 | 实现 | 输出锚点 |
|---|---|---|
| ① J11/J12 挂载器件地址枚举 | 0x03–0x77 全窗探测（真档 `esp_port_i2c_probe`；SIM 档回显 0x6A + 0x40） | `SCAN dev=0x6A kind=KNOWN` / `SCAN dev=0x40 kind=NEW` |
| ② 与板载 IMU 同总线不冲突（地址表管理） | 已登记表 {0x6A:IMU}；表外地址标 `NEW (needs reg)` 防重复分配 | `TBL round=N total=... known=0x6A:IMU new=...` |
| ③ N 轮连续扫描零异常 | 5 轮 ×1s，逐轮 total/new 汇总 | `SCAN DONE run=5 scan=PASS` + `VERDICT PASS` |

## 二、硬件引脚与总线拓扑

| 信号 | 引脚 | 说明 |
|---|---|---|
| I2C1 SDA | D4 (GPIO5) | 唯一事实源：`port/esp_idf/imu_hal_i2c.c` |
| I2C1 SCL | D5 (GPIO6) | 同左 |
| 总线挂载 | J11 + J12（Grove I2C ×2）+ **板载 IMU U18 (0x6A)** | 三者共线（任务书 D4）；400kHz |

地址表（board 固定 + 登记区）：
- `0x6A` — IMU LSM6DS3TR-C（已登记 KNOWN，新挂器件切勿复用）
- 新挂器件 → 出现在 `kind=NEW`，由用户登记入表后升 KNOWN

## 三、双数据源宏（编译期，见 main/CMakeLists.txt）

- `P_SIM_SCAN=1`（默认）：合成扫描（零 I2C 依赖，QEMU 可测；回显 0x6A + 演示 0x40 NEW）
- `P_SIM_SCAN=0`：真 I2C 扫描——复用 port 层 `esp_port_i2c_probe` 端点（i2c_master_probe，
  不建器件句柄，任意地址探测）；QEMU 档必须走 SIM（无真实 I2C 器件）
- `EXP_QEMU_UART=1`：输出层 UART0（QEMU chardev 出口）；默认 USB-Serial-JTAG（真板）

## 四、验证记录

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 168,048 B（2026-11-07 实测） |
| QEMU 档构建 | `run_qemu.bat`（`-DP_SIM_SCAN=1 -DEXP_QEMU_UART=1`，build-qemu） | ✅ bin 188,880 B（2026-11-07 实测） |
| QEMU 文本断言 | 本项目内部 CI 流程（公开仓不含 QEMU runner 与断言脚本） | 本仓仅构建 SIM 变体；`run_qemu.bat` 只做编译验证 |

## 五、组件对接（P-08 → Base_Library）

- **扫描端点**：`port/esp_idf` 新增 `esp_port_i2c_probe(addr, timeout_ms)`（P-08 引入，
  真档扫任意地址；单器件 read/write 的 addr=0x6A 契约不变）。
- **地址表管理**：先例程为 hardcode（s_addr_book）；P-11 pstore_params 可升级为
  NVS 持久化登记表（T-033 承接）。
- **与 P-01 关系**：P-01 是单器件轮询（WHO/读/单地址），P-08 是全窗扫描 + 地址表——
  P-08 可视为 P-01 的扩展视图；两者共用同一 I2C 平台端点（只读）。
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

- J11/J12 I2C 总线挂载器件（板载 IMU 0x6A 为已知表项；探测 0x03–0x77 全窗）
