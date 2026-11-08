# P-09 uart_passthrough — UART 环回/透传（J10, 115200 8N1, 与 USB CDC 并存）

> 类目：外设例程（periph）· Wave 3 ｜ 任务卡 T-032 ｜ 状态：**构建 + QEMU 断言已收口，真板未验证**（无板，联调项待 T-017）

## 一、覆盖点（任务书 §3 P-09）

| 验收 | 实现 | 输出锚点 |
|---|---|---|
| ① 环回/透传 115200 双向 10min 零丢包 | 每轮 1000B 测试模式环回（SIM=软件回环 / real=UART0 回环），逐轮 loss 统计 | `LOOP n=1 tx=1000 rx=1000 loss=0 crc=OK` |
| ② 与 USB CDC 并存 | 数据面（UART0/D6-D7, J10）与日志面（USB-Serial-JTAG）分离；QEMU 档文本走 UART0 出口 | `UART loop 115200 8N1` / `INFO text-plane=USB-CDC data-plane=UART0 coexist` |

> 真板 10min 为验收时长；QEMU 档跑 10 轮短窗演示，判据（零丢包）相同。

## 二、硬件引脚

| 信号 | 引脚 | 说明 |
|---|---|---|
| UART0_TX | D6 | J10 Grove UART（R27 0R） |
| UART0_RX | D7 | J10 Grove UART（R28 0R） |
| 串口参数 | 115200 8N1 | 数据口与日志口一致 |

真板回环接线：J10 TX←→RX 短接（或接对端器件做透传）。

## 三、双数据源宏（编译期，见 main/CMakeLists.txt）

- `P_SIM_UART=1`（默认）：软件回环（生成测试模式→ring buffer→读回比对，零外设依赖，QEMU 可测）
- `P_SIM_UART=0`：真 UART0 回环/透传（J10 短接或对端回显；数据走 UART0，日志走 USB CDC）
- `EXP_QEMU_UART=1`：文本输出层 UART0（QEMU chardev 出口）；默认 USB-Serial-JTAG（真板）

## 四、验证记录

| 验证 | 命令 | 结果 |
|---|---|---|
| default 档构建 | `run.bat` | ✅ bin 167,888 B（2026-11-07 实测） |
| QEMU 档构建 | `run_qemu.bat`（`-DP_SIM_UART=1 -DEXP_QEMU_UART=1`，build-qemu） | ✅ bin 188,720 B（2026-11-07 实测） |
| QEMU 文本断言 | 本项目内部 CI 流程（公开仓不含 QEMU runner 与断言脚本） | 本仓仅构建 SIM 变体；`run_qemu.bat` 只做编译验证 |

## 五、组件对接（P-09 → xiao_* 组件）

- **数据面/日志面分离**：UART0 透传数据、USB-Serial-JTAG 日志——为 P-09~P-13 的
  "设备口 + 调试口"共存模式提供模板（真板 J10 接外设时日志不占数据带宽）。
- **回环判据**：逐字节比对 + 累计 loss=0 → VERDICT PASS；真板超时/半包走 BAD 分支
  （现为计数上报，供后续 P-11 持久化扩展）。
## 依赖声明（Dependencies）

### 构建依赖（Build）

| 依赖 | 路径 | 缺失时脚本行为 |
|---|---|---|
| ESP-IDF v6.1 SDK（含 `export.bat`） | `%IDF_PATH%（默认 C:\esp\v6.1\esp-idf）` | `run.bat`/`run_qemu.bat` 启动检查，缺失打印 `[ERR] ... not found` 并退出 |
| IDF Python 虚拟环境 | `%IDF_PYTHON_ENV_PATH%（默认 C:\Espressif\python_env\idf6.1_py3.11_env）` | 同上 |
| esp32s3 工具链 | 由 `export.bat` 注入 | 激活 IDF 环境失败即退出 |

### 运行依赖（Run）

| 场景 | 依赖 | 缺失时行为 |
|---|---|---|
| 真板（`run.bat COMx`） | XIAO ESP32-S3 + USB-C 数据线（USB-Serial-JTAG），COM 口 | 无 COM 参数时仅构建不烧录（脚本提示） |
| QEMU（`run_qemu.bat`） | `%QEMU_XTENSA%（内部 CI 注入）` | bat 启动检查，缺失打印 `[ERR] QEMU xtensa binary not found` 并退出 |
| QEMU（内部 CI） | 见上：QEMU 引导与文本断言在私仓 CI 管道运行，公开副本不包含 |

### 硬件依赖（Hardware，真板档）

- J10（D6/D7）UART0 115200 8N1 环回/透传；与 USB-CDC（USB-Serial-JTAG 日志面）并存
