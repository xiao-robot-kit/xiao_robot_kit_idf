# P-00 hello_bringup — 工程脚手架 + USB 串口日志

> XIAO Robot Board V1.0 · 外设例程开发任务书 P-00（Wave 1 基础链路首例）
> 状态：**已构建**（见下）· 上板实测：**未验证**（待真板）

## 目的

最小工程脚手架：验证编译链、烧录链、串口日志链是否打通——验收标准的
「一切例程共同底座」。

## 硬件

- **零 GPIO 占用**（任务书 §4.1：GPIO 无空余，本例程不碰任何外设）
- 输出 = USB-Serial-JTAG（板载 USB-C 直连，115200 8N1）

## 接线表

| 信号 | 位置 | 说明 |
| --- | --- | --- |
| USB | 模块 USB-C | 上电即出日志，无需接线 |
| （无） | — | 本例外接零器件 |

## 预期现象

上电 ≤1s 串口打印（验收 5.1 判据）：

```
OK boot P-00_hello_bringup v0.1.0 | board=XIAO Robot Board V1.0 | mcu=XIAO ESP32-S3 Plus
INFO reset_reason=POWERON
HELLO workflow: USB-CDC log link up
HEART 1 uptime=1 s
HEART 2 uptime=2 s
… 每秒一行，连跑 10min 无重启
```

`reset_reason` 取值：`POWERON`（上电）/ `SOFTWARE`/`PANIC`/`TASK_WDT`/`BROWNOUT`
（欠压——怀疑供电问题先看这行）/ `DEEPSLEEP` 等。

## 构建与运行

```bash
# 真板（USB 已插）
run.bat COMx        # 构建 + 烧录 + 串口监控（idf.py flash monitor）

# QEMU 无板验证（EXP_QEMU_UART=1 → UART0 chardev 出口）
run_qemu.bat        # 构建 EXP_QEMU_UART 档 → 起 QEMU 收 15s 日志 → 必杀
```

## 已知限制

- USB-Serial-JTAG 掉线重连（插拔）会丢开机头部日志；断线重连后继续打印，
  心跳行保证状态可见。
- Task WDT：默认 5s 看门狗已在 FreeRTOS 空闲任务喂——心跳循环本身不触发
 （10min 连跑无重启即验收）。
- QEMU 档 UART0 与 USB-Serial-JTAG 二选一由 `EXP_QEMU_UART` 编译宏决定；
  真板默认 0，QEMU 默认 1。

## 提交自证（本轮）

- 构建：`idf.py build` 通过（详见本日提交记录）
- QEMU 日志链路：`run_exp_sim.sh` 收 15s 心跳行
- 真板 10min 连跑：**未验证**（待 P-00 上板实测补齐，任务书 §5.2 纪律）
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

- 零 GPIO 占用（任务书 §4.1：GPIO 无空余，本例不碰任何外设）；输出 = USB-Serial-JTAG（板载 USB-C，115200 8N1）
