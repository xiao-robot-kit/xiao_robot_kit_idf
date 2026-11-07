# P-12_spare_gpio_tpxx — 备用 GPIO：TP20/TP21（MTDI/MTDO）飞线双向验证

## 目标

任务书 P-12：板载 20 个 GPIO 全占用（任务书 §4.1），唯一备用 GPIO 是
U3 pad24/25（MTDI/MTDO），仅以测试点 TP20/TP21 引出，需飞线使用。
本例程验证 MTDI/MTDO 可作为普通输入/输出 GPiIO：

- **角色 1**：MTDI=输出、MTDO=输入（写 4 拍电平 → 读回比对）
- **角色 2**：MTDI=输入、MTDO=输出（角色互换再测一轮）
- 输出 `SPARE done mismatch=(m1,m2)` → 全 0 即 `VERDICT PASS`

## 覆盖点（任务书 §1.1 / §4.1）

- U3 pad24/25 = MTDI/MTDO，仅 TP20/TP21 测试点（唯一备用 GPIO，飞线用）
- 引脚映射：**TP20 = MTDI = GPIO41**、**TP21 = MTDO = GPIO42**
  （ESP32-S3 strapping 引脚；Arduino variant TX1=42 / RX1=41 佐证）
- 真板路径保留（`P_SIM_SPARE=0` 含真实 `gpio_config` / `gpio_set_level` /
  `gpio_get_level` 代码）；QEMU 档走合成回环（`P_SIM_SPARE=1`）

## 硬件约束（⚠ strapping 属性，任务书要求提示）

- GPIO42（MTDO/TP21）是 ESP32-S3 **strapping 引脚**（VDD_SPI 电压选择，
  boot 早期采样）。真板飞线时必须保证**上电瞬间两脚无强下拉到 GND**，
  否则可能改变启动行为（固件启动行打印该警告；QEMU 无此语义，仅透传）。
- GPIO41（MTDI/TP20）同样为 strapping 引脚（JTAG 模式选择侧）。
- 真板验证需要：一片飞线把 TP20 ↔ TP21 互联（loopback），或分别接
  外部电平源/负载；任务书 §4.1 说明扩展只能走 I2C(J11/J12) 或 UART(J10)。

## 双数据源（编译期宏）

| 宏 | 1（QEMU 档，默认） | 0（真板档） |
|---|---|---|
| `P_SIM_SPARE` | 合成回环：内存电平表模拟飞线互联（写任一脚→双槽同步→读回同电平） | 真实 GPIO41/42：`gpio_config` + `gpio_set_level`/`gpio_get_level` |
| `EXP_QEMU_UART` | 遥测出口 UART0（QEMU `-serial tcp:...,server` 槽） | 遥测出口 USB-Serial-JTAG（真板 USB-C） |

构建：`idf.py build`（默认档）或
`idf.py -C examples\periph\P-12_spare_gpio_tpxx -B ...\build-qemu -DP_SIM_SPARE=1 -DEXP_QEMU_UART=1 build`（QEMU 档）。

## 输出格式（SIM/QEMU 档）

```
OK boot P-12_spare_gpio_tpxx v0.1.0 | board=XIAO Robot Board V1.0 | mcu=XIAO ESP32-S3 Plus | (SIM LOOPBACK)
SPARE strapping-warn GPIO42=MTDO=TP21 is VDD_SPI strap pin: ...
SPARE pins MTDI=GPIO41(TP20) MTDO=GPIO42(TP21)
SPARE role1 MTDI=OUT MTDO=IN
SPARE r1 out=0 -> in=0 OK
SPARE r1 out=1 -> in=1 OK
SPARE role2 MTDI=IN MTDO=OUT
SPARE r2 out=0 -> in=0 OK
...
SPARE done mismatch=(0,0) -> VERDICT PASS
```

完成后每 1s 全量重放（REPLAY_MAX 16 行，含 done/verdict，晚连探针可见）。

## 验证（QEMU 2026-11-08）

`run_qemu.bat` 或：
```
bash testing/qemu/run_qemu_tcp_hello.sh examples/periph/P-12_spare_gpio_tpxx/build-qemu \
  examples/periph/P-00_hello_bringup/testing/hello_check.py 5562 20 \
  'OK boot P-12_spare_gpio_tpxx' 'SPARE role1' 'SPARE role2' 'SPARE done' 'VERDICT PASS'
```

结果：**5/5 全部命中，check exit=0（ALL OK）**；构建 default/QEMU 两档 EXIT=0。
真板路径（`P_SIM_SPARE=0`）待真板 + TP20↔TP21 飞线实测（无真板环境）。

## 依赖声明（Dependencies）

### 构建依赖（Build）

| 依赖 | 路径 | 缺失时脚本行为 |
|---|---|---|
| ESP-IDF v6.1 SDK（含 `export.bat`） | `%IDF_PATH%（默认 C:\esp\v6.1\esp-idf）` | `run.bat`/`run_qemu.bat` 启动检查，缺失打印 `[ERR] ... not found` 并退出 |
| IDF Python 虚拟环境 | `%IDF_PYTHON_ENV_PATH%（默认 C:\Espressif\python_env\idf6.1_py3.11_env）` | 同上 |
| esp32s3 工具链 | 由 export.bat 注入 | 激活 IDF 环境失败即退出 |

### 运行依赖（Run）

| 场景 | 依赖 | 缺失时行为 |
|---|---|---|
| 真板（`run.bat COMx`） | XIAO ESP32-S3 + USB-C 数据线（USB-Serial-JTAG），COM 口 | 无 COM 参数时仅构建不烧录（脚本提示） |
| QEMU（`run_qemu.bat`） | `%QEMU_XTENSA%（内部 CI 注入）` | bash 脚本检查，缺失打印 `QEMU 不存在` 并退出 |
| QEMU（内部 CI） | 见上：QEMU 引导与文本断言在私仓 CI 管道运行，公开副本不包含 |

### 硬件依赖（Hardware，真板档 `P_SIM_SPARE=0`）

- TP20（MTDI/GPIO41）、TP21（MTDO/GPIO42）测试点，飞线互联（loopback）
- ⚠ strapping：上电瞬间两脚无强下拉（见上「硬件约束」）