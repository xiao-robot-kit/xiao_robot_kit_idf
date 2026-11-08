# P-11 pstore_params — 参数持久化（断电重启读回一致）

## 目标
电机校准 / 舵机中位参数**写入断电重启后读回一致**
——pstore 组件（`xiao_pstore`）做参数序列化，flash 裸分区后端持久化。

## 覆盖点（任务书 §4）
- pstore 组件：多条目注册（`motor.cal` / `servo.center`）、整帧 CRC32、两阶段原子加载
- 断电重启一致性：QEMU 强杀模拟掉电，同镜像二次启动读回校验

## 硬件约束
- 零外设依赖（纯存储演示）；介质 = flash 自定义 `store` 分区（partitions.csv，
  同 store_demo 布局）；端口 5561（QEMU chardev 出口）
- 控制台：真板 USB-Serial-JTAG 115200；QEMU 档 `EXP_QEMU_UART=1` 切 UART0

## 运行
- 真板（默认档，USB 输出）：`run.bat` → `idf.py -C examples\periph\P-11_pstore_params -p COMx flash monitor`
- QEMU 断电实验（两段式断言）：先 `run_qemu.bat` 建 QEMU 档，再：
  `python examples\periph\P-11_pstore_params\testing\qemu_powercycle_p11.py --build <build-qemu> [--port 5561]`

## 验证（QEMU 2026-11-08）
| 段 | 断言 | 结果 |
|---|------|------|
| 段一 fresh | boot 行含 `P-11_pstore_params`；含 `P11 SET rc=0`（写入非默认校准值 + save） | ✅ 1/1（实际回放见 `testing/logs/p11_phase1.txt`） |
| 段二 power-cycle | 含 `P11 GET` + `P11 MATCH=1`（读回 = 写入值）；无 `MATCH=0` | ✅ 2/2（`testing/logs/p11_phase2.txt`） |

> 首次断言运行 0/2（段一收 0B）：TCP `server,nowait` 时序——固件启动即刻输出完毕，客户端未连上期间字节被丢弃 → 改 `server`（QEMU 阻塞等客户端连接后再 boot，输出不丢）。
> 段二首跑重复 fresh：`pstore_save` 返回 0 但介质空白——pstore 脏检查「不脏零后端调用」：`create`(calloc) dirty=0、空介质 `load` 失败不置 dirty、`register` 不置 dirty → 应用层写参数必须显式 `pstore_mark_dirty` 再 save（ERR-009，见 .ai/errors/raw/ERR-009.md）。

> 状态：真板断电重启待实测（无真板环境）。

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

- 零外设依赖（纯存储演示）；介质 = flash 自定义 `store` 分区（`partitions.csv`，同 store_demo 布局）
