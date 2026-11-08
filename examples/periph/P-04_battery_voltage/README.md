# P-04_battery_voltage — 电池电量监控：分压采样 + SOC/容量计算 + 低电告警

任务书 P-04。链路：电池 → 板上 100K:30K 分压 → ESP32-S3 ADC2_CH5 (GPIO16) → battery 组件（SOC/容量/告警）→ 串口。

## 硬件约束

- **分压**：100K:30K（ratio = 4.3333），量程覆盖 4.2V 满电（满电压 ~0.97V 峰值 < 3.3V 参考安全）。
- **ADC**：ADC2_CH5（GPIO16），12bit，DB_11 衰减（0~3.3V）。注意 v6.1 组件已重命名为 `esp_adc`（非 `esp_driver_adc`）。
- **参数 preset**：`xiao_1s_1400mah`（v_full=4.2 / v_cutoff=3.0 / cut=... / 1400mAh），battery 组件单测 35/35 通过。

## 双数据源

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `P_ADC_RAW` | 1（默认） | 合成 ADC：4.2→3.0V 线性放电 20 步（每步 0.06V），alarm(≤3.3V) 在第 ~15s 触发，20s QEMU 断言窗口可见；走完回满重来 |
| `P_ADC_RAW` | 0 | 真 ADC2_CH5（GPIO16）single-shot，DB_11 |

## 输出格式

```
OK boot p04_battery_voltage | divider=100K:30K v_full=4.20 v_cutoff=3.00 cap=1400 mAh
SRC synthetic-adc (no battery)
BATT v=4.14 soc=97% mah=1360 alarm=0
BATT v=3.29 soc=5% mah=70 alarm=1
...
```

`alarm=1` 在 v≤3.30V 触发（低电告警）。

## 构建 / 运行

- 构建：`run.bat`（无参 = build；带 COMx = flash+monitor）
- QEMU 文本断言：`run_qemu.bat`（合成档 + UART0，端口 5554）
  - 期望：`OK boot p04_battery_voltage` / `SRC synthetic-adc` / `BATT v=` / `alarm=1`

## 验证状态

- ✅ 构建（SIM 档 + QEMU 档）+ QEMU 文本断言 4/4 命中（`run_qemu.bat`）
- ⚠️ 真板分压实测（ADC 校准曲线/低电告警实时性）**未验证**（无真板）

## 已知限制

- QEMU 档走合成 ADC（QEMU 无 ADC 外设链路）；真 ADC 档需真板。
- SIM 放电 20 步为演示语义（覆盖满电→截止全区间），非电池真实放电曲线。
- 合成与真 ADC 的 SRC 行不同（SIM 强调无电池，防误判）。
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

- 电池 → 板载 100K:30K 分压 → ADC2_CH5（GPIO16）；量程覆盖 4.2V 满电
