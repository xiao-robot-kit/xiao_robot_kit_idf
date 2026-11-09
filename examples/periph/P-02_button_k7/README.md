# P-02_button_k7 — D11 按键：消抖 + 短按/长按事件

任务书 P-02。链路：K7 (D11, GPIO11, 板上拉) → 软件消抖 ≥20ms → 短按(<800ms)/长按(≥800ms) 事件。

## 硬件约束

- **GPIO11**：内部上拉 45K，按下 = 低电平（SW 闭合到 GND）。
- **消抖**：软件 1ms 节拍轮询，≥20ms 稳定才翻转（`KEY_DEBOUNCE_MS`）。
- **按钮语义**：短按 = 按下到抬起 <800ms；长按 = ≥800ms 且**到点即触发**（无需等抬起）。
- 验收边界：795ms → 短按；805ms → 长按（合成时序含 795/805 两边界用例）。

## 双数据源

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `P_SIM_KEY` | 1（默认） | 合成按键序列：短按×3 + 长按×2 + 4~8ms 抖动注入，5.6s 跑完；无板可测 |
| `P_SIM_KEY` | 0 | 真 GPIO11（内部上拉，按下=低），真板手按 |

合成时序（`s_seq[]`）：E1 800-1120 短按(hold 320)；E2 2200 起长按(到点 3000 触发 LONG)；E3 3500-3800 短按；E4 4300-5105 长按边界(hold 805)；E5 5600-6395 短按边界(hold 795)。事件间隙插 4~8ms 抖动(< 消抖半窗口,吸收不计数)。

## 输出格式

```
OK boot P-02_button_k7 v0.1.0 | board=XIAO Robot Board V1.0 | mcu=XIAO ESP32-S3 Plus (BYPASS P_SIM_KEY)
KEY SHORT n=1 hold=320 ms
KEY LONG n=2 hold=900 ms
KEY SHORT n=3 hold=300 ms
KEY LONG n=4 hold=805 ms
KEY SHORT n=5 hold=795 ms
KEY done n=5 deb=0 jitter=0
```

`deb/jitter` 为诊断计数（≥10ms 半窗口反跳才计）——合成序号全部抖动 <10ms 吸收 ⇒ 恒 0，证明无抖动误报。

## 构建 / 运行

- 构建：`run.bat`（无参 = build；带 COMx = flash+monitor）
  - 期望：`OK boot P-02_button_k7` / `KEY SHORT` / `KEY LONG` / `KEY done`

## 验证状态

- ⚠️ 真板按键实测（真 GPIO11 消抖/长按手感）**未验证**（无真板）

## 已知限制

- 消抖用软件轮询而非 GPIO 中断（例程教学语义；真板手按 1ms 轮询足够）。
## 依赖声明（Dependencies）

### 构建依赖（Build）

| 依赖 | 路径 | 缺失时脚本行为 |
|---|---|---|
| IDF Python 虚拟环境 | `%IDF_PYTHON_ENV_PATH%（默认 C:\Espressif\python_env\idf6.1_py3.11_env）` | 同上 |
| esp32s3 工具链 | 由 `export.bat` 注入 | 激活 IDF 环境失败即退出 |

### 运行依赖（Run）

| 场景 | 依赖 | 缺失时行为 |
|---|---|---|
| 真板（`run.bat COMx`） | XIAO ESP32-S3 + USB-C 数据线（USB-Serial-JTAG），COM 口 | 无 COM 参数时仅构建不烧录（脚本提示） |

### 硬件依赖（Hardware，真板档）

- K7（D11 / GPIO11）：内部上拉 45K，按下 = 低电平（SW 闭合到 GND）
