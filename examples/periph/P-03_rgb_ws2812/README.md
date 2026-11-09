# P-03_rgb_ws2812 — D13→J7 外接 WS2812 灯带：RMT 驱动 + 自研色轮

任务书 P-03。链路：GPIO13 → R79 22R → U4 (SN74LVC1G17 5V 缓冲) → R52 22R → J7 外接 WS2812 灯带。

## 硬件约束

- **必须走 J7**：J7 数据已 5V 电平缓冲（U4），勿绕过缓冲直连灯带（3.3V 逻辑对 WS2812 不足）。
- **WS2812**：800kHz 数据率，GRB (MSB first)，RESET 低 ≥50us；灯带默认 24 灯（`P03_LED_N`）。
- **RMT 时序**（resolution=10MHz, 0.1us/tick）：0 码 T0H=0.35us/T0L=0.8us；1 码 T1H=0.7us/T1L=0.6us。

## 双数据源

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `P_SIM_LED` | 1（默认） | 合成灯带：相位→色轮渐变 + 单灯定点，内存缓冲零硬件；无板可测 |
| `P_SIM_LED` | 0 | 真 RMT 发 GPIO13（真板 + J7 灯带；`esp_driver_rmt` 自带零外部依赖） |

色轮：0°红 → 120°绿 → 240°蓝 → 360°回红（`hue_to_rgb` 线性插值，三分点命名常量）。全带相位渐变 + 灯 0/5 反相位定点（视觉上固定不动，验证单灯定点控制）。

## 输出格式（SIM 档）

```
OK boot P-03_rgb_ws2812 v0.1.0 | board=XIAO Robot Board V1.0 | mcu=XIAO ESP32-S3 Plus (BYPASS P_SIM_LED)
LED phase=000 led0=r255g  0b  0 led5=r255g  0b  0
HUE 0 r255 g0 b0
LED phase=006 led0=r255g  5b  0 led5=r255g  5b  0
...
HUE 120 r0 g255 b0
HUE 240 r0 g0 b255
```

相位每帧 +6°(20ms),1.2s 满轮;0/120/240 三分点每轮各打一次 HUE 断言行。

## 构建 / 运行

- 构建：`run.bat`（无参 = build；带 COMx = flash+monitor）
  - 期望：`OK boot P-03_rgb_ws2812` / `LED phase=` / `HUE 0 r255 g0 b0` / `HUE 120 r0 g255 b0` / `HUE 240 r0 g0 b255`

## 验证状态

- ⚠️ 真灯带实测（RMT 时序/5V 缓冲链路/J7 亮度）**未验证**（无真板/灯带）

## 已知限制

- RMT 驱动为自研（copy encoder 组帧），未走 led_strip 组件（IDF v6.1 无预置 led_strip 于本工程 REQUIRES 层；零外部依赖纪律）。
- 色轮为派度线性插值（非 HSV 圆柱模型），满足验收三分点断言（0/120/240 纯色）。
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

- J7 外接 WS2812 灯带（数据经 R79 → U4 5V 缓冲 → R52）；勿绕过缓冲直连（3.3V 逻辑对 WS2812 不足）\n- 灯带默认 24 灯（`P03_LED_N`），GRB 800kHz
