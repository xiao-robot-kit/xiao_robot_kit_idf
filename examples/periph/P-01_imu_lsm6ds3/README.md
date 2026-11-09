# P-01_imu_lsm6ds3 — I2C 轮询 LSM6DS3TR-C(0x6A):WHO_AM_I + 100Hz 连读 + 总线扫描

任务书 P-01。链路:板载 LSM6DS3TR-C (U18, 地址 0x6A) → I2C 总线 → WHO_AM_I 校验 + 100Hz 连读 5min + 总线扫描。

## 硬件约束

- **LSM6DS3TR-C (U18)**：器件地址 0x6A（SA0 接地）；总线实线（SDA/SCL 脚位、速率 400kHz）以 `port/esp_idf` 平台端点为唯一事实源。
- **INT1/INT2 未接** → 本例只能轮询（无中断采样；100Hz 为软件语义节拍，传感器 ODR 由配置预设决定）。
- 板载无固件可控 LED（D1 绿灯/充电红×2 均为硬件驱动）→ 调试可视输出一律串口。

## 验收

① WHO_AM_I 校验：probe 通过 → `WHO 0x6A ok`；
② 100Hz 连读 5min 零报错：每秒 `IMU n=100 err=0 …`，300s 后 `DONE run=300 s …` + `VERDICT PASS`；
③ I2C 总线扫描：→ `SCAN dev=0x6A`（板面 I2C 唯一器件）。

## 双数据源

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `P_SIM_IMU` | 1（默认） | 合成 IMU 数据（testing/imu_sim，跨实验共享单源），WHO/SCAN 合成回显 0x6A；无板可测 |
| `P_SIM_IMU` | 0 | 真 I2C 传感器（port_i2c + vendored imu 驱动，预设 `imu.exp1.raw` 104Hz） |

## 输出格式（SIM 档）

```
OK boot P-01_imu_lsm6ds3 v0.1.0 | board=XIAO Robot Board V1.0 | mcu=XIAO ESP32-S3 Plus (BYPASS P_SIM_IMU)
INFO source=sim sim.exp2.static 100Hz
WHO 0x6A ok
SCAN dev=0x6A who=0x6A (sim echo)
SCAN total=1
IMU n=100 err=0 ax= 123 ay= -45 az= 1000 gx= 10 gy= -20 gz= 0 tmp= 25000
...
```

## 构建 / 运行

- 构建：`run.bat`（无参 = build；带 COMx = flash+monitor）
  - 期望：`OK boot` / `WHO 0x6A ok` / `IMU n=100 err=0` / `SCAN dev=0x6A`

## 验证状态

- ⚠️ 真板 I2C 连读 5min 零报错**未验证**（无真板）

## 已知限制

- 100Hz 为软件轮询语义节拍，传感器 ODR 由预设 `imu.exp1.raw`（104Hz）决定——轮询与 ODR 差 4Hz，属正常教学语义。
- 真传感器档的总线扫描受 port/esp_idf 单器件总线约束（addr 契约强制 0x6A），扫描结果为板面 I2C 唯一器件；SIM 档为合成回显。
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

- LSM6DS3TR-C（U18）：I2C 器件地址 0x6A（SA0 接地），总线 SDA/SCL 实线（400kHz；`port/esp_idf` 为唯一事实源）\n- INT1/INT2 未接（只能轮询，无中断采样）\n- 板载无固件可控 LED → 调试输出一律串口
