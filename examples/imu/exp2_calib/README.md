# exp2_calib — 零偏标定 + 温度补偿，参数跨重启保持

> 类目：深度实验（examples/imu）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

本系列的能力基线：读 N 帧求零偏（重力轴自动判定）→ 去偏 → 按温度阶阶梯重算
偏置 → `0x02` IMU_CAL 帧输出；标定结果经 `xiao_pstore` 落盘，重启后无需重标。

## 二、标定动作（只由显式动作触发）

| 入口 | 说明 |
|---|---|
| 开机一次 | 数据源就绪后立即采 N 帧（默认 N=500，100 Hz 下约 5 s），窗内不出数据帧，只用 TEXT 报进度 |
| `CAL` 行命令 | 运行期在 UART0 收到 ASCII 行 `CAL` 后重跑同一标定窗；窗内再收 `CAL` 不排队 |

TEXT（`0x7F`）逐字契约：`OK calib start N=<n>` / `OK calib progress <n>/<N>` /
`OK calib done bias ax=.. ay=.. az=.. gx=.. gy=.. gz=.. temp=..`。

## 三、帧与参数

- `0x02` IMU_CAL：每次标定完成连发 3 帧，负载 19×float32 LE，块序
  `raw6 | deb6 | bias6 | temp`；布局单源 `src/proto/frame.h`。
- 参数单源：`xiao_imu` 预设 `imu.attitude.208`、`xiao_calibration` 预设
  `cal.exp2.bias`、温补系数表见 `main/`（零写死）。
- 持久化：`xiao_pstore` + `examples/_shared/pstore_nvs.c`（NVS 介质后端）。

## 四、双数据源宏

| 宏 | 1（默认） | 0 |
|---|---|---|
| `EXP_SIM` | `examples/_shared/imu_sim.c` 合成源（注入零偏/温漂真值，无传感器可跑通全链） | 真实 LSM6DS3TR-C（I2C @ GPIO5/6） |

## 构建与运行（本仓）

本工程以库形态消费仓库根目录的组件（`xiao_imu, xiao_calibration, xiao_pstore, xiao_proto`），不含任何外部依赖副本：

```bat
run.bat COM5      :: build + flash + serial monitor (real board)
run_qemu.bat      :: build the synthetic-data variant only (no board needed)
```

等价命令行：

```bat
idf.py build
idf.py -B build-sim -DEXP_SIM=1 -DEXP_QEMU_UART=1 build
```



## 切真实硬件源

本工程的 `EXP_SIM=0` 分支使用真实传感器（LSM6DS3，I2C @ GPIO5/6）：

```bat
idf.py -DEXP_SIM=0 build
```
