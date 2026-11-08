# exp1_raw — IMU 裸数据直出

> 类目：深度实验（examples/imu）｜ 状态：**SIM/真板均构建通过，真板电气行为未实测**

## 一、意图

最小 IMU 链路：LSM6DS3TR-C（0x6A，I2C @ GPIO5/6）→ 驱动换算（g / dps）→
`0x01` 帧直出。后续 exp2~exp5 全部在这条链路上叠加能力，本工程是它们的基线。

## 二、现象（串口）

上电后连续出现 `0x01` IMU 帧（每帧 6×float32 LE：ax,ay,az,gx,gy,gz）。
静置时 `az ≈ ±1 g`，其余轴接近 0；晃动板子波形随之变化。

## 三、帧与参数

- 帧布局单源：`src/proto/frame.h`（`PROTO_TYPE_IMU_SAMPLE = 0x01`）。
- 采样率 / 量程零写死：全部来自 `xiao_imu` 预设 `imu.raw.104`。
- 帧出口：真板档 USB-Serial-JTAG；`EXP_QEMU_UART=1` 时改为 UART0。

## 四、源码结构

| 文件 | 职责 |
|---|---|
| `main/exp1_main.c` | init → probe → configure → loop read → 组帧直出 |
| `examples/_shared/imu_hal_i2c.c` | I2C 端口层（`port_i2c.h` 契约） |
| `xiao_imu` / `xiao_proto` | 驱动 + 预设表 / 帧编解码 |

## 构建与运行（本仓）

本工程以库形态消费仓库根目录的组件（`xiao_imu, xiao_proto`），不含任何外部依赖副本：

```bat
run.bat COM5      :: build + flash + serial monitor (real board)
run_qemu.bat      :: build the synthetic-data variant only (no board needed)
```

等价命令行：

```bat
idf.py build
idf.py -B build-sim -DEXP_SIM=1 -DEXP_QEMU_UART=1 build
```

