# exp5_trajectory — 姿态 + 比力双流（轨迹实验数据源）

> 类目：深度实验（examples/imu）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

轨迹重建算法住在上位机（`xiao_robot_kit_host` 侧），固件只负责提供
**轨迹实验数据流**：每拍两帧——`0x04` QUAT（姿态估计）+ `0x01` IMU
（原始六轴比力，供上位机去重力后双重积分重建位移）。

与 exp4 的差异：exp4 只发 `0x04`，缺加速度通道；本工程补齐双流。

## 二、帧与参数

| 帧 | 内容 |
|---|---|
| `0x04` QUAT | Mahony 姿态估计（4×float32 LE），渲染/重建的姿态单源 |
| `0x01` IMU | 原始比力（6×float32 LE） |
| `0x02` IMU_CAL | 标定完成时 3 帧连发 |
| `0x7F` TEXT | 标定进度 / 段标记 |

- 段标记与三段轨迹场景单源：`main/traj_scen.c`。
- 参数单源：`main/att_cfg_presets.c`；标定预设 `cal.exp2.bias`。

## 三、双数据源宏

| 宏 | 1（默认） | 0 |
|---|---|---|
| `EXP_SIM` | `main/traj_scen.c` + `examples/_shared/imu_sim.c` 三段轨迹合成源（含线加速度项，标定窗用静止预设派生源） | 真实 LSM6DS3（I2C @ GPIO5/6） |

## 构建与运行（本仓）

本工程以库形态消费仓库根目录的组件（`xiao_attitude, xiao_imu, xiao_calibration, xiao_pstore, xiao_proto`），不含任何外部依赖副本：

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
