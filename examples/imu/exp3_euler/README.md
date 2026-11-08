# exp3_euler — Mahony 欧拉角直出

> 类目：深度实验（examples/imu）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

读六轴 → 零点校准去偏 → Mahony 互补滤波 → 欧拉角（ZYX，弧度）`0x03` 帧。
与 exp4 的**唯一变量是姿态表示法**：两工程跑同一滤波器、同一组预设
（`att.exp3.mahony` = `att.exp4.mahony`），exp3 出欧拉角（直观，但 pitch ±90°
处 ZYX 分解有万向锁），exp4 出四元数（无奇异）。

## 二、能力继承

| 数据源 | 零偏校准 | 姿态输出 | 帧型 |
|---|---|---|---|
| IMU / 合成摇摆源 | 继承 exp2 同口径（预设 `cal.exp2.bias`，N 帧均值 + 重力轴判定；不含温补） | Mahony 欧拉 | `0x03` + 校准 3×`0x02` |

## 三、帧与参数

- `0x03` EULER：12 B = roll,pitch,yaw 三个 float32 LE（弧度，ZYX 约定）。
- 校准：只走显式触发（开机一次 + `CAL` 行命令），TEXT 契约同 exp2。
- 参数单源：`main/att_cfg_presets.c`（`att.exp3.mahony`）；采样/量程 = `xiao_imu`
  预设 `imu.attitude.208`；输出行格式与 `0x02` 打包单源 = `main/zero_cal.c`
  （与 exp4/exp5 逐字同源）。
- 传输面：真板 USB-Serial-JTAG；`EXP_QEMU_UART=1` 时 UART0（同时是 `CAL` 通道）。

## 四、双数据源宏

| 宏 | 1（默认） | 0 |
|---|---|---|
| `EXP_SIM` | `examples/_shared/imu_sim.c` 预设 `sim.exp3.sway`（绕 Y 纯摆 ±20°、4 s 周期、零误差注入） | 真实 LSM6DS3（I2C @ GPIO5/6） |

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
