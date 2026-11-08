# exp4_quat — Mahony 四元数直出

> 类目：深度实验（examples/imu）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

读样 → 去偏 → 互补滤波 → 四元数 `0x04` 帧。欧拉角不在固件侧计算：上位机从
四元数反算（单源解算，本固件不建欧拉帧）。与 exp3 构成姿态表示法的对照。

## 二、帧与参数

- `0x04` QUAT：4×float32 LE（w,x,y,z，单位四元数）。
- 校准帧 `0x02`：每次标定完成连发 3 帧（同 exp2 口径）。
- 参数单源：`main/att_cfg_presets.c`（`att.exp4.mahony`）；标定预设 `cal.exp2.bias`。
- 传输面：真板 USB-Serial-JTAG；`EXP_QEMU_UART=1` 时 UART0。

## 三、双数据源宏

| 宏 | 1 | 0（本仓默认 1，便于无板跑通） |
|---|---|---|
| `EXP_SIM` | `examples/_shared/imu_sim.c` 合成静止源 | 真实 LSM6DS3（I2C @ GPIO5/6） |

> 私有开发树里 `EXP_SIM` 默认 0（真板优先）；公开发布版默认 1，保证拿到代码即可
> 无传感器编译并观察帧流。切真板：`idf.py -DEXP_SIM=0 build`。

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
