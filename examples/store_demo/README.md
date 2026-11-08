# store_demo — 参数存储：跨掉电持久化

> 类目：深度实验（examples）｜ 状态：**SIM 构建通过；真板电气行为未实测**

## 一、意图

`xiao_pstore` 的独立例程（不依赖 IMU / 标定）：参数写进 flash 裸分区，掉电后
重新上电仍在。演示三件事：

1. **裸分区后端**：分区 label `store`（`partitions.csv` 自定义 data 子类型），
   经 `esp_partition` 直接读写，不经 NVS 中间层（`examples/_shared/pstore_flash.c`）；
2. **跨掉电续跑**：`boot_count` 每次开机 +1 并落盘，断电重启后继续累加；
3. **命令通道**（`0x7D` CMD / `0x7E` ACK）：`MOD_STORE` 三条命令
   `GET` / `STATUS` / `SET`，另有 `0x7F` TEXT 周期状态行便于串口直接观察。

## 二、运行要点

- `partitions.csv` 必须随工程一起使用（自定义 `store` 分区）；换分区表需
  `rm -rf build` 后重新 configure。
- 上电后 TEXT 输出当前 `boot_count` 与已存参数，可用上位机或手打命令行验证。

## 三、源码结构

| 文件 | 职责 |
|---|---|
| `main/store_demo.c` | 会话实现（纯 C，不依赖 ESP-IDF 头，可 host 编译） |
| `main/store_demo_main.c` | IDF 主程序：分区装配 + 帧出口 + 调度 |
| `examples/_shared/pstore_flash.c` | flash 裸分区介质后端 |
| `xiao_pstore` / `xiao_proto` | 两阶段原子写 / 帧编解码 |

## 构建与运行（本仓）

本工程以库形态消费仓库根目录的组件（`xiao_pstore, xiao_proto`），不含任何外部依赖副本：

```bat
run.bat COM5      :: build + flash + serial monitor (real board)
run_qemu.bat      :: build the synthetic-data variant only (no board needed)
```

等价命令行：

```bat
idf.py build
idf.py -B build-sim -DEXP_SIM=1 -DEXP_QEMU_UART=1 build
```

