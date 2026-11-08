# Examples

Every directory here is a standalone ESP-IDF project (except `_shared`, which
holds shared sources compiled *into* the projects). Each project links the
`xiao_*` library targets defined in the repository root `CMakeLists.txt`; no
project copies component sources.

| Group | What it covers | Projects |
|---|---|---|
| [`periph/`](periph) | one board peripheral per example | 13 (`P-00` … `P-12`) |
| [`imu/`](imu) | the IMU pipeline, staged | 5 (`exp1_raw` … `exp5_trajectory`) |
| [`motor/`](motor) | motor control strategies | 8 (`exp_motor_*`) |
| [`store_demo/`](store_demo) | parameter persistence across power cycles | 1 |
| [`idf/`](idf) | minimal streaming project for the quickstart tutorial | 1 |
| [`_shared/`](_shared) | port layer + simulated data sources | (sources only) |

## Running one

```bash
cd examples/motor/exp_motor_dc
idf.py set-target esp32s3
idf.py build                 # real-board variant
run.bat COM5                 # Windows helper: build + flash + monitor
run_qemu.bat                 # Windows helper: build the SIM variant only
```

`run.bat` / `run_qemu.bat` assume ESP-IDF is installed and set `IDF_PATH` /
`IDF_PYTHON_ENV_PATH` themselves when those variables are missing.

## Variant macros

| Macro | 1 | 0 |
|---|---|---|
| `EXP_SIM` | synthetic data source (no sensor/motor hardware needed) | real hardware source — where the hardware HAL is not published, the build is rejected at compile time on purpose instead of silently producing a "fake hardware" firmware |
| `EXP_QEMU_UART` | frames on UART0 (works with a QEMU/serial harness) | frames on USB-Serial-JTAG (real board) |

`periph/` projects use the older `P_SIM_*` names for the same idea; see each
project's README.

## Wire format

All examples that speak the serial protocol use `xiao_proto`; the byte layout is
defined once in [`src/proto/frame.h`](../src/proto/frame.h) — that header is the
contract, not a duplicate of one.