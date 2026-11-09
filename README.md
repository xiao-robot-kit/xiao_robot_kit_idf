<!-- SPDX-License-Identifier: MIT -->
# XIAO Robot Kit IDF Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

POSIX-style C component library for the **XIAO Robot Kit** (ESP32-S3): **eight
components** — IMU driver, attitude fusion, motor control, parameter
persistence, battery gauge, IMU calibration, IMU heater and the serial frame
codec — with **zero hardcoded parameters**: every tunable lives in a named
preset table.

The C core is platform-neutral; it never includes a platform header. Buses and
PWM are injected as function-pointer tables, so the same code runs on
ESP-IDF, Arduino (see [`xiao_robot_kit_arduino`](https://github.com/xiao-robot-kit/xiao_robot_kit_arduino))
or bare metal.

**28 example projects**, grouped by the hardware they exercise — the full index
with per-project pins, acceptance anchors and library usage is
[`examples/README.md`](examples/README.md):

| Group | Count | Location |
| --- | --- | --- |
| Board peripherals — one per on-board device | 13 | [`examples/periph/`](examples/periph) (P-00 … P-12) |
| IMU pipeline — raw → calibrated → euler → quaternion → trajectory | 5 | [`examples/imu/`](examples/imu) |
| Motor control — open loop → encoder → PID → S-curve → nested timers | 8 | [`examples/motor/`](examples/motor) |
| Persistence across power loss | 1 | [`examples/store_demo/`](examples/store_demo) |
| Quickstart template to copy | 1 | [`examples/idf/imu_quickstart/`](examples/idf/imu_quickstart) |

Each is a standalone ESP-IDF project built against the `xiao_*` libraries in
this repo; every one also builds with no board attached (simulated data source
is the default — see the variant macros in [`examples/README.md`](examples/README.md#8-real-hardware-vs-simulated-data)).

## Design rules

- **POSIX flavor** — `snake_case`, `0` on success / negative on error,
  validated pointers, unit-annotated fields (`g`, `dps`, `ms`).
- **Zero hardcoded parameters** — rates, ranges, periods and controller
  gains all come from preset registries (`imu.raw.104`, `motor.servo.sg90`,
  …). No magic numbers in code; presets are data, not `#define`s.
- **Tested** — 5,052 assertions across 11 test binaries, all green (`ctest` 11/11).
  Counts below are the actual printed numbers from each binary, not estimates.

## Components

| Component | Headers in | What you get | Unit tests |
| --- | --- | --- | --- |
| `xiao_imu` | `src/imu/` | LSM6DS3TR-C six-axis driver: probe → configure from semantic presets (Hz / g / dps) → samples in physical units | 29 assertions |
| `xiao_attitude` | `src/attitude/` | Quaternion math + Mahony complementary filter (gyro + accel → roll/pitch/yaw) | 107 assertions |
| `xiao_motor` | `src/motor/` | Motor instances on a HAL: PWM timer, PID speed loop with anti-windup, 7-phase constant-jerk S-curve trajectories | 1238 assertions (motor 1022 + pid 78 + pwm_timer 119 + scurve 19) |
| `xiao_pstore` | `src/pstore/` | Parameter persistence: item registry, framed serialization, CRC32, two-phase atomic load | 153 assertions |
| `xiao_battery` | `src/battery/` | Battery voltage / remaining capacity (SOC) curve computation, pure algorithm, zero platform deps | 35 assertions |
| `xiao_calibration` | `src/calibration/` | IMU calibration (6-axis bias + temp compensation), uses xiao_imu types | 3402 assertions |
| `xiao_imu_heater` | `src/imu_heater/` | Heater controller state machine for IMU preheating (hysteresis + overheat guard + watchdog) | 40 assertions |
| `xiao_proto` | `src/proto/` | Serial frame codec shared by firmware and host app: SOF/LEN/TYPE/SEQ/PAYLOAD/CRC8 (poly 0x07), frame type table and CMD/ACK addressing. The header is the wire-format contract | 48 assertions |

## Quickstart (ESP-IDF, esp32s3)

Use the runnable reference project
[`examples/idf/imu_quickstart`](examples/idf/imu_quickstart) (built with
ESP-IDF v6.1), or copy the two-line CMake pattern into your own project:

```cmake
# in your project's CMakeLists.txt (EXTRA_COMPONENT_DIRS or add_subdirectory)
add_subdirectory(xiao_robot_kit_idf)          # this repository
```

Then in `main.c` (~20 lines):

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu/lsm6ds3.h"    /* xiao_imu — headers keep their component prefix */
#include "imu/imu_cfg.h"
#include "bus_i2c_idf.h"    /* your glue: fills imu_i2c_bus_t — see examples/idf */

void app_main(void)
{
    const imu_cfg_t *cfg = imu_cfg_find("imu.raw.104");  /* preset, zero hardcoded params */
    bus_i2c_init(cfg->i2c_speed_hz);

    lsm6ds3_t imu;
    lsm6ds3_init(&imu, &bus_i2c);
    while (lsm6ds3_probe(&imu) != 0) vTaskDelay(pdMS_TO_TICKS(1000)); /* wait for chip */
    lsm6ds3_configure(&imu, cfg);

    for (;;) {
        imu_sample_t s;                    /* units: g, dps */
        if (lsm6ds3_read(&imu, &s) == 0)
            printf("az=%+.3fg gz=%+.1fdps\n", s.az, s.gz);
        vTaskDelay(pdMS_TO_TICKS(cfg->sample_period_ms));
    }
}
```

### Plain C (CMake, platform-neutral)

```cmake
add_subdirectory(xiao-robot-kit-idf)          # this repository
target_link_libraries(app PRIVATE xiao_imu xiao_attitude)   # pick what you use
```

```c
#include "imu/lsm6ds3.h"   /* headers keep their component prefix */
#include "att/mahony.h"
```

Standalone build with tests:

```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build          # per-library unit tests
```

## Repository layout

```
xiao_robot_kit_idf/
├── idf_component.yml       # ESP-IDF component manifest (esp32s3)
├── CMakeLists.txt          # plain-CMake library mode: all eight xiao_* libs
├── LICENSE                 # MIT
├── src/
│   ├── imu/                #   lsm6ds3 driver + preset registry
│   │   ├── include/imu/    #     public headers (include root = include/)
│   │   └── src/            #     implementation
│   ├── attitude/           #   quat + mahony            (include/att/,  src/)
│   ├── motor/              #   pwm_timer, pid, scurve, motor + presets (include/mtr/)
│   ├── pstore/             #   persistence: registry + framing + CRC32 (include/pstore/)
│   ├── battery/            #   ADC voltage → SOC curve (include/battery/)
│   ├── calibration/        #   IMU bias + temp compensation (include/cal/)
│   ├── imu_heater/         #   heater control state machine (include/imu_heater/)
│   └── proto/              #   serial frame codec, wire-format contract (include/proto/)
├── tests/                  #   per-component unit tests (each defines main())
└── examples/
    ├── idf/imu_quickstart/ #   ESP-IDF: minimal streaming project
    ├── _shared/            #   port layer + simulated data sources (shared by all examples)
    ├── imu/                #   5 staged IMU experiments + their shared command layer
    │   ├── exp1_raw/       #     raw samples
    │   ├── exp2_calib/     #     bias calibration + temperature compensation
    │   ├── exp3_euler/     #     Mahony filter -> Euler angles
    │   ├── exp4_quat/      #     Mahony filter -> quaternion
    │   ├── exp5_trajectory/#     attitude + specific-force dual stream
    │   └── common/         #     calibration command session (single source)
    ├── motor/              #   8 motor experiments
    │   ├── exp_motor_qemu/ #     five scenarios behind one selector
    │   ├── exp_motor_pwm/  #     open-loop duty staircase
    │   ├── exp_motor_dc/   #     H-bridge direction switching
    │   ├── exp_motor_servo/#     five-point servo angle calibration
    │   ├── exp_motor_enc/  #     encoder closed-loop step response
    │   ├── exp_motor_nested/#    nested timers, channel isolation
    │   ├── exp_motor_pidscan/#   PID gain grid scan (3x3)
    │   └── exp_motor_scurve/ #   seven-phase S-curve trajectories
    ├── store_demo/         #   parameter persistence across power cycles
    └── periph/             #   13 board examples, one per peripheral (P-00 ~ P-12)
        ├── P-00_hello_bringup/     toolchain / UART self-check
        ├── P-01_imu_lsm6ds3/       on-board IMU
        ├── P-02_button_k7/         D11 user button
        ├── P-03_rgb_ws2812/        D13 WS2812 strip
        ├── P-04_battery_voltage/   D16 battery ADC → SOC
        ├── P-05_motor_gc9112/      2× GC9112 H-bridge
        ├── P-06_encoder_read/      encoder counting + closed loop
        ├── P-07_servo_pwm/         4× 50 Hz servo PWM
        ├── P-08_i2c_scan_ports/    I2C bus enumeration (J11/J12)
        ├── P-09_uart_passthrough/  Grove UART loopback (J10)
        ├── P-10_imu_heater/        heater closed loop + temp compensation
        ├── P-11_pstore_params/     param persist across power cycle
        └── P-12_spare_gpio_tpxx/   TP20/TP21 spare pins
```

## Board examples (P-00 ~ P-12)

Each example is a **standalone ESP-IDF project**; `cd` into one and build it.
Nothing is shared except `examples/_shared/` (port layer + simulated data
sources), so an example never depends on another example's build. Every example
consumes the library targets from the root `CMakeLists.txt` instead of copying
component sources.

```bash
cd examples/periph/P-00_hello_bringup
idf.py set-target esp32s3 && idf.py build flash monitor
```

| Example | Peripheral | What you should see | Uses |
| --- | --- | --- | --- |
| P-00 hello_bringup | — (toolchain) | version + reset reason + heartbeat, no reboot for 10 min | — |
| P-01 imu_lsm6ds3 | LSM6DS3TR-C (I2C) | `WHO 0x6A ok` then 100 Hz samples, 5 min zero errors | `xiao_imu` |
| P-02 button_k7 | D11 button K7 | short/long press events, no debounce glitches | — |
| P-03 rgb_ws2812 | D13 → J7 strip | colour fade + single-pixel sweep | — |
| P-04 battery_voltage | D16 ADC | `vbat` mV + SOC %, low-battery warning | `xiao_battery` |
| P-05 motor_gc9112 | D1/D2, D19/D18 | fwd/rev/brake/duty sweep + duty-vs-rpm curve | `xiao_motor` |
| P-06 encoder_read | D3/D14, D17/D10 | manual counts + closed loop to target angle | `xiao_motor` |
| P-07 servo_pwm | D15/D9/D0/D8 | 0°/90°/180° positioning + supply-sag warning | `xiao_motor` |
| P-08 i2c_scan_ports | D4/D5 → J11/J12 | address table (on-board IMU 0x6A + external) | — |
| P-09 uart_passthrough | D6/D7 → J10 | 115200 loopback, zero loss / CRC ok | — |
| P-10 imu_heater | D12 → heater + IMU temp | hold within ±2 °C, overheat guard, temp compensation | `xiao_imu_heater` `xiao_calibration` `xiao_imu` |
| P-11 pstore_params | `store` flash partition | write → power cycle → read back identical | `xiao_pstore` |
| P-12 spare_gpio_tpxx | TP20/TP21 | spare pins as input/output (strapping caveat) | — |

Every example also ships a `run_qemu.bat` that builds the **SIM variant**
(simulated data source instead of real hardware) — useful to verify the project
compiles without a board attached. Booting QEMU and asserting on its serial
output is done by the project's internal CI, not shipped here.

Usage walkthroughs, wiring and troubleshooting for all 13 examples live in the
docs repo: [`xiao_robot_kit_docs/examples/periph-index.md`](https://github.com/xiao-robot-kit/xiao_robot_kit_docs/blob/main/examples/periph-index.md).

## Deeper experiments (14)

Same library, one new capability per project — a failure localises to the stage
you just added. Pins, frames and acceptance anchors per project:
[`examples/README.md`](examples/README.md).

| Group | Projects | What each one adds |
| --- | --- | --- |
| IMU pipeline (5) | `exp1_raw` → `exp2_calib` → `exp3_euler` → `exp4_quat` → `exp5_trajectory` | raw g/dps → bias calibration + temperature compensation **with persistence** → euler angles → quaternion (exp3 vs exp4 is a controlled A/B on the representation only) → attitude + specific-force dual stream for host-side trajectory |
| Motor control (8) | `exp_motor_pwm` `_dc` `_servo` `_enc` `_pidscan` `_scurve` `_nested` `_qemu` | duty staircase (dead zone) → H-bridge sign → 5-point servo calibration → encoder speed loop step → 3×3 `kp×ki` grid → 7-phase S-curve with velocity feed-forward → channel isolation across two timers → all four kinds behind one `SCEN <n>` selector |
| Persistence (1) | `store_demo` | `xiao_pstore` on a raw flash partition, no IMU dependency, with a QEMU power-cycle harness |

Arduino users: the same C core ships as a self-contained Arduino library in
[`xiao_robot_kit_arduino`](https://github.com/xiao-robot-kit/xiao_robot_kit_arduino).

## License

[MIT](LICENSE) © XIAO Robot Kit Contributors