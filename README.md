<!-- SPDX-License-Identifier: MIT -->
# XIAO Robot Kit IDF Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

POSIX-style C component library for the **XIAO Robot Kit** (ESP32-S3): **IMU
driver, attitude fusion, motor control and parameter persistence**, with **zero
hardcoded parameters** — every tunable lives in a named preset table.

The C core is platform-neutral; it never includes a platform header. Buses and
PWM are injected as function-pointer tables, so the same code runs on
ESP-IDF, Arduino (see [`xiao_robot_kit_arduino`](https://github.com/xiao-robot-kit/xiao_robot_kit_arduino))
or bare metal.

## Design rules

- **POSIX flavor** — `snake_case`, `0` on success / negative on error,
  validated pointers, unit-annotated fields (`g`, `dps`, `ms`).
- **Zero hardcoded parameters** — rates, ranges, periods and controller
  gains all come from preset registries (`imu.raw.104`, `motor.servo.sg90`,
  …). No magic numbers in code; presets are data, not `#define`s.
- **Tested** — 1,311 assertions across the four component suites, all green
  (plus separate PID / PWM-timer / S-curve module suites under `tests/`).

## Components

| Component | Headers in | What you get | Unit tests |
| --- | --- | --- | --- |
| `xiao_imu` | `src/imu/` | LSM6DS3TR-C six-axis driver: probe → configure from semantic presets (Hz / g / dps) → samples in physical units | 29 assertions |
| `xiao_attitude` | `src/attitude/` | Quaternion math + Mahony complementary filter (gyro + accel → roll/pitch/yaw) | 107 assertions |
| `xiao_motor` | `src/motor/` | Motor instances on a HAL: PWM timer, PID speed loop with anti-windup, 7-phase constant-jerk S-curve trajectories | 1022 assertions |
| `xiao_pstore` | `src/pstore/` | Parameter persistence: item registry, framed serialization, CRC32, two-phase atomic load | 153 assertions |

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
#include "lsm6ds3.h"        /* xiao_imu */
#include "imu_cfg.h"
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
#include "lsm6ds3.h"     /* flat: each src/<component>/ dir is on its include path */
#include "mahony.h"
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
├── CMakeLists.txt          # plain-CMake library mode: xiao_imu/attitude/motor/pstore
├── LICENSE                 # MIT
├── src/
│   ├── imu/                #   lsm6ds3 driver + preset registry (flat: headers + sources)
│   ├── attitude/           #   quat + mahony
│   ├── motor/              #   pwm_timer, pid, scurve, motor + preset registry
│   └── pstore/             #   persistence: registry + framing + CRC32
├── tests/                  #   per-component unit tests (each defines main())
└── examples/
    └── idf/imu_quickstart/ #   ESP-IDF: minimal streaming project (CI-built)
```

Arduino users: the same C core ships as a self-contained Arduino library in
[`xiao_robot_kit_arduino`](https://github.com/xiao-robot-kit/xiao_robot_kit_arduino).

## License

[MIT](LICENSE) © XIAO Robot Kit Contributors