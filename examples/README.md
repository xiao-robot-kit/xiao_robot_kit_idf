<!-- SPDX-License-Identifier: MIT -->
# Examples

**28 standalone ESP-IDF projects**, grouped by *what hardware they exercise* —
not by experiment number. Every project is its own `idf.py build` target and
links the `xiao_*` library targets from the repository root `CMakeLists.txt`;
no project copies component sources.

| Group | Count | What it is for |
| --- | --- | --- |
| [Board peripherals](#1-board-peripheral-examples-13) (`periph/`) | **13** | one example per on-board peripheral — the "make sure the hardware works" layer |
| [IMU pipeline](#2-imu-pipeline-5-stages-imu) (`imu/`) | **5** | five stages stacked on the same link: raw → calibrated → euler → quaternion → trajectory data source |
| [Motor control](#3-motor-control-8-projects-motor) (`motor/`) | **8** | one control strategy per project: open loop → encoder → PID → S-curve → nested timers |
| [Persistence](#4-persistence-1-project) (`store_demo/`) | **1** | parameters that survive power loss |
| [Quickstart template](#5-quickstart-template-1-project-idf) (`idf/imu_quickstart/`) | **1** | minimal ~20-line `main.c` to copy into your own project |
| `_shared/` | sources | port layer (I2C endpoint, flash/NVS backends) + synthetic data sources, compiled *into* the projects |

Recommended order for a first pass over the board: **P-00 → P-01 → P-02 → P-03**
(see [suggested routes](#6-suggested-routes)).

---

## 1. Board peripheral examples (13)

One per on-board peripheral, `periph/P-XX_*`. Each README states the acceptance
criteria, the pin map and the exact console anchors to look for. The **Pins**
column is the real wiring; the **You should see** column is the output anchor to
grep for.

| Periph-eral | # | Project | What the example does | Pins (board net) | Uses | You should see |
| --- | --- | --- | --- | --- | --- | --- |
| USB serial (no GPIO) | P-00 | [`P-00_hello_bringup`](periph/P-00_hello_bringup) | Smallest possible skeleton: proves toolchain → flash → UART log chain. Touches **zero** GPIOs | — (USB-C, USB-Serial-JTAG 115200 8N1) | ESP-IDF only | `OK boot … board=XIAO Robot Board V1.0`, `HELLO workflow: USB-CDC log link up`, then `HEART <n> uptime=<s> s` once a second |
| IMU (6-axis) | P-01 | [`P-01_imu_lsm6ds3`](periph/P-01_imu_lsm6ds3) | `WHO_AM_I` probe → configure from preset (`imu.raw.104`) → 100 Hz polling for 5 min + I2C bus scan | SDA=D4/GPIO5, SCL=D5/GPIO6 (on-board bus, 400 kHz, defined in `_shared/imu_hal_i2c.c`) | `xiao_imu` | `WHO 0x6A ok`, `IMU n=100 err=0 …`, after 300 s `VERDICT PASS`; `SCAN dev=0x6A` |
| Button K7 | P-02 | [`P-02_button_k7`](periph/P-02_button_k7) | Debounce (≥20 ms) + short-press (<800 ms) / long-press (≥800 ms, fires at the threshold) events | D11/GPIO11 (internal pull-up, press = low) | ESP-IDF `gpio` | `KEY SHORT n=… hold=… ms` / `KEY LONG n=… hold=… ms`, final `KEY done n=… deb=… jitter=…` |
| RGB / WS2812 | P-03 | [`P-03_rgb_ws2812`](periph/P-03_rgb_ws2812) | RMT-driven WS2812 strip: phase-shifted colour wheel + two fixed pixels to prove single-LED addressing | D13/GPIO13 → R79 → U4 buffer → J7 (**must go through J7**) | ESP-IDF `rmt` | `HUE 0 r255 g0 b0` / `HUE 120 …` / `HUE 240 …`, per-phase `LED phase=… led0=… led5=…` |
| Battery gauge | P-04 | [`P-04_battery_voltage`](periph/P-04_battery_voltage) | One-shot ADC on the 100K:30K divider → SOC / remaining capacity from `xiao_1s_1400mah` preset → low-voltage alarm | GPIO16 (ADC2_CH5, DB_11) | `xiao_battery` + `esp_adc` | `BATT v=… soc=…% mah=… alarm=0/1`, plus `SRC synthetic-adc` vs `SRC adc2_ch5 GPIO16` |
| DC motor ×2 | P-05 | [`P-05_motor_gc9112`](periph/P-05_motor_gc9112) | Two GC9112 H-bridges: forward / reverse / brake / duty sweep (+50 → −50 …), duty→rpm curve | M1 = D1/GPIO1 + D2/GPIO2, M2 = D19/GPIO19 + D18/GPIO18 | `xiao_motor` (`motor.dc.generic`, `pwm.motor.20k`) | `CURVE M1 duty=… rpm=…`, `DONE` then 1 Hz replay |
| Encoder ×2 | P-06 | [`P-06_encoder_read`](periph/P-06_encoder_read) | Quadrature read (manual SPIN phase) then closed-loop angle move ±120° (DRIVE phase) with `motor.enc.n20` (cpr 7200) | ENC1 = D3/GPIO3 + D14/GPIO14, ENC2 = D17/GPIO17 + D10/GPIO10 | `xiao_motor` (encoder + PID) | `ENC SPIN t+…ms count=±…`, `ENC PIK count=…`, then `DRIVE target=+120` → `ENC ACT t+… rpm=… count=…` |
| Servo ×4 | P-07 | [`P-07_servo_pwm`](periph/P-07_servo_pwm) | Four independent 50 Hz LEDC channels sweeping 0→90→180→90→0 with staggered phase; clamps to 0° and warns when battery < 6.20 V | S1..S4 = D15/GPIO15, D9/GPIO9, D0/GPIO0, D8/GPIO8 | `xiao_motor` (`motor.servo.sg90`, `pwm.servo.50`) + `esp_adc` | `SERVO vbat=… ang0=… (us:…/…/…/…)`, `WARN vbat=… low (cutoff 6.20V) servo hold`, `SERVO DONE n=…` |
| I2C expansion ports | P-08 | [`P-08_i2c_scan_ports`](periph/P-08_i2c_scan_ports) | Full 0x03–0x77 probe of the Grove ports on the *same* bus as the on-board IMU + address-table bookkeeping (`KNOWN` vs `NEW`) | J11/J12 (shared SDA=D4, SCL=D5) | ESP-IDF `i2c_master` | `SCAN dev=0x6A kind=KNOWN`, `SCAN DONE run=5 scan=PASS` |
| Grove UART | P-09 | [`P-09_uart_passthrough`](periph/P-09_uart_passthrough) | 115200 8N1 loopback / passthrough, data plane on UART0 while logs stay on USB-CDC, per-round CRC + loss stats | J10: D6 (UART0 TX), D7 (UART0 RX) | ESP-IDF `uart` | `LOOP n=1 tx=1000 rx=1000 loss=0 crc=OK` |
| IMU heater | P-10 | [`P-10_imu_heater`](periph/P-10_imu_heater) | Closed-loop heater (`thermo45`: target 45 °C ±2, overheat 60) with soft watchdog, wired into `temp_comp` bias compensation | D12/GPIO12 → R76 100R → Q1 NMOS | `xiao_imu_heater` + `xiao_calibration` | `HT cfg preset=thermo45 target=…`, per-round `HT  r=… st=HOLD d=0.12 tc=1 bias=(…)`, `WDT fail seq=3 … forced duty=0`, `TC init t_ref=25.0` |
| Flash parameter partition | P-11 | [`P-11_pstore_params`](periph/P-11_pstore_params) | Multi-item registry (`motor.cal`, `servo.center`) → CRC32 frame → atomic write/load; QEMU power-kill then boot again and compare | custom `store` partition (`partitions.csv`) | `xiao_pstore` | `P11 SET ok`, then after the power cycle `P11 GET … MATCH=1` (`MATCH=0 MISMATCH params=…` on failure) |
| Spare pins | P-12 | [`P-12_spare_gpio_tpxx`](periph/P-12_spare_gpio_tpxx) | The only spare GPIOs on the board: MTDI/MTDO exercised as plain I/O, both roles, read-back compared | TP20 = MTDI = GPIO41, TP21 = MTDO = GPIO42 (**test points, requires flying leads**) | ESP-IDF `gpio` | `SPARE done mismatch=(0,0)` → `VERDICT PASS` |

**Counts by peripheral group:** serial/bring-up 1 · I2C sensors & ports 2 (IMU
P-01, bus scan P-08) · user I/O 2 (button P-02, RGB P-03) · power 1 (battery
P-04) · motion 3 (DC P-05, encoder P-06, servo P-07) · thermal 1 (heater P-10) ·
storage 1 (P-11) · spare 1 (P-12) · serial data path 1 (UART P-09) — **total 13**.

> The board has **no free GPIO left**: all 20 are consumed by the peripherals
> above. That is why P-12 has to fly-lead TP20/TP21, and why examples never
> invent extra pins.

## 2. IMU pipeline: 5 stages (`imu/`)

Same link, one capability added per stage — so a failure localises to the stage
you just added. `imu/common/` holds the shared command layer (`cal_cmd.c/h`)
they all use.

| Stage | Project | Adds on top of the previous one | Frame out |
| --- | --- | --- | --- |
| 1 | [`exp1_raw`](imu/exp1_raw) | The baseline: probe → `imu.raw.104` preset → g / dps in physical units. Nothing else | `0x01` IMU_SAMPLE |
| 2 | [`exp2_calib`](imu/exp2_calib) | Gyro bias from N frames (gravity axis auto-detected) → debias → temperature-step bias recompute → **persist via `xiao_pstore`**, so no re-calibration after reboot | `0x02` IMU_CAL |
| 3 | [`exp3_euler`](imu/exp3_euler) | Mahony complementary filter → **euler angles (ZYX, rad)** | `0x03` EULER |
| 4 | [`exp4_quat`](imu/exp4_quat) | Same filter, **quaternion** output. Euler is computed *on the host* from the quaternion — one solver, no duplicate math in firmware | `0x04` QUAT |
| 5 | [`exp5_trajectory`](imu/exp5_trajectory) | Two frames per tick (quaternion + raw specific force) as the data source for host-side trajectory reconstruction | `0x04` + `0x01` |

exp3 vs exp4 is a **controlled A/B**: same samples, same presets, same filter —
the only variable is the attitude representation.

## 3. Motor control: 8 projects (`motor/`)

| Project | Control面 | Scenario |
| --- | --- | --- |
| [`exp_motor_pwm`](motor/exp_motor_pwm) | open-loop PWM | duty staircase 0/25/50/75/100 %, 2 s each — shows monotonicity and the dead zone |
| [`exp_motor_dc`](motor/exp_motor_dc) | H-bridge (signed duty) | +50 % ↔ −50 % every 2 s; the sign bit *is* the direction |
| [`exp_motor_servo`](motor/exp_motor_servo) | servo position | five-point calibration 0/45/90/135/180 ° up then down — hysteresis / repeatability |
| [`exp_motor_enc`](motor/exp_motor_enc) | encoder speed loop | 30 ↔ 90 rpm steps, 4 s per segment, synthetic first-order plant |
| [`exp_motor_pidscan`](motor/exp_motor_pidscan) | PID grid scan | 3×3 `kp × ki` grid over the same step, so you can compare overshoot / settling side by side |
| [`exp_motor_scurve`](motor/exp_motor_scurve) | S-curve trajectory | 7-phase constant-jerk point-to-point on both slots; velocity output feeds the controller as feed-forward |
| [`exp_motor_nested`](motor/exp_motor_nested) | timer nesting | two motors sharing one 20 kHz timer (ch0/ch1) + a servo on an independent 50 Hz timer — channel isolation |
| [`exp_motor_qemu`](motor/exp_motor_qemu) | all four kinds + selector | runs a demo on boot, then `SCEN <n>` over UART0 switches kind at runtime — the one to use with the QEMU harness |

All eight emit `0x05` MOTOR telemetry at a 500 Hz control loop.

## 4. Persistence: 1 project

[`store_demo`](store_demo) — `xiao_pstore` on a raw flash partition, no IMU or
calibration dependency: write params → power-cycle → read back identical. Ships
with a QEMU power-cycle harness (`testing/qemu_powercycle.py`).

## 5. Quickstart template: 1 project (`idf/`)

[`idf/imu_quickstart`](idf/imu_quickstart) — the ~20-line `main.c` used by the
tutorial: register your glue table, link the libraries, stream six-axis samples.
Copy this into your own project rather than starting from P-01.

## 6. Suggested routes

| You want | Path |
| --- | --- |
| Confirm the board is alive | P-00 → P-01 → P-02 → P-03 |
| Add your own I2C sensor | P-08 → P-01 (preset table) → P-09 (data out) |
| Make the IMU trustworthy | `imu/exp1_raw` → `exp2_calib` (bias + persistence) → `exp3_euler` |
| Close a motion loop | P-05 → P-06 → `motor/exp_motor_enc` → `exp_motor_pidscan` → `exp_motor_scurve` |
| Keep parameters across power loss | P-11 → `store_demo` |

## 7. Running one

```bash
cd examples/periph/P-01_imu_lsm6ds3
idf.py set-target esp32s3
idf.py build                 # build only
run.bat COM5                 # Windows: build + flash + monitor
run_qemu.bat                 # Windows: build the simulated variant only
```

`run.bat` / `run_qemu.bat` set `IDF_PATH` / `IDF_PYTHON_ENV_PATH` themselves when
missing. Build one project at a time — `idf.py build` and `idf.py reconfigure`
share `build/` and will produce bogus failures if run concurrently.

## 8. Real hardware vs simulated data

Every project ships **two data sources**, selected at build time. The default is
the simulated one, so all examples build and run without the board attached.

| Macro | `1` (default) | `0` |
| --- | --- | --- |
| `EXP_SIM` (deep experiments) / `P_SIM_*` (peripherals) | synthetic samples — no sensor, motor or heater needed; runs under QEMU | real hardware source. Where a HAL is not published yet (motor PWM, heater PWM) the build is **rejected at compile time on purpose**, instead of silently shipping a fake-hardware firmware |
| `EXP_QEMU_UART` | frames + logs on UART0 (QEMU / serial harness) | logs on USB-Serial-JTAG (real board over USB-C) |

```bash
idf.py -DEXP_SIM=0 build      # deep experiments: real hardware path
idf.py -DP_SIM_IMU=0 build    # peripherals: per-project macro name, see its README
```

## 9. Shared sources (`_shared/`)

Not a project. Compiled into the examples that need it: the ESP-IDF I2C endpoint,
flash + NVS `pstore` backends, and the synthetic sources (IMU, motor HAL,
encoder). Changing a file here affects every example — the built-in variant
switches keep the real/simulated boundary explicit.

## 10. Wire format

Examples that speak the serial protocol use `xiao_proto`; the byte layout is
defined once in [`src/proto/include/proto/frame.h`](../src/proto/include/proto/frame.h) — SOF/LEN/TYPE/SEQ/
PAYLOAD/CRC8, frame types `0x01`…`0x7F`. That header is the contract; do not
duplicate it in a doc or a sketch.

## 11. Verification status

Per-example detail (acceptance criteria, console anchors, what to change) lives
in each project's README. Build status:

- `idf.py reconfigure` — **28/28 projects pass** (ESP-IDF v6.1, target esp32s3)
- full build verified — P-00, P-01, P-05, P-11, `imu/exp1_raw`, `store_demo`
- remaining projects share the same template but were **not** individually built
  in the last release window; re-run before trusting a green checkmark
- **on-board electrical behaviour is not verified in this repository** — no
  hardware was attached. The "you should see" anchors are the contract to check
  yourself.