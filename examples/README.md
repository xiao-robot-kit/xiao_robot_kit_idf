<!-- SPDX-License-Identifier: MIT -->
# Examples

**13 个板载外设例程**（`periph/P-XX_*`），一外设一个工程——回答「这颗器件
能不能动」。每个例程是独立的 `idf.py build` 目标，链接仓根 `CMakeLists.txt`
的 `xiao_*` 库目标，不复制组件源码。

| Group | Count | What it is for |
| --- | --- | --- |
| [Board peripherals](#1-board-peripheral-examples-13) (`periph/`) | **13** | 每个板载外设一个例程——「确认硬件能用」这一层 |
| `_shared/` | sources | 端口层（I2C 端点、flash 后端）+ 合成数据源，编进各例程 |

推荐首次过板的顺序：**P-00 → P-01 → P-02 → P-03**（见[建议路径](#5-suggested-routes)）。

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
| Flash parameter partition | P-11 | [`P-11_pstore_params`](periph/P-11_pstore_params) | Multi-item registry (`motor.cal`, `servo.center`) → CRC32 frame → atomic write/load; power-cycle then boot again and compare | custom `store` partition (`partitions.csv`) | `xiao_pstore` | `P11 SET ok`, then after the power cycle `P11 GET … MATCH=1` (`MATCH=0 MISMATCH params=…` on failure) |
| Spare pins | P-12 | [`P-12_spare_gpio_tpxx`](periph/P-12_spare_gpio_tpxx) | The only spare GPIOs on the board: MTDI/MTDO exercised as plain I/O, both roles, read-back compared | TP20 = MTDI = GPIO41, TP21 = MTDO = GPIO42 (**test points, requires flying leads**) | ESP-IDF `gpio` | `SPARE done mismatch=(0,0)` → `VERDICT PASS` |

**Counts by peripheral group:** serial/bring-up 1 · I2C sensors & ports 2 (IMU
P-01, bus scan P-08) · user I/O 2 (button P-02, RGB P-03) · power 1 (battery
P-04) · motion 3 (DC P-05, encoder P-06, servo P-07) · thermal 1 (heater P-10) ·
storage 1 (P-11) · spare 1 (P-12) · serial data path 1 (UART P-09) — **total 13**.

> The board has **no free GPIO left**: all 20 are consumed by the peripherals
> above. That is why P-12 has to fly-lead TP20/TP21, and why examples never
> invent extra pins.

## 2. Running one

```bash
cd examples/periph/P-01_imu_lsm6ds3
idf.py set-target esp32s3
idf.py build                 # build only
run.bat COM5                 # Windows: build + flash + monitor
```

`run.bat` sets `IDF_PATH` / `IDF_PYTHON_ENV_PATH` itself when missing. Build one
project at a time — `idf.py build` and `idf.py reconfigure` share `build/` and
will produce bogus failures if run concurrently.

## 3. 数据源（默认合成，真板可切真实）

每个例程默认用**合成数据源**（`P_SIM_*` 宏），无板也能编译验证；接真板时切
真实数据源（`P_SIM_* = 0`）。电机 / 存储系的真实 HAL 未随本仓发布，切 `0` 会
在编译期显式拒绝（不会静默编出「假硬件」固件）。

```bash
idf.py -DP_SIM_IMU=0 build     # 例：P-01 切真实 I2C 数据源
```

## 4. Shared sources (`_shared/`)

Not a project. Compiled into the examples that need it: the ESP-IDF I2C endpoint,
flash `pstore` backend, and the synthetic sources (IMU, motor HAL). Changing a
file here affects every example — the built-in variant switches keep the
real/simulated boundary explicit.

## 5. Suggested routes

| You want | Path |
| --- | --- |
| Confirm the board is alive | P-00 → P-01 → P-02 → P-03 |
| Add your own I2C sensor | P-08 → P-01 (preset table) → P-09 (data out) |
| Make it move | P-05 → P-06 → P-07 |
| Keep parameters across power loss | P-11 |

## 6. Wire format

Examples that speak the serial protocol use `xiao_proto`; the byte layout is
defined once in [`src/proto/include/proto/frame.h`](../src/proto/include/proto/frame.h) — SOF/LEN/TYPE/SEQ/
PAYLOAD/CRC8, frame types `0x01`…`0x7F`. That header is the contract; do not
duplicate it in a doc or a sketch.

## 7. Verification status

Per-example detail (acceptance criteria, console anchors, what to change) lives
in each project's README. Build status (ESP-IDF v6.1, target esp32s3):

- `idf.py reconfigure` — **13/13 pass**（本轮删除算法例程后由 PM 复跑）
- full build verified — P-00, P-01, P-05, P-11
- remaining projects share the same template but were **not** individually built
  in the last release window; re-run before trusting a green checkmark
- **on-board electrical behaviour is not verified in this repository** — no
  hardware was attached. The "you should see" anchors are the contract to check
  yourself.