<!-- SPDX-License-Identifier: MIT -->
# imu_quickstart — ESP-IDF

Minimal ESP-IDF (v5.3+ / v6.x) project: streams six-axis samples from an
LSM6DS3TR-C over the console UART. Uses the library straight from this
repository — no component manager involved.

```bash
idf.py set-target esp32s3     # or keep the default from sdkconfig.defaults
idf.py build
idf.py -p COMx flash monitor
```

Wiring (XIAO ESP32-S3): SDA = GPIO5 (D4), SCL = GPIO6 (D5), IMU at
0x6A. Expected output (board at rest):

```
streaming: 104 Hz, +-2 g, +-250 dps
ax=+0.012 ay=-0.003 az=+0.987 g | gx=+0.4 gy=-0.2 gz=-0.6 dps
```

The whole point of this example is visible in `main/main.c`: no numeric
parameter is written in code — rates, ranges and the sampling period come
from the `imu.raw.104` preset. `main/bus_i2c_idf.c` is the platform glue:
it fills the library's bus contract (`imu_i2c_bus_t`) with the ESP-IDF
`i2c_master` driver, so the library core stays free of any platform header.
