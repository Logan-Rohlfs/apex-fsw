# Apex — HIL Simulation

Hardware-in-the-loop and software-in-the-loop simulation for the Apex flight computer.
The laptop side of the HIL link: runs a RocketPy physics simulation, injects simulated
sensor data into the Teensy over USB serial, and reads back actuator commands to close
the loop.

---

## Repository Layout

```
apex/sim/
├── apex_sim/          # installable Python package
│   ├── sensors/       # sensor noise and bias models
│   ├── sim/           # RocketPy rocket and flight wrappers
│   ├── hil/           # serial protocol and hardware interface
│   ├── analysis/      # logging and plotting
│   └── config/        # YAML config loader
├── config/            # parameter files (rocket, sensors, airbrakes, environment)
├── scripts/           # run_hil.py, run_sil.py entry points
├── tests/             # unit test suite
└── docs/
    └── sensors/       # per-sensor spec sheets and HIL parameters
```

---

## Requirements

> *Not yet implemented — fill in once `pyproject.toml` is complete.*

---

## Installation

> *Not yet implemented.*

---

## Configuration

All physical parameters live in `config/`. Edit these files to match the rocket —
no code changes required for a new vehicle or launch site.

| File | Contents |
|---|---|
| `config/rocket.yaml` | Mass, geometry, motor designation |
| `config/sensors.yaml` | Noise, bias, and ODR for each sensor |
| `config/airbrakes.yaml` | Drag coefficients, servo travel, PWM range |
| `config/environment.yaml` | Launch site coordinates, elevation, wind model |
| `config/hil.yaml` | Serial port, baud rate, packet rate, timeouts |

> *Config files not yet created — see `docs/sensors/` for the parameter values that will populate `sensors.yaml`.*

---

## Running the Simulation

### HIL Mode (Teensy connected)

> *Not yet implemented.*

### SIL Mode (no hardware)

> *Not yet implemented.*

---

## Sensor Models

Noise and bias parameters for each sensor are documented in `docs/sensors/`.
Values in those files should be used to populate `config/sensors.yaml`.

| Sensor | Part | Doc |
|---|---|---|
| Barometer | BMP581 | [docs/sensors/bmp581_barometer.md](docs/sensors/bmp581_barometer.md) |
| 6-DOF IMU | ICM-45686 | [docs/sensors/icm45686_imu.md](docs/sensors/icm45686_imu.md) |
| Magnetometer | MMC5983MA | [docs/sensors/mmc5983ma_magnetometer.md](docs/sensors/mmc5983ma_magnetometer.md) |
| High-G Accelerometer | ADXL375 | [docs/sensors/adxl375_high_g_accelerometer.md](docs/sensors/adxl375_high_g_accelerometer.md) |
| GPS | MAX-M10S | [docs/sensors/max_m10s_gps.md](docs/sensors/max_m10s_gps.md) |
| Radio | RF4463PRO-433 | [docs/sensors/rf4463pro_433_radio.md](docs/sensors/rf4463pro_433_radio.md) |

---

## Serial Protocol

> *Not yet defined — see `apex_sim/hil/protocol.py` once implemented.*

The protocol spec will also be documented in `../interface/protocol_spec.md` at the
repo root as the shared contract between this package and the firmware.

---

## Testing

> *Not yet implemented.*

---

## Contributing

Follow PEP 8. Use NumPy-style docstrings. All public classes and functions must have
a docstring. Run the linter and test suite before opening a PR.
