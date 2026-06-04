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
│   ├── config/        # YAML config loader (load_environment, SiteProfile, ...)
│   ├── sensors/       # sensor noise and bias models (not yet implemented)
│   ├── sim/           # RocketPy wrappers — environment, rocket, airbrakes, flight
│   ├── hil/           # serial protocol and hardware interface (not yet implemented)
│   └── analysis/      # logging and plotting (not yet implemented)
├── config/            # parameter files — edit these, not the Python
│   ├── environment.yaml    # atmosphere model, rail, active_site pointer
│   ├── rocket.yaml         # mass, geometry, motor, recovery
│   ├── airbrakes.yaml      # Cd values (CFD), servo travel, PID gains
│   └── sites/
│       ├── irec_2026_pecos_tx.yaml        # competition site (active by default)
│       └── seymour_tx_2026_05_24.yaml     # Seymour TX test flight — use for validation
├── data/
│   ├── flights/seymour_2026_05_24/        # TeleMega + Blue Raven flight recordings
│   ├── motor/                              # N3355 static fire .eng thrust curve
│   └── openrocket/                         # Team307 PR3 .ork design file
├── docs/sensors/      # per-sensor spec sheets and HIL noise parameters
├── scripts/           # runnable entry points (see below)
└── tests/
```

---

## Setup

From the repo root, run once to create the virtualenv and install dependencies:

```
source apex-setup
```

After that, `apex-setup` is always available as a command. Re-run it any time to sync
dependencies. Source it (rather than run it) to activate the venv in your shell.

---

## Configuration

All physical parameters live in `config/`. No code changes are required to swap a
launch site or update a Cd value.

| File | Contents |
|---|---|
| `config/environment.yaml` | Active site pointer, atmosphere model selection, rail |
| `config/rocket.yaml` | Mass, geometry, fins, motor, recovery system |
| `config/airbrakes.yaml` | Drag coefficients (CFD), servo travel, PID gains |
| `config/sites/*.yaml` | One file per launch site — coordinates, elevation, launch time |

To switch between the competition site and the Seymour TX validation site, change one
line in `config/environment.yaml`:

```yaml
active_site: "irec_2026_pecos_tx"        # competition
active_site: "seymour_tx_2026_05_24"     # Seymour TX test flight — for validation
```

---

## Scripts

Run all scripts from `apex/sim/` with the venv active (`source apex-setup`).

### `scripts/run_sim.py` — flight simulation

Runs a full RocketPy flight simulation and prints key results.

```
python scripts/run_sim.py [options]
```

| Flag | Description |
|---|---|
| *(none)* | Competition sim — IREC Pecos TX, PID airbrakes active |
| `--baseline` | Clean flight, no airbrakes — use for validating against real data |
| `--site PROFILE` | Override the active site (e.g. `seymour_tx_2026_05_24`) |
| `--model MODEL` | Override atmosphere model: `RAP`, `NAM`, `GFS`, `standard_atmosphere` |
| `--full-descent` | Simulate through landing (default: stop at apogee) |
| `--save` | Write summary CSV and plots to `output/` |
| `--verbose` | Print RocketPy integration progress |

**Examples:**

```bash
# Competition sim
python scripts/run_sim.py

# Validate against Seymour TX flight — standard atmosphere (fast, ~4.5% low vs real data)
python scripts/run_sim.py --baseline --site seymour_tx_2026_05_24

# Same validation with ERA5 historical weather (accurate, requires Copernicus CDS key)
python scripts/run_sim.py --baseline --site seymour_tx_2026_05_24 --model ERA5
```

**Atmosphere note:** NOAA's OpenDAP service (RAP/NAM/GFS) was retired in 2025.
Forecast sims will fall back to `standard_atmosphere` until RocketPy adds support for
the replacement service. For near-launch sims, force the model explicitly with
`--model standard_atmosphere`. For historical validation, use ERA5.

---

## Sensor Models

Noise and bias parameters for each sensor are documented in `docs/sensors/`.

| Sensor | Part | Doc |
|---|---|---|
| Barometer | BMP581 | [docs/sensors/bmp581_barometer.md](docs/sensors/bmp581_barometer.md) |
| 6-DOF IMU | ICM-45686 | [docs/sensors/icm45686_imu.md](docs/sensors/icm45686_imu.md) |
| Magnetometer | MMC5983MA | [docs/sensors/mmc5983ma_magnetometer.md](docs/sensors/mmc5983ma_magnetometer.md) |
| High-G Accelerometer | ADXL375 | [docs/sensors/adxl375_high_g_accelerometer.md](docs/sensors/adxl375_high_g_accelerometer.md) |
| GPS | MAX-M10S | [docs/sensors/max_m10s_gps.md](docs/sensors/max_m10s_gps.md) |
| Radio | RF4463PRO-433 | [docs/sensors/rf4463pro_433_radio.md](docs/sensors/rf4463pro_433_radio.md) |

---

## HIL Mode

> *Not yet implemented — serial protocol definition pending.*

The HIL loop will be started with `scripts/run_hil.py` (not yet created).
The serial protocol spec lives in `apex_sim/hil/` once defined.

---

## Testing

```bash
python -m pytest tests/
```

> *Test suite not yet implemented.*

---

## Contributing

Follow PEP 8. Use NumPy-style docstrings. All public classes and functions must have
a docstring. Run the linter and test suite before opening a PR.
