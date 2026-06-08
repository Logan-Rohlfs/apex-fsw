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
│   └── analysis/      # flight comparison — scalar table, trace plots, OR .ork parser
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
| `--compare` | Baseline against Seymour TX site; prints scalar table and saves trace plot to `output/comparison.png` |
| `--run-or` | Also run a live OpenRocket sim with the same conditions as RocketPy (requires Java + OR JAR — see below) |
| `--or-rod-angle DEG` | Launch rod angle from vertical for the OR sim (e.g. `15.33` for observed Seymour TX weathercock) |
| `--site PROFILE` | Override the active site (e.g. `seymour_tx_2026_05_24`) |
| `--model MODEL` | Override atmosphere model: `RAP`, `NAM`, `GFS`, `standard_atmosphere` |
| `--full-descent` | Simulate through landing (default: stop at apogee) |
| `--save` | Write summary CSV and plots to `output/` |
| `--verbose` | Print RocketPy integration progress |

**Examples:**

```bash
# Competition sim
python scripts/run_sim.py

# Validate against Seymour TX flight — standard atmosphere (fast, ~2.5% high vs real data)
python scripts/run_sim.py --baseline --site seymour_tx_2026_05_24

# Full comparison: scalar table + trace plot vs TeleMega, Blue Raven, and stored OR trace
python scripts/run_sim.py --compare

# Same but also run a live OR sim with matched conditions (needs Java + OR JAR)
python scripts/run_sim.py --compare --run-or

# Live OR with Seymour TX weathercock angle (15.33° observed at t=1.12s)
python scripts/run_sim.py --compare --run-or --or-rod-angle 15.33

# Same with ERA5 historical weather (accurate, requires Copernicus CDS key)
python scripts/run_sim.py --compare --model ERA5
```

**`--compare` output:**

The scalar table shows RocketPy alongside TeleMega (baro), Blue Raven (baro and
inertial), and OpenRocket — all read live from the data files in `data/`.  The trace
plot (`output/comparison.png`) overlays altitude AGL and speed vs time for all sources
through apogee.

Without `--run-or`, OR data is parsed directly from the stored simulation inside
`data/openrocket/Team307_TexasTechUniversity_PR3.ork` — no separate OpenRocket install
needed.

**Live OpenRocket setup (`--run-or`):**

1. Install Java: `brew install --cask temurin`
2. Download OR JAR:
   ```
   curl -L -o data/openrocket/OpenRocket-23.09.jar \
     https://github.com/openrocket/openrocket/releases/download/release-23.09/OpenRocket-23.09.jar
   ```
3. Run: `python scripts/run_sim.py --compare --run-or`

The live OR run uses the same `.ork` rocket file but overrides conditions (lat, lon,
elevation, wind, launch rod angle) to match the RocketPy config exactly.

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

Two HIL modes exist:

**Replay (implemented)** — plays a recorded Telemega CSV back to the Teensy:

```bash
python scripts/run_replay.py --port /dev/cu.usbmodem<N>
python scripts/run_replay.py  # auto-detects Teensy
```

See `scripts/run_replay.py` for full options (`--csv`, `--pre-pad`, `--speed`, `--out`).
Requires a Teensy flashed with the HIL build (`pio run -e teensy41_hil -t upload`).

**Real-time HIL (not yet implemented)** — closes the loop between a live RocketPy
simulation and the Teensy so `deployment_frac` feeds back into airbrake drag.
Entry point will be `scripts/run_hil.py`. Python-side serial framing lives in
`apex_sim/hil/` once built.

The serial protocol (packet structs, CRC-8, magic bytes, baud) is fully defined on the
Teensy side in `fsw/src/hil.h`.

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
