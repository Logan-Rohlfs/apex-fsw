# MATLAB Reference Files

These files are the original 1D control system design and Monte Carlo analysis.
They are **not** the source of truth for flight parameters — they are a snapshot
of the control system state as of April 23, 2026 and were tuned against the
Seymour, TX test flight conditions, not IREC.

| File | Purpose |
|---|---|
| `rocket_sim_4_23_26.m` | Single-run 1D airbrake simulation with selectable controller (BangBang / PID / Energy Management / Clean Flight) |
| `monte_carlo_4_23_2026.m` | 1000-run Monte Carlo over the same sim to find PID and Energy Management gains |

## Parameters extracted into `config/airbrakes.yaml`

Values from these files that were used to seed the RocketPy config.
Update `airbrakes.yaml` when gains are re-tuned.

| Parameter | MATLAB value | Notes |
|---|---|---|
| Rocket weight | 67.03 lbf (30.4 kg) | Used for mass in physics engine |
| Reference area | 0.2045143 ft² (0.01900 m²) | Cross-sectional area |
| Cd clean | 0.576 | No brake deployment |
| Cd full brakes | 0.900 | Full deployment |
| Airbrake full-travel time | 0.24 s | 0% → 100% deployment |
| Target apogee | 10,000 ft (3,048 m) AGL | IREC competition target |
| PID Kp / Ki / Kd | 0.4 / −0.004 / −0.04 | Output range [−15, 30] |
| Energy Mgmt Kp / Ki / Kd | 0.5 / 0.004 / 0.004 | Output range [0, 0.05] |

## Test flight initial conditions (Seymour, TX — do not use for IREC)

These are the burnout conditions the sim was tuned against.
IREC burnout conditions will differ and must come from OpenRocket.

| Parameter | Value |
|---|---|
| Launch site elevation | 393 m (1289 ft) MSL |
| Burnout velocity | 1015.66 ft/s (309.6 m/s) |
| Burnout altitude (AGL) | 1699.96 ft (518.2 m) |
