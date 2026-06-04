# Seymour TX Test Flight — 2026-05-24

Test flight for PR3. No active airbrake control. Both the TeleMega and Blue Raven
were flying. This data is the primary validation dataset for the RocketPy baseline sim.

## Files

| File | Instrument | Contents |
|---|---|---|
| `telemega_flight_2026-05-24.csv` | TeleMega (s/n 16370, KJ5LDI) | Full time-series: baro altitude, GPS, IMU (accel/gyro/mag), speed, state |
| `blueraven_flight_2026-05-24.csv` | Blue Raven | 50 Hz time-series: baro altitude, inertial nav, velocity, tilt, events |
| `blueraven_summary_2026-05-24.csv` | Blue Raven | Single-row summary of key flight statistics |
| `blueraven_deploy_config_2026-05-24.xlsx` | Blue Raven | Pyro channel firing logic configuration for this flight |
| `conditions.yaml` | — | Launch site, atmosphere, and measured outcomes for sim comparison |

## Key Outcomes

| Metric | Blue Raven (baro) | Blue Raven (inertial) | TeleMega |
|---|---|---|---|
| Max altitude AGL | 10,523 ft (3,208 m) | 11,338 ft (3,456 m) | TBD |
| Max velocity | 930 ft/s (284 m/s) | — | TBD |
| Time to burnout | 3.6 s | — | TBD |
| Time to apogee | 28.4 s | — | TBD |

OpenRocket predicted **11,310 ft (3,448 m)** — within 28 ft (0.25%) of the Blue Raven
inertial reading. Baro reading is ~815 ft lower, likely due to dynamic pressure effects
on the sensor during the high-velocity coast phase.

## Notes

- Rail angle: 1.0°, tilt at burnout: 5.0°, roll rate at burnout: 527 deg/s
- TeleMega GPS lat/lon: 33.4986968, -99.3329858 (pad position, Seymour TX)
- Pad altitude: 1,199 ft (365.5 m) ASL — note MATLAB used 393 m (1,289 ft), ~27 m discrepancy
- Temperature on pad: 89.2°F (31.8°C)
- Motor: N3355 static fire data (.eng) from same day
