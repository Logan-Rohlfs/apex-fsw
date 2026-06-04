# MAX-M10S — GNSS Receiver Module

**Manufacturer:** u-blox
**Datasheet:** [MAX-M10S Data Sheet UBX-20035208](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf)
**Integration Manual:** [UBX-20053088](https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf)

---

## Overview

The MAX-M10S is a u-blox M10 standard precision GNSS module in a compact LCC form factor. It supports concurrent reception of GPS, Galileo, BeiDou B1I, QZSS, and SBAS. On Apex it provides absolute position (latitude, longitude, altitude) and velocity for post-flight trajectory reconstruction and as a secondary altitude reference. GPS update rates are too slow for real-time airbrake control but are useful for telemetry and recovery.

---

## Key Specifications

| Parameter | Value |
|---|---|
| Position accuracy (CEP) | 1.5 m |
| Velocity accuracy | 0.05 m/s |
| Max navigation update rate | 18 Hz (default config) |
| Max update rate (high performance) | 25 Hz |
| Cold start TTFF | 29 s |
| Hot start TTFF | 1 s |
| Concurrent GNSS constellations | GPS + Galileo + BeiDou + QZSS + SBAS |
| Interface | UART, I²C, SPI |
| Supply voltage | 1.71 – 1.89 V (VCC); 1.71 – 3.6 V (V_BCKP) |
| Current (continuous tracking) | ~25 mW |
| Operating temperature | −40 – +85 °C |
| Package | LCC-18 (9.6 × 14.0 × 2.4 mm) |

---

## HIL Simulation Parameters

These values should be reflected in `config/sensors.yaml` under `gps`:

| Config Field | Value | Source |
|---|---|---|
| `position_noise_stddev_m` | 1.5 | CEP → 1-sigma (CEP ≈ 1.18σ for 2D Gaussian) |
| `altitude_noise_stddev_m` | 3.0 | Vertical accuracy ~2× horizontal is typical for GNSS |
| `velocity_noise_stddev_ms` | 0.05 | Datasheet velocity accuracy |
| `update_rate_hz` | 10.0 | Conservative default; increase if needed |
| `cold_start_delay_s` | 29.0 | Set to 0 if simulating a hot start (board was recently powered) |
| `fix_delay_s` | 29.0 | Same as cold start — no GPS output until fix is acquired |

> GPS is not used for real-time airbrake control. The HIL sim primarily uses GPS for telemetry validation and recovery system testing. The `fix_delay_s` parameter is important for testing the flight computer's behavior before GPS lock is acquired.

---

## Notes & Quirks

- **1.8 V VCC:** The MAX-M10S requires a 1.71–1.89 V supply on VCC. This is lower than the typical 3.3 V system rail and requires a dedicated LDO or level shifting. Unlike many modules, this is the bare chip — if your PCB routes 3.3 V directly to VCC it will damage the part.

- **UART vs. SPI vs. I²C:** The module supports all three interfaces but UART is the most commonly used and best-supported by u-blox's UBX protocol. SPI is available if UART pins are scarce. I²C is available but u-blox recommends against it for high-rate applications.

- **UBX binary protocol:** u-blox's UBX binary protocol is more efficient and provides richer data than NMEA strings. For a flight computer, parse UBX NAV-PVT messages which provide position, velocity, and fix quality in a single packet. Avoid NMEA if possible — it's ASCII-heavy and slower to parse on an embedded system.

- **Altitude datum:** The MAX-M10S reports altitude above the WGS84 ellipsoid, not above mean sea level (MSL). The difference (geoid undulation) can be several meters to tens of meters depending on launch site. Ensure the firmware uses the correct altitude reference for apogee detection, or compensate using the module's built-in geoid model (available via UBX-NAV-PVT `hMSL` field).

- **Velocity during ascent:** GPS velocity is derived from Doppler shift and is typically more accurate than position-derived velocity. For a rocket ascending at 100+ m/s, GPS velocity can be a useful cross-check on the barometer-derived velocity estimate.

- **Fix loss during high-G burn:** Some GNSS receivers lose fix during high-acceleration phases due to oscillator stress. The MAX-M10S has not been characterized for this specifically — assume fix may be lost during boost and verify during post-flight log analysis.

- **Backup power (V_BCKP):** Connecting V_BCKP to a small capacitor or coin cell allows the module to retain almanac and ephemeris data between power cycles, dramatically reducing TTFF to hot-start times. Recommended for a competition rocket where pre-launch power cycling is common.
