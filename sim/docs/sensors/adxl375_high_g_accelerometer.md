# ADXL375 — High-G 3-Axis Accelerometer

**Manufacturer:** Analog Devices
**Part:** ADXL375BCCZ-RL (RoHS compliant, tape-and-reel)
**Datasheet:** [ADXL375 Rev. B](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.PDF)

---

## Overview

The ADXL375 is a 3-axis MEMS digital accelerometer with a fixed ±200 g full-scale range, designed specifically for high-shock applications. On Apex it serves as the primary accelerometer during motor burn, where the ICM-45686 will saturate at ±32 g. It provides reliable axial acceleration data for burnout detection, dynamic pressure estimation, and as a redundant launch detect source. The device survives 10,000 g shock events.

---

## Key Specifications

| Parameter | Value |
|---|---|
| Full-scale range | ±200 g (fixed) |
| Sensitivity | 49 mg/LSB (typical) |
| Scale factor range | 44 – 54 mg/LSB |
| Resolution | 13-bit at ODR ≤ 800 Hz |
| Resolution | 12-bit at ODR 1600 / 3200 Hz |
| Scale factor at high ODR | ~98 mg/LSB (12-bit mode) |
| Noise density | ~5 mg/√Hz |
| Max ODR | 3200 Hz |
| FIFO depth | 32 levels |
| Shock survival | 10,000 g |
| Interface | SPI (4-wire), I²C |
| Supply voltage | 2.0 – 3.6 V |
| Current (measurement mode) | 35 µA |
| Current (standby) | 0.1 µA |
| Operating temperature | −40 – +85 °C |
| Package | LFCSP-14 (3.0 × 3.0 × 1.06 mm) |

> **⚠️ Noise density note:** The 5 mg/√Hz value is from a secondary source. Verify against Table 1 in the official datasheet before using in the noise model — the actual value may be lower (~0.3–0.6 mg/√Hz based on ADI part family conventions). Update `config/sensors.yaml` accordingly after verification.

---

## HIL Simulation Parameters

These values should be reflected in `config/sensors.yaml` under `imu_high_g`:

| Config Field | Value | Source |
|---|---|---|
| `noise_stddev_mss` | 0.049 | ~5 mg/√Hz × √(ODR/2) — **verify from datasheet** |
| `accel_bias_mss` | [0.0, 0.0, 0.0] | Calibrate from bench |
| `update_rate_hz` | 800.0 | Highest 13-bit ODR; use 3200 Hz only if 12-bit is acceptable |
| `clip_mss` | 1962.0 | ±200 g × 9.81 m/s² |
| `scale_factor_mss_per_lsb` | 0.481 | 49 mg/LSB × 9.81 m/s²/g |

---

## Notes & Quirks

- **Fixed ±200 g range:** Unlike the ICM-45686 where FSR is configurable, the ADXL375 is always ±200 g. There is no way to increase resolution by reducing range. This is intentional — the part is designed as a dedicated high-shock sensor, not a general-purpose accelerometer.

- **Resolution degrades at high ODR:** At ODR ≤ 800 Hz the output is 13-bit. At 1600 Hz and 3200 Hz it drops to 12-bit, effectively doubling the scale factor. If the firmware uses high ODR, the HIL sim must use the correct scale factor for that ODR setting.

- **I²C address conflict risk:** The ADXL375 has a configurable I²C address (0x1D or 0x53 via the ALT ADDRESS pin). If sharing an I²C bus with other sensors, verify no address collisions. Using SPI is strongly recommended for this sensor to avoid bus contention and to achieve higher data rates.

- **FIFO usage:** The 32-level FIFO allows the flight computer to burst-read accumulated samples rather than polling at the full ODR. For a 100 Hz flight loop reading a 3200 Hz sensor, the FIFO should be used to avoid dropped samples. Ensure the firmware drains the FIFO before it overflows.

- **Self-test:** The ADXL375 supports a self-test function that applies an electrostatic force to the sensing elements. This should be run during pre-flight checks to verify the sensor is functional before launch.

- **BCCZ vs. plain ADXL375:** The BCCZ-RL suffix denotes RoHS compliant material set and tape-and-reel packaging. Electrically and functionally identical to the base part.
