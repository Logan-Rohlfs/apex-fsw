# ICM-45686 — 6-Axis IMU (Accelerometer + Gyroscope)

**Manufacturer:** TDK InvenSense
**Datasheet:** [DS-000577](https://www.mouser.com/catalog/specsheets/TDK_DS_000577_ICM_45686.pdf)
**Product page:** [invensense.tdk.com](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/)

---

## Overview

The ICM-45686 is a high-performance 6-axis MEMS IMU combining a 3-axis accelerometer and 3-axis gyroscope. It is TDK's flagship low-noise 6-axis device, featuring their proprietary BalancedGyro™ architecture for superior vibration rejection. On Apex it is the primary inertial reference for attitude estimation, state machine transitions (launch detect, burnout, coast), and as feedback into the airbrake control law.

---

## Key Specifications

### Accelerometer

| Parameter | Value |
|---|---|
| Full-scale range | ±2 / ±4 / ±8 / ±16 / ±32 g (selectable) |
| Noise density | 70 µg/√Hz |
| Temperature stability | ±0.15 mg/°C |
| Resolution | 20-bit |

### Gyroscope

| Parameter | Value |
|---|---|
| Full-scale range | ±15.625 / ±31.25 / ±62.5 / ±125 / ±250 / ±500 / ±1000 / ±2000 / ±4000 dps (selectable) |
| Noise density | 3.8 mdps/√Hz |
| Temperature stability | ±0.005 °/s/°C |
| Architecture | BalancedGyro™ |

### General

| Parameter | Value |
|---|---|
| Max ODR | 6400 Hz (accel), 6400 Hz (gyro) |
| Interface | SPI, I²C, I3C |
| Supply voltage | 1.71 – 3.6 V |
| Current (low noise, 6-axis) | 0.42 mA |
| Current (low power, 6-axis) | 0.22 mA |
| Operating temperature | −40 – +85 °C |
| Package | LGA-14 (2.5 × 3.0 × 0.86 mm) |

---

## HIL Simulation Parameters

These values should be reflected in `config/sensors.yaml` under `imu_low_g`:

| Config Field | Value | Source |
|---|---|---|
| `accel_noise_stddev_mss` | 0.069 | 70 µg/√Hz × √(ODR/2) at 1 kHz → ≈ 2.2 mg RMS; use noise density directly in sim |
| `accel_bias_mss` | [0.0, 0.0, 0.0] | Calibrate from bench |
| `gyro_noise_stddev_rads` | 0.000066 | 3.8 mdps/√Hz converted to rad/s/√Hz |
| `gyro_bias_rads` | [0.0, 0.0, 0.0] | Calibrate from bench |
| `update_rate_hz` | 1000.0 | Typical flight use; sensor supports up to 6400 Hz |
| `accel_fsr_g` | 32 | Full range for launch/burnout; can switch to ±16 g during coast |
| `gyro_fsr_dps` | 2000 | Typical for a rocket with limited spin |

> For the HIL noise model, apply noise as white Gaussian noise with the above standard deviations per axis per sample. A more accurate model would apply the noise density and integrate over the sensor bandwidth, but white noise per sample is sufficient for control law validation.

---

## Notes & Quirks

- **BalancedGyro™ architecture:** The balanced design physically cancels vibration-induced gyro error by using two mechanically anti-phase resonators. This makes the ICM-45686 significantly more resistant to motor vibration than previous InvenSense parts (e.g. ICM-42688-P). This is a meaningful advantage during the boost phase.

- **I3C support:** The ICM-45686 supports I3C in addition to SPI/I2C. Unless the Teensy 4.1 has I3C support (it does not natively), use SPI for the highest data rate and lowest latency.

- **Dual interface (UI + OIS):** The ICM-45686 has two separate serial interfaces — the primary User Interface (UI) and an OIS (Optical Image Stabilization) interface. The OIS interface can output raw IMU data at very high rates to a secondary host. This could be used for a secondary logging path but is unnecessary for the primary flight computer use case.

- **APEX motion functions:** The on-chip pedometer, tap detection, free-fall, and wake-on-motion functions are consumer-focused and not useful for rocketry. Disable them to avoid any interference with raw data output.

- **FSR switching during flight:** The ±32 g range has lower resolution than ±16 g. Consider whether the flight computer switches FSR between boost and coast phases, and ensure the HIL sim reflects the active range when computing expected ADC counts.

- **Gyro startup time:** The gyroscope requires a settling time after power-on before it outputs valid data. The datasheet specifies this — account for it in the boot sequence and do not process gyro data until the sensor flags ready.
