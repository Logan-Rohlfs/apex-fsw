# BMP581 — Barometric Pressure Sensor

**Manufacturer:** Bosch Sensortec
**Datasheet:** [BST-BMP581-DS004](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf)

---

## Overview

The BMP581 is a 24-bit absolute barometric pressure and temperature sensor. It is the successor to the BMP390 and represents Bosch's highest-performance consumer barometer. On Apex it is the primary altitude source used for apogee detection, state transitions, and airbrake control input.

---

## Key Specifications

| Parameter | Value |
|---|---|
| Pressure range | 30 – 125 kPa |
| Absolute accuracy | ±0.3 hPa |
| Relative accuracy | ±0.06 hPa / 10 kPa |
| RMS noise (highest OSR) | < 0.1 Pa |
| Resolution | 24-bit |
| Max ODR (normal mode) | 240 Hz |
| Max ODR (forced mode) | > 240 Hz |
| Interface | SPI (4-wire), I²C |
| Supply voltage | 1.71 – 3.6 V |
| Operating temperature | −40 – +85 °C |
| Package | LGA-8 (2.0 × 2.0 × 0.75 mm) |

### Oversampling vs. Noise Trade-off

| OSR Setting | Pressure Noise (RMS) | Current Draw |
|---|---|---|
| OSR x1 | ~1.3 Pa | ~700 µA |
| OSR x4 | ~0.65 Pa | ~1.4 mA |
| OSR x16 | ~0.32 Pa | ~5.6 mA |
| OSR x128 | ~0.11 Pa | ~44 mA |

---

## HIL Simulation Parameters

These values should be reflected in `config/sensors.yaml` under `barometer`:

| Config Field | Value | Source |
|---|---|---|
| `noise_stddev_pa` | 0.32 | OSR x16 RMS noise (recommended operating mode) |
| `bias_pa` | 0.0 | Calibrate from bench measurement |
| `update_rate_hz` | 50.0 | Typical flight computer polling rate |
| `quantization_pa` | 0.0244 | 24-bit over 30–125 kPa range |

> Altitude is derived from pressure in the simulation using the ISA atmosphere model.
> 1 Pa ≈ 0.083 m altitude error near sea level.

---

## Notes & Quirks

- **IIR filter:** The BMP581 has a configurable on-chip IIR filter (coefficients 0–127). At high coefficients it heavily smooths transient pressure spikes but introduces group delay. For a rocket with fast altitude changes this can lag behind true apogee — keep the IIR coefficient low or disabled and filter in software if needed.

- **Forced mode vs. normal mode:** In normal mode the sensor runs autonomously at the configured ODR. In forced mode a measurement is triggered on demand, allowing ODRs higher than 240 Hz. If the flight computer uses forced mode, the HIL sim should deliver sensor packets at the same triggered rate to avoid desync.

- **Temperature co-measurement:** Every pressure reading includes a temperature measurement used for internal compensation. The temperature output can also be used by the flight computer as a secondary temperature source.

- **Pressure vs. altitude nonlinearity:** The ISA model used to convert pressure to altitude is only accurate to within ~1% at altitudes relevant to IREC (3,000–10,000 m). For higher fidelity, use the full hypsometric formula with launch-site QFE.

- **Vibration sensitivity:** The BMP581 is a MEMS pressure sensor and can be affected by acoustic and structural vibration during motor burn. Consider applying a software low-pass filter on pressure readings during the boost phase.
