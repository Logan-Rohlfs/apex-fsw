# MMC5983MA — 3-Axis Magnetometer

**Manufacturer:** MEMSIC
**Datasheet:** [MMC5983MA Rev A](https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5983MADatasheetRevA.pdf)

---

## Overview

The MMC5983MA is an 18-bit anisotropic magnetoresistive (AMR) sensor that measures the 3-axis magnetic field vector. On Apex it is used for heading estimation and complementary filtering alongside the gyroscope to track the rocket's orientation. At IREC altitudes and velocities, heading drift from gyro integration is significant — the magnetometer provides the absolute reference needed to bound that drift.

---

## Key Specifications

| Parameter | Value |
|---|---|
| Full-scale range | ±8 Gauss |
| Resolution | 18-bit (0.0625 mG/LSB) |
| RMS noise | 0.4 mG |
| Heading accuracy | ±0.5° |
| Max ODR | 1000 Hz |
| Interface | SPI (4-wire), I²C (up to 400 kHz) |
| Supply voltage | 1.71 – 3.6 V |
| Operating temperature | −40 – +105 °C |
| Package | LGA-8 (3.0 × 3.0 × 1.0 mm) |
| Qualification | AEC-Q100 (automotive grade) |

---

## HIL Simulation Parameters

These values should be reflected in `config/sensors.yaml` under `magnetometer`:

| Config Field | Value | Source |
|---|---|---|
| `noise_stddev_ut` | 0.04 | 0.4 mG RMS converted to µT (1 mG = 0.1 µT) |
| `hard_iron_bias_ut` | [0.0, 0.0, 0.0] | Calibrate via figure-8 routine before flight |
| `update_rate_hz` | 100.0 | Sufficient for attitude estimation; saves power vs. 1000 Hz |
| `field_strength_ut` | 50.0 | Approximate at IREC launch site (New Mexico ~50-55 µT); verify with IGRF model |

> The local field strength and declination angle should be updated for each launch site using the [NOAA IGRF calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).

---

## Notes & Quirks

- **SET/RESET degaussing:** The AMR sensing elements can become magnetized by strong external fields (e.g. the rocket motor or nearby electronics). The MMC5983MA has built-in SET and RESET strobe circuitry that flips the magnetization of the sensing film to remove residual magnetization. This must be called periodically in firmware (typically every N measurements). Failing to do so causes progressive offset drift. The datasheet provides a recommended SET/RESET scheduling strategy.

- **Hard-iron vs. soft-iron calibration:** Hard-iron distortion (constant offsets from nearby permanent magnets — batteries, motor magnets) must be calibrated before each flight. Soft-iron distortion (field shape distortion from ferromagnetic structures) requires a more complex calibration matrix. At minimum, always perform hard-iron calibration. The HIL sim uses `hard_iron_bias_ut` to replicate uncalibrated offset for testing the calibration routine.

- **18-bit output assembly:** The MMC5983MA output is spread across multiple registers, including 2 high-resolution bits stored in a shared register for all three axes. Ensure the driver assembles the full 18-bit value correctly — a common bug is reading only the 16 MSBs, which gives correct values but discards resolution.

- **Magnetic interference from avionics:** Switching regulators, the radio PA, and any relay or solenoid on the board will cause magnetic interference. PCB layout should maximize distance between the magnetometer and these sources. This is worth noting in HIL testing — if the sim shows heading drift that doesn't match the real hardware, magnetic interference is a likely cause.

- **Orientation in rocket body frame:** The magnetometer axes must be aligned to the rocket body frame in firmware. Verify the chip orientation on the PCB against the datasheet axis diagram and apply the correct rotation matrix.
