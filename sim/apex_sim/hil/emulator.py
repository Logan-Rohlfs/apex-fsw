"""Sensor emulation — RocketPy flight state → SimPacket sensor fields.

Converts the true rigid-body state RocketPy exposes to its airbrake
controller callback into what the flight computer's sensors would actually
read, in the body frame and units the firmware expects (see
``fsw/src/hil.h``).

Frame conventions
-----------------
* RocketPy inertial frame: ENU (x east, y north, z up, z is altitude ASL).
* RocketPy body frame: z along the rocket axis toward the nose.
* Apex firmware body frame: **x axial toward the nose** (``accel_x ≈ +9.81``
  on the pad).  Mapping used here (right-handed, cyclic)::

      fsw_x = rp_z      fsw_y = rp_x      fsw_z = rp_y

Accelerometers measure *specific force* f = a − g, not coordinate
acceleration: on the pad they read +1 g up.  Coordinate acceleration is not
part of the RocketPy state vector, so it is reconstructed by finite
differencing velocity between consecutive controller calls (100 Hz).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from apex_sim.hil.protocol import SimSensors

_G = 9.80665  # m/s²

# Per-sample 1σ white Gaussian noise, from the HIL noise model tables in
# docs/sensors/*.md (the docs bless white-per-sample as sufficient for
# control-law validation). Disabled by default — deterministic loops are
# easier to debug; enable for realism runs.
_NOISE_ACCEL_MSS = 0.069     # ICM-45686: doc noise-model value (70 µg/√Hz)
_NOISE_GYRO_RADS = 0.0013    # ICM-45686: 3.8 mdps/√Hz × √(400 Hz BW)
_NOISE_BARO_PA = 0.32        # BMP581 at OSR×16 (recommended operating mode)
_NOISE_HIGHG_MSS = 0.049     # ADXL375: doc value, flagged "verify vs datasheet"
_NOISE_MAG_GAUSS = 0.0004    # MMC5983MA: 0.4 mG RMS
_NOISE_GPS_ALT_M = 3.0       # MAX-M10S vertical (~2× horizontal CEP)


def _quat_to_matrix(e0: float, e1: float, e2: float, e3: float) -> np.ndarray:
    """Rotation matrix (body → inertial) from a scalar-first quaternion."""
    n = math.sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3)
    if n < 1e-12:
        return np.eye(3)
    e0, e1, e2, e3 = e0 / n, e1 / n, e2 / n, e3 / n
    return np.array([
        [1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2)],
        [2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1)],
        [2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2)],
    ])


def rail_quaternion(inclination_deg: float, heading_deg: float) -> np.ndarray:
    """Scalar-first quaternion tilting the body z-axis onto the rail direction.

    Inclination is measured from horizontal (90° = vertical); heading is
    degrees east of north.  Roll about the rocket axis is left at zero.
    """
    inc = math.radians(inclination_deg)
    head = math.radians(heading_deg)
    axis_world = np.array([
        math.cos(inc) * math.sin(head),   # east
        math.cos(inc) * math.cos(head),   # north
        math.sin(inc),                    # up
    ])
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, axis_world))
    if c > 1.0 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    rot_axis = np.cross(z, axis_world)
    rot_axis /= np.linalg.norm(rot_axis)
    half = 0.5 * math.acos(max(-1.0, min(1.0, c)))
    return np.array([math.cos(half), *(math.sin(half) * rot_axis)])


class SensorEmulator:
    """Turn true flight state into the SimSensors the firmware ingests.

    Parameters
    ----------
    pressure_fn : callable
        ``pressure_fn(altitude_asl_m) -> Pa`` — pass ``env.pressure`` from
        the RocketPy Environment so baro readings match the sim atmosphere.
    pad_elevation_m : float
        Launch site ground elevation ASL.
    mag_declination_deg, mag_inclination_deg, mag_strength_ut : float
        Local geomagnetic field (site profile ``magnetic:`` block).
        Strength in microtesla; SimPacket wants Gauss (1 G = 100 µT).
    noise : bool
        Add representative Gaussian sensor noise.
    seed : int, optional
        RNG seed for reproducible noisy runs.
    """

    def __init__(self, pressure_fn, pad_elevation_m: float,
                 mag_declination_deg: float = 0.0,
                 mag_inclination_deg: float = 60.0,
                 mag_strength_ut: float = 48.0,
                 noise: bool = False,
                 seed: Optional[int] = None):
        self._pressure_fn = pressure_fn
        self.pad_elevation_m = pad_elevation_m
        self._noise = noise
        self._rng = np.random.default_rng(seed)

        # NED geomagnetic components → ENU world vector, in Gauss.
        dec = math.radians(mag_declination_deg)
        dip = math.radians(mag_inclination_deg)
        b = mag_strength_ut / 100.0
        north = b * math.cos(dip) * math.cos(dec)
        east = b * math.cos(dip) * math.sin(dec)
        down = b * math.sin(dip)
        self._mag_world = np.array([east, north, -down])

    # ── Internals ─────────────────────────────────────────────────────────────

    def _gauss(self, sigma: float) -> float:
        return float(self._rng.normal(0.0, sigma)) if self._noise else 0.0

    def _make(self, quat: np.ndarray, accel_world: np.ndarray,
              omega_body_rp: np.ndarray, alt_asl_m: float,
              gps_valid: bool = True) -> SimSensors:
        rot = _quat_to_matrix(*quat)           # body → inertial
        # Specific force: f = a − g, with g = (0, 0, −9.81) in ENU.
        f_world = accel_world + np.array([0.0, 0.0, _G])
        f_body = rot.T @ f_world               # RocketPy body frame
        mag_body = rot.T @ self._mag_world

        def fsw(v):
            # fsw(x, y, z) = rp(z, x, y) — firmware x is axial.
            return float(v[2]), float(v[0]), float(v[1])

        ax, ay, az = fsw(f_body)
        gx, gy, gz = fsw(omega_body_rp)
        mx, my, mz = fsw(mag_body)
        baro_pa = float(self._pressure_fn(alt_asl_m))

        return SimSensors(
            accel_x_mss=ax + self._gauss(_NOISE_ACCEL_MSS),
            accel_y_mss=ay + self._gauss(_NOISE_ACCEL_MSS),
            accel_z_mss=az + self._gauss(_NOISE_ACCEL_MSS),
            gyro_x_rads=gx + self._gauss(_NOISE_GYRO_RADS),
            gyro_y_rads=gy + self._gauss(_NOISE_GYRO_RADS),
            gyro_z_rads=gz + self._gauss(_NOISE_GYRO_RADS),
            baro_pa=baro_pa + self._gauss(_NOISE_BARO_PA),
            # ADXL375 mirrors the ICM channel (same convention as replay);
            # the firmware only switches to it above 14 g.
            highg_x_mss=ax + self._gauss(_NOISE_HIGHG_MSS),
            highg_y_mss=ay + self._gauss(_NOISE_HIGHG_MSS),
            highg_z_mss=az + self._gauss(_NOISE_HIGHG_MSS),
            mag_x_gauss=mx + self._gauss(_NOISE_MAG_GAUSS),
            mag_y_gauss=my + self._gauss(_NOISE_MAG_GAUSS),
            mag_z_gauss=mz + self._gauss(_NOISE_MAG_GAUSS),
            gps_alt_msl_m=(alt_asl_m + self._gauss(_NOISE_GPS_ALT_M))
            if gps_valid else float("nan"),
            gps_valid=1 if gps_valid else 0,
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def pad_sensors(self, rail_inclination_deg: float = 90.0,
                    rail_heading_deg: float = 0.0) -> SimSensors:
        """Stationary on-pad readings, nose along the rail direction."""
        quat = rail_quaternion(rail_inclination_deg, rail_heading_deg)
        return self._make(quat, np.zeros(3), np.zeros(3), self.pad_elevation_m)

    def flight_sensors(self, state, accel_world: np.ndarray) -> SimSensors:
        """Readings from a RocketPy 13-element state vector.

        ``state`` is ``[x, y, z, vx, vy, vz, e0, e1, e2, e3, w1, w2, w3]``
        as passed to the airbrake controller.  ``accel_world`` is the
        finite-differenced coordinate acceleration in the inertial frame.
        """
        quat = np.asarray(state[6:10], dtype=float)
        omega = np.asarray(state[10:13], dtype=float)
        return self._make(quat, np.asarray(accel_world, dtype=float),
                          omega, float(state[2]))
