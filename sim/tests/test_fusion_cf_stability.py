"""Complementary-filter stability guard for the HIL dt decision.

The firmware's altitude/velocity complementary filter (fsw/src/fusion.cpp) is
an alpha-beta tracker:

    vel_pred = vel + accel*dt
    alt_pred = alt + vel*dt
    err      = clamp(baro_agl - alt_pred, +-CF_ALT_ERR_CLAMP_M)
    alt      = alt_pred + alpha*err
    vel      = vel_pred + beta *err          # beta applied per-tick

With the COAST gains (alpha=0.02, beta=1.0) the velocity-correction loop is
lightly damped. At a *uniform* dt that is fine (the 200 Hz flight timer and the
fixed-dt HIL path both stay within a few m/s). But if dt is taken from
wall-clock micros() while packets arrive with USB jitter, the predict step
desynchronises from the correction and the estimate rings into a ±100 m/s
sawtooth — exactly what was seen on the bench HIL run.

This test reproduces that and locks in the fix: the firmware uses a fixed,
sim-authoritative dt in the APEX_HIL build (fusion.cpp / control.cpp), so the
estimator must be well-behaved at uniform dt and would *not* be at jittered dt.
If someone reverts to wall-clock dt in HIL, the jitter case here documents why
it breaks.
"""

from __future__ import annotations

import numpy as np
import pytest

_G = 9.81
_CLAMP = 5.0           # CF_ALT_ERR_CLAMP_M
_COAST_ALPHA = 0.02    # CF_COAST_ALPHA
_COAST_BETA = 1.00     # CF_COAST_BETA


def _coast_truth(n, nom_dt):
    """Smooth ballistic coast: 520 m AGL, +300 m/s, drag+gravity decel."""
    k = 0.0009
    v, h = 300.0, 520.0
    alt = np.zeros(n); vel = np.zeros(n); acc = np.zeros(n)
    for i in range(n):
        a = -_G - k * v * v
        acc[i], vel[i], alt[i] = a, v, h
        v += a * nom_dt
        h += v * nom_dt
        if v < 0:
            v, a = 0.0, -_G
    return alt, vel, acc


def _run_cf(alpha, beta, dt_fn, seconds=20.0, seed=1):
    """Port of the fusion.cpp COAST complementary filter. dt_fn(rng) -> dt."""
    nom = 0.01
    n = int(seconds / nom)
    rng = np.random.default_rng(seed)
    true_alt, true_vel, true_acc = _coast_truth(n, nom)
    alt, vel = true_alt[0], true_vel[0]
    est_alt = np.zeros(n); est_vel = np.zeros(n)
    for i in range(n):
        dt = dt_fn(rng)
        acc = true_acc[i] + rng.normal(0, 0.07)      # ICM accel noise
        baro = true_alt[i] + rng.normal(0, 0.03)     # BMP581 noise (~0.32 Pa)
        vel_pred = vel + acc * dt
        alt_pred = alt + vel * dt
        err = max(-_CLAMP, min(_CLAMP, baro - alt_pred))
        alt = alt_pred + alpha * err
        vel = vel_pred + beta * err
        est_alt[i], est_vel[i] = alt, vel
    mask = np.arange(n) * nom > 0.5                   # skip initial settle
    return (np.abs(est_alt - true_alt)[mask].max(),
            np.abs(est_vel - true_vel)[mask].max())


def _fixed_dt(_rng):
    return 0.01


def _jitter_dt(rng):
    # ±8 ms wall-clock jitter, like USB packet delivery at speed 1.0.
    return 0.01 + rng.uniform(-0.008, 0.008)


class TestComplementaryFilterDt:
    def test_fixed_dt_is_stable(self):
        """Sim-authoritative fixed dt (what the APEX_HIL build now uses):
        the COAST gains keep the estimate tight — no sawtooth."""
        alt_err, vel_err = _run_cf(_COAST_ALPHA, _COAST_BETA, _fixed_dt)
        assert alt_err < 5.0, f"altitude ripple {alt_err:.1f} m too large"
        assert vel_err < 25.0, f"velocity ripple {vel_err:.1f} m/s too large"

    def test_wallclock_jitter_rings_documenting_the_bug(self):
        """The reverted behaviour: wall-clock dt under USB jitter rings into
        the ±100 m/s sawtooth observed on the bench. This is the regression
        the fixed-dt HIL path prevents — if this ever stops ringing the model
        has drifted from the firmware and the guard above is meaningless."""
        _, vel_err = _run_cf(_COAST_ALPHA, _COAST_BETA, _jitter_dt)
        assert vel_err > 60.0, (
            f"expected wall-clock jitter to ring (>60 m/s), got {vel_err:.1f}")

    def test_fixed_dt_far_better_than_jitter(self):
        _, vel_fixed = _run_cf(_COAST_ALPHA, _COAST_BETA, _fixed_dt)
        _, vel_jit = _run_cf(_COAST_ALPHA, _COAST_BETA, _jitter_dt)
        assert vel_jit > 3 * vel_fixed
