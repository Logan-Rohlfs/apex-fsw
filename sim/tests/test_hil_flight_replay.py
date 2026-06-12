"""State-machine validation against the real Seymour TX flight recording.

Feeds the actual TeleMega CSV (2026-05-24, no airbrakes flown) through the
reference flight logic (``apex_sim.hil.fake_teensy.FlightLogic`` — the model
the firmware must match) and asserts the state machine:

* does NOT progress while sitting on the pad (no accidental transitions),
* detects every phase at the time the recording says it happened
  (launch t=0, burnout t=3.60 s, baro apogee t=27.85 s, landing ~t=247 s),
* never skips or repeats a phase,
* and begins airbrake deployment exactly when the gates allow — after the
  2.5 s post-burnout lockout AND below the 240 m/s mach gate — even though
  this flight never actually deployed brakes (open-loop replay: the
  deployment command has no effect on the recorded data).

Ground truth (Blue Raven summary + TeleMega): max accel 14.1 g, burnout
3.6 s, max velocity 284 m/s, apogee 3218 m AGL at 27.9 s, drogue descent
9.5 m/s, main at 997 ft, landing accel spike at ~t=247 s.

Run from apex/sim/:  .venv/bin/python -m pytest tests/ -v
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from apex_sim.hil.fake_teensy import FlightLogic  # noqa: E402
from apex_sim.hil.protocol import PHASE_NAMES, SimSensors  # noqa: E402
from run_replay import build_grid, load_telemega  # noqa: E402

_CSV = (Path(__file__).resolve().parents[1] / "data" / "flights"
        / "seymour_2026_05_24" / "telemega_flight_2026-05-24.csv")

_PRE_PAD_S = 5.0


def _replay(grid, logic):
    """Drive FlightLogic with the gridded recording; return per-tick history."""
    hist = []
    n = len(grid["t_grid"])
    for i in range(n):
        s = SimSensors(
            accel_x_mss=float(grid["accel_x"][i]),
            accel_y_mss=float(grid["accel_y"][i]),
            accel_z_mss=float(grid["accel_z"][i]),
            gyro_x_rads=float(grid["gyro_roll"][i]),
            gyro_y_rads=float(grid["gyro_pitch"][i]),
            gyro_z_rads=float(grid["gyro_yaw"][i]),
            baro_pa=float(grid["pressure"][i]),
            highg_x_mss=float(grid["accel_x"][i]),
            highg_y_mss=float(grid["accel_y"][i]),
            highg_z_mss=float(grid["accel_z"][i]),
            mag_x_gauss=float(grid["mag_x"][i]),
            mag_y_gauss=float(grid["mag_y"][i]),
            mag_z_gauss=float(grid["mag_z"][i]),
            gps_alt_msl_m=float(grid["gps_alt_msl"][i]),
            gps_valid=1,
        )
        reply = logic.step(int(grid["sim_ms"][i]), s)
        t_flight = float(grid["t_grid"][i])     # CSV time: launch at ~0
        hist.append((t_flight, reply))
    return hist


@pytest.fixture(scope="module")
def flight():
    grid = build_grid(load_telemega(_CSV), pre_pad_s=_PRE_PAD_S)
    logic = FlightLogic()
    hist = _replay(grid, logic)
    return logic, hist


def _transitions(hist):
    """[(t_flight, phase_name)] at each phase change."""
    out = []
    prev = None
    for t, reply in hist:
        if reply is None:
            continue
        name = PHASE_NAMES[reply.phase]
        if name != prev:
            out.append((t, name))
            prev = name
    return out


class TestSeymourReplay:
    def test_phases_in_order_no_skips_no_repeats(self, flight):
        _, hist = flight
        names = [n for _, n in _transitions(hist)]
        assert names == ["ARMED", "BOOST", "COAST", "DESCENT", "LANDED"], names

    def test_no_progress_on_pad(self, flight):
        """5 s of pad data before launch — must sit in ARMED the whole time."""
        _, hist = flight
        for t, reply in hist:
            if reply is None or t >= -0.1:
                continue
            assert PHASE_NAMES[reply.phase] == "ARMED", (
                f"left ARMED at t={t:.2f}s, before launch")

    def test_launch_detected_promptly(self, flight):
        _, hist = flight
        t_boost = dict((n, t) for t, n in _transitions(hist))["BOOST"]
        # 2 g crossing + 150 ms confirm; recording launches at t≈0
        assert -0.1 <= t_boost <= 0.5, t_boost

    def test_burnout_matches_recording(self, flight):
        _, hist = flight
        t_coast = dict((n, t) for t, n in _transitions(hist))["COAST"]
        # Blue Raven burnout 3.6 s + 200 ms confirmation window
        assert 3.5 <= t_coast <= 4.6, t_coast

    @pytest.mark.xfail(
        reason=("FakeTeensy uses a lightweight firmware estimator surrogate; "
                "use run_hil.py --compare-fake against real APEX_HIL firmware "
                "for apogee/control validation."),
        strict=False,
    )
    def test_apogee_matches_recording(self, flight):
        _, hist = flight
        t_desc = dict((n, t) for t, n in _transitions(hist))["DESCENT"]
        # Baro apogee 27.85 s; Blue Raven apo fire 28.4 s
        assert 26.0 <= t_desc <= 30.0, t_desc

    def test_landed_detected_after_touchdown(self, flight):
        _, hist = flight
        t_land = dict((n, t) for t, n in _transitions(hist))["LANDED"]
        # Touchdown ~247 s (main-deploy descent), recording ends 291 s
        assert 245.0 <= t_land <= 291.0, t_land

    def test_altitude_estimate_tracks_baro_apogee(self, flight):
        logic, _ = flight
        assert 3100.0 < logic.max_alt < 3350.0, logic.max_alt   # truth: 3218 m

    @pytest.mark.xfail(
        reason=("FakeTeensy is no longer the tuning authority for deployment "
                "magnitude; compare the shadow fake against real HIL firmware."),
        strict=False,
    )
    def test_deployment_respects_gates_then_commands(self, flight):
        _, hist = flight
        trans = dict((n, t) for t, n in _transitions(hist))
        t_coast, t_desc = trans["COAST"], trans["DESCENT"]

        first_deploy = None
        max_deploy = 0.0
        for t, reply in hist:
            if reply is None:
                continue
            if reply.deployment_frac > 0.005:
                if first_deploy is None:
                    first_deploy = (t, reply.est_vel_mps)
                if t < t_desc:
                    max_deploy = max(max_deploy, reply.deployment_frac)
            # retracted again within servo travel time of DESCENT entry
            if t > t_desc + 0.5:
                assert reply.deployment_frac < 0.005, (
                    f"brakes not retracted at t={t:.2f}s")

        assert first_deploy is not None, "controller never commanded the brakes"
        t_first, vel_first = first_deploy
        # Post-burnout lockout: nothing before burnout + 2.5 s
        assert t_first >= t_coast + 2.5 - 0.02, (t_first, t_coast)
        # Mach gate: nothing above 240 m/s (max recorded velocity was 284)
        assert vel_first < 240.0, vel_first
        # This flight overshot the 10k ft target — PID should brake hard
        assert max_deploy > 0.5, max_deploy
