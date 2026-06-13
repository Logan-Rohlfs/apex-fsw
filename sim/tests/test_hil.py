"""End-to-end tests for the HIL stack — no hardware required.

Follows the project pattern: synthesise the full loop (RocketPy → emulator →
wire bytes → fake flight computer → reply bytes → decoder) and assert
everything round-trips.  The fake Teensy runs the reference state machine +
PID over a real pty, so the serial path is exercised byte-for-byte.

Run from apex/sim/:  .venv/bin/python -m pytest tests/ -v
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apex_sim.hil import protocol  # noqa: E402
from apex_sim.hil.emulator import SensorEmulator, rail_quaternion  # noqa: E402
from apex_sim.hil.fake_teensy import FakeTeensy  # noqa: E402
from apex_sim.hil.link import HilLink  # noqa: E402
from apex_sim.hil.protocol import SimSensors, TeensyPkt  # noqa: E402

_G = 9.80665


def _isa_pressure(alt_m):
    return 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588


def _pad_sensors(elev=889.0):
    em = SensorEmulator(_isa_pressure, pad_elevation_m=elev)
    return em, em.pad_sensors(rail_inclination_deg=90.0)


# ─── Protocol ─────────────────────────────────────────────────────────────────


class TestProtocol:
    def test_struct_sizes_match_firmware(self):
        assert protocol.SIM_SIZE == 64    # static_assert in fsw/src/hil.h
        assert protocol.TEENSY_SIZE == 24

    def test_crc8_check_value(self):
        # CRC-8/SMBUS (poly 0x07, init 0x00) check value for "123456789"
        assert protocol.crc8(b"123456789") == 0xF4

    def test_sim_packet_roundtrip(self):
        s = SimSensors(*[float(i) for i in range(14)], gps_valid=1)
        wire = protocol.pack_sim(123456, s)
        assert len(wire) == 64
        assert wire[:2] == bytes([0xCD, 0xAB])   # LE magic, parser sync bytes
        sim_ms, out = protocol.unpack_sim(wire)
        assert sim_ms == 123456
        assert out == s

    def test_teensy_packet_roundtrip(self):
        pkt = TeensyPkt(0, 99, 0.42, 1500.0, 250.0, 3100.0, 3, 0)
        wire = protocol.pack_teensy(pkt)
        assert len(wire) == 24
        out = protocol.unpack_teensy(wire)
        assert out is not None
        assert out.sim_time_ms == 99
        assert out.phase == 3
        assert abs(out.deployment_frac - 0.42) < 1e-6

    def test_corrupted_crc_rejected(self):
        wire = bytearray(protocol.pack_sim(1, _pad_sensors()[1]))
        wire[10] ^= 0xFF
        assert protocol.unpack_sim(bytes(wire)) is None


# ─── Emulator physics ─────────────────────────────────────────────────────────


class TestEmulator:
    def test_pad_specific_force_is_plus_1g_axial(self):
        _, s = _pad_sensors()
        assert abs(s.accel_x_mss - _G) < 1e-6    # nose-up: +1 g on body x
        assert abs(s.accel_y_mss) < 1e-9
        assert abs(s.accel_z_mss) < 1e-9
        assert s.gyro_x_rads == 0.0

    def test_pad_baro_matches_site_pressure(self):
        _, s = _pad_sensors(elev=889.0)
        assert abs(s.baro_pa - _isa_pressure(889.0)) < 0.5

    def test_rail_tilt_reduces_axial_component(self):
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0)
        s = em.pad_sensors(rail_inclination_deg=85.0, rail_heading_deg=0.0)
        assert abs(s.accel_x_mss - _G * math.cos(math.radians(5.0))) < 1e-6
        mag = math.sqrt(s.accel_x_mss**2 + s.accel_y_mss**2 + s.accel_z_mss**2)
        assert abs(mag - _G) < 1e-6              # gravity magnitude preserved

    def test_boost_specific_force(self):
        # Vertical flight, 5 g upward coordinate accel → 6 g specific force.
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0)
        state = [0, 0, 500.0, 0, 0, 100.0, 1, 0, 0, 0, 0, 0, 0]
        s = em.flight_sensors(state, np.array([0.0, 0.0, 5 * _G]))
        assert abs(s.accel_x_mss - 6 * _G) < 1e-6

    def test_mag_field_magnitude_preserved(self):
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0,
                            mag_declination_deg=4.0, mag_inclination_deg=58.5,
                            mag_strength_ut=47.5)
        s = em.pad_sensors(rail_inclination_deg=85.0, rail_heading_deg=120.0)
        mag = math.sqrt(s.mag_x_gauss**2 + s.mag_y_gauss**2 + s.mag_z_gauss**2)
        assert abs(mag - 0.475) < 1e-6

    def test_rail_quaternion_is_unit(self):
        q = rail_quaternion(85.0, 230.0)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9

    def test_gps_drops_above_4g_and_reacquires(self):
        """AIRBORNE4g receiver model: fix lost during boost, back ~2 s after
        dynamics settle — mirrors gps_monitor_update expectations."""
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0)
        state = [0, 0, 500.0, 0, 0, 100.0, 1, 0, 0, 0, 0, 0, 0]
        boost = em.flight_sensors(state, np.array([0.0, 0.0, 8 * _G]))
        assert boost.gps_valid == 0                       # 8 g > 4 g envelope
        s = em.flight_sensors(state, np.array([0.0, 0.0, -_G - 5.0]))
        assert s.gps_valid == 0                           # still reacquiring
        for _ in range(250):                              # 2.5 s of coast
            s = em.flight_sensors(state, np.array([0.0, 0.0, -_G - 5.0]))
        assert s.gps_valid == 1, "fix never reacquired after dynamics settled"
        assert not math.isnan(s.gps_alt_msl_m)

    def test_gps_model_can_be_disabled(self):
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0, gps_model=False)
        state = [0, 0, 500.0, 0, 0, 100.0, 1, 0, 0, 0, 0, 0, 0]
        s = em.flight_sensors(state, np.array([0.0, 0.0, 8 * _G]))
        assert s.gps_valid == 1


# ─── Closed loop over a pty ───────────────────────────────────────────────────


@pytest.fixture
def fake_link():
    fake = FakeTeensy()
    link = HilLink(fake.port)
    yield fake, link
    link.close()
    fake.close()


class TestFakeTeensyLoop:
    def test_ready_banner_and_arming(self, fake_link):
        fake, link = fake_link
        assert link.wait_for_line("HIL_READY", timeout=3.0)
        em, pad = _pad_sensors()
        for i in range(60):                       # settle window is 50 packets
            link.send(i * 10, pad)
            time.sleep(0.001)
        assert link.wait_for_line("ARMED", timeout=2.0)
        reply = link.transact(700, pad, timeout=1.0)
        assert reply is not None
        assert protocol.PHASE_NAMES[reply.phase] == "ARMED"
        assert abs(reply.est_alt_agl_m) < 2.0     # on the pad
        assert reply.deployment_frac == 0.0

    def test_replies_every_packet_idle_before_armed(self, fake_link):
        """New HIL contract: the FC replies to every SimPacket from the first
        one, reporting IDLE during pad capture and ARMED once settled. The
        host's warm_up() depends on watching this phase field — a regression
        to 'no reply until armed' would hang the loop."""
        fake, link = fake_link
        assert link.wait_for_line("HIL_READY", timeout=3.0)
        em, pad = _pad_sensors()

        # First packet must already get a reply, and it must read IDLE.
        first = link.transact(0, pad, timeout=1.0)
        assert first is not None, "no reply to the first SimPacket"
        assert protocol.PHASE_NAMES[first.phase] == "IDLE"
        assert first.deployment_frac == 0.0

        # Drive past the settle window; the phase must transition to ARMED
        # and every packet keeps getting a reply.
        saw_armed = False
        for i in range(1, 70):
            reply = link.transact(i * 10, pad, timeout=1.0)
            assert reply is not None, f"missing reply at packet {i}"
            if protocol.PHASE_NAMES[reply.phase] == "ARMED":
                saw_armed = True
        assert saw_armed, "FC never reported ARMED across the settle window"

    def test_synthetic_flight_reaches_coast_and_deploys(self, fake_link):
        """Drive a kinematic vertical flight; expect BOOST → COAST and PID
        deployment after the post-burnout lockout."""
        fake, link = fake_link
        assert link.wait_for_line("HIL_READY", timeout=3.0)
        em = SensorEmulator(_isa_pressure, pad_elevation_m=0.0)

        pad = em.pad_sensors()
        sim_ms = 0
        for _ in range(60):
            link.send(sim_ms, pad)
            sim_ms += 10
            time.sleep(0.001)
        assert link.wait_for_line("ARMED", timeout=2.0)

        # Kinematics: 4 s boost at 8 g net, then coast with ~0.6 g of drag
        # (burnout detection keys on negative axial specific force).
        dt = 0.01
        alt, vel = 0.0, 0.0
        phases = set()
        max_deploy = 0.0
        t = 0.0
        while t < 12.0:
            a = 8 * _G if t < 4.0 else -_G - 6.0   # net coordinate accel
            vel += a * dt
            alt += vel * dt
            state = [0, 0, alt, 0, 0, vel, 1, 0, 0, 0, 0, 0, 0]
            s = em.flight_sensors(state, np.array([0.0, 0.0, a]))
            reply = link.transact(sim_ms, s, timeout=0.5)
            assert reply is not None, f"no reply at t={t:.2f}"
            phases.add(protocol.PHASE_NAMES[reply.phase])
            max_deploy = max(max_deploy, reply.deployment_frac)
            sim_ms += 10
            t += dt

        assert "BOOST" in phases
        assert "COAST" in phases
        assert max_deploy > 0.05, "PID never commanded the brakes"
        assert link.rx_crc_errors == 0


# ─── Full RocketPy closed loop (slower, ~10–30 s) ────────────────────────────


class TestRocketPyClosedLoop:
    def test_full_hil_flight_against_fake_teensy(self):
        from apex_sim.config.loader import load_environment
        from apex_sim.hil.runner import run_closed_loop
        from apex_sim.sim.environment import build_environment
        from apex_sim.sim.rocket import build_rocket
        import yaml

        env_cfg = load_environment()
        env_cfg.atmosphere.model_override = "standard_atmosphere"
        env = build_environment(env_cfg)
        rocket = build_rocket()
        cfg_root = Path(__file__).resolve().parents[1] / "config"
        with (cfg_root / "airbrakes.yaml").open() as fh:
            airbrakes_cfg = yaml.safe_load(fh)

        fake = FakeTeensy()
        link = HilLink(fake.port)
        try:
            assert link.wait_for_line("HIL_READY", timeout=3.0)
            result = run_closed_loop(
                link, env, rocket, env_cfg, airbrakes_cfg,
                speed=0.0, warmup_s=1.0, terminate_on_apogee=True)
        finally:
            link.close()
            fake.close()

        assert result.missed == 0
        assert result.crc_errors == 0

        names = [n for _, n in result.transitions]
        assert "BOOST" in names
        assert "COAST" in names

        replies = [r for r in result.rows if r.reply is not None]
        assert replies, "no closed-loop ticks"

        # The brakes must actually have been commanded and have bitten:
        # apogee should land near (and not above) the uncontrolled altitude.
        max_deploy = max(r.reply.deployment_frac for r in replies)
        assert max_deploy > 0.05

        apogee_agl = result.flight.apogee - env.elevation
        target = airbrakes_cfg["control"]["target_apogee_m"]
        assert 0.8 * target < apogee_agl < 1.15 * target, (
            f"apogee {apogee_agl:.0f} m implausible vs target {target:.0f} m")

        # FC altitude estimate should track truth (clean sensors, no noise).
        coast = [r for r in replies
                 if protocol.PHASE_NAMES.get(r.reply.phase) == "COAST"]
        worst = max(abs(r.reply.est_alt_agl_m - r.true_alt_agl_m) for r in coast)
        assert worst < 50.0, f"FC altitude diverged from truth by {worst:.1f} m"
