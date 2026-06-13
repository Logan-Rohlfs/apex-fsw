"""Real-time closed-loop HIL runner.

Closes the loop between a live RocketPy simulation and a flight computer
speaking the ``fsw/src/hil.h`` protocol (real Teensy or
:class:`~apex_sim.hil.fake_teensy.FakeTeensy`):

1. **Pad warm-up** — stream stationary on-pad SimPackets until the flight
   computer captures its pad reference and reports ARMED (the firmware needs
   ~0.5 s of baro settling plus ~3 s of Mahony convergence).
2. **Flight** — RocketPy integrates the dynamics; its airbrake controller
   callback (100 Hz of sim time) emulates the sensors from true state, sends
   them down the wire, blocks for the TeensyPacket reply, and applies the
   returned ``deployment_frac`` to the simulated airbrakes.  The loop is
   paced to wall clock because the firmware's complementary filter
   integrates with wall-clock dt (``micros()``), so sim time and real time
   must advance 1:1.

Speed caveat: ``speed != 1`` distorts the *firmware's* velocity estimate on
real hardware (Mahony runs at a fixed 100 Hz assumption but the CF uses
measured dt).  The fake Teensy uses sim-time dt and is immune — max-speed
runs (``speed=0``) are only meaningful against the fake.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import numpy as np
from rocketpy import Flight

from apex_sim.hil.emulator import SensorEmulator
from apex_sim.hil.link import HilLink
from apex_sim.hil.protocol import PHASE_NAMES, SimSensors, TeensyPkt


@dataclass
class HilRow:
    """One controller tick of the closed loop."""

    t_s: float                 # sim time since ignition
    sim_ms: int                # wire timestamp (includes pre-pad offset)
    true_alt_agl_m: float
    true_vel_z_mps: float
    sensors: SimSensors        # the injected sensor readings
    reply: Optional[TeensyPkt]
    latency_ms: float
    shadow_replies: Dict[str, Optional[TeensyPkt]] = field(default_factory=dict)
    shadow_latencies_ms: Dict[str, float] = field(default_factory=dict)


@dataclass
class HilResult:
    flight: object                      # rocketpy.Flight
    rows: List[HilRow] = field(default_factory=list)
    transitions: List[tuple] = field(default_factory=list)   # (t_s, phase name)
    lines: List[str] = field(default_factory=list)
    missed: int = 0
    crc_errors: int = 0

    @property
    def latencies_ms(self) -> List[float]:
        return [r.latency_ms for r in self.rows if r.reply is not None]


def warm_up(link: HilLink, emulator: SensorEmulator,
            rail_inclination_deg: float, rail_heading_deg: float,
            duration_s: float = 6.0, speed: float = 1.0,
            arm_timeout: float = 30.0,
            shadow_links: Optional[Dict[str, HilLink]] = None,
            result: "Optional[HilResult]" = None,
            tick_cb: Optional[Callable] = None) -> int:
    """Sit on the pad with arm switches OPEN, then CLOSE them to arm.

    This mirrors the real launch sequence with no HIL-specific arming: the FC
    sits in IDLE on the pad (capturing its baro pad reference and converging the
    Mahony filter) while the arm switches are open, exactly as during pad ops.
    After ``duration_s`` of pad time we close the switches (HIL_ARM_SWITCH_BIT),
    and the FC's normal IDLE→ARMED gate fires — same code as flight.

    Pad ticks are streamed at 100 Hz and (if ``result`` is given) recorded as
    HilRows with negative ``t_s`` (time-to-launch), so the on-pad period appears
    in the host CSV/telemetry. The firmware logs its own IDLE/ARMED samples to
    the binary flight log throughout, so a HIL log carries real pad context.

    Returns the last sim_time_ms sent (the flight phase continues from it).
    Raises RuntimeError if the FC never reports ARMED after the switches close.
    """
    pad_open = emulator.pad_sensors(rail_inclination_deg, rail_heading_deg)
    pad_armed = pad_open._replace(arm_switch=1)   # switches closed
    shadows = shadow_links or {}
    wall0 = time.monotonic()
    pad_ticks = max(int(duration_s * 100), 0)
    state = {"i": 0, "sim_ms": 0}

    def _tick(sensors: SimSensors) -> Optional[TeensyPkt]:
        t_send = time.monotonic()
        reply = link.transact(state["sim_ms"], sensors, timeout=0.1)
        latency_ms = (time.monotonic() - t_send) * 1e3
        for shadow in shadows.values():
            shadow.transact(state["sim_ms"], sensors, timeout=0.1)
        if speed > 0:
            rem = wall0 + (state["i"] + 1) * 0.01 / speed - time.monotonic()
            if rem > 0:
                time.sleep(rem)
        if result is not None:
            row = HilRow(
                t_s=(state["i"] - pad_ticks) * 0.01,   # negative = time-to-launch
                sim_ms=state["sim_ms"],
                true_alt_agl_m=0.0, true_vel_z_mps=0.0,
                sensors=sensors, reply=reply, latency_ms=latency_ms)
            result.rows.append(row)
            if tick_cb is not None:
                tick_cb(row)
        state["i"] += 1
        state["sim_ms"] = state["i"] * 10
        return reply

    # Phase 1 — pad sit with switches OPEN. The FC stays IDLE (it cannot arm
    # without the switches), capturing its pad reference and converging fusion.
    for _ in range(pad_ticks):
        _tick(pad_open)

    # Phase 2 — close the switches; the FC's real IDLE→ARMED gate now fires.
    deadline = time.monotonic() + arm_timeout
    armed = False
    last_phase = None
    while time.monotonic() < deadline:
        reply = _tick(pad_armed)
        if reply is not None:
            last_phase = PHASE_NAMES.get(reply.phase, reply.phase)
            if last_phase == "ARMED":
                armed = True
                break
    if not armed:
        # Surface the firmware's own diagnostic (#INFO/#WARN lines explaining
        # why it has not armed — pad capture, storage-not-ready, auto-arm
        # countdown) so the reason reaches the operator instead of a blind
        # timeout. The HIL build emits these once per second while IDLE.
        fc_lines = [ln for ln in link.drain_lines()
                    if ln.startswith("#")][-4:]
        detail = ("\n  Flight computer said:\n    " + "\n    ".join(fc_lines)
                  if fc_lines else "")
        raise RuntimeError(
            "Flight computer never reported ARMED after the arm switches closed "
            f"(last phase: {last_phase}). Check the APEX_HIL build is flashed "
            "and the port is correct. Most common cause is 'no log, no arm' — "
            "arming is refused unless the QSPI flash is mounted and writable "
            "on the board." + detail)

    return state["sim_ms"]


def run_closed_loop(link: HilLink, env, rocket, env_cfg, airbrakes_cfg: dict,
                    speed: float = 1.0, warmup_s: float = 6.0,
                    terminate_on_apogee: bool = True, max_time: float = 120.0,
                    noise: bool = False,
                    sensor_kwargs: Optional[dict] = None,
                    shadow_links: Optional[Dict[str, HilLink]] = None,
                    tick_cb: Optional[Callable] = None) -> HilResult:
    """Warm up, then fly the RocketPy sim with the flight computer in the loop.

    Parameters
    ----------
    link : HilLink
        Open link to a HIL-flashed Teensy or a FakeTeensy pty.
    env, rocket
        Built RocketPy Environment and Rocket (no airbrakes attached yet).
    env_cfg
        Resolved config from ``load_environment()`` (rail + magnetic data).
    airbrakes_cfg : dict
        Parsed ``config/airbrakes.yaml`` (aero curve for the sim side).
    speed : float
        Wall-clock pacing factor.  1.0 = real time (required for real
        hardware); 0 = as fast as the serial roundtrip allows (fake only).
    noise : bool
        Add per-sample sensor noise from the docs/sensors noise model.
    tick_cb : callable, optional
        ``tick_cb(HilRow)`` invoked every controller tick (e.g. for live
        printing).  Must be fast — it runs inside the control loop.

    Returns
    -------
    HilResult
    """
    mag = env_cfg.site.magnetic
    emulator = SensorEmulator(
        pressure_fn=env.pressure,
        pad_elevation_m=env_cfg.site.elevation_m,
        mag_declination_deg=mag.declination_deg,
        mag_inclination_deg=mag.inclination_deg,
        mag_strength_ut=mag.field_strength_ut,
        noise=noise,
        **(sensor_kwargs or {}),
    )
    rail = env_cfg.rail

    result = HilResult(flight=None)
    pad_ms = warm_up(link, emulator, rail.inclination_deg, rail.heading_deg,
                     duration_s=warmup_s, speed=speed,
                     shadow_links=shadow_links, result=result, tick_cb=tick_cb)
    result.lines.extend(link.drain_lines())

    aero = airbrakes_cfg["aerodynamics"]
    delta_cd = aero["delta_cd_max"]

    ctx = {
        "prev_t": None, "prev_v": None, "accel": np.zeros(3),
        "deploy": 0.0, "phase": None, "wall0": None, "last_sim_ms": None,
    }

    def _controller(t, sampling_rate, state, state_history,
                    observed_variables, interactive_objects, sensors):
        # RocketPy 1.10 passes a lone interactive object bare, not in a list.
        airbrakes = (interactive_objects[0]
                     if isinstance(interactive_objects, (list, tuple))
                     else interactive_objects)

        sim_ms = pad_ms + 10 + int(round(t * 1000.0))
        if ctx["last_sim_ms"] == sim_ms:
            # RocketPy may evaluate controllers more than once at the same
            # integration time. The real HIL contract is one packet per 100 Hz
            # controller tick; duplicate transactions would advance firmware
            # time and servo rate limits without advancing the simulated state.
            airbrakes.deployment_level = ctx["deploy"]
            return [t, ctx["deploy"]]
        ctx["last_sim_ms"] = sim_ms

        # Coordinate acceleration via finite difference of true velocity.
        v = np.asarray(state[3:6], dtype=float)
        if ctx["prev_t"] is not None and t > ctx["prev_t"]:
            ctx["accel"] = (v - ctx["prev_v"]) / (t - ctx["prev_t"])
        ctx["prev_t"], ctx["prev_v"] = t, v

        # arm_switch stays closed for the whole flight (operator armed on the
        # pad); pre-BOOST it holds ARMED, post-BOOST the flight is latched.
        sim_sensors = emulator.flight_sensors(state, ctx["accel"])._replace(arm_switch=1)

        # Pace sim time to wall clock (anchor at the first flight tick).
        if ctx["wall0"] is None:
            ctx["wall0"] = time.monotonic() - (t / speed if speed > 0 else 0.0)
        if speed > 0:
            rem = ctx["wall0"] + t / speed - time.monotonic()
            if rem > 0:
                time.sleep(rem)

        t_send = time.monotonic()
        reply = link.transact(sim_ms, sim_sensors, timeout=0.05)
        latency_ms = (time.monotonic() - t_send) * 1e3
        shadow_replies: Dict[str, Optional[TeensyPkt]] = {}
        shadow_latencies: Dict[str, float] = {}
        for name, shadow in (shadow_links or {}).items():
            st = time.monotonic()
            shadow_replies[name] = shadow.transact(sim_ms, sim_sensors, timeout=0.05)
            shadow_latencies[name] = (time.monotonic() - st) * 1e3

        if reply is not None:
            ctx["deploy"] = max(0.0, min(1.0, reply.deployment_frac))
            name = PHASE_NAMES.get(reply.phase, str(reply.phase))
            if name != ctx["phase"]:
                if ctx["phase"] is not None:
                    result.transitions.append((t, name))
                ctx["phase"] = name
        else:
            result.missed += 1
        airbrakes.deployment_level = ctx["deploy"]

        row = HilRow(
            t_s=t, sim_ms=sim_ms,
            true_alt_agl_m=float(state[2]) - emulator.pad_elevation_m,
            true_vel_z_mps=float(state[5]),
            sensors=sim_sensors,
            reply=reply, latency_ms=latency_ms,
            shadow_replies=shadow_replies,
            shadow_latencies_ms=shadow_latencies)
        result.rows.append(row)
        if tick_cb is not None:
            tick_cb(row)
        return [t, ctx["deploy"]]

    rocket.add_air_brakes(
        drag_coefficient_curve=lambda deployment, mach: deployment * delta_cd,
        controller_function=_controller,
        sampling_rate=100,
        clamp=True,
        reference_area=aero["reference_area_m2"],
        initial_observed_variables=[0.0, 0.0],
        override_rocket_drag=False,
        return_controller=True,
        name="HIL AirBrakes",
    )

    result.flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=rail.length_m,
        inclination=rail.inclination_deg,
        heading=rail.heading_deg,
        terminate_on_apogee=terminate_on_apogee,
        max_time=max_time,
        name="Apex HIL",
    )

    # Post-landing static tail (full flights only). RocketPy stops integrating at
    # touchdown, but the FC's DESCENT→LANDED gate needs |baro_rate|<2 m/s and
    # |accel-1g|<2 m/s² sustained LANDED_CONFIRM_MS (3 s). Feed a few seconds of
    # at-rest data — a real landed rocket sitting still — so LANDED fires and the
    # once-per-flight post-landing QSPI→SD dump runs (otherwise untested in HIL).
    if not terminate_on_apogee:
        # Only feed the at-rest tail if the rocket ACTUALLY reached the ground.
        # If max_time truncated descent mid-air, fabricating stillness here would
        # be a bandaid (a fake LANDED that never happens in flight) — refuse it.
        last_true_alt = result.rows[-1].true_alt_agl_m if result.rows else 0.0
        if last_true_alt > 30.0:
            result.lines.append(
                f"#WARN: descent truncated at {last_true_alt:.0f} m AGL — "
                f"max_time={max_time:.0f}s too short for the recovery profile. "
                "Skipping post-landing tail so LANDED is NOT fabricated; raise "
                "--max-time to fly all the way down.")
            result.lines.extend(link.drain_lines())
            result.crc_errors = link.rx_crc_errors
            return result
        tail = emulator.pad_sensors(
            rail.inclination_deg, rail.heading_deg)._replace(arm_switch=1)
        last_ms = result.rows[-1].sim_ms if result.rows else pad_ms
        last_t = result.rows[-1].t_s if result.rows else 0.0
        wall0 = time.monotonic()
        for k in range(int(5.0 * 100)):       # 5 s ≥ LANDED_CONFIRM_MS
            sim_ms = last_ms + (k + 1) * 10
            t_send = time.monotonic()
            reply = link.transact(sim_ms, tail, timeout=0.05)
            latency_ms = (time.monotonic() - t_send) * 1e3
            if speed > 0:
                rem = wall0 + (k + 1) * 0.01 / speed - time.monotonic()
                if rem > 0:
                    time.sleep(rem)
            if reply is not None:
                name = PHASE_NAMES.get(reply.phase, str(reply.phase))
                if name != ctx["phase"]:
                    result.transitions.append((last_t + (k + 1) * 0.01, name))
                    ctx["phase"] = name
            row = HilRow(
                t_s=last_t + (k + 1) * 0.01, sim_ms=sim_ms,
                true_alt_agl_m=0.0, true_vel_z_mps=0.0,
                sensors=tail, reply=reply, latency_ms=latency_ms)
            result.rows.append(row)
            if tick_cb is not None:
                tick_cb(row)

    result.lines.extend(link.drain_lines())
    result.crc_errors = link.rx_crc_errors
    return result
