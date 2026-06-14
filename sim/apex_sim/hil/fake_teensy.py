"""In-process fake Teensy — a Python reference of the APEX_HIL firmware loop.

Serves two purposes:

1. **Hardware-free testing** of the entire laptop-side HIL stack (emulator →
   link → packets → closed loop) over a pty, used by ``tests/test_hil.py``
   and ``run_hil.py --fake``.
2. **Reference implementation** of the flight state machine + airbrake
   control that the firmware (``fsw/src/flight_state.cpp`` /
   ``fsw/src/control.cpp``) must match.  All thresholds and gains mirror
   ``fsw/src/config.h`` — keep them in sync.

State machine hardening (validated against the Seymour TX 2026-05-24
recordings, see tests/test_hil_flight_replay.py):

* Launch acceleration creates a candidate, but a sustained 30 m barometric
  rise is required before burnout progression or airbrake control is allowed.
  A candidate that settles back near 1 g with low rotation returns to ARMED.
  Other transitions retain confirmation windows and independent backups:

  ARMED→BOOST    accel > 2 g for 150 ms          | baro AGL > 30 m for 200 ms
  BOOST→COAST    axial specific force < 0, 200 ms| time in BOOST > 8 s
  COAST→DESCENT  fused vel < 2 m/s for 500 ms    | baro 10 m below max, 500 ms
                                                   (backup armed 8 s after
                                                    burnout — transonic baro
                                                    spikes can't reach it)
  DESCENT→LANDED |baro rate| < 2 m/s AND accel magnitude within ±2 m/s² of
                 1 g for 3 s.  Both signals are orientation-independent: a
                 rocket lying on its side breaks any axial-axis assumption
                 (the CF velocity converges to −1 g·s there, never to zero)

* Deployment gates (config.h "Airbrake Gates"): COAST only, ≥ 2.5 s after
  burnout, velocity < 240 m/s (Mach 0.7 — transonic actuation forbidden),
  altitude > 100 m AGL, ascending.  Brakes retract on DESCENT entry.

Differences from real firmware (intentional): dt comes from the echoed
``sim_time_ms`` instead of wall-clock micros(), so the fake runs correctly
at any replay speed including max-speed test runs.
"""

from __future__ import annotations

import math
import os
import select
import threading
import tty
from typing import Optional

from apex_sim.hil.protocol import (
    HIL_MAGIC_SIM_TO_TEENSY,
    SIM_SIZE,
    SimSensors,
    TeensyPkt,
    pack_teensy,
    unpack_sim,
)

# Mirrors fsw/src/config.h — keep in sync when tuning firmware.
_LAUNCH_ACCEL_THRESH_MSS = 19.62      # 2 g
_LAUNCH_CONFIRM_MS = 150
_LAUNCH_BARO_BACKUP_M = 30.0          # baro-only launch detect (accel failure)
_LAUNCH_BARO_CONFIRM_MS = 200
_LAUNCH_ABORT_CONFIRM_MS = 500
_BURNOUT_CONFIRM_MS = 200
_BOOST_MAX_MS = 8000                  # forced burnout (N3355 burns ~4.6 s)
_POST_BURNOUT_LOCKOUT_MS = 1000
_MACH_GATE_MPS = 240.0                # Mach 0.7 — no transonic actuation
_MIN_DEPLOY_ALT_M = 100.0
_APOGEE_VEL_THRESH_MPS = 2.0
_APOGEE_CONFIRM_MS = 500
_APOGEE_BACKUP_LOCKOUT_MS = 8000      # baro-fall backup arms this long after burnout
_APOGEE_BARO_FALL_M = 10.0
_LANDED_ACCEL_THRESH_MSS = 2.0        # |accel - 1 g| band
_LANDED_VEL_MAX_MPS = 2.0
_LANDED_CONFIRM_MS = 3000
_TARGET_APOGEE_M = 3048.0
_PID_KP = 0.4 / 0.3048
_PID_KI = -0.004 / 0.3048
_PID_KD = -0.04 / 0.3048   # D-term is Kd × velocity — velocity rescales too
_PID_U_MIN = -15.0
_PID_U_MAX = 30.0
_SERVO_MAX_RATE_PER_S = 1.0 / 0.24    # SERVO_FULL_TRAVEL_S — mirrors the firmware
# Servo motion profile parity with fsw/src/control.cpp: the reported
# deployment_frac is rate-limited to a full 0→1 stroke in SERVO_FULL_TRAVEL_S,
# same as the firmware. The firmware's µs endpoints / inverted mapping / smooth
# arm sweep are hardware specifics that don't change the 0–1 fraction the host
# sees, so this fraction-domain rate limit is the faithful mirror.
_ROCKET_MASS_KG = 30.44
_REF_AREA_M2 = 0.019001
_CD_CLEAN = 0.576
_G = 9.81
_SETTLE_PACKETS = 50
_CF_BOOST_ALPHA = 0.005
_CF_COAST_ALPHA = 0.02
_CF_BOOST_BETA = 0.10
_CF_COAST_BETA = 1.00
_CF_ALT_ERR_CLAMP_M = 5.0
_STATIONARY_ACCEL_MIN_MSS = 9.3
_STATIONARY_ACCEL_MAX_MSS = 10.3
_STATIONARY_GYRO_MAX_RADS = 0.05

_PH_IDLE, _PH_ARMED, _PH_BOOST, _PH_COAST, _PH_DESCENT, _PH_LANDED = range(6)


def _pressure_to_alt_msl(pa: float) -> float:
    return 44330.0 * (1.0 - (pa / 101325.0) ** (1.0 / 5.255))


def _isa_density(alt_msl_m: float) -> float:
    t = max(288.15 - 0.0065 * alt_msl_m, 1.0)
    return 1.225 * (t / 288.15) ** 4.256


class FlightLogic:
    """Estimator + state machine + airbrake control — the firmware model.

    Pure logic, no I/O: feed packets with :meth:`step`, get TeensyPkt fields
    back.  ``FakeTeensy`` wraps this in a pty; the flight-data replay test
    drives it directly.
    """

    def __init__(self):
        self.packets = 0
        self.pad_baro_sum = 0.0
        self.pad_alt_msl = 0.0
        self.armed = False
        self.phase = _PH_IDLE
        self.alt = 0.0
        self.vel = 0.0
        self.max_alt = 0.0
        self.deploy = 0.0
        self.messages = []          # ASCII lines the firmware would print
        self._prev_ms = None
        self._launch_ms = None      # confirmation-window start timestamps
        self._launch_baro_ms = None
        self._launch_abort_ms = None
        self._launch_validated = False
        self._burnout_ms_start = None
        self._apogee_ms = None
        self._apogee_baro_ms = None
        self._landed_ms = None
        self._boost_entry_ms = 0
        self._burnout_ms = None     # COAST entry time
        self._integral = 0.0
        # Baro vertical rate over a 1 s baseline (10 slots × 100 ms).
        # A per-tick derivative is useless on real data — recorded baro
        # arrives in ZOH steps (low post-landing log rates) that spike it.
        self._alt_ring = [0.0] * 10
        self._ring_idx = 0
        self._ring_full = False
        self._ring_last_ms = None
        self._baro_rate = 0.0       # m/s over the 1 s baseline

    # ── helpers ───────────────────────────────────────────────────────────────

    def _say(self, msg: str) -> None:
        self.messages.append(msg)

    @staticmethod
    def _confirm(now_ms: int, since_ms: Optional[int], cond: bool,
                 window_ms: int) -> "tuple":
        """Consecutive-condition window. Returns (fired, new_since_ms)."""
        if not cond:
            return False, None
        if since_ms is None:
            return False, now_ms
        return (now_ms - since_ms >= window_ms), since_ms

    # ── main tick (100 Hz) ────────────────────────────────────────────────────

    def step(self, sim_ms: int, s: SimSensors) -> Optional[TeensyPkt]:
        dt = 0.01
        if self._prev_ms is not None and sim_ms > self._prev_ms:
            dt = (sim_ms - self._prev_ms) * 1e-3
        self._prev_ms = sim_ms

        self.packets += 1

        # Settle: average baro for the pad reference. Reply to every packet
        # from the first one (phase IDLE during capture, then ARMED) — mirrors
        # the firmware's reply-every-packet HIL contract; the host watches the
        # phase field to know when the FC has armed.
        if not self.armed:
            self.pad_baro_sum += s.baro_pa
            # Arm only once the pad reference has settled AND the operator's arm
            # switches are closed (s.arm_switch) — mirrors the firmware gate
            # (board_switches_armed + pad_ready). Switches open → stay IDLE.
            if self.packets >= _SETTLE_PACKETS and s.arm_switch:
                avg = self.pad_baro_sum / self.packets
                self.pad_alt_msl = _pressure_to_alt_msl(avg)
                self.armed = True
                self.phase = _PH_ARMED
                self._launch_validated = False
                # Arm = fresh flight: clear PID state (parity with the
                # firmware's control_reset() called from flight_state_arm()).
                self._integral = 0.0
                self.deploy = 0.0
                self._say("#INFO: Pad reference captured - state ARMED")
            return TeensyPkt(
                magic=0, sim_time_ms=sim_ms, deployment_frac=0.0,
                est_alt_agl_m=0.0, est_vel_mps=0.0, pred_apogee_m=0.0,
                phase=self.phase, crc=0)

        baro_agl = _pressure_to_alt_msl(s.baro_pa) - self.pad_alt_msl
        # Vertical accel proxy: axial specific force minus gravity (rocket
        # near vertical through the phases where it matters).
        vert_accel = s.accel_x_mss - _G
        accel_mag = math.sqrt(s.accel_x_mss ** 2 + s.accel_y_mss ** 2
                              + s.accel_z_mss ** 2)
        gyro_mag = math.sqrt(s.gyro_x_rads ** 2 + s.gyro_y_rads ** 2
                             + s.gyro_z_rads ** 2)

        # ── Estimator: α-β complementary filter (mirrors fusion.cpp) ──────────
        in_flight = self.phase in (_PH_BOOST, _PH_COAST, _PH_DESCENT)
        if in_flight:
            if self.phase == _PH_BOOST:
                alpha = _CF_BOOST_ALPHA
                beta = _CF_BOOST_BETA
            else:
                alpha = _CF_COAST_ALPHA
                beta = _CF_COAST_BETA
            vel_pred = self.vel + vert_accel * dt
            alt_pred = self.alt + self.vel * dt
            err = max(-_CF_ALT_ERR_CLAMP_M,
                      min(_CF_ALT_ERR_CLAMP_M, baro_agl - alt_pred))
            self.alt = alt_pred + alpha * err
            self.vel = vel_pred + beta * err
        else:
            self.alt = baro_agl
            self.vel = 0.0
        if self.alt > self.max_alt:
            self.max_alt = self.alt

        # Orientation-independent vertical rate for the LANDED gate:
        # sample altitude every 100 ms, difference across the 1 s ring.
        if self._ring_last_ms is None or sim_ms - self._ring_last_ms >= 100:
            self._ring_last_ms = sim_ms
            oldest = self._alt_ring[self._ring_idx]
            if self._ring_full:
                self._baro_rate = (baro_agl - oldest) / 1.0
            self._alt_ring[self._ring_idx] = baro_agl
            self._ring_idx = (self._ring_idx + 1) % len(self._alt_ring)
            if self._ring_idx == 0:
                self._ring_full = True

        # ── Phase machine ──────────────────────────────────────────────────────
        if self.phase == _PH_ARMED:
            # Safing: arm switches opening before launch drops back to IDLE
            # (mirrors flight_state.cpp; only pre-BOOST — once boosted the
            # flight is latched and a switch glitch must not abort airbrakes).
            if not s.arm_switch:
                self.armed = False
                self.phase = _PH_IDLE
                self._launch_validated = False
                self._launch_ms = self._launch_baro_ms = None
                return TeensyPkt(
                    magic=0, sim_time_ms=sim_ms, deployment_frac=0.0,
                    est_alt_agl_m=0.0, est_vel_mps=0.0, pred_apogee_m=0.0,
                    phase=self.phase, crc=0)
            # Primary: sustained 2 g.  Backup: sustained baro climb (covers a
            # dead accelerometer — Seymour data clears the primary trivially
            # at 14.1 g).
            fired, self._launch_ms = self._confirm(
                sim_ms, self._launch_ms,
                accel_mag > _LAUNCH_ACCEL_THRESH_MSS, _LAUNCH_CONFIRM_MS)
            fired_b, self._launch_baro_ms = self._confirm(
                sim_ms, self._launch_baro_ms,
                baro_agl > _LAUNCH_BARO_BACKUP_M, _LAUNCH_BARO_CONFIRM_MS)
            if fired or fired_b:
                self.phase = _PH_BOOST
                self._boost_entry_ms = sim_ms
                self._launch_validated = fired_b
                # Seed velocity — rocket is already moving when confirmed
                # (mirrors fusion.cpp launch seeding).
                self.vel = 0.5 * vert_accel * (_LAUNCH_CONFIRM_MS * 1e-3)
                self._say("#INFO: BOOST detected (%s)"
                          % ("accel" if fired else "baro backup"))

        elif self.phase == _PH_BOOST:
            if not self._launch_validated:
                validated, self._launch_baro_ms = self._confirm(
                    sim_ms, self._launch_baro_ms,
                    baro_agl > _LAUNCH_BARO_BACKUP_M,
                    _LAUNCH_BARO_CONFIRM_MS)
                stationary, self._launch_abort_ms = self._confirm(
                    sim_ms, self._launch_abort_ms,
                    (_STATIONARY_ACCEL_MIN_MSS <= accel_mag
                     <= _STATIONARY_ACCEL_MAX_MSS
                     and gyro_mag <= _STATIONARY_GYRO_MAX_RADS),
                    _LAUNCH_ABORT_CONFIRM_MS)
                if validated:
                    self._launch_validated = True
                    self._burnout_ms_start = None
                    self._say("#INFO: Launch validated at 30 m AGL")
                elif stationary:
                    self.phase = _PH_ARMED
                    self.alt = baro_agl
                    self.vel = 0.0
                    self.max_alt = max(0.0, baro_agl)
                    self.deploy = 0.0
                    self._launch_ms = None
                    self._launch_baro_ms = None
                    self._launch_abort_ms = None
                    self._burnout_ms_start = None
                    self._launch_validated = False
                    self._say("#WARN: Launch candidate rejected - vehicle stationary")
                return TeensyPkt(
                    magic=0, sim_time_ms=sim_ms, deployment_frac=0.0,
                    est_alt_agl_m=self.alt, est_vel_mps=self.vel,
                    pred_apogee_m=self.alt, phase=self.phase, crc=0)

            # Primary: axial specific force flips negative (drag deceleration;
            # −0.78 g early coast on the Seymour flight).  Backup: max burn
            # time — the N3355 burns ~4.6 s, a stuck gate can't hold BOOST.
            fired, self._burnout_ms_start = self._confirm(
                sim_ms, self._burnout_ms_start,
                s.accel_x_mss < 0.0, _BURNOUT_CONFIRM_MS)
            if fired or sim_ms - self._boost_entry_ms > _BOOST_MAX_MS:
                self.phase = _PH_COAST
                self._burnout_ms = sim_ms
                self._say("#INFO: COAST - burnout detected (%s)"
                          % ("accel" if fired else "timeout"))

        elif self.phase == _PH_COAST:
            # Primary: fused velocity through zero (accel-verified → immune
            # to transonic baro spikes).  Backup: baro fell 10 m below the
            # running max — armed only 8 s after burnout so the transonic
            # regime (first ~4 s of coast) can never reach it.
            fired, self._apogee_ms = self._confirm(
                sim_ms, self._apogee_ms,
                self.vel < _APOGEE_VEL_THRESH_MPS, _APOGEE_CONFIRM_MS)
            backup_armed = (self._burnout_ms is not None and
                            sim_ms - self._burnout_ms > _APOGEE_BACKUP_LOCKOUT_MS)
            fired_b, self._apogee_baro_ms = self._confirm(
                sim_ms, self._apogee_baro_ms,
                backup_armed and baro_agl < self.max_alt - _APOGEE_BARO_FALL_M,
                _APOGEE_CONFIRM_MS)
            if fired or fired_b:
                self.phase = _PH_DESCENT
                self.deploy = 0.0          # retract for recovery
                self._say("#INFO: DESCENT - apogee detected (%s)"
                          % ("velocity" if fired else "baro backup"))

        elif self.phase == _PH_DESCENT:
            fired, self._landed_ms = self._confirm(
                sim_ms, self._landed_ms,
                (abs(self._baro_rate) < _LANDED_VEL_MAX_MPS and
                 abs(accel_mag - _G) < _LANDED_ACCEL_THRESH_MSS),
                _LANDED_CONFIRM_MS)
            if fired:
                self.phase = _PH_LANDED
                self._say("#INFO: LANDED")

        # ── Apogee prediction (closed-form, clean Cd) ──────────────────────────
        alt_msl = self.alt + self.pad_alt_msl
        rho = _isa_density(alt_msl)
        k = 0.5 * rho * _REF_AREA_M2 * _CD_CLEAN
        if self.vel > 0.0:
            pred = self.alt + (_ROCKET_MASS_KG / (2.0 * k)) * math.log1p(
                k * self.vel ** 2 / (_ROCKET_MASS_KG * _G))
        else:
            pred = self.alt

        # ── Airbrake control (PID, COAST only, all gates must pass) ───────────
        active = (self.phase == _PH_COAST
                  and self._launch_validated
                  and self._burnout_ms is not None
                  and sim_ms - self._burnout_ms >= _POST_BURNOUT_LOCKOUT_MS
                  and self.vel > 0.0
                  and self.vel < _MACH_GATE_MPS
                  and self.alt > _MIN_DEPLOY_ALT_M)
        if active:
            error = pred - _TARGET_APOGEE_M
            self._integral += error * dt
            u = _PID_KP * error + _PID_KI * self._integral + _PID_KD * self.vel
            u = max(_PID_U_MIN, min(_PID_U_MAX, u))
            desired = (u - _PID_U_MIN) / (_PID_U_MAX - _PID_U_MIN)
        elif self.phase == _PH_COAST:
            desired = self.deploy      # gated — hold position
        else:
            desired = 0.0              # retracted everywhere else
        max_delta = _SERVO_MAX_RATE_PER_S * dt
        delta = max(-max_delta, min(max_delta, desired - self.deploy))
        self.deploy = max(0.0, min(1.0, self.deploy + delta))

        return TeensyPkt(
            magic=0, sim_time_ms=sim_ms, deployment_frac=self.deploy,
            est_alt_agl_m=self.alt, est_vel_mps=self.vel,
            pred_apogee_m=pred, phase=self.phase, crc=0)


class FakeTeensy:
    """Pty-backed responder implementing the HIL flight-computer behaviour.

    Open :attr:`port` with :class:`~apex_sim.hil.link.HilLink` exactly as you
    would a real Teensy.  Call :meth:`close` when done.
    """

    def __init__(self):
        self._master_fd, slave_fd = os.openpty()
        tty.setraw(slave_fd)
        self.port = os.ttyname(slave_fd)
        self._slave_fd = slave_fd          # keep open so the pty persists
        self._run = True
        self.logic = FlightLogic()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        # The reader polls _run via select timeout — closing the fds before
        # the thread exits would race a blocked os.read on macOS.
        self._run = False
        self._thread.join(timeout=2.0)
        os.close(self._master_fd)
        os.close(self._slave_fd)

    # ── Serial loop ───────────────────────────────────────────────────────────

    def _write(self, data: bytes) -> None:
        try:
            os.write(self._master_fd, data)
        except OSError:
            self._run = False

    def _loop(self) -> None:
        self._write(b"#WARN: *** FAKE TEENSY (apex_sim.hil.fake_teensy) ***\n")
        buf = bytearray()
        m0 = HIL_MAGIC_SIM_TO_TEENSY & 0xFF          # 0xCD
        m1 = (HIL_MAGIC_SIM_TO_TEENSY >> 8) & 0xFF   # 0xAB
        sent_msgs = 0
        while self._run:
            try:
                # Re-announce until the host talks — pyserial discards the
                # pty input buffer when it opens the slave, so a single
                # boot-time banner can be lost before HilLink attaches.
                if self.logic.packets == 0:
                    self._write(b"#HIL_READY\n")
                ready, _, _ = select.select([self._master_fd], [], [], 0.1)
                if not ready:
                    continue
                chunk = os.read(self._master_fd, 256)
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            while True:
                start = buf.find(bytes([m0, m1]))
                if start < 0:
                    del buf[:-1]
                    break
                if len(buf) - start < SIM_SIZE:
                    del buf[:start]
                    break
                parsed = unpack_sim(bytes(buf[start:start + SIM_SIZE]))
                if parsed is None:
                    del buf[:start + 1]   # bad CRC — resync past this magic
                    continue
                del buf[:start + SIM_SIZE]
                reply = self.logic.step(parsed[0], parsed[1])
                while sent_msgs < len(self.logic.messages):
                    self._write(self.logic.messages[sent_msgs].encode() + b"\n")
                    sent_msgs += 1
                if reply is not None:
                    self._write(pack_teensy(reply))
