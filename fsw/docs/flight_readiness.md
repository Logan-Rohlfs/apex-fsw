# Apex Flight-Readiness Plan

Goal: get the flight computer **feature-complete** so the only remaining work is
HIL tuning. Driven by the worst-case launch-day scenario:

> Integrate the FC into the rocket, power on, let it sit ~1 hour being rotated
> (sideways ↔ upright), bumped, and carried. The **only** human interface is two
> screw switches reached on the rod via a ladder. There is **no uplink/TX** to
> the rocket. We may be the 8th flight of the salvo after long recovery delays.
> When it finally lships, it must be armed and ready — and during all the handling
> it must never enter a dead-end state or false-trigger.

Design philosophy (from the team):
- **Fail toward flying.** Block arming only on the minimum needed to be *safe*
  (the one hard rule: never deploy during motor burn). Anything the airbrakes can
  function without should not block a launch. Airbrakes are non-safety-critical —
  recovery (drogue/main) is a **separate altimeter**, unchanged from the test
  flight. A failed airbrake attempt = slightly high apogee, not a lost vehicle.
- **The screw switches are the arming commit.** During handling the switches are
  OFF → the FC sits in a safe pre-armed state with launch detection disabled, so
  bumps/rotation/carrying can never false-launch. Screwing them in on the rod is
  the last action → arms → launch detection goes live.
- **Survive hangs.** A watchdog reboots a wedged FC back to a flight-ready state.

## Hardware map (confirmed pins)

| Signal | Pin | Notes |
|---|---|---|
| 12V rail enable | 20 / A6 | `PIN_12V_EN` — high-side enable for 12 V |
| Servo power enable | 38 / A14 | `PIN_SRV_EN` — NPN + PMOS high-side switch to servo |
| Servo PWM | 37 / D37 | `PIN_SERVO` (moved from 6) |
| Arm switch 1 | 32 / D32 | `PIN_SWITCH_1` — screw switch, closes to GND when ON |
| Arm switch 2 | 31 / D31 | `PIN_SWITCH_2` — screw switch, closes to GND when ON |
| Buzzer | 3 / D3 | `PIN_BUZZER` — passive piezo, `tone()` patterns |

**Switch wiring (fail-safe).** Use `INPUT_PULLUP`. Wire each switch so the
**armed/ON** position **closes the pin to GND** (pin reads LOW = armed). Then a
broken wire or unplugged switch floats HIGH = **disarmed/safe** — the safe
default. This is the recommended orientation; `SWITCH_ARMED_LEVEL` makes it
configurable if the mechanical switch is normally-closed instead.
Yes, a screw switch shorting the pin to GND with `INPUT_PULLUP` works perfectly.

**Buzzer.** Passive piezo (most Arduino kits) from pin 3 → buzzer → GND, optional
100 Ω series. Driven with `tone()` so we get distinct status patterns. An active
(self-driving) buzzer also works — set `BUZZER_ACTIVE 1` and it falls back to
on/off `digitalWrite`. For more volume, drive a louder buzzer through an NPN.

## Phases

Status: ☐ todo · ◐ in progress · ☑ done

### Phase A — Board I/O foundation  ☑
New `board.cpp/.h`. `board_init()` sets pin modes, drives `PIN_12V_EN` high at
boot, `PIN_SRV_EN` low (servo unpowered during the pad sit — saves battery, no
hour of servo chatter), switches `INPUT_PULLUP`, buzzer idle.
APIs: `board_switches_armed()` (both switches in armed position),
`board_servo_power(bool)`, buzzer (Phase D). HIL build: switches read as armed
(no hardware), `board_servo_power` is a no-op-safe stub.

### Phase B — Switch-gated arming (pad hardening)  ☑
Replace the pure 10 s auto-arm with: **storage logging ready AND pad reference
captured AND minimum settle elapsed AND both arm switches ON → ARMED.** While a
switch is OFF the FC holds in IDLE with launch detection disabled. ARMED → IDLE
if a switch leaves the armed position — but **only before BOOST**; once BOOST is
detected the flight is latched and switches are ignored (a switch vibrating open
mid-flight must not abort airbrakes). Servo power follows: on at ARM, off at
IDLE. Minimum sensor-health gate: require IMU + baro present to arm (the CF needs
both); everything else is allowed to be degraded ("fail toward flying").

### Phase C — Watchdog  ☑ (WDT_T4 lib, WDT1, 2 s; fed in both loops)
Enable a hardware watchdog (~2 s). Feed it in the main loop and around long ops
(storage flush, radio TX). On a hang it resets; the FC reboots, re-inits, returns
to IDLE, and re-arms automatically if storage + switches + pad ref still hold —
so a wedge during the 1-hour pad sit self-heals with no human action. Mid-flight
reset is best-effort only (the FC will reboot but likely miss the rest of the
flight; acceptable because recovery is independent). Document this clearly.

### Phase D — Buzzer status patterns (on-pad feedback, no TX)  ☑
Non-blocking tone scheduler. Patterns (audible confirmation the operator on the
ladder relies on, since there is no telemetry to the rocket):
- **Storage / can't-arm fault** — urgent fast triple, repeating.
- **Pre-arm, waiting** — slow single beep (alive, not yet armed).
- **Armed** — distinct rising chirp on the ARM transition, then occasional short
  chirp so you can hear "armed" from the pad.
- **GPS 3D lock acquired** — one chirp.

### Phase E — Servo motion profile  ☑
`SERVO_MIN_US = 500`, `SERVO_MAX_US = 1833`, **inverted** mapping
(fraction 0 → MAX_US retracted, 1 → MIN_US deployed). Smooth init sweep from the
assumed center pulse to fully retracted over ~800 ms (avoids the attach() center
snap). In-flight rate limiting already exists (full stroke in 0.24 s). Mirror the
same motion profile (rate limit + the inverted/endpoint semantics where they
affect the reported fraction) in `apex_sim/hil/fake_teensy.py`.

### Phase F — Logging diagnostics  ☐
- Record firmware **git/build hash** in the boot log record (currently only a
  config hash). Inject via a build-time `-DAPEX_GIT_HASH=...` flag.
- `BENCH_STORAGE` serial command: time N record writes to QSPI and SD, report
  mean/p95/max latency. (Author can't flash the board, so this ships for the
  user to run; results decide whether SD stays real-time or mirrors after
  landing — the open question in logging.md.)

### Phase G — Magnetometer hard-iron calibration  ☐
Running per-axis min/max tracker on the raw magnetometer while IDLE (the pad
handling/rotation naturally sweeps orientation); hard-iron offset = midpoint,
subtracted before fusion. Counters the PCB's own iron. Marginal for airbrake
control (mag mainly fixes yaw, which barely affects the vertical estimate) — kept
lightweight, no soft-iron ellipsoid fit, no extra library.

### Phase H — Tuning (user-run, documented, NOT code)  ☐
- **PID retuning workflow** via the HIL loop — documented in this file.
- Dynamic mach gate: **intentionally skipped** (one more failure point for ~10%
  accuracy; the fixed 240 m/s gate stays).

## PID retuning workflow (Phase H reference)

The airbrake PID lives in two mirrored places — keep them in sync:
`fsw/src/config.h` (`PID_KP/KI/KD`, flight) and `sim/config/airbrakes.yaml`
(RocketPy reference). Error = predicted_apogee − target; D-term uses velocity.

1. **Baseline:** run the closed loop — `python scripts/run_hil.py --fake` (or
   against real `teensy41_hil`). Note apogee vs the 10,000 ft target and watch
   the deployment trace on the HORIZON HIL tab.
2. **Read the behavior:**
   - Overshoots target, brakes saturate at 100% late → need earlier/stronger
     response: raise |Kp|, or reduce the post-burnout lockout (already 1000 ms).
   - Oscillates / hunts around target → too aggressive: lower |Kp|, add a touch
     of |Kd| (velocity damping).
   - Slow steady drift off target → integral: nudge |Ki| (small; Ki is negative).
3. **Change one gain**, re-run, compare apogee error and deployment smoothness.
4. **Dispersion / "Monte-Carlo":** run many `--fake` flights with noise on
   (default) and varied seeds (`--seed`) / sensor error to see the apogee spread,
   not just one trajectory. Tune for a tight distribution near target, not a
   single perfect run.
5. When happy, set the same gains in **both** config.h and airbrakes.yaml.

Note on Cd: `delta_cd_max = 0.324` is the best brake-authority figure we will
have; if the rocket simply can't shed enough energy the PID saturates at 100% and
apogee lands high — that's a real-world performance limit, not a tuning bug.

## Out of scope (confirmed)
- Recovery / pyro (separate altimeter, unchanged from test flight).
- Battery ADC sensing (no hardware for it).
- Dynamic mach gate.
