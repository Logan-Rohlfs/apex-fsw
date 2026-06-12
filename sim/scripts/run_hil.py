#!/usr/bin/env python3
"""Apex real-time closed-loop HIL.

Runs a live RocketPy flight simulation with the flight computer in the loop:
sensor data synthesised from the sim is injected over USB serial, the Teensy
runs fusion / state machine / airbrake control, and the returned deployment
fraction feeds back into the simulated drag.

Usage (from apex/sim/ with venv active):

    # Against real hardware (Teensy flashed with: pio run -e teensy41_hil -t upload)
    python scripts/run_hil.py
    python scripts/run_hil.py --port /dev/cu.usbmodem<N>

    # No hardware — in-process fake Teensy (reference flight-computer model)
    python scripts/run_hil.py --fake
    python scripts/run_hil.py --fake --speed 0     # max speed (fake only)

Options:
    --port      Serial port (default: auto-detect Teensy)
    --fake      Use the in-process FakeTeensy instead of hardware
    --speed     Pacing: 1.0 = real time (default; required for real hardware
                because the firmware CF integrates wall-clock dt), 0 = max
    --site      Site profile override (default: active_site in environment.yaml)
    --model     Atmosphere model (default: standard_atmosphere — offline and
                deterministic; pass 'auto' for the forecast ladder)
    --warmup    Seconds of on-pad streaming before ignition (default: 6.0)
    --full      Simulate past apogee instead of terminating there
    --max-time  Simulation time limit in seconds (default: 120)
    --out       CSV path for the tick log (default: output/hil_run_<ts>.csv)

The firmware implements the full chain (fusion, state machine, PID, servo);
apex_sim/hil/fake_teensy.py is the matching reference model — a real-Teensy
run and a --fake run should behave the same. If they diverge, the firmware
port has drifted from the reference.
"""

from __future__ import annotations

import argparse
import csv
import sys
import warnings
from datetime import datetime
from pathlib import Path

warnings.filterwarnings("ignore", message=".*LibreSSL.*", category=Warning)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apex_sim.config.loader import load_environment                    # noqa: E402
from apex_sim.hil.emulator import SensorErrors                         # noqa: E402
from apex_sim.hil.link import HilLink                                  # noqa: E402
from apex_sim.hil.protocol import PHASE_NAMES, find_port               # noqa: E402
from apex_sim.hil.runner import HilRow, run_closed_loop                # noqa: E402
from apex_sim.sim.environment import build_environment                 # noqa: E402
from apex_sim.sim.rocket import build_rocket                           # noqa: E402

_SIM_ROOT = Path(__file__).resolve().parents[1]

_FT_PER_M = 3.28084   # apogee figures display in feet — the target is 10,000 ft

RESET = "\033[0m"; BOLD = "\033[1m"; RED = "\033[31m"; GRN = "\033[32m"; YEL = "\033[33m"
PHASE_COLOR = {"IDLE": "\033[90m", "ARMED": "\033[34m", "BOOST": "\033[33m",
               "COAST": "\033[32m", "DESCENT": "\033[35m", "LANDED": "\033[36m"}


def main():
    ap = argparse.ArgumentParser(description="Apex real-time closed-loop HIL")
    ap.add_argument("--port", default=None)
    ap.add_argument("--fake", action="store_true",
                    help="use the in-process fake Teensy (no hardware)")
    ap.add_argument("--compare-fake", action="store_true",
                    help="shadow the same run through FakeTeensy; real hardware remains primary")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="pacing factor; 1.0 = real time, 0 = max (fake only)")
    ap.add_argument("--site", default=None)
    ap.add_argument("--model", default="standard_atmosphere",
                    help="atmosphere model, or 'auto' for the forecast ladder")
    ap.add_argument("--warmup", type=float, default=6.0)
    ap.add_argument("--full", action="store_true",
                    help="simulate past apogee (default: stop at apogee)")
    ap.add_argument("--no-noise", action="store_true",
                    help="disable the per-sensor noise model (deterministic run)")
    ap.add_argument("--sensor-seed", type=int, default=None,
                    help="RNG seed for reproducible noisy realism runs")
    ap.add_argument("--accel-bias-mg", default="0,0,0",
                    help="fixed ICM accel bias x,y,z in milli-g")
    ap.add_argument("--highg-bias-mg", default="0,0,0",
                    help="fixed ADXL375 accel bias x,y,z in milli-g")
    ap.add_argument("--gyro-bias-dps", default="0,0,0",
                    help="fixed gyro bias x,y,z in deg/s")
    ap.add_argument("--mag-bias-mg", default="0,0,0",
                    help="fixed magnetometer bias x,y,z in milli-gauss")
    ap.add_argument("--baro-bias-pa", type=float, default=0.0,
                    help="fixed barometer pressure bias in Pa")
    ap.add_argument("--gps-bias-m", type=float, default=0.0,
                    help="fixed GPS altitude bias in metres")
    ap.add_argument("--misalign-deg", default="0,0,0",
                    help="fixed sensor mount rotation x,y,z in degrees")
    ap.add_argument("--sensor-delay-ms", type=float, default=0.0,
                    help="integer-tick sensor delay; rounded to 10 ms HIL ticks")
    ap.add_argument("--max-time", type=float, default=120.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.fake and args.compare_fake:
        print(f"{YEL}--compare-fake is ignored when --fake is the primary FC.{RESET}")
        args.compare_fake = False

    if not args.fake and args.speed != 1.0:
        print(f"{YEL}WARNING: speed={args.speed} against real hardware distorts the "
              f"firmware velocity estimate (CF uses wall-clock dt). Use 1.0.{RESET}")

    # ── Build sim ──────────────────────────────────────────────────────────────
    env_cfg = load_environment(site_override=args.site)
    if args.model != "auto":
        env_cfg.atmosphere.model_override = args.model
    print(f"Site: {env_cfg.site.name}  |  atmosphere: "
          f"{env_cfg.atmosphere.model_override or 'auto'}")
    env = build_environment(env_cfg)
    rocket = build_rocket()
    airbrakes_cfg = _load_airbrakes()

    # ── Open link ──────────────────────────────────────────────────────────────
    fake = None
    fake_shadow = None
    shadow_links = {}
    if args.fake:
        from apex_sim.hil.fake_teensy import FakeTeensy
        fake = FakeTeensy()
        port = fake.port
        print(f"Fake Teensy on {port}")
    else:
        port = args.port or find_port()
        if not port:
            print(f"{RED}No Teensy found. Specify --port or use --fake.{RESET}")
            sys.exit(1)
        print(f"Port: {port}")

    link = HilLink(port)
    try:
        if not args.fake:
            link.reset_input()
        if not link.wait_for_line("READY", timeout=15.0):
            print(f"{RED}No #HIL_READY within 15 s. Check the APEX_HIL build "
                  f"is flashed (pio run -e teensy41_hil -t upload).{RESET}")
            sys.exit(1)
        print(f"{GRN}Flight computer ready.{RESET}  Warming up "
              f"({args.warmup:.0f} s on pad)...")

        if args.compare_fake:
            from apex_sim.hil.fake_teensy import FakeTeensy
            fake_shadow = FakeTeensy()
            fake_link = HilLink(fake_shadow.port)
            if not fake_link.wait_for_line("READY", timeout=5.0):
                print(f"{RED}Fake shadow did not become ready.{RESET}")
                sys.exit(1)
            shadow_links["fake"] = fake_link
            print(f"{GRN}Fake shadow ready.{RESET}  Same sensor packets will be mirrored.")

        # ── Live tick printer (1 Hz of sim time + transitions) ────────────────
        state = {"phase": None, "count": 0}

        def tick(row: HilRow):
            state["count"] += 1
            if row.reply is None:
                return
            name = PHASE_NAMES.get(row.reply.phase, "?")
            changed = name != state["phase"]
            state["phase"] = name
            if changed or state["count"] % 100 == 0:
                c = PHASE_COLOR.get(name, "")
                mark = f"{BOLD}{YEL}>>> {RESET}" if changed else "    "
                print(f"{mark}t={row.t_s:6.2f}s  {c}{name:8}{RESET}"
                      f"  alt {row.reply.est_alt_agl_m:7.1f}/{row.true_alt_agl_m:7.1f} m"
                      f"  vel {row.reply.est_vel_mps:6.1f} m/s"
                      f"  pred {row.reply.pred_apogee_m*_FT_PER_M:7.0f} ft"
                      f"  brakes {row.reply.deployment_frac*100:5.1f}%"
                      f"  lat {row.latency_ms:4.1f} ms")

        result = run_closed_loop(
            link, env, rocket, env_cfg,
            airbrakes_cfg=airbrakes_cfg,
            speed=args.speed, warmup_s=args.warmup,
            terminate_on_apogee=not args.full, max_time=args.max_time,
            noise=not args.no_noise,
            sensor_kwargs=_sensor_kwargs(args),
            shadow_links=shadow_links,
            tick_cb=tick)
    finally:
        link.close()
        for shadow in shadow_links.values():
            shadow.close()
        if fake is not None:
            fake.close()
        if fake_shadow is not None:
            fake_shadow.close()

    # ── CSV log ────────────────────────────────────────────────────────────────
    out_path = Path(args.out) if args.out else (
        _SIM_ROOT / "output" / f"hil_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["t_s", "true_alt_agl_m", "true_vel_z_mps",
                  "est_alt_agl_m", "est_vel_mps", "pred_apogee_m",
                  "deployment_frac", "phase", "latency_ms"]
        if "fake" in shadow_links:
            header.extend([
                "fake_est_alt_agl_m", "fake_est_vel_mps", "fake_pred_apogee_m",
                "fake_deployment_frac", "fake_phase", "fake_latency_ms",
                "fake_minus_primary_alt_m", "fake_minus_primary_vel_mps",
                "fake_minus_primary_deployment",
            ])
        w.writerow(header)
        for r in result.rows:
            if r.reply is not None:
                row = [f"{r.t_s:.3f}", f"{r.true_alt_agl_m:.2f}",
                       f"{r.true_vel_z_mps:.3f}",
                       f"{r.reply.est_alt_agl_m:.2f}",
                       f"{r.reply.est_vel_mps:.3f}",
                       f"{r.reply.pred_apogee_m:.1f}",
                       f"{r.reply.deployment_frac:.4f}",
                       PHASE_NAMES.get(r.reply.phase, r.reply.phase),
                       f"{r.latency_ms:.2f}"]
                fake_pkt = r.shadow_replies.get("fake")
                if "fake" in shadow_links:
                    if fake_pkt is not None:
                        row.extend([
                            f"{fake_pkt.est_alt_agl_m:.2f}",
                            f"{fake_pkt.est_vel_mps:.3f}",
                            f"{fake_pkt.pred_apogee_m:.1f}",
                            f"{fake_pkt.deployment_frac:.4f}",
                            PHASE_NAMES.get(fake_pkt.phase, fake_pkt.phase),
                            f"{r.shadow_latencies_ms.get('fake', 0.0):.2f}",
                            f"{fake_pkt.est_alt_agl_m - r.reply.est_alt_agl_m:.2f}",
                            f"{fake_pkt.est_vel_mps - r.reply.est_vel_mps:.3f}",
                            f"{fake_pkt.deployment_frac - r.reply.deployment_frac:.4f}",
                        ])
                    else:
                        row.extend([""] * 9)
                w.writerow(row)
            else:
                row = [f"{r.t_s:.3f}", f"{r.true_alt_agl_m:.2f}",
                       f"{r.true_vel_z_mps:.3f}", "", "", "", "", "MISS", ""]
                if "fake" in shadow_links:
                    row.extend([""] * 9)
                w.writerow(row)

    # ── Summary ────────────────────────────────────────────────────────────────
    flight = result.flight
    replies = [r for r in result.rows if r.reply is not None]
    target = airbrakes_cfg["control"]["target_apogee_m"]
    apogee_agl = flight.apogee - env.elevation
    lat = sorted(result.latencies_ms)

    print(f"\n{'='*60}\n{BOLD}HIL Run Summary{RESET}")
    print(f"  Ticks: {len(result.rows)}  replies: {len(replies)}  "
          f"missed: {result.missed}  CRC errors: {result.crc_errors}")
    if lat:
        print(f"  Latency: mean={sum(lat)/len(lat):.1f} ms  "
              f"p95={lat[int(len(lat)*0.95)]:.1f} ms  max={lat[-1]:.1f} ms")
    print(f"  Sim apogee:     {apogee_agl*_FT_PER_M:7.0f} ft AGL  "
          f"(target {target*_FT_PER_M:.0f} ft, error {(apogee_agl-target)*_FT_PER_M:+.0f} ft"
          f" = {apogee_agl:.1f} m)")
    if replies:
        max_est = max(r.reply.est_alt_agl_m for r in replies)
        alt_err = [abs(r.reply.est_alt_agl_m - r.true_alt_agl_m) for r in replies]
        max_dep = max(r.reply.deployment_frac for r in replies)
        print(f"  FC max est alt: {max_est*_FT_PER_M:7.0f} ft AGL  "
              f"(|est-true| max {max(alt_err):.1f} m)")
        print(f"  Max deployment: {max_dep*100:.1f}%")
        if "fake" in shadow_links:
            pairs = [(r.reply, r.shadow_replies.get("fake")) for r in result.rows
                     if r.reply is not None and r.shadow_replies.get("fake") is not None]
            if pairs:
                max_alt_delta = max(abs(f.est_alt_agl_m - p.est_alt_agl_m)
                                    for p, f in pairs)
                max_vel_delta = max(abs(f.est_vel_mps - p.est_vel_mps)
                                    for p, f in pairs)
                max_dep_delta = max(abs(f.deployment_frac - p.deployment_frac)
                                    for p, f in pairs)
                phase_mismatch = sum(1 for p, f in pairs if p.phase != f.phase)
                print("  Fake shadow delta:")
                print(f"    max |alt|={max_alt_delta:.1f} m  "
                      f"max |vel|={max_vel_delta:.2f} m/s  "
                      f"max |deploy|={max_dep_delta*100:.1f}%  "
                      f"phase mismatches={phase_mismatch}/{len(pairs)}")
    print("  Phase transitions:")
    for t, name in result.transitions:
        print(f"    t={t:7.2f}s  →  {PHASE_COLOR.get(name, '')}{name}{RESET}")
    if result.lines:
        print("  FC messages:")
        for ln in result.lines:
            if ln.startswith("#"):
                print(f"    {ln}")
    print(f"  Log → {out_path}")
    print("=" * 60)


def _load_airbrakes() -> dict:
    import yaml
    with (_SIM_ROOT / "config" / "airbrakes.yaml").open() as fh:
        return yaml.safe_load(fh)


def _triple(text: str, scale: float = 1.0):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return tuple(float(p) * scale for p in parts)


def _sensor_kwargs(args) -> dict:
    g = 9.80665
    deg = 3.141592653589793 / 180.0
    delay_ticks = max(0, int(round(args.sensor_delay_ms / 10.0)))
    errors = SensorErrors(
        accel_bias_mss=_triple(args.accel_bias_mg, g / 1000.0),
        highg_bias_mss=_triple(args.highg_bias_mg, g / 1000.0),
        gyro_bias_rads=_triple(args.gyro_bias_dps, deg),
        mag_bias_gauss=_triple(args.mag_bias_mg, 1e-3),
        baro_bias_pa=args.baro_bias_pa,
        gps_alt_bias_m=args.gps_bias_m,
        misalignment_deg=_triple(args.misalign_deg),
        delay_ticks=delay_ticks,
    )
    return {"seed": args.sensor_seed, "errors": errors}


if __name__ == "__main__":
    main()
