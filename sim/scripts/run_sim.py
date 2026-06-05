#!/usr/bin/env python3
"""Run the Apex RocketPy simulation.

Usage
-----
Run from apex/sim/ with the venv active (source apex-setup):

  # Competition sim — IREC Pecos TX, airbrakes active
  python scripts/run_sim.py

  # Validation run — Seymour TX, clean flight, no brakes
  python scripts/run_sim.py --baseline --site seymour_tx_2026_05_24

  # Validate and compare against real Seymour TX flight data (stored OR trace)
  python scripts/run_sim.py --compare

  # Same comparison but also run a live OpenRocket sim with matched conditions
  python scripts/run_sim.py --compare --run-or

  # Live OR with weathercock angle (15.33° observed at t=1.12s, Seymour TX)
  python scripts/run_sim.py --compare --run-or --or-rod-angle 15.33

  # Force a specific atmospheric model
  python scripts/run_sim.py --model standard_atmosphere

  # Save summary to output/
  python scripts/run_sim.py --save
"""

import argparse
import sys
import warnings
from pathlib import Path

# macOS ships LibreSSL; urllib3 v2 dropped LibreSSL support and warns on every import.
warnings.filterwarnings("ignore", message=".*LibreSSL.*", category=Warning)
warnings.filterwarnings("ignore", message=".*NotOpenSSLWarning.*", category=Warning)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apex_sim.analysis.compare import FlightMetrics, plot_traces, print_comparison
from apex_sim.config.loader import load_environment
from apex_sim.sim.flight import run_baseline, run_flight

_SEYMOUR_SITE = "seymour_tx_2026_05_24"


def main() -> None:
    parser = argparse.ArgumentParser(description="Apex RocketPy flight simulation")
    parser.add_argument(
        "--baseline",
        action="store_true",
        help="Clean flight — no airbrakes (use for validation against real data)",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help=(
            "Run a clean-flight baseline against the Seymour TX site and print a "
            "comparison table vs TeleMega, Blue Raven, and OpenRocket. "
            "Implies --baseline and --site seymour_tx_2026_05_24."
        ),
    )
    parser.add_argument(
        "--site",
        metavar="PROFILE",
        default=None,
        help="Site profile stem from config/sites/ (default: active_site in environment.yaml)",
    )
    parser.add_argument(
        "--model",
        metavar="MODEL",
        default=None,
        help="Override atmosphere model: RAP, NAM, GFS, ERA5, standard_atmosphere",
    )
    parser.add_argument(
        "--full-descent",
        action="store_true",
        help="Simulate through landing (default: stop at apogee)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Write summary CSV and plots to output/",
    )
    parser.add_argument(
        "--run-or",
        action="store_true",
        help=(
            "Run a live OpenRocket simulation with conditions matched to RocketPy "
            "and include it in the comparison. Requires Java + data/openrocket/OpenRocket-23.09.jar."
        ),
    )
    parser.add_argument(
        "--or-rod-angle",
        metavar="DEG",
        type=float,
        default=None,
        help=(
            "Launch rod angle from VERTICAL for the OR sim, in degrees "
            "(0 = straight up, 15.33 = Seymour TX observed weathercock). "
            "Defaults to rail inclination from site config."
        ),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print RocketPy integration progress",
    )
    args = parser.parse_args()

    if args.compare:
        args.baseline = True
        if args.site is None:
            args.site = _SEYMOUR_SITE

    env_cfg = load_environment(site_override=args.site)

    if args.model:
        env_cfg.atmosphere.model_override = args.model

    terminate_on_apogee = not args.full_descent

    print(f"Site:       {env_cfg.site.name}")
    print(f"Sim time:   {env_cfg.site.launch_window.sim_datetime}")
    print(f"Airbrakes:  {'disabled (baseline)' if args.baseline else 'active (PID)'}")
    print()

    if args.baseline:
        flight = run_baseline(
            site_override=args.site,
            terminate_on_apogee=terminate_on_apogee,
            verbose=args.verbose,
        )
    else:
        flight = run_flight(
            env_cfg=env_cfg,
            active_airbrakes=True,
            terminate_on_apogee=terminate_on_apogee,
            verbose=args.verbose,
        )

    apogee_m = flight.apogee - flight.env.elevation
    max_v_ms = flight.max_speed
    t_apogee = flight.apogee_time

    print(f"Apogee AGL:     {apogee_m:.1f} m  ({apogee_m / 0.3048:.0f} ft)")
    print(f"Max velocity:   {max_v_ms:.1f} m/s  ({max_v_ms / 0.3048:.0f} ft/s)")
    print(f"Time to apogee: {t_apogee:.1f} s")

    if args.compare:
        sim_metrics = FlightMetrics(
            source="RocketPy (ISA)",
            max_alt_agl_m=round(apogee_m, 1),
            max_velocity_ms=round(max_v_ms, 1),
            time_to_apogee_s=round(t_apogee, 1),
        )

        or_live_metrics = None
        or_live_trace = None
        if args.run_or:
            from apex_sim.analysis.openrocket_runner import write_or_file, read_or_results
            ork_out = write_or_file(
                site=env_cfg.site,
                atmosphere_cfg=env_cfg.atmosphere,
                rail_cfg=env_cfg.rail,
                launch_rod_angle_deg=args.or_rod_angle,
            )
            result = read_or_results(ork_out)
            if result is None:
                print(
                    f"\n  OR file written to {ork_out}\n"
                    f"  Open it in OpenRocket, run simulation 'IREC 2026', save, then re-run.\n"
                )
            else:
                or_live_metrics, or_live_trace = result
                print(f"OpenRocket (matched): {or_live_metrics.max_alt_agl_m:.1f} m  "
                      f"({or_live_metrics.max_alt_agl_m / 0.3048:.0f} ft)")

        print_comparison(sim_metrics, or_live=or_live_metrics)
        out = plot_traces(flight, or_live_trace=or_live_trace)
        if out:
            print(f"  Trace plot saved to {out}")

    if args.save:
        out_dir = Path(__file__).resolve().parents[1] / "output"
        out_dir.mkdir(exist_ok=True)
        flight.export_data(str(out_dir / "flight_data.csv"))
        print(f"Saved to {out_dir}/")


if __name__ == "__main__":
    main()
