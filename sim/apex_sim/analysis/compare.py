"""Compare RocketPy sim results against real Seymour TX flight data.

Data sources:
  TeleMega s/n 16370 (KJ5LDI) — barometric altimeter + IMU, metric units
  Blue Raven SN1403             — barometric + inertial nav, imperial units

Baro readings (TeleMega and BR baro) read ~250 m low vs BR inertial at apogee.
This is a known effect: dynamic pressure from high-velocity airflow over the
static port inflates the measured pressure, making the rocket appear lower than
it is. The BR inertial reading (3456 m) matches OpenRocket (3448 m) within 0.2%.
"""

from __future__ import annotations

import io
import re
from dataclasses import dataclass
from pathlib import Path

import pandas as pd

_FLIGHT_DIR = Path(__file__).resolve().parents[2] / "data" / "flights" / "seymour_2026_05_24"
_FT_PER_M = 0.3048


@dataclass
class FlightMetrics:
    """Key scalar outcomes from one data source."""

    source: str
    max_alt_agl_m: float | None = None
    max_velocity_ms: float | None = None
    time_to_apogee_s: float | None = None


def load_telemega() -> FlightMetrics:
    """Parse TeleMega AltosUI CSV export and extract key flight metrics."""
    path = _FLIGHT_DIR / "telemega_flight_2026-05-24.csv"
    with path.open() as f:
        raw = f.read()

    # Header row starts with '#version' — strip the '#' so pandas can use it
    cleaned = raw.replace("#version", "version", 1)
    df = pd.read_csv(io.StringIO(cleaned))
    df.columns = df.columns.str.strip()

    # Only use the ascending phase (height increasing, above pad) for timing
    flight = df[df["height"] > 0].copy()

    max_alt = flight["height"].max()
    max_vel = flight["speed"].max()
    t_apogee = float(flight.loc[flight["height"].idxmax(), "time"])

    return FlightMetrics(
        source="TeleMega (baro)",
        max_alt_agl_m=round(max_alt, 1),
        max_velocity_ms=round(max_vel, 1),
        time_to_apogee_s=round(t_apogee, 1),
    )


def load_blueraven() -> tuple[FlightMetrics, FlightMetrics]:
    """Parse Blue Raven summary CSV and return baro and inertial metrics."""
    path = _FLIGHT_DIR / "blueraven_summary_2026-05-24.csv"

    def _extract(label: str) -> float | None:
        with path.open() as f:
            for line in f:
                if line.startswith(label + ","):
                    val_str = line.split(",", 1)[1].strip()
                    m = re.search(r"[-+]?\d+\.?\d*", val_str)
                    return float(m.group()) if m else None
        return None

    baro_ft = _extract("Max Altitude")
    inertial_ft = _extract("Inertial navigation max alt")
    vel_fps = _extract("Max velocity")
    # "Time to Apo channel fire" includes pyro firing delay — not true apogee time,
    # but it's the closest timestamp the Blue Raven records for the apogee event.
    t_apo = _extract("Time to Apo channel fire")

    baro = FlightMetrics(
        source="Blue Raven (baro)",
        max_alt_agl_m=round(baro_ft * _FT_PER_M, 1) if baro_ft else None,
        max_velocity_ms=round(vel_fps * _FT_PER_M, 1) if vel_fps else None,
        time_to_apogee_s=t_apo,
    )
    inertial = FlightMetrics(
        source="Blue Raven (inertial)",
        max_alt_agl_m=round(inertial_ft * _FT_PER_M, 1) if inertial_ft else None,
    )
    return baro, inertial


def print_comparison(sim: FlightMetrics) -> None:
    """Print a comparison table of sim results vs all real-data sources.

    Parameters
    ----------
    sim : FlightMetrics
        Results from the RocketPy simulation.
    """
    telemega = load_telemega()
    br_baro, br_inertial = load_blueraven()

    or_ref = FlightMetrics(
        source="OpenRocket ref",
        max_alt_agl_m=3447.8,
        max_velocity_ms=287.4,
        time_to_apogee_s=26.1,
    )

    sources = [sim, telemega, br_baro, br_inertial, or_ref]

    def _fmt_alt(m: float | None) -> str:
        if m is None:
            return "—"
        return f"{m:.0f} m  ({m / _FT_PER_M:.0f} ft)"

    def _fmt_vel(ms: float | None) -> str:
        if ms is None:
            return "—"
        return f"{ms:.1f} m/s  ({ms / _FT_PER_M:.0f} ft/s)"

    def _fmt_time(s: float | None, note: str = "") -> str:
        if s is None:
            return "—"
        return f"{s:.1f} s{note}"

    col_w = 26
    headers = [s.source for s in sources]
    row_label_w = 16

    def _row(label: str, values: list[str]) -> str:
        return f"  {label:<{row_label_w}}" + "".join(f"{v:<{col_w}}" for v in values)

    divider = "  " + "-" * (row_label_w + col_w * len(sources))

    print()
    print("  Seymour TX — sim vs real flight data")
    print(divider)
    print(_row("", [f"{h:<{col_w}}" for h in headers]))
    print(divider)

    print(_row("Max alt AGL", [
        _fmt_alt(s.max_alt_agl_m) for s in sources
    ]))
    print(_row("Max velocity", [
        _fmt_vel(s.max_velocity_ms) for s in sources
    ]))

    # Note that BR apogee time is channel-fire time, not true apogee
    t_notes = ["", "", " †", "", ""]
    print(_row("Time to apogee", [
        _fmt_time(s.time_to_apogee_s, note=t_notes[i])
        for i, s in enumerate(sources)
    ]))

    print(divider)

    # Delta vs BR inertial (most accurate — matches OR within 0.2%)
    ref_alt = br_inertial.max_alt_agl_m
    if sim.max_alt_agl_m and ref_alt:
        delta_m = sim.max_alt_agl_m - ref_alt
        delta_pct = 100.0 * delta_m / ref_alt
        print(f"\n  RocketPy vs BR inertial: {delta_m:+.0f} m ({delta_pct:+.1f}%)")

    print(
        "\n  † Blue Raven time is apogee-channel fire, not true apogee"
        " — includes pyro delay."
    )
    print(
        "  Baro readings (TeleMega, BR baro) run ~250 m low vs inertial at apogee."
        "\n  Dynamic pressure on the static port during high-velocity coast inflates"
        "\n  measured pressure → barometer reads lower than actual altitude."
    )
    print()
