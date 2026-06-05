"""Compare RocketPy sim results against real Seymour TX flight data.

Data sources
------------
TeleMega s/n 16370 (KJ5LDI) — full ascent trace, metric, barometric altitude
Blue Raven SN1403             — first 7 s only (powered + early coast), imperial
OpenRocket                    — time-series from stored .ork simulation output

Baro is the competition-relevant measurement.  BR baro (10,523 ft) and TeleMega
baro (10,553 ft) agree within 30 ft — treat their midpoint (~10,538 ft / 3212 m)
as ground truth.

Blue Raven "Inertial navigation max alt" (11,338 ft / 3456 m) has no AGL/ASL
label in the summary CSV.  Pad altitude ASL is listed separately as 1,199 ft.
If the inertial figure is ASL, AGL = 10,139 ft — below both baro readings,
implying downward drift from double-integrating accelerometer bias over the coast.
Either interpretation makes it unreliable; it is shown for reference only.

OpenRocket .ork datapoint column layout (54 columns):
  0  time (s)    1  altitude AGL (m)    4  total speed (m/s)
Confirmed by matching col-1 max (3447.8 m) and col-4 max (287.4 m/s) against
the known OpenRocket scalar output for this .ork file.
"""

from __future__ import annotations

import io
import re
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

_FLIGHT_DIR = Path(__file__).resolve().parents[2] / "data" / "flights" / "seymour_2026_05_24"
_OR_FILE = Path(__file__).resolve().parents[2] / "data" / "openrocket" / "Team307_TexasTechUniversity_PR3.ork"
_OUTPUT_DIR = Path(__file__).resolve().parents[2] / "output"
_FT_PER_M = 0.3048


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------


@dataclass
class FlightMetrics:
    """Key scalar outcomes from one data source."""

    source: str
    max_alt_agl_m: float | None = None
    max_velocity_ms: float | None = None
    time_to_apogee_s: float | None = None


@dataclass
class FlightTrace:
    """Time-series trajectory from one data source."""

    source: str
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    alt_m: np.ndarray = field(default_factory=lambda: np.array([]))
    speed_ms: np.ndarray = field(default_factory=lambda: np.array([]))


# ---------------------------------------------------------------------------
# Scalar loaders
# ---------------------------------------------------------------------------


def load_telemega() -> FlightMetrics:
    """Parse TeleMega AltosUI CSV and extract key flight metrics."""
    _, tm = _read_telemega()
    flight = tm[tm["height"] > 0]
    return FlightMetrics(
        source="TeleMega (baro)",
        max_alt_agl_m=round(float(flight["height"].max()), 1),
        max_velocity_ms=round(float(flight["speed"].max()), 1),
        time_to_apogee_s=round(float(tm.loc[tm["height"].idxmax(), "time"]), 1),
    )


def load_blueraven() -> tuple[FlightMetrics, FlightMetrics]:
    """Parse Blue Raven summary CSV and return baro and inertial metrics."""
    path = _FLIGHT_DIR / "blueraven_summary_2026-05-24.csv"

    def _get(label: str) -> float | None:
        with path.open() as f:
            for line in f:
                if line.startswith(label + ","):
                    m = re.search(r"[-+]?\d+\.?\d*", line.split(",", 1)[1])
                    return float(m.group()) if m else None
        return None

    baro_ft = _get("Max Altitude")
    inertial_ft = _get("Inertial navigation max alt")
    vel_fps = _get("Max velocity")
    # "Time to Apo channel fire" includes pyro firing delay after apogee detection.
    t_apo = _get("Time to Apo channel fire")

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


# ---------------------------------------------------------------------------
# Trace loaders
# ---------------------------------------------------------------------------


def load_telemega_trace() -> FlightTrace:
    """Load TeleMega time-series for the full ascent (t ≥ 0 to apogee)."""
    _, tm = _read_telemega()
    ascent = tm[tm["time"] >= 0].drop_duplicates(subset=["time"]).copy()
    ascent = ascent[ascent["height"] >= 0]  # trim any post-apogee rows
    return FlightTrace(
        source="TeleMega (baro)",
        t=ascent["time"].to_numpy(dtype=float),
        alt_m=ascent["height"].to_numpy(dtype=float),
        speed_ms=ascent["speed"].to_numpy(dtype=float),
    )


def load_blueraven_trace() -> FlightTrace:
    """Load Blue Raven time-series.

    The CSV covers only the first ~7 s (powered flight + early coast).
    Velocity_Up is in ft/s; Baro_Altitude_AGL_(feet) is in feet.
    """
    path = _FLIGHT_DIR / "blueraven_flight_2026-05-24.csv"
    br = pd.read_csv(path)
    flight = br[br["Flight_Time_(s)"] >= 0].drop_duplicates(
        subset=["Flight_Time_(s)"]
    ).copy()
    return FlightTrace(
        source="Blue Raven (baro, 0–7 s)",
        t=flight["Flight_Time_(s)"].to_numpy(dtype=float),
        alt_m=flight["Baro_Altitude_AGL_(feet)"].to_numpy(dtype=float) * _FT_PER_M,
        speed_ms=flight["Velocity_Up"].to_numpy(dtype=float) * _FT_PER_M,
    )


def rocketpy_trace(flight, n_points: int = 500) -> FlightTrace:
    """Sample a RocketPy Flight object into a FlightTrace.

    Parameters
    ----------
    flight : rocketpy.Flight
        Completed simulation.
    n_points : int
        Number of time samples.
    """
    t_end = flight.apogee_time
    t = np.linspace(0.0, t_end, n_points)
    alt = np.array([flight.z(ti) - flight.env.elevation for ti in t])
    spd = np.array([flight.speed(ti) for ti in t])
    return FlightTrace(source="RocketPy (ISA atm)", t=t, alt_m=alt, speed_ms=spd)


def load_openrocket_trace(or_file: Path = _OR_FILE) -> FlightTrace:
    """Load OpenRocket time-series from the stored .ork simulation output.

    The .ork is a ZIP containing rocket.ork XML.  Each <datapoint> row has 54
    comma-separated values; col 0 = time (s), col 1 = altitude AGL (m),
    col 4 = total speed (m/s).  Only ascent rows (col 2 vertical vel ≥ 0) are
    returned so the trace ends at apogee.
    """
    with zipfile.ZipFile(or_file) as z:
        xml_bytes = z.read("rocket.ork")
    root = ET.fromstring(xml_bytes)
    db = root.find(".//databranch")
    if db is None:
        return FlightTrace(source="OpenRocket")
    rows = []
    for dp in db.findall("datapoint"):
        if dp.text:
            rows.append([float(x) for x in dp.text.strip().split(",")])
    arr = np.array(rows)
    ascent = arr[arr[:, 2] >= 0]  # keep rows where vertical velocity ≥ 0
    return FlightTrace(
        source="OpenRocket",
        t=ascent[:, 0],
        alt_m=ascent[:, 1],
        speed_ms=ascent[:, 4],
    )


def load_openrocket_metrics(or_file: Path = _OR_FILE) -> FlightMetrics:
    """Extract scalar metrics from the stored OpenRocket .ork simulation."""
    trace = load_openrocket_trace(or_file)
    if trace.t.size == 0:
        return FlightMetrics(source="OpenRocket")
    apogee_idx = int(np.argmax(trace.alt_m))
    return FlightMetrics(
        source="OpenRocket",
        max_alt_agl_m=round(float(trace.alt_m[apogee_idx]), 1),
        max_velocity_ms=round(float(trace.speed_ms.max()), 1),
        time_to_apogee_s=round(float(trace.t[apogee_idx]), 1),
    )


# ---------------------------------------------------------------------------
# Scalar comparison table
# ---------------------------------------------------------------------------


def print_comparison(
    sim: FlightMetrics,
    or_live: FlightMetrics | None = None,
) -> None:
    """Print a scalar comparison table of sim results vs all real-data sources.

    Parameters
    ----------
    sim : FlightMetrics
        RocketPy simulation result.
    or_live : FlightMetrics, optional
        Live OR run from ``openrocket_runner.run_openrocket_sim``.  When
        provided, replaces the stored-trace OR column.
    """
    telemega = load_telemega()
    br_baro, br_inertial = load_blueraven()
    # Live OR replaces stored-trace metrics when available.
    or_metrics = or_live if or_live is not None else load_openrocket_metrics()
    # Post-flight OR re-run with 15.33° weathercock observed at t=1.12s.
    # Only shown when we're using stored data — live run already sets its own angle.
    or_weathercocked = (
        None
        if or_live is not None
        else FlightMetrics(
            source="OR (weathercocked)",
            max_alt_agl_m=round(10438 * _FT_PER_M, 1),
        )
    )

    sources = [sim, telemega, br_baro, br_inertial, or_metrics]
    if or_weathercocked is not None:
        sources.append(or_weathercocked)

    def _alt(m):
        return f"{m:.0f} m  ({m / _FT_PER_M:.0f} ft)" if m is not None else "—"

    def _vel(ms):
        return f"{ms:.1f} m/s  ({ms / _FT_PER_M:.0f} ft/s)" if ms is not None else "—"

    def _time(s, note=""):
        return f"{s:.1f} s{note}" if s is not None else "—"

    col_w, label_w = 26, 16
    div = "  " + "-" * (label_w + col_w * len(sources))

    def row(label, vals):
        return "  " + f"{label:<{label_w}}" + "".join(f"{v:<{col_w}}" for v in vals)

    print()
    print("  Seymour TX — sim vs real flight data")
    print(div)
    print(row("", [s.source for s in sources]))
    print(div)
    print(row("Max alt AGL",    [_alt(s.max_alt_agl_m) for s in sources]))
    print(row("Max velocity",   [_vel(s.max_velocity_ms) for s in sources]))
    t_notes = (["", "", " †", "", ""] + ([""] if or_weathercocked else []))
    print(row("Time to apogee", [_time(s.time_to_apogee_s, t_notes[i]) for i, s in enumerate(sources)]))
    print(div)

    # Baro consensus is competition-relevant ground truth.
    # BR inertial excluded — ASL/AGL ambiguous, shows likely drift.
    baro_ref = (
        (telemega.max_alt_agl_m + br_baro.max_alt_agl_m) / 2
        if telemega.max_alt_agl_m and br_baro.max_alt_agl_m
        else telemega.max_alt_agl_m or br_baro.max_alt_agl_m
    )
    print()
    for src in [sim, or_metrics] + ([or_weathercocked] if or_weathercocked else []):
        if src and src.max_alt_agl_m and baro_ref:
            d = src.max_alt_agl_m - baro_ref
            print(f"  {src.source:<30} vs baro consensus: {d:+.0f} m ({100*d/baro_ref:+.1f}%)")

    print(
        "\n  † Blue Raven time is apogee-channel fire, not true apogee — includes pyro delay."
        "\n  BR inertial alt shown for reference — no AGL/ASL label in summary CSV; likely"
        "\n  unreliable due to accelerometer drift over coast phase."
        "\n"
    )


# ---------------------------------------------------------------------------
# Trace plot
# ---------------------------------------------------------------------------


def plot_traces(
    sim_flight,
    or_live_trace: FlightTrace | None = None,
    save: bool = True,
) -> Path | None:
    """Plot altitude and velocity traces for RocketPy, OR, TeleMega, and Blue Raven.

    Parameters
    ----------
    sim_flight : rocketpy.Flight
        Completed RocketPy simulation.
    or_live_trace : FlightTrace, optional
        Live OR trace from ``openrocket_runner.run_openrocket_sim``.  When
        provided, replaces the stored .ork trace.
    save : bool
        Write the figure to ``output/comparison.png``.

    Returns
    -------
    Path or None
        Path to the saved figure, or None if matplotlib is unavailable.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend — no display required
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot")
        return None

    sim_trace = rocketpy_trace(sim_flight)
    tm_trace = load_telemega_trace()
    br_trace = load_blueraven_trace()
    or_trace = or_live_trace if or_live_trace is not None else load_openrocket_trace()

    burnout_t = 3.6  # from Blue Raven summary

    fig, (ax_alt, ax_vel) = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    fig.suptitle("Seymour TX — RocketPy vs real flight  (ISA atmosphere, no airbrakes)")

    # Altitude
    ax_alt.plot(sim_trace.t, sim_trace.alt_m, label="RocketPy (ISA)", color="steelblue", lw=2)
    ax_alt.plot(or_trace.t, or_trace.alt_m, label=or_trace.source, color="mediumpurple", lw=1.5, ls="-.")
    ax_alt.plot(tm_trace.t, tm_trace.alt_m, label="TeleMega (baro)", color="darkorange", lw=1.5, ls="--")
    ax_alt.plot(br_trace.t, br_trace.alt_m, label="Blue Raven baro (0–7 s)", color="seagreen", lw=1.5, ls=":")
    ax_alt.axvline(burnout_t, color="gray", ls="--", lw=1, alpha=0.6, label=f"Burnout (~{burnout_t} s)")
    ax_alt.set_ylabel("Altitude AGL (m)")
    ax_alt.legend(fontsize=9)
    ax_alt.grid(True, alpha=0.3)

    # Velocity
    ax_vel.plot(sim_trace.t, sim_trace.speed_ms, label="RocketPy (ISA)", color="steelblue", lw=2)
    ax_vel.plot(or_trace.t, or_trace.speed_ms, label=or_trace.source, color="mediumpurple", lw=1.5, ls="-.")
    ax_vel.plot(tm_trace.t, tm_trace.speed_ms, label="TeleMega", color="darkorange", lw=1.5, ls="--")
    ax_vel.plot(br_trace.t, br_trace.speed_ms, label="Blue Raven (0–7 s)", color="seagreen", lw=1.5, ls=":")
    ax_vel.axvline(burnout_t, color="gray", ls="--", lw=1, alpha=0.6)
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Speed (m/s)")
    ax_vel.legend(fontsize=9)
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()

    if save:
        _OUTPUT_DIR.mkdir(exist_ok=True)
        out = _OUTPUT_DIR / "comparison.png"
        fig.savefig(out, dpi=150)
        plt.close(fig)
        return out

    plt.close(fig)
    return None


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _read_telemega() -> tuple[pd.DataFrame, pd.DataFrame]:
    """Return (raw_df, cleaned_df) from the TeleMega CSV."""
    path = _FLIGHT_DIR / "telemega_flight_2026-05-24.csv"
    with path.open() as f:
        raw = f.read().replace("#version", "version", 1)
    df = pd.read_csv(io.StringIO(raw))
    df.columns = df.columns.str.strip()
    return df, df
