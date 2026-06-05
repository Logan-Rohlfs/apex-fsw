"""Generate and read OpenRocket simulations matched to our RocketPy conditions.

OR 23.09 explicitly blocks headless operation ("OpenRocket cannot currently be
run without the graphical user interface"), and JPype crashes on macOS 26 during
JVM stub initialisation.  We work around both by manipulating the .ork XML:

  1. ``write_or_file`` — copies the team .ork and patches the <conditions> block
     to match our RocketPy site/atmosphere/rail config, then saves to output/.
     Open the result in OpenRocket, run the simulation, and save.

  2. ``read_or_results`` — reads the simulation results back from a saved .ork.

Typical workflow
----------------
  python scripts/run_sim.py --compare --run-or        # writes output/or_matched.ork
  # open output/or_matched.ork in OpenRocket, run sim "IREC 2026", save
  python scripts/run_sim.py --compare --run-or        # now reads saved results
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from .compare import FlightMetrics, FlightTrace, _OR_FILE, _OUTPUT_DIR

import numpy as np

_OR_MATCHED = _OUTPUT_DIR / "or_matched.ork"

# Columns in the OR databranch datapoint rows (confirmed from .ork inspection).
_COL_TIME = 0
_COL_ALT  = 1
_COL_VVEL = 2   # vertical velocity — used to identify ascent
_COL_SPD  = 4   # total speed


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def write_or_file(
    site,
    atmosphere_cfg,
    rail_cfg,
    launch_rod_angle_deg: float | None = None,
    src_ork: Path = _OR_FILE,
    dst_ork: Path = _OR_MATCHED,
) -> Path:
    """Patch the team .ork with our conditions and save to output/or_matched.ork.

    Parameters
    ----------
    site : SiteProfile
    atmosphere_cfg : AtmosphereConfig
    rail_cfg : RailConfig
    launch_rod_angle_deg : float, optional
        Angle from VERTICAL in degrees (0 = straight up).  Defaults to
        ``90 - rail_cfg.inclination_deg``.
    src_ork, dst_ork : Path
        Source team .ork and output path.

    Returns
    -------
    Path
        Path to the written .ork file.
    """
    rod_angle = (
        launch_rod_angle_deg
        if launch_rod_angle_deg is not None
        else 90.0 - rail_cfg.inclination_deg
    )

    with zipfile.ZipFile(src_ork) as z:
        xml_bytes = z.read("rocket.ork")

    root = ET.fromstring(xml_bytes)

    for cond in root.iter("conditions"):
        _set(cond, "launchlatitude",   f"{site.latitude}")
        _set(cond, "launchlongitude",  f"{site.longitude}")
        _set(cond, "launchaltitude",   f"{site.elevation_m}")
        _set(cond, "launchrodangle",   f"{rod_angle}")
        _set(cond, "launchroddirection", f"{rail_cfg.heading_deg}")
        _set(cond, "windaverage",      f"{atmosphere_cfg.wind_speed_ms}")
        _set(cond, "windturbulence",   "0.0")   # deterministic

    xml_out = ET.tostring(root, encoding="unicode", xml_declaration=False)

    _OUTPUT_DIR.mkdir(exist_ok=True)
    with zipfile.ZipFile(dst_ork, "w", compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr("rocket.ork", xml_out)

    return dst_ork


def read_or_results(ork_file: Path = _OR_MATCHED) -> tuple[FlightMetrics, FlightTrace] | None:
    """Read simulation results from a saved .ork file.

    Returns None if the file has no databranch (simulation has not been run yet).
    """
    if not ork_file.exists():
        return None

    with zipfile.ZipFile(ork_file) as z:
        xml_bytes = z.read("rocket.ork")

    root = ET.fromstring(xml_bytes)
    db = root.find(".//databranch")
    if db is None:
        return None

    rows = []
    for dp in db.findall("datapoint"):
        if dp.text:
            rows.append([float(x) for x in dp.text.strip().split(",")])

    if not rows:
        return None

    arr = np.array(rows)
    ascent = arr[arr[:, _COL_VVEL] >= 0]
    if ascent.size == 0:
        return None

    t   = ascent[:, _COL_TIME]
    alt = ascent[:, _COL_ALT]
    spd = ascent[:, _COL_SPD]

    apogee_idx = int(np.argmax(alt))
    label = "OpenRocket (matched)"
    metrics = FlightMetrics(
        source=label,
        max_alt_agl_m=round(float(alt[apogee_idx]), 1),
        max_velocity_ms=round(float(arr[:, _COL_SPD].max()), 1),
        time_to_apogee_s=round(float(t[apogee_idx]), 1),
    )
    trace = FlightTrace(source=label, t=t, alt_m=alt, speed_ms=spd)
    return metrics, trace


def results_are_stale(ork_file: Path = _OR_MATCHED) -> bool:
    """True if the matched .ork exists but has not been simulated yet."""
    if not ork_file.exists():
        return False
    return read_or_results(ork_file) is None


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _set(parent: ET.Element, tag: str, value: str) -> None:
    el = parent.find(tag)
    if el is not None:
        el.text = value
    else:
        child = ET.SubElement(parent, tag)
        child.text = value
