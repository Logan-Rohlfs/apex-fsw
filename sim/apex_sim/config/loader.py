"""Configuration loader for apex_sim.

Loads environment.yaml and resolves the ``active_site`` pointer to the
corresponding profile under ``config/sites/``.  All paths resolve relative to
the ``config/`` directory that lives alongside the ``apex_sim`` package.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

# ---------------------------------------------------------------------------
# Resolve the config root at import time.
# Package lives at apex/sim/apex_sim/; config lives at apex/sim/config/.
# ---------------------------------------------------------------------------
_CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config"


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


@dataclass
class MagneticField:
    """Magnetic field parameters at the launch site.

    Parameters
    ----------
    declination_deg : float
        Declination in degrees east (negative = west of true north).
    inclination_deg : float
        Dip angle in degrees.
    field_strength_ut : float
        Total field magnitude in microtesla.
    """

    declination_deg: float
    inclination_deg: float
    field_strength_ut: float


@dataclass
class LaunchWindow:
    """Launch window timing for a site profile.

    Parameters
    ----------
    sim_datetime : str
        ISO 8601 datetime string used as the default simulation time.
        Always UTC for historical sites; local for forecast sites.
    open : str or None
        ISO 8601 window open time (competition sites only).
    close : str or None
        ISO 8601 window close time (competition sites only).
    """

    sim_datetime: str
    open: Optional[str] = None
    close: Optional[str] = None


@dataclass
class SiteProfile:
    """Everything specific to one launch site.

    Parameters
    ----------
    name : str
        Human-readable site name.
    latitude : float
        Decimal degrees N.
    longitude : float
        Decimal degrees E (negative = west).
    elevation_m : float
        Pad altitude above mean sea level in metres.
    timezone : str
        IANA timezone string (e.g. ``"US/Central"``).
    magnetic : MagneticField
        Magnetic field parameters at this site.
    launch_window : LaunchWindow
        Timing information for this site.
    """

    name: str
    latitude: float
    longitude: float
    elevation_m: float
    timezone: str
    magnetic: MagneticField
    launch_window: LaunchWindow


@dataclass
class AtmosphereConfig:
    """Atmospheric model selection configuration.

    Parameters
    ----------
    model_override : str or None
        Force a specific model. ``None`` = auto-select by date.
    rap_days : float
        Use RAP when launch is this many days or fewer away.
    nam_days : float
        Use NAM when launch is within this many days.
    gfs_days : float
        Use GFS when launch is within this many days.
    wind_speed_ms : float
        Fallback wind speed (m/s) used with ``standard_atmosphere``.
    wind_direction_deg : float
        Fallback wind direction in degrees (FROM, meteorological convention).
    wind_shear_ms_per_m : float
        Fallback wind shear in (m/s) per metre altitude.
    """

    model_override: Optional[str]
    rap_days: float
    nam_days: float
    gfs_days: float
    wind_speed_ms: float
    wind_direction_deg: float
    wind_shear_ms_per_m: float


@dataclass
class RailConfig:
    """Launch rail configuration.

    Parameters
    ----------
    length_m : float
        Rail length in metres.
    inclination_deg : float
        Inclination from horizontal in degrees (90 = vertical).
    heading_deg : float
        Heading in degrees from north (0 = north).
    """

    length_m: float
    inclination_deg: float
    heading_deg: float


@dataclass
class EnvironmentConfig:
    """Full resolved environment configuration for one simulation run.

    Combine with ``SiteProfile`` to fully configure a ``rocketpy.Environment``.

    Parameters
    ----------
    active_site : str
        Stem of the active site profile file (e.g. ``"irec_2026_pecos_tx"``).
    site : SiteProfile
        Resolved site profile.
    atmosphere : AtmosphereConfig
        Atmospheric model settings.
    rail : RailConfig
        Launch rail geometry.
    """

    active_site: str
    site: SiteProfile
    atmosphere: AtmosphereConfig
    rail: RailConfig


# ---------------------------------------------------------------------------
# Public loader
# ---------------------------------------------------------------------------


def load_environment(
    config_root: Optional[Path] = None,
    site_override: Optional[str] = None,
) -> EnvironmentConfig:
    """Load and resolve the full environment configuration.

    Reads ``environment.yaml`` and resolves ``active_site`` to the matching
    profile in ``config/sites/``.  Site-agnostic settings (atmosphere, rail)
    come from ``environment.yaml``; all site-specific data comes from the
    resolved profile.

    Parameters
    ----------
    config_root : Path, optional
        Override the config directory.  Defaults to ``apex/sim/config/``.
    site_override : str, optional
        Override ``active_site`` from the YAML (useful in tests and scripts).
        Pass the filename stem (e.g. ``"seymour_tx_2026_05_24"``).

    Returns
    -------
    EnvironmentConfig
        Fully resolved environment configuration.

    Raises
    ------
    FileNotFoundError
        If the resolved site profile YAML does not exist.
    KeyError
        If a required field is missing from either config file.
    """
    root = Path(config_root) if config_root is not None else _CONFIG_ROOT

    env_path = root / "environment.yaml"
    with env_path.open() as fh:
        env_raw = yaml.safe_load(fh)

    active_site = site_override or env_raw["active_site"]
    site_path = root / "sites" / f"{active_site}.yaml"
    if not site_path.exists():
        raise FileNotFoundError(
            f"Site profile not found: {site_path}\n"
            f"Available profiles: {_list_sites(root)}"
        )

    with site_path.open() as fh:
        site_raw = yaml.safe_load(fh)

    return EnvironmentConfig(
        active_site=active_site,
        site=_parse_site(site_raw),
        atmosphere=_parse_atmosphere(env_raw["atmosphere"]),
        rail=_parse_rail(env_raw["rail"]),
    )


# ---------------------------------------------------------------------------
# Internal parsers
# ---------------------------------------------------------------------------


def _parse_site(raw: dict) -> SiteProfile:
    s = raw["site"]
    m = raw["magnetic"]
    w = raw["launch_window"]
    return SiteProfile(
        name=s["name"],
        latitude=float(s["latitude"]),
        longitude=float(s["longitude"]),
        elevation_m=float(s["elevation_m"]),
        timezone=str(s["timezone"]),
        magnetic=MagneticField(
            declination_deg=float(m["declination_deg"]),
            inclination_deg=float(m["inclination_deg"]),
            field_strength_ut=float(m["field_strength_ut"]),
        ),
        launch_window=LaunchWindow(
            sim_datetime=str(w["sim_datetime"]),
            open=w.get("open"),
            close=w.get("close"),
        ),
    )


def _parse_atmosphere(raw: dict) -> AtmosphereConfig:
    t = raw["model_thresholds"]
    w = raw["wind"]
    return AtmosphereConfig(
        model_override=raw.get("model_override"),
        rap_days=float(t["RAP_days"]),
        nam_days=float(t["NAM_days"]),
        gfs_days=float(t["GFS_days"]),
        wind_speed_ms=float(w["speed_ms"]),
        wind_direction_deg=float(w["direction_deg"]),
        wind_shear_ms_per_m=float(w["shear_ms_per_m"]),
    )


def _parse_rail(raw: dict) -> RailConfig:
    return RailConfig(
        length_m=float(raw["length_m"]),
        inclination_deg=float(raw["inclination_deg"]),
        heading_deg=float(raw["heading_deg"]),
    )


def _list_sites(root: Path) -> list[str]:
    sites_dir = root / "sites"
    if not sites_dir.exists():
        return []
    return [p.stem for p in sorted(sites_dir.glob("*.yaml"))]
