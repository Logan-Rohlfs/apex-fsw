"""RocketPy Environment builder with automatic atmospheric model selection.

Selects the most accurate available weather model based on how far the
simulation date is from today. Falls back gracefully down the model ladder
if a forecast fetch fails (no network, outside availability window, etc.).

Model ladder (best → fallback):
    RAP  →  NAM  →  GFS  →  standard_atmosphere
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import TYPE_CHECKING

from rocketpy import Environment

if TYPE_CHECKING:
    from apex_sim.config.loader import EnvironmentConfig

logger = logging.getLogger(__name__)

# Ordered fallback chain. Each entry is attempted in sequence until one succeeds.
_FORECAST_LADDER: list[str] = ["RAP", "NAM", "GFS", "standard_atmosphere"]


def build_environment(cfg: EnvironmentConfig) -> Environment:
    """Build a RocketPy Environment from config, auto-selecting the best weather model.

    If ``cfg.atmosphere.model_override`` is set, that model is used directly with
    no fallback. Otherwise, the model is chosen based on how many days remain until
    ``cfg.launch_window.sim_datetime`` and attempted in fallback order until one
    succeeds.

    Parameters
    ----------
    cfg : EnvironmentConfig
        Parsed environment configuration, typically from ``config/environment.yaml``.

    Returns
    -------
    rocketpy.Environment
        Fully configured environment ready to attach to a RocketPy Flight.

    Raises
    ------
    RuntimeError
        If every model in the fallback chain fails. Should only happen if
        ``standard_atmosphere`` itself errors, which indicates a RocketPy issue.
    """
    env = Environment(
        latitude=cfg.launch_site.latitude,
        longitude=cfg.launch_site.longitude,
        elevation=cfg.launch_site.elevation_m,
        date=_parse_sim_datetime(cfg.launch_window.sim_datetime),
    )

    if cfg.atmosphere.model_override:
        logger.info("Atmospheric model forced by config: %s", cfg.atmosphere.model_override)
        _set_model(env, cfg.atmosphere.model_override, cfg)
        return env

    selected = _select_model(cfg)
    _set_model_with_fallback(env, selected, cfg)
    return env


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _select_model(cfg: EnvironmentConfig) -> str:
    """Choose the best model based on days remaining until the sim date.

    Parameters
    ----------
    cfg : EnvironmentConfig
        Environment configuration containing thresholds and sim datetime.

    Returns
    -------
    str
        Model name: ``"RAP"``, ``"NAM"``, ``"GFS"``, or ``"standard_atmosphere"``.
    """
    sim_dt = _parse_sim_datetime(cfg.launch_window.sim_datetime)
    now = datetime.now(timezone.utc)
    days_until = (sim_dt - now).total_seconds() / 86400.0

    thresholds = cfg.atmosphere.model_thresholds

    if days_until <= thresholds.rap_days:
        model = "RAP"
    elif days_until <= thresholds.nam_days:
        model = "NAM"
    elif days_until <= thresholds.gfs_days:
        model = "GFS"
    else:
        model = "standard_atmosphere"

    logger.info(
        "Auto-selected atmospheric model: %s (%.1f days until sim date)",
        model,
        days_until,
    )
    return model


def _set_model_with_fallback(
    env: Environment,
    model: str,
    cfg: EnvironmentConfig,
) -> None:
    """Attempt to set the given model, falling back down the ladder on failure.

    Parameters
    ----------
    env : rocketpy.Environment
        The environment object to configure.
    model : str
        The preferred model to attempt first.
    cfg : EnvironmentConfig
        Full environment config, needed for standard_atmosphere wind parameters.

    Raises
    ------
    RuntimeError
        If all models in the fallback chain fail.
    """
    start_idx = _FORECAST_LADDER.index(model) if model in _FORECAST_LADDER else 0

    for candidate in _FORECAST_LADDER[start_idx:]:
        try:
            _set_model(env, candidate, cfg)
            if candidate != model:
                logger.warning(
                    "Fell back from %s to %s — preferred model unavailable.",
                    model,
                    candidate,
                )
            return
        except Exception as exc:  # noqa: BLE001
            logger.warning("Model %s failed (%s), trying next fallback.", candidate, exc)

    raise RuntimeError(
        "All atmospheric models failed. Check RocketPy installation and network access."
    )


def _set_model(env: Environment, model: str, cfg: EnvironmentConfig) -> None:
    """Set a specific atmospheric model on the environment.

    Parameters
    ----------
    env : rocketpy.Environment
        The environment to configure.
    model : str
        One of ``"RAP"``, ``"NAM"``, ``"GFS"``, or ``"standard_atmosphere"``.
    cfg : EnvironmentConfig
        Full environment config.

    Raises
    ------
    ValueError
        If ``model`` is not a recognised model name.
    Exception
        Re-raises any exception from RocketPy (e.g. network error, data not
        available) so the fallback chain can catch it.
    """
    if model == "standard_atmosphere":
        env.set_atmospheric_model(type="standard_atmosphere")
        _apply_custom_wind(env, cfg)
        logger.info("Atmospheric model set: standard_atmosphere (with custom wind profile)")
    elif model in ("RAP", "NAM", "GFS"):
        env.set_atmospheric_model(type="forecast", file=model)
        logger.info("Atmospheric model set: %s forecast", model)
    else:
        raise ValueError(f"Unknown atmospheric model: {model!r}")


def _apply_custom_wind(env: Environment, cfg: EnvironmentConfig) -> None:
    """Apply the config wind profile to a standard_atmosphere environment.

    RocketPy's ISA standard atmosphere has no wind. This injects the
    climatological wind defined in ``config/environment.yaml`` as a simple
    linear shear profile.

    Parameters
    ----------
    env : rocketpy.Environment
        Environment using standard_atmosphere.
    cfg : EnvironmentConfig
        Config containing wind speed, direction, and shear parameters.
    """
    import math

    wind = cfg.atmosphere.wind
    u = -wind.speed_ms * math.sin(math.radians(wind.direction_deg))
    v = -wind.speed_ms * math.cos(math.radians(wind.direction_deg))

    def wind_u(altitude: float) -> float:
        return u + wind.shear_ms_per_m * altitude * (u / max(wind.speed_ms, 1e-9))

    def wind_v(altitude: float) -> float:
        return v + wind.shear_ms_per_m * altitude * (v / max(wind.speed_ms, 1e-9))

    env.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=wind_u,
        wind_v=wind_v,
    )


def _parse_sim_datetime(dt_str: str) -> datetime:
    """Parse an ISO 8601 datetime string and attach UTC timezone if naive.

    Parameters
    ----------
    dt_str : str
        Datetime string from config, e.g. ``"2026-06-17T08:00:00"``.

    Returns
    -------
    datetime
        Timezone-aware datetime in UTC.
    """
    dt = datetime.fromisoformat(dt_str)
    if dt.tzinfo is None:
        # Config times are local (US/Central = UTC-5). Attach UTC offset.
        # CDT (summer) is UTC-5; CST (winter) is UTC-6.
        from datetime import timedelta
        dt = dt.replace(tzinfo=timezone(timedelta(hours=-5)))
    return dt.astimezone(timezone.utc)
