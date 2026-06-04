"""RocketPy Environment builder with automatic atmospheric model selection.

Selects the most accurate available weather model based on how far the
simulation date is from today. Falls back gracefully down the model ladder
if a forecast fetch fails (no network, outside availability window, etc.).

Model ladder (best → fallback):
    RAP  →  NAM  →  GFS  →  standard_atmosphere
"""

from __future__ import annotations

import logging
import math
from datetime import datetime, timezone

from rocketpy import Environment

from apex_sim.config.loader import EnvironmentConfig

logger = logging.getLogger(__name__)

_FORECAST_LADDER: list[str] = ["RAP", "NAM", "GFS", "standard_atmosphere"]


def build_environment(cfg: EnvironmentConfig) -> Environment:
    """Build a RocketPy Environment from config, auto-selecting the best weather model.

    If ``cfg.atmosphere.model_override`` is set, that model is used directly with
    no fallback. Otherwise, the model is chosen based on how many days remain until
    ``cfg.site.launch_window.sim_datetime`` and attempted in fallback order until one
    succeeds.

    Parameters
    ----------
    cfg : EnvironmentConfig
        Resolved environment configuration from ``load_environment()``.

    Returns
    -------
    rocketpy.Environment
        Fully configured environment ready to attach to a RocketPy Flight.

    Raises
    ------
    RuntimeError
        If every model in the fallback chain fails.
    """
    env = Environment(
        latitude=cfg.site.latitude,
        longitude=cfg.site.longitude,
        elevation=cfg.site.elevation_m,
        date=_parse_sim_datetime(cfg.site.launch_window.sim_datetime),
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
    sim_dt = _parse_sim_datetime(cfg.site.launch_window.sim_datetime)
    now = datetime.now(timezone.utc)
    days_until = (sim_dt - now).total_seconds() / 86400.0

    atm = cfg.atmosphere

    # Forecast models are only useful for future dates. A historical date would
    # always fail the NOMADS fetch — skip straight to standard_atmosphere so we
    # don't burn time on guaranteed network timeouts.
    if days_until < 0:
        model = "standard_atmosphere"
    elif days_until <= atm.rap_days:
        model = "RAP"
    elif days_until <= atm.nam_days:
        model = "NAM"
    elif days_until <= atm.gfs_days:
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
    """Inject a linear-shear wind profile into a standard_atmosphere environment.

    ISA has no wind by default.  This converts the climatological speed and
    direction from config into East (u) and North (v) components with a linear
    altitude shear.
    """
    atm = cfg.atmosphere
    speed = atm.wind_speed_ms
    direction = atm.wind_direction_deg
    shear = atm.wind_shear_ms_per_m

    u_base = -speed * math.sin(math.radians(direction))
    v_base = -speed * math.cos(math.radians(direction))
    denom = max(speed, 1e-9)

    def wind_u(altitude: float) -> float:
        return u_base + shear * altitude * (u_base / denom)

    def wind_v(altitude: float) -> float:
        return v_base + shear * altitude * (v_base / denom)

    env.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=wind_u,
        wind_v=wind_v,
    )


def _parse_sim_datetime(dt_str: str) -> datetime:
    """Parse an ISO 8601 datetime string and attach UTC timezone if naive.

    Seymour TX and IREC site times are stored as UTC in the site profiles
    (Seymour: confirmed from TeleMega GPS; IREC: local US/Central, UTC-5 CDT).
    """
    dt = datetime.fromisoformat(dt_str)
    if dt.tzinfo is None:
        from datetime import timedelta
        dt = dt.replace(tzinfo=timezone(timedelta(hours=-5)))
    return dt.astimezone(timezone.utc)
