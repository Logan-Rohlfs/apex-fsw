"""Run a RocketPy Flight simulation from configuration.

Entry points
------------
``run_flight`` — full simulation (with or without airbrakes)
``run_baseline`` — clean-flight simulation for validation against real data
"""

from __future__ import annotations

from pathlib import Path

import yaml
from rocketpy import Flight

from apex_sim.config.loader import EnvironmentConfig, SiteProfile, load_environment
from apex_sim.sim.environment import build_environment
from apex_sim.sim.rocket import build_rocket

_CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config"


def run_flight(
    env_cfg: EnvironmentConfig | None = None,
    rocket_cfg: dict | None = None,
    airbrakes_cfg: dict | None = None,
    active_airbrakes: bool = True,
    terminate_on_apogee: bool = False,
    max_time: float = 600.0,
    verbose: bool = False,
) -> Flight:
    """Run a full flight simulation.

    Parameters
    ----------
    env_cfg : EnvironmentConfig, optional
        Resolved environment + site config.  Loaded from disk if omitted.
    rocket_cfg : dict, optional
        Parsed ``config/rocket.yaml``.  Loaded from disk if omitted.
    airbrakes_cfg : dict, optional
        Parsed ``config/airbrakes.yaml``.  Loaded from disk if omitted.
    active_airbrakes : bool
        If ``True``, attach and run the PID airbrake controller.
        Set ``False`` for a clean-flight (validation) run.
    terminate_on_apogee : bool
        Stop integration at apogee.  Faster when only altitude is needed.
    max_time : float
        Maximum simulation time in seconds.
    verbose : bool
        Print RocketPy progress output.

    Returns
    -------
    Flight
        Completed simulation.  Query ``flight.apogee``, ``flight.max_speed``,
        etc., or call ``flight.all_info()`` for a full report.
    """
    if env_cfg is None:
        env_cfg = load_environment()
    if rocket_cfg is None:
        with (_CONFIG_ROOT / "rocket.yaml").open() as fh:
            rocket_cfg = yaml.safe_load(fh)
    if airbrakes_cfg is None:
        with (_CONFIG_ROOT / "airbrakes.yaml").open() as fh:
            airbrakes_cfg = yaml.safe_load(fh)

    env = build_environment(env_cfg)
    rocket = build_rocket(rocket_cfg, airbrakes_cfg)

    if active_airbrakes:
        from apex_sim.sim.airbrakes import attach_airbrakes

        burn_time = rocket_cfg["motor"]["burn_time_s"]
        attach_airbrakes(
            rocket=rocket,
            airbrakes_cfg=airbrakes_cfg,
            rocket_cfg=rocket_cfg,
            burnout_time_s=burn_time,
        )

    site: SiteProfile = env_cfg.site
    rail = env_cfg.rail

    flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=rail.length_m,
        inclination=rail.inclination_deg,
        heading=rail.heading_deg,
        terminate_on_apogee=terminate_on_apogee,
        max_time=max_time,
        verbose=verbose,
        name=f"Apex — {site.name}",
    )

    return flight


def run_baseline(
    site_override: str | None = None,
    terminate_on_apogee: bool = True,
    verbose: bool = False,
) -> Flight:
    """Clean-flight simulation for validating against real flight data.

    No airbrakes deployed.  Intended to be compared against the Seymour TX
    TeleMega/Blue Raven recordings and the OpenRocket prediction.

    Parameters
    ----------
    site_override : str, optional
        Site profile stem to use (e.g. ``"seymour_tx_2026_05_24"``).
        Defaults to ``active_site`` in ``config/environment.yaml``.
    terminate_on_apogee : bool
        Stop at apogee by default since the baseline comparison only cares
        about max altitude.  Set ``False`` to simulate full descent.
    verbose : bool
        Print RocketPy progress output.

    Returns
    -------
    Flight
        Completed clean-flight simulation.
    """
    env_cfg = load_environment(site_override=site_override)
    return run_flight(
        env_cfg=env_cfg,
        active_airbrakes=False,
        terminate_on_apogee=terminate_on_apogee,
        verbose=verbose,
    )
