"""Build a RocketPy Rocket from config/rocket.yaml and config/airbrakes.yaml.

All positions use the ``nose_to_tail`` coordinate system with origin at the
nose tip.  Positive z runs from nose toward tail.

Component layout (z from nose tip):
  0.000 m  — nose tip
  0.762 m  — nose cone base / Body Tube 1 start
  2.896 m  — Body Tube 1 end / airbrake housing start
  2.959 m  — airbrake housing end / Aft Body Tube start
  3.213 m  — motor forward end
  3.992 m  — fin root leading edge
  4.305 m  — aft body tube end
  4.318 m  — motor retainer end / aft tip
  4.432 m  — motor nozzle exit (extends 0.114 m past aft tip)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml
from rocketpy import GenericMotor, Rocket

_SIM_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_ROOT = _SIM_ROOT / "config"


def _load_yaml(path: Path) -> dict:
    with path.open() as fh:
        return yaml.safe_load(fh)


def build_rocket(
    rocket_cfg: dict | None = None,
    airbrakes_cfg: dict | None = None,
) -> Rocket:
    """Build a RocketPy ``Rocket`` from configuration dicts.

    Parameters
    ----------
    rocket_cfg : dict, optional
        Parsed contents of ``config/rocket.yaml``.  Loaded from disk if omitted.
    airbrakes_cfg : dict, optional
        Parsed contents of ``config/airbrakes.yaml``.  Loaded from disk if
        omitted.  Used for the clean-flight Cd applied to ``power_off_drag``
        and ``power_on_drag``.

    Returns
    -------
    Rocket
        Configured RocketPy ``Rocket`` instance, ready for ``add_air_brakes``
        and ``Flight``.
    """
    if rocket_cfg is None:
        rocket_cfg = _load_yaml(_CONFIG_ROOT / "rocket.yaml")
    if airbrakes_cfg is None:
        airbrakes_cfg = _load_yaml(_CONFIG_ROOT / "airbrakes.yaml")

    mass = rocket_cfg["mass"]
    geom = rocket_cfg["geometry"]
    fins_cfg = rocket_cfg["fins"]
    motor_cfg = rocket_cfg["motor"]
    recovery_cfg = rocket_cfg["recovery"]
    cd_clean = airbrakes_cfg["aerodynamics"]["cd_clean"]

    # Inertia — solid-cylinder approximation about the dry CG.
    # Replace with OpenRocket or physical measurement when available.
    # Significantly under-estimates I_transverse if mass is end-heavy.
    m = mass["dry_mass_kg"]
    r = geom["body_radius_m"]
    L = geom["total_length_m"]
    I_transverse = m * (r**2 / 4.0 + L**2 / 12.0)
    I_axial = 0.5 * m * r**2

    rocket = Rocket(
        radius=r,
        mass=m,
        inertia=(I_transverse, I_transverse, I_axial),
        power_off_drag=cd_clean,
        power_on_drag=cd_clean,
        center_of_mass_without_motor=mass["center_of_dry_mass_from_nose_m"],
        coordinate_system_orientation="nose_to_tail",
    )

    nose = geom["nose_cone"]
    rocket.add_nose(
        length=nose["length_m"],
        kind=nose["shape"],
        position=0.0,
    )

    rocket.add_trapezoidal_fins(
        n=fins_cfg["count"],
        root_chord=fins_cfg["root_chord_m"],
        tip_chord=fins_cfg["tip_chord_m"],
        span=fins_cfg["span_m"],
        position=fins_cfg["position_from_nose_m"],
        sweep_length=fins_cfg["leading_edge_sweep_m"],
        cant_angle=0.0,
    )

    motor = _build_motor(motor_cfg)
    # Nozzle exit sits past the aft body tube — motor length > motor mount tube.
    nozzle_pos_m = motor_cfg["motor_position_from_nose_m"] + motor_cfg["length_mm"] / 1000.0
    rocket.add_motor(motor, position=nozzle_pos_m)

    drogue = recovery_cfg["drogue"]
    drogue_cd_s = drogue["cd"] * np.pi * (drogue["diameter_m"] / 2.0) ** 2
    rocket.add_parachute(
        name=drogue["name"],
        cd_s=drogue_cd_s,
        trigger="apogee",
        sampling_rate=100,
    )

    main = recovery_cfg["main"]
    main_cd_s = main["cd"] * np.pi * (main["diameter_m"] / 2.0) ** 2
    rocket.add_parachute(
        name=main["name"],
        cd_s=main_cd_s,
        trigger=main["deploy_altitude_m"],
        sampling_rate=100,
    )

    return rocket


def _build_motor(motor_cfg: dict) -> GenericMotor:
    eng_path = str(_SIM_ROOT / motor_cfg["eng_file"])
    motor_radius = motor_cfg["diameter_mm"] / 2000.0
    motor_length = motor_cfg["length_mm"] / 1000.0

    # Motor casing dry mass = total loaded mass - propellant mass (.eng header).
    # N3355 loaded: 12.645 kg, propellant: 6.682 kg → casing: 5.963 kg.
    dry_mass = 5.963

    # Nozzle throat radius not provided — estimated for a 98 mm N-class motor.
    # Affects specific impulse bookkeeping but not the thrust curve itself.
    nozzle_radius_estimate = 0.020

    return GenericMotor(
        thrust_source=eng_path,
        burn_time=None,  # inferred from .eng thrust curve — avoids off-by-epsilon warning
        chamber_radius=motor_radius,
        chamber_height=motor_length,
        chamber_position=motor_length / 2.0,
        propellant_initial_mass=motor_cfg["propellant_mass_kg"],
        nozzle_radius=nozzle_radius_estimate,
        dry_mass=dry_mass,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
