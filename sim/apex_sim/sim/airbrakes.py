"""Attach airbrakes and a PID controller to a RocketPy Rocket.

Controller design notes
-----------------------
The PID controller is a direct port of the MATLAB reference in
``sim/matlab/rocket_sim_4_23_26.m``.  Key differences from a textbook PID:

* The D-term uses vertical velocity directly as a proxy for d(error)/dt.
  This matches both the MATLAB reference and the firmware implementation —
  do not change it without updating both.

* MATLAB operated in feet/fps.  All gains are rescaled by 1/0.3048 here so
  the controller response is identical in SI units.  The rescale applies to
  all three gains (Kp, Ki, Kd), despite a comment in airbrakes.yaml claiming
  Kd is unaffected — velocity has units too.

* The raw PID output u is in the range [u_min, u_max] = [-15, 30].  This
  maps linearly to deployment fraction [0, 1].  A deployment of 0 corresponds
  to u = u_min (brakes retracted); full deployment corresponds to u = u_max.

* A rate limiter caps deployment change per time step to match servo travel
  time (0.24 s full travel).  The previous deployment is retrieved from
  ``observed_variables[-1]`` so no mutable state lives outside the function.

Apogee prediction
-----------------
Uses the closed-form drag-deceleration model from the MATLAB reference::

    predicted_apogee = h + (m / (2k)) × ln(1 + k×v² / (m×g))

where k = 0.5 × ρ(h) × S × Cd_clean.  Density uses a tropospheric ISA
approximation.  The prediction assumes clean-flight Cd (no brakes), which
is intentionally conservative — over-prediction drives earlier brake
deployment.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from rocketpy import Rocket

_CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config"

_FT_PER_M = 0.3048

# ISA sea-level constants used in density model
_RHO0 = 1.225   # kg/m³
_T0 = 288.15    # K
_L = 0.0065     # K/m lapse rate (troposphere)
_G = 9.80665    # m/s²
_R = 287.058    # J/(kg·K)


def _isa_density(altitude_m: float) -> float:
    """Tropospheric ISA air density at a given MSL altitude."""
    T = max(_T0 - _L * altitude_m, 1.0)
    return _RHO0 * (T / _T0) ** (_G / (_R * _L) - 1.0)


def attach_airbrakes(
    rocket: Rocket,
    airbrakes_cfg: dict | None = None,
    rocket_cfg: dict | None = None,
    burnout_time_s: float = 4.583,
) -> object:
    """Attach airbrakes and a PID controller to ``rocket``.

    Parameters
    ----------
    rocket : Rocket
        The RocketPy ``Rocket`` to modify in place.
    airbrakes_cfg : dict, optional
        Parsed ``config/airbrakes.yaml``.  Loaded from disk if omitted.
    rocket_cfg : dict, optional
        Parsed ``config/rocket.yaml``.  Needed for rocket mass and reference
        area used inside the apogee prediction.  Loaded from disk if omitted.
    burnout_time_s : float
        Simulation time (s) at motor burnout.  The controller is inactive
        before this point so it does not interfere with powered flight.

    Returns
    -------
    AirBrakes
        The attached ``AirBrakes`` instance (returned by ``add_air_brakes``).
    """
    if airbrakes_cfg is None:
        with (_CONFIG_ROOT / "airbrakes.yaml").open() as fh:
            airbrakes_cfg = yaml.safe_load(fh)
    if rocket_cfg is None:
        with (_CONFIG_ROOT / "rocket.yaml").open() as fh:
            rocket_cfg = yaml.safe_load(fh)

    aero = airbrakes_cfg["aerodynamics"]
    ctrl = airbrakes_cfg["control"]
    servo = airbrakes_cfg["servo"]
    pid_cfg = ctrl["pid"]

    cd_clean = aero["cd_clean"]
    delta_cd = aero["delta_cd_max"]
    ref_area = aero["reference_area_m2"]
    mass = rocket_cfg["mass"]["dry_mass_kg"]
    target_apogee_m = ctrl["target_apogee_m"]
    loop_rate = ctrl["loop_rate_hz"]
    max_rate_per_s = servo["max_rate_per_s"]

    # Rescale MATLAB (feet) gains to SI (metres/s).
    kp = pid_cfg["kp"] / _FT_PER_M
    ki = pid_cfg["ki"] / _FT_PER_M
    kd = pid_cfg["kd"] / _FT_PER_M
    u_min = float(pid_cfg["output_min"])
    u_max = float(pid_cfg["output_max"])

    def _drag_coeff_curve(deployment: float, mach: float) -> float:
        """Additional drag from brakes, linear with deployment fraction."""
        return deployment * delta_cd

    def _controller(
        time: float,
        sampling_rate: float,
        state: list,
        state_history: list,
        observed_variables: list,
        interactive_objects: list,
        sensors: list,
    ) -> list:
        """PID controller — sets airbrakes deployment level each time step.

        ``observed_variables`` carries persistent state between calls:
        ``[time, deployment, predicted_apogee_m, error_m, integral_m_s]``
        """
        airbrakes = interactive_objects[0]

        # Unpack state [x, y, z, vx, vy, vz, ...]
        alt_asl = state[2]
        vz = state[5]

        # Not ascending or still in powered flight — retract and wait.
        if vz <= 0.0 or time < burnout_time_s:
            airbrakes.deployment_level = 0.0
            prev_integral = observed_variables[-1][4] if observed_variables else 0.0
            return [time, 0.0, alt_asl, 0.0, prev_integral]

        # Apogee prediction (closed-form, clean-flight Cd, current density)
        rho = _isa_density(alt_asl)
        k = 0.5 * rho * ref_area * cd_clean
        arg = (k * vz**2) / (mass * _G)
        predicted_apogee = alt_asl + (mass / (2.0 * k)) * math.log1p(arg)

        error = predicted_apogee - target_apogee_m

        # Integral — persisted via observed_variables
        prev_integral = observed_variables[-1][4] if observed_variables else 0.0
        integral = prev_integral + error / sampling_rate

        u = kp * error + ki * integral + kd * vz
        u = max(u_min, min(u_max, u))

        # Map [-15, 30] → [0, 1] linearly
        desired_deployment = (u - u_min) / (u_max - u_min)

        # Rate limiter — caps δ-deployment to servo travel constraint
        prev_deployment = observed_variables[-1][1] if observed_variables else 0.0
        max_delta = max_rate_per_s / sampling_rate
        delta = desired_deployment - prev_deployment
        delta = max(-max_delta, min(max_delta, delta))
        deployment = prev_deployment + delta
        deployment = max(0.0, min(1.0, deployment))

        airbrakes.deployment_level = deployment
        return [time, deployment, predicted_apogee, error, integral]

    airbrakes = rocket.add_air_brakes(
        drag_coefficient_curve=_drag_coeff_curve,
        controller_function=_controller,
        sampling_rate=loop_rate,
        clamp=True,
        reference_area=ref_area,
        initial_observed_variables=[0.0, 0.0, 0.0, 0.0, 0.0],
        override_rocket_drag=False,
        return_controller=True,
        name="AirBrakes",
    )

    return airbrakes
