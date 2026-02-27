"""Propulsion - RCS and vernier thrusters."""

import numpy as np
from ..config import MAIN_RCS_THRUST_N, VERNIER_RCS_THRUST_N, RCS_ISP


class PropulsionSystem:
    """Main RCS and vernier thrusters."""

    def __init__(self, dry_mass_kg: float = 4000):
        self.main_thrust = MAIN_RCS_THRUST_N
        self.vernier_thrust = VERNIER_RCS_THRUST_N
        self.isp = RCS_ISP
        self.mass_kg = dry_mass_kg
        self.fuel_kg = 500  # Residual propellant in second stage

    def thrust_from_acceleration(self, accel_mps2: np.ndarray) -> tuple[np.ndarray, float]:
        """Convert desired acceleration to thrust vector and fuel consumption."""
        f = self.mass_kg * accel_mps2
        mag = np.linalg.norm(f)
        # Clamp to available thrust
        f_max = np.sqrt(3) * self.main_thrust
        if mag > f_max:
            f = f * (f_max / mag)
        # Vernier for small corrections
        if mag < self.vernier_thrust:
            f = f * (self.vernier_thrust / (mag + 1e-8))
        dt_flow = 0.1  # Assume 0.1s step
        mdot = np.linalg.norm(f) / (self.isp * 9.81)
        fuel_used = mdot * dt_flow
        self.fuel_kg -= min(fuel_used, self.fuel_kg)
        return f, fuel_used

    def has_fuel(self) -> bool:
        return self.fuel_kg > 1.0
