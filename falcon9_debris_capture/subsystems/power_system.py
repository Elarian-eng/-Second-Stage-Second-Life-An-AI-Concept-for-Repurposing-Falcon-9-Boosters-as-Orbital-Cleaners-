"""Power system - battery management for second stage."""

import numpy as np
from ..config import BATTERY_CAPACITY_KWH, BATTERY_EFFICIENCY, MAX_DISCHARGE_RATE_KW


class PowerSystem:
    """Battery-powered electrical system."""

    def __init__(self):
        self.capacity_kwh = BATTERY_CAPACITY_KWH
        self.soc = 1.0  # State of charge 0-1
        self.efficiency = BATTERY_EFFICIENCY
        self.max_discharge_kw = MAX_DISCHARGE_RATE_KW

    def request_power(self, kw: float, dt: float) -> float:
        """Request power. Returns actual power delivered."""
        available = min(kw, self.max_discharge_kw)
        energy_kwh = available * dt / 3600
        if energy_kwh > self.soc * self.capacity_kwh:
            available = self.soc * self.capacity_kwh * 3600 / dt
            energy_kwh = self.soc * self.capacity_kwh
        self.soc -= energy_kwh / self.capacity_kwh
        self.soc = max(0, self.soc)
        return available * self.efficiency

    def can_support(self, kw: float) -> bool:
        return self.soc > 0.01 and kw <= self.max_discharge_kw
