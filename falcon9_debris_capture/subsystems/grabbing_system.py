"""Grabbing system - net deployment, capture, retraction."""

import numpy as np
from ..config import NET_DEPLOYMENT_TIME_S, NET_RETRACTION_TIME_S, MAX_DEBRIS_MASS_KG


class GrabbingSystem:
    """Net-based debris capture with deployment thrusters and retraction."""

    def __init__(self):
        self.net_deployed = False
        self.net_deploying = False
        self.capture_confirmed = False
        self.retracting = False
        self.deploy_timer = 0.0
        self.retract_timer = 0.0
        self.captured_mass_kg = 0.0
        self.deployment_thruster_active = False

    def deploy_net(self, dt: float) -> bool:
        """Deploy net. Returns True when deployment complete."""
        if self.net_deployed:
            return True
        if not self.net_deploying:
            self.net_deploying = True
            self.deployment_thruster_active = True
        self.deploy_timer += dt
        if self.deploy_timer >= NET_DEPLOYMENT_TIME_S:
            self.net_deployed = True
            self.net_deploying = False
            self.deployment_thruster_active = False
        return self.net_deployed

    def confirm_capture(self, mass_kg: float) -> bool:
        """Confirm debris is in net."""
        if mass_kg <= MAX_DEBRIS_MASS_KG and self.net_deployed:
            self.capture_confirmed = True
            self.captured_mass_kg = mass_kg
            return True
        return False

    def retract_net(self, dt: float) -> bool:
        """Retract net with captured debris."""
        if not self.capture_confirmed:
            return False
        if not self.retracting:
            self.retracting = True
        self.retract_timer += dt
        return self.retract_timer >= NET_RETRACTION_TIME_S
