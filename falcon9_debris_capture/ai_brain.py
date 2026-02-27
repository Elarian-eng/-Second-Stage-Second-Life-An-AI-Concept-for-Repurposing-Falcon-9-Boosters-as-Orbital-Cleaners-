"""
Autonomous AI Brain - Orchestrates all control and subsystems.
Makes decisions: debris selection, phase switching, capture, re-entry.
"""

import numpy as np
from .controls import LQRController, MPCController, SlidingModeController, AdaptiveController
from .subsystems import PowerSystem, GrabbingSystem, PropulsionSystem
from .config import (
    LQR_COARSE_THRESHOLD,
    MPC_FINE_THRESHOLD,
    SLIDING_CAPTURE_THRESHOLD,
    CAPTURE_TOLERANCE_MM,
)


class MissionPhase:
    PAYLOAD_DEPLOYED = "payload_deployed"
    SEEKING_DEBRIS = "seeking_debris"
    COARSE_APPROACH = "coarse_approach"   # LQR
    FINE_APPROACH = "fine_approach"       # MPC
    CAPTURE = "capture"                   # SMC + Adaptive
    RETRACTING = "retracting"
    REENTRY = "reentry"
    COMPLETE = "complete"


class AIBrain:
    """Autonomous AI for debris capture mission."""

    def __init__(self):
        self.phase = MissionPhase.PAYLOAD_DEPLOYED
        self.lqr = LQRController(dt=0.1)
        self.mpc = MPCController(dt=0.1)
        self.smc = SlidingModeController(dt=0.01)
        self.adaptive = AdaptiveController(dt=0.01)
        self.power = PowerSystem()
        self.grabbing = GrabbingSystem()
        self.propulsion = PropulsionSystem(dry_mass_kg=4000)
        self.state = np.zeros(6)  # [x,y,z,vx,vy,vz] in LVLH
        self.target_debris = None
        self.debris_catalog = []

    def identify_nearest_debris(self, state: np.ndarray, catalog: list):
        """Select nearest debris from catalog (each: {pos, vel, mass})."""
        if not catalog:
            return None
        pos = state[:3]
        best = None
        best_dist = np.inf
        for d in catalog:
            dist = np.linalg.norm(np.array(d["pos"]) - pos)
            if dist < best_dist:
                best_dist = dist
                best = d
        return best

    def decide_phase(self, state: np.ndarray, target: np.ndarray | None) -> str:
        """Autonomous phase transition logic."""
        if target is None:
            return MissionPhase.SEEKING_DEBRIS
        dist = np.linalg.norm(state[:3] - target[:3]) if target.size >= 3 else np.inf
        if self.phase == MissionPhase.COARSE_APPROACH and dist < LQR_COARSE_THRESHOLD:
            return MissionPhase.FINE_APPROACH
        if self.phase == MissionPhase.FINE_APPROACH and dist < MPC_FINE_THRESHOLD:
            return MissionPhase.CAPTURE
        if self.phase == MissionPhase.CAPTURE and dist < SLIDING_CAPTURE_THRESHOLD:
            return MissionPhase.RETRACTING
        if self.grabbing.retract_timer >= 5.0 and self.grabbing.capture_confirmed:
            return MissionPhase.REENTRY
        if self.phase == MissionPhase.REENTRY:
            return MissionPhase.COMPLETE
        return self.phase

    def compute_control(self, state: np.ndarray, dt: float) -> tuple[np.ndarray, dict]:
        """
        Main control loop. Returns (thrust_acceleration, telemetry).
        """
        self.state = state
        telemetry = {"phase": self.phase, "control_mode": None}

        # Power check
        if not self.power.can_support(5.0):
            return np.zeros(3), {**telemetry, "error": "low_power"}

        # Identify target if needed
        if self.target_debris is None and self.debris_catalog:
            self.target_debris = self.identify_nearest_debris(state, self.debris_catalog)
            if self.target_debris:
                self.phase = MissionPhase.COARSE_APPROACH

        target = np.array(self.target_debris["pos"]) if self.target_debris else None

        # Phase update
        self.phase = self.decide_phase(state, target)

        # Control computation by phase
        u = np.zeros(3)
        if self.phase == MissionPhase.COARSE_APPROACH and target is not None:
            u = self.lqr.compute_control(state, target)
            telemetry["control_mode"] = "LQR"
        elif self.phase == MissionPhase.FINE_APPROACH and target is not None:
            u = self.mpc.compute_control(state, target)
            telemetry["control_mode"] = "MPC"
        elif self.phase == MissionPhase.CAPTURE and target is not None:
            u_smc = self.smc.compute_control(state, target)
            u = self.adaptive.compute_control(state, target, u_smc)
            telemetry["control_mode"] = "SMC+Adaptive"
            # Deploy net when within mm precision
            if np.linalg.norm(state[:3] - target[:3]) < CAPTURE_TOLERANCE_MM / 1000:
                self.grabbing.deploy_net(dt)
        elif self.phase == MissionPhase.RETRACTING:
            self.grabbing.confirm_capture(self.target_debris.get("mass_kg", 100))
            self.grabbing.retract_net(dt)
            telemetry["control_mode"] = "retract"
        elif self.phase == MissionPhase.REENTRY:
            # Point toward Earth, apply deorbit burn
            u = -0.5 * state[:3] / (np.linalg.norm(state[:3]) + 1e-6)
            telemetry["control_mode"] = "reentry"

        # Consume power and fuel
        self.power.request_power(np.linalg.norm(u) * 0.5, dt)
        self.propulsion.thrust_from_acceleration(u)

        return u, telemetry
