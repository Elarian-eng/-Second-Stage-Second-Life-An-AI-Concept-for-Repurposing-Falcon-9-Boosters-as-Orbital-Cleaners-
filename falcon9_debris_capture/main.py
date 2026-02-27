"""
MVP Simulation: Falcon 9 Second Stage - Orbital Debris Capture AI
Run: python main.py
"""

import numpy as np
from falcon9_debris_capture.ai_brain import AIBrain, MissionPhase


def run_simulation(duration_s: float = 120, dt: float = 0.1):
    """Run mission simulation."""

    ai = AIBrain()

    # Initial state: just after payload deploy (LVLH frame, meters)
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Simulated debris catalog (pos [x,y,z] m, vel [vx,vy,vz] m/s, mass kg)
    ai.debris_catalog = [
        {"pos": [800, 200, 100], "vel": [0.1, -0.05, 0], "mass_kg": 150},
        {"pos": [300, 50, 20], "vel": [-0.02, 0.01, 0], "mass_kg": 80},
        {"pos": [5000, 500, 300], "vel": [0, 0, 0], "mass_kg": 200},
    ]

    # Pick nearest and set as target (AI would do this)
    ai.target_debris = ai.identify_nearest_debris(state, ai.debris_catalog)
    target_pos = np.array(ai.target_debris["pos"])
    target_vel = np.array(ai.target_debris["vel"])

    print("=" * 60)
    print("FALCON 9 SECOND STAGE - DEBRIS CAPTURE MISSION")
    print("=" * 60)
    print(f"Target debris: {target_pos} m, mass {ai.target_debris['mass_kg']} kg")
    print(f"Simulation: {duration_s}s @ dt={dt}s\n")

    t = 0
    phase_history = []
    while t < duration_s and ai.phase != MissionPhase.COMPLETE:

        u, telemetry = ai.compute_control(state, dt)

        # Simple kinematic integration
        state[3:6] += u * dt
        state[:3] += state[3:6] * dt

        # Simulate debris drift (slight)
        target_pos += target_vel * dt * 0.1
        ai.target_debris["pos"] = target_pos.tolist()

        if phase_history == [] or phase_history[-1] != ai.phase:
            phase_history.append(ai.phase)
            dist = np.linalg.norm(state[:3] - target_pos)
            print(f"t={t:6.1f}s | Phase: {ai.phase:20s} | dist={dist:8.2f}m | u_norm={np.linalg.norm(u):.4f}")

        t += dt

        # Trigger capture/retract after close approach
        if ai.phase == MissionPhase.CAPTURE and np.linalg.norm(state[:3] - target_pos) < 0.01:
            ai.grabbing.deploy_net(dt)
            ai.grabbing.confirm_capture(ai.target_debris["mass_kg"])
            ai.phase = MissionPhase.RETRACTING

        if ai.phase == MissionPhase.RETRACTING and ai.grabbing.retract_timer > 5.0:
            ai.phase = MissionPhase.REENTRY

        if ai.phase == MissionPhase.REENTRY and t > 90:
            ai.phase = MissionPhase.COMPLETE

    print("\n" + "=" * 60)
    print("MISSION COMPLETE" if ai.phase == MissionPhase.COMPLETE else "MISSION SIMULATION ENDED")
    print("=" * 60)
    print(f"Phases: {' -> '.join(phase_history)}")
    print(f"Captured mass: {ai.grabbing.captured_mass_kg} kg")
    print(f"Battery SOC: {ai.power.soc * 100:.1f}%")
    print(f"Fuel remaining: {ai.propulsion.fuel_kg:.1f} kg")


if __name__ == "__main__":
    run_simulation(duration_s=120, dt=0.1)
