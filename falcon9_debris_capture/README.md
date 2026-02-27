# Falcon 9 Second Stage – Orbital Debris Capture AI (MVP)

Autonomous AI control system for a modified Falcon 9 second stage that captures orbital debris and returns to Earth.

## Control Architecture

| Control | Role | Phase |
|---------|------|-------|
| **LQR** | Coarse trajectory toward debris (>500 m) | `COARSE_APPROACH` |
| **MPC** | Precise positioning (5–500 m) | `FINE_APPROACH` |
| **Sliding Mode** | Millimeter precision near capture (<5 m) | `CAPTURE` |
| **Adaptive** | Stabilization and disturbance rejection | `CAPTURE` |

## Subsystems

- **Power**: Battery, power allocation for thrusters and sensors
- **Grabbing**: Net deployment, capture confirmation, retraction
- **Propulsion**: Main RCS, vernier RCS, fuel management

## Mission Flow

1. Payload deployed → AI selects nearest debris
2. LQR → MPC → SMC+Adaptive approach
3. Net deployment at mm precision
4. Capture confirmation → retraction → re-entry burn

## Setup & Run

```bash
pip install -r requirements.txt
python main.py
```

## Project Layout

```
falcon9_debris_capture/
├── config.py           # Mission constants
├── ai_brain.py         # Autonomous decision engine
├── controls/           # LQR, MPC, SMC, Adaptive
└── subsystems/         # Power, grabbing, propulsion
main.py                 # Simulation entry point
```
