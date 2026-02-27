"""Configuration constants for debris capture mission."""

# Orbital parameters (LEO reference)
ORBIT_ALTITUDE_KM = 400
EARTH_RADIUS_KM = 6371
MU_EARTH = 398600.4418  # km³/s²

# Debris capture thresholds (meters)
LQR_COARSE_THRESHOLD = 500      # Switch from LQR to MPC
MPC_FINE_THRESHOLD = 5          # Switch from MPC to Sliding Mode
SLIDING_CAPTURE_THRESHOLD = 0.01  # 1 cm - net deployment
CAPTURE_TOLERANCE_MM = 1.0      # Millimeter precision target

# Power system (battery)
BATTERY_CAPACITY_KWH = 50
BATTERY_EFFICIENCY = 0.95
MAX_DISCHARGE_RATE_KW = 10

# Thruster specifications (typical RCS)
MAIN_RCS_THRUST_N = 450
VERNIER_RCS_THRUST_N = 50
RCS_ISP = 300  # seconds

# Net deployment
NET_DEPLOYMENT_TIME_S = 3.0
NET_RETRACTION_TIME_S = 5.0
MAX_DEBRIS_MASS_KG = 500
