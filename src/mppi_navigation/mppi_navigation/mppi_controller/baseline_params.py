"""MPPI baseline parameters (design doc section 7).

Single source of truth mirrored in config/nav2_pure_mppi_params.yaml.
These are starting values for the tuning workflow, not final values.
"""

CONTROLLER_FREQUENCY = 20.0   # Hz -> 0.05 s deadline per cycle
MODEL_DT = 0.05               # s
TIME_STEPS = 44               # horizon = 2.2 s
BATCH_SIZE = 1200
ITERATION_COUNT = 1

VX_STD = 0.25                 # m/s exploration noise
WZ_STD = 0.35                 # rad/s exploration noise

TEMPERATURE = 0.24
GAMMA = 0.022

VX_MIN = 0.0                  # m/s (no reverse during normal tracking)
VX_MAX = 1.2                  # m/s
WZ_MAX = 2.0                  # rad/s

MOTION_MODEL = 'DiffDrive'
