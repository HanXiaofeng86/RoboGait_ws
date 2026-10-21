"""MPPI baseline parameters (design doc section 7).

Single source of truth mirrored in config/nav2_pure_mppi_params.yaml.
These are starting values for the tuning workflow, not final values.
"""

CONTROLLER_FREQUENCY = 20.0   # Hz; must equal 1/model_dt (Humble MPPI)
MODEL_DT = 0.05               # s
TIME_STEPS = 30               # horizon = 1.5 s
BATCH_SIZE = 512
ITERATION_COUNT = 1

VX_STD = 0.22                 # m/s exploration noise
WZ_STD = 0.35                 # rad/s exploration noise

TEMPERATURE = 0.32
GAMMA = 0.022

VX_MIN = 0.0                  # m/s no reverse; MPPI rotates then drives
VX_MAX = 1.00                 # m/s ceiling (open space may use more of it)
WZ_MAX = 1.20                 # rad/s

MOTION_MODEL = 'DiffDrive'
