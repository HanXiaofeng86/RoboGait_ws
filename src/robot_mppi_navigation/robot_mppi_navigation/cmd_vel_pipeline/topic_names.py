"""Canonical topic names for the pure MPPI stack."""

# Velocity command chain (order matters; see remap_chain.py)
CMD_VEL_MPPI = '/cmd_vel_nav'          # controller_server (MPPI) output
CMD_VEL_SMOOTHED = '/cmd_vel_smoothed'  # velocity_smoother output
CMD_VEL_FINAL = '/cmd_vel'             # safety gate output -> rover driver

# Sensors / state
SCAN = '/scan'
IMU = '/imu/data'
WHEEL_ODOM = '/odometry/wheels'
FILTERED_ODOM = '/odometry/filtered'

# Navigation
MAP = '/map'
PLAN = '/plan'
TRANSFORMED_PLAN = '/transformed_global_plan'
LOCAL_COSTMAP = '/local_costmap/costmap'

# Safety
ESTOP = '/safety_estop'
