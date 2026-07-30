"""The /cmd_vel publishing chain (design doc section 2).

    controller_server (MPPI)  publishes  cmd_vel_nav
    velocity_smoother         subscribes cmd_vel_nav, publishes cmd_vel_smoothed
    safety_gate               subscribes cmd_vel_smoothed, publishes cmd_vel
    roverrobotics_driver      subscribes cmd_vel

Invariant: no node other than the safety gate may write /cmd_vel, and the
gate only forwards unchanged or zeroes the command.
"""

from robot_mppi_navigation.cmd_vel_pipeline.topic_names import (
    CMD_VEL_FINAL,
    CMD_VEL_MPPI,
    CMD_VEL_SMOOTHED,
)

CHAIN = (CMD_VEL_MPPI, CMD_VEL_SMOOTHED, CMD_VEL_FINAL)

# Remappings applied in nav2_mppi_controller.launch.py
CONTROLLER_REMAP = ('cmd_vel', 'cmd_vel_nav')
SMOOTHER_INPUT_REMAP = ('cmd_vel', 'cmd_vel_nav')
