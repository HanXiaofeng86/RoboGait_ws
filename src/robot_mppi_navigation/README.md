# robot_mppi_navigation

Nav2 / **MPPI** stack for Rover Mini: hardware, localization, planner, MPPI
controller, smoother, safety gate. Used for **Mode 1 (RViz Goal)**.

**Independent Nav2 from `robot_simple_commander`.** Path record/follow is a
separate package — do not run both full stacks at once.

Gamepad: `ros2 launch robot_mppi_navigation mini_teleop.launch.py`
(not vendor `roverrobotics_driver mini_teleop` — that skips lidar RESET).

## Mode 1

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

## Control chain

```text
RViz Goal → bt_navigator → planner → MPPI → smoother → safety_gate → /cmd_vel
```

## Layout

```text
launch/   rviz_goal_navigation, nav2_mppi_controller, rover_mini_hardware, …
config/   nav2_pure_mppi_params.yaml, ekf, …
maps/     lab2_new
robot_mppi_navigation/safety/   safety_gate
```

See workspace Chinese handbook `PROJECT_CHANGES.md` §2 for file-by-file notes
(English twin: `PROJECT_CHANGES_EN.md`).
