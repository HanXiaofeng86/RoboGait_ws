# RoboGait_ws

ROS 2 Humble workspace for **pure MPPI** navigation on a Rover Robotics Mini.

Nav2 MPPI produces the full velocity `(v, ω)`. Other nodes may only **pass through or zero** commands (safety gate)—no PID velocity mixing on `/cmd_vel`.

## Docs

| File | Content |
|------|---------|
| [PROJECT_CHANGES.md](PROJECT_CHANGES.md) | Full handbook (**Chinese** — keep as the CN source of truth) |
| [PROJECT_CHANGES_EN.md](PROJECT_CHANGES_EN.md) | Full handbook (English) |
| [TEST_PLAN.md](TEST_PLAN.md) | On-robot test checklist (English) |
| [src/mppi_navigation/README.md](src/mppi_navigation/README.md) | Package README (English) |

## Quick start

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mppi_navigation --symlink-install
source install/setup.bash

# can0 must be UP (see handbook §4.8)
ros2 launch mppi_navigation rviz_goal_navigation.launch.py
```

## Layout

- `src/mppi_navigation` — pure MPPI stack (main package)
- `src/roverrobotics_ros2`, `src/rplidar_ros`, `src/bno055`, `src/navigation2` — dependencies vendored in this workspace

Build artifacts (`build/`, `install/`, `log/`) are not tracked.
