# robot_simple_commander

**Mode A2** — teleop record / follow. Own Nav2; Mini hardware uses
`robot_mppi_navigation` `rover_mini_hardware` (lidar RESET).

## Layout

```text
launch/                 # ros2 launch entry points
  record_trajectory.launch.py
  follow_trajectory.launch.py
  parts/                # fragments included by entries (not entries)
    hardware.launch.py
    nav_stack.launch.py
config/                 # parameter yaml
maps/                   # maps
paths/                  # built-in exam path geometry
recorded/               # teleop-recorded routes
rviz/                   # record/follow RViz
src/follow_panel.cpp    # RViz 开始 / 回到起点 / 停止
bin/                    # ros2 run entries
test/                   # unit tests
robot_simple_commander/
  nodes/                # ros2 run nodes only
  lib/                  # libraries (not nodes)
  safety/               # safety-gate logic
```

## Commands

```bash
# edit src/robot_simple_commander/config/route_name.yaml  (name: my_loop)
ros2 launch robot_simple_commander record_trajectory.launch.py
# Wait for the laser → 2D Pose Estimate → drive → Ctrl+C saves

ros2 launch robot_simple_commander follow_trajectory.launch.py
# Wait ~15–20 s (lidar RESET + CAN; RViz opens after that; do not Ctrl+C)
# → 2D Pose Estimate at the route start → 开始
# After the run, 回到起点 drives forward along the recording back to the start
# 停止 cancels motion and leaves the launch up. Ctrl+C exits.
```
