# robot_simple_commander

**Mode A2** — teleop record / follow. Independent of `robot_mppi_navigation`.

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
scripts/                # offline scripts (not ros2 run nodes)
test/                   # unit tests
robot_simple_commander/
  nodes/                # ros2 run nodes only
  lib/                  # libraries (not nodes)
  safety/               # safety-gate logic
```

## Commands

```bash
ros2 launch robot_simple_commander record_trajectory.launch.py
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=left_90
```
