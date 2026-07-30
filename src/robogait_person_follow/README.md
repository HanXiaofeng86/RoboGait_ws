# robogait_person_follow

**Mode B** — person provides a Nav2 goal only; chassis velocity comes entirely from **MPPI** (no PID blend on `/cmd_vel`).

## Layout

```text
launch/                 # flat entry launches
  person_follow.launch.py   # B1 one-shot: Nav2/MPPI + person layer
  person_layer.launch.py    # B2 terminal 2: layer on running Nav
config/                 # standoff distance / goal update params
robogait_person_follow/
  nodes/                # distance_filter, person_nav_goal
```

```bash
# One-shot
ros2 launch robogait_person_follow person_follow.launch.py

# Or two terminals
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py
```
