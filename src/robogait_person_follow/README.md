# robogait_person_follow

**Mode B** — two separate person lines. Do not run them together.

| Line | What moves the chassis |
|------|------------------------|
| **B1 / B2** | Person becomes a Nav2 goal. **MPPI** owns `[v, ω]`. |
| **B3** | Straight distance only. No Nav2, no steering. Gimbal stays at idle. |

## Layout

```text
launch/                 # flat entry launches
  person_follow.launch.py   # B1 one-shot: Nav2/MPPI + person layer
  person_layer.launch.py    # B2 terminal 2: layer on running Nav
  straight_follow.launch.py # B3 distance only, gimbal frozen, no Nav2
config/                 # B1 standoff (person_follow.yaml); B3 band/speed (straight_follow.yaml)
robogait_person_follow/
  nodes/                # distance_filter, person_nav_goal, straight_follow
```

```bash
# One-shot (Nuitrack skeleton; overlay rover_workspace_final first)
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_person_follow person_follow.launch.py

# Or two terminals
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py

# B3 straight line (no Nav2). Stop B1 first. Speed/band: config/straight_follow.yaml
ros2 launch robogait_person_follow straight_follow.launch.py
```
