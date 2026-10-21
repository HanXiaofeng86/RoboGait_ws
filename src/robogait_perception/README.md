# robogait_perception

**Mode B** — person in front of the Femto Mega. Two front-ends, **one camera owner**.

| `person_source` | What it is |
|-----------------|------------|
| **`nuitrack`** | **Gait / gimbal skeleton** (old working method). `orbbec_person_publisher` / `nuitrack_person_node` from `~/rover_workspace_final`. Lock point = **torso**. |
| **`sdk`** | Orbbec K4A wrapper point cloud / color (teacher demo). Azure k4abt is **off** (`enable_body: false`) — this NUC has Intel Iris Xe, no CUDA; CPU body tracking never locks a pelvis. |

Both publish `/orbbec/person_position`, `/orbbec/person_center_pixel`, `/orbbec/num_users`. `distance_orbbec` turns those into `user_raw` (x right, y up, z forward) for the gimbal / follow stack.

**Do not start SDK and Nuitrack at the same time** — they open the same USB camera.

```bash
# Tracking / gait (default for test_gimbal). Overlay first:
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_perception person_source.launch.py person_source:=nuitrack

# Point cloud only (no Nuitrack, no k4abt)
ros2 launch robogait_perception person_source.launch.py person_source:=sdk
```

RViz Fixed Frame: `base_link`. This launch is **view only** (no Dynamixel). To have the camera follow you:

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```
