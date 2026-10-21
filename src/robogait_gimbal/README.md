# robogait_gimbal

**Mode B** — two-axis camera gimbal (Dynamixel pan + tilt). Does **not** publish `/cmd_vel`.

Keeps the Orbbec Femto Mega pointed at the person for **gait capture**: pan from torso/pelvis X pixel, tilt from camera-frame elevation `atan2(y, z)`. Tilt is slow (large dead zone) so walking does not nod the camera. Pan stays snappy for corridor corners.

**Skeleton comes from Nuitrack** (default `person_source:=nuitrack`), not from Azure k4abt on CPU. Overlay `~/rover_workspace_final` so `nuitrack_person_node` is found. SDK (`person_source:=sdk`) is the point-cloud demo only. Do not start both — same USB camera.

`person_source` alone does **not** move the servos. Use `test_gimbal` so the gimbal tracks you and RViz rotates the cyan camera.

Straight-follow test: set `freeze_gimbal: true` on both `camera_tracker` and `dynamixel_driver` in `config/gimbal.yaml` (lens stays at idle). Corners: `freeze_gimbal: false` and `enable_gimbal_pan: true`.

## Layout

```text
launch/                 # entry
  test_gimbal.launch.py
config/                 # U2D2 by-id, pan/tilt IDs and tick limits, PID
robogait_gimbal/
  nodes/                # camera_tracker, dynamixel_driver
```

Same U2D2 bus as `rover_workspace_final` `tower_controller`: **ID 1 = pan**, **ID 2 = tilt**. Servos need **12 V** (U2D2 is data only). On start the driver slowly goes to home (forward / slightly up).

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py

# Teacher point-cloud demo (no skeleton on this NUC):
ros2 launch robogait_gimbal test_gimbal.launch.py person_source:=sdk
```

Manual jog (after `dynamixel_driver` is up; values are relative degrees, dead zone 2.5°):

```bash
ros2 topic pub --once /camera_position std_msgs/msg/Float32 "{data: 20.0}"
ros2 topic pub --once /camera_tilt std_msgs/msg/Float32 "{data: 15.0}"
```
