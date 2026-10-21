# Lightweight online SLAM for Rover Mini (no full Nav2).

## Map (one terminal)

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
```

Starts: hardware (+ lidar reset) · EKF · slam_toolbox · safety_gate · PS4 pad · RViz.

Park the robot at the experiment start **before** launching. That pose becomes the map origin. Confirm `/scan` ~10 Hz, then drive slowly. Check `ros2 topic hz /map`.

`use_rviz:=False` skips the window. The view is `rviz/mapping.rviz` (map, cyan laser, robot). The save bar writes `yaml` + `pgm`; the default path is `~/RoboGait_ws/src/robot_slam_mapping/maps/today`.

**Tune SLAM:** edit `config/mapper_params_online_async.yaml`, then relaunch.

## Save map

While the launch is still running, use the RViz **保存** button, or:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
```

Then record or follow with that map:

```bash
ros2 launch robot_simple_commander record_trajectory.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/my_lab.yaml
```

## Pad only

```bash
ros2 launch robot_slam_mapping mini_teleop.launch.py
```
