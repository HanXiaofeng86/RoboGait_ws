# mppi_navigation

Pure MPPI navigation package for the Rover Robotics Mini (Mode 1: RViz goal; Mode 2: predefined path).  
MPPI produces the full velocity pair `(v, ω)`; other nodes may only pass commands through unchanged or zero them as a group.

## Control chain

```text
RViz 2D Goal
    ↓
bt_navigator → planner_server → /plan
    ↓
controller_server (nav2_mppi_controller)  →  /cmd_vel_nav
    ↓
velocity_smoother                          →  /cmd_vel_smoothed
    ↓
safety_gate (pass-through or zero only)    →  /cmd_vel
    ↓
roverrobotics_driver → chassis
    ↓
wheel odometry + IMU → EKF → /odometry/filtered → next MPPI cycle
```

## Launch (Mode 1)

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Default: map_server + AMCL + lab2_new (set initial pose with 2D Pose Estimate after start)
ros2 launch mppi_navigation rviz_goal_navigation.launch.py

# Online SLAM (no prior map):
ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True

# Software pipe check only — disable AMCL (static map→odom):
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False

# Without RViz:
ros2 launch mppi_navigation rviz_goal_navigation.launch.py use_rviz:=False
```

Note: boolean launch args must be `True` / `False` (capitalized).

## Launch (Mode 2: predefined path)

Start the full nav stack first (Mode 1), then send a path (geometry only; MPPI computes velocity online):

```bash
# Built-in test paths: straight / left_90 / right_90 / s_curve / obstacle_avoid
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

# Or ros2 run:
ros2 run mppi_navigation follow_path --ros-args -p path_file:=/absolute/path/my_path.yaml
```

- Paths with `frame: local` are transformed into `map` from the robot’s current pose
- Path files live in `paths/*.yaml`; regenerate with `python3 scripts/generate_test_paths.py`
- The sent path is also published on `/predefined_path` (for RViz)

Cancel active navigation (Test 7):

```bash
ros2 run mppi_navigation cancel_navigation
```

## Launch files

| File | Role |
|------|------|
| `rviz_goal_navigation.launch.py` | Top-level one-shot (hardware + nav + RViz) |
| `rover_mini_hardware.launch.py` | Chassis driver + URDF + lidar + IMU (includes upstream `mini.launch.py`) |
| `nav2_mppi_controller.launch.py` | EKF + localization (SLAM / AMCL / static TF) + Nav2 stack + velocity chain + safety gate |
| `follow_predefined_path.launch.py` | Mode 2: FollowPath to an already-running stack |
| `ekf_amcl_localization.launch.py` | EKF-only (debug) |

## Config files

| File | Role |
|------|------|
| `config/nav2_pure_mppi_params.yaml` | Full Nav2 params; MPPI baseline: 20 Hz, time_steps 44, batch 1200, vx_std 0.25, wz_std 0.35, temperature 0.24, gamma 0.022, vx∈[0,1.2], wz≤2.0, 8 critics |
| `config/robot_localization_ekf.yaml` | EKF: wheel odom (vx, vy) + IMU (yaw rate) |
| `config/sensor_topics_frames.yaml` | Topic / TF frame reference |
| `config/diagnostic_recording_topics.yaml` | Bag recording topic list |

## Safety gate (`safety_gate`)

- Input: `cmd_vel_smoothed` → output: `/cmd_vel`
- Only two behaviors: **pass through** or **publish zero**; never invent substitute commands
- Zero when: command timeout (0.5 s), odometry timeout (0.5 s), or `/safety_estop` is true
- Software estop:

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"   # stop
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"  # clear
```

## Preflight and test tools

```bash
# Key topics publishing? (default watch window 5 s)
./scripts/preflight_check_topics.sh

# TF chain map→odom→base_link complete?
./scripts/preflight_check_tf.sh

# Test 1 (idle): /cmd_vel must stay zero with no goal
ros2 run mppi_navigation idle_check 10
# or: python3 -m mppi_navigation.preflight.idle_cmd_vel_check 10

# Lidar internal error: software reset (no power cycle)
ros2 run mppi_navigation reset_rplidar

# Record a navigation test bag
./scripts/record_navigation_test.sh straight_test

# Analyze bag: metrics JSON + plots
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/<bag_dir>
```

## MPPI tuning entry points

- Baseline constants: `mppi_navigation/mppi_controller/baseline_params.py`
- Critic notes (8): `mppi_navigation/mppi_controller/critics_reference.py`
- Live parameters: `config/nav2_pure_mppi_params.yaml`  
  (with `--symlink-install`, edit yaml and restart launch—no rebuild required)

## Custom critic (M10)

Nav2 sources are in this workspace: `src/navigation2/nav2_mppi_controller/`.  
Add a critic under `src/critics/*.cpp`, register in `critics.xml`, then:

```bash
colcon build --packages-select nav2_mppi_controller --symlink-install
```

## Difference from the old `rover_workspace`

- No person detection / Orbbec / gimbal / `robot_controller_remapping`
- Final `/cmd_vel` `linear.x` and `angular.z` both come from MPPI (old stack overwrote linear speed with a person-distance PID)
- Do not `source` both workspaces in the same shell

## Further docs

- Chinese handbook: `PROJECT_CHANGES.md` (workspace root)
- English handbook: `PROJECT_CHANGES_EN.md`
- On-robot test plan: `TEST_PLAN.md`
