# RoboGait_ws Project Handbook (English)

> This file documents the architecture, file roles, and common commands for the `mppi_navigation` package (pure MPPI navigation).  
> **Chinese counterpart:** `PROJECT_CHANGES.md` — **keep both in sync** when updating (see `.cursor/rules/update-changelog.mdc`).

---

## 1. Project overview

- Workspace: `~/RoboGait_ws`
- Core package: `src/mppi_navigation` (ament_python)
- Goal: Rover Mini navigates with **Nav2 MPPI only**; MPPI produces the full velocity pair `(v, ω)`; other nodes may only pass through or zero the whole command
- Current stage: Mode 1 + Mode 2 code complete; **hardware restored (2026-07-21)**; AMCL on by default + default map `lab2_new`

> Opening `.md` in this workspace defaults to preview (`.vscode/settings.json`). Tables are standard Markdown.  
> **Who calls whom / relationship diagrams: see Section 2.**

---

### 1.1 Why launches are split this way

Same idea as common Nav2 / Rover bringup: three layers; the top level only assembles them.

| Layer | Single responsibility | Benefit |
|-------|----------------------|---------|
| `rover_mini_hardware` | Sensors + chassis only | Debug hardware without Nav2 |
| `nav2_mppi_controller` | Localization + Nav2 + velocity chain | Change map / AMCL / SLAM without restarting hardware |
| `rviz_goal_navigation` | One-shot: both layers + RViz | One command for daily use |

**Why this helps**

- **Thin top level, reusable lower layers**: Mode 2 only needs `follow_predefined_path` on top
- **RViz optional**: `use_rviz:=False`
- **Velocity remaps centralized** in `nav2_mppi_controller`

**Note:** Run only the top level day-to-day; do not also launch `nav2_mppi_controller` separately or you get two Nav2 stacks.

---

### 1.2 Launch include tree

```text
                    rviz_goal_navigation.launch.py
                           (daily entry)
                                  │
          ┌───────────────────────┼───────────────────────┐
          ▼                       ▼                       ▼
   rover_mini_hardware            nav2_mppi_controller                 rviz2
      (hardware)               (navigation)                (optional)
          │                       │
          │                       ├── EKF
          │                       ├── map + AMCL (default)
          ├── driver              ├── planner / BT / MPPI
          ├── rplidar             └── smoother → safety_gate
          ├── bno055
          └── robot_state_publisher

                    follow_predefined_path.launch.py
                         (Mode 2: separate terminal)
                                  │
                                  └──► FollowPath to the already-running nav stack
```

---

### 1.3 What starts what / when to use

| Launch | Role | Hardware | Nav2/MPPI | RViz |
|--------|------|:--------:|:---------:|:----:|
| `rviz_goal_navigation.launch.py` | **Top-level one-shot** | Y | Y | Y (can disable) |
| `nav2_mppi_controller.launch.py` | **Nav stack** | N | Y | N |
| `rover_mini_hardware.launch.py` | **Hardware layer** | Y | N | N |
| `follow_predefined_path.launch.py` | **Mode 2 path send** | N | N (needs running stack) | N |
| `ekf_amcl_localization.launch.py` | EKF debug | N | N | N |

| Launch | When to use |
|--------|-------------|
| `rviz_goal_navigation` | Daily nav, set pose, send Goal |
| `nav2_mppi_controller` | Usually **do not run alone**; included by top level |
| `rover_mini_hardware` | Test lidar / IMU / chassis only |
| `follow_predefined_path` | After full stack is up, send a predefined path |
| `ekf_amcl_localization` | Rarely needed |

**One line:** `rviz_goal_navigation` = hardware + navigation + RViz; RViz only views and clicks, it is not the controller.

---

### 1.4 Localization modes

| Launch arg | Who publishes map→odom | Needs map? | 2D Pose Estimate | Use case |
|------------|------------------------|------------|------------------|----------|
| **Default** `amcl:=True` | AMCL | Yes (`lab2_new`) | Yes (set first) | **Real navigation** |
| `amcl:=False` | Static identity TF | Map loaded but no loc | No effect | Software pipe check only |
| `slam:=True` | SLAM Toolbox | No (build online) | Usually unused | No prior map |

---

### 1.5 Control chain

```text
RViz (Goal / Pose)
        │
        ▼
  bt_navigator ──► planner ──► /plan ──► MPPI
                                           │
                              /cmd_vel_nav ▼
                              velocity_smoother
                                           │
                         /cmd_vel_smoothed ▼
                              safety_gate (pass or zero)
                                           │
                                /cmd_vel ▼
                              driver → chassis
                                           │
                    wheels + IMU ──► EKF ──┘
                    /scan + map ──► AMCL ──┘ (back to MPPI)
```

| Topic | Publisher | Subscriber | Meaning |
|-------|-----------|------------|---------|
| `/cmd_vel_nav` | MPPI / behavior | velocity_smoother | Raw MPPI output |
| `/cmd_vel_smoothed` | velocity_smoother | safety_gate | After limits / accel smoothing |
| `/cmd_vel` | **Only** safety_gate | driver | Final command; pass-through or all zeros |

---

### 1.6 Node groups after default bringup

| Group | Nodes | Role |
|-------|-------|------|
| Hardware | driver / rplidar / bno055 / rsp | Chassis, lidar, IMU, robot TF |
| Localization | map_server + amcl + ekf | Map, particle filter, odom fusion |
| Nav2 | controller(MPPI) / planner / bt / smoother / … | Plan and track |
| Custom | safety_gate | Final `/cmd_vel` gate |
| UI | rviz2 | Viz and manual goals |

---

## 2. File relationships (who calls whom)

> Section 2 = how files connect; Section 3 = what each file does.  
> Diagrams are plain text (no Mermaid needed for preview).

### 2.1 Big picture: from your command to the wheels

```text
Your command
    │
    ▼
┌───────────────────────────────────────────────────────────┐
│  launch/                                                  │
│   rviz_goal_navigation.launch.py     ← daily one-shot     │
│        │                                                  │
│        ├── include → rover_mini_hardware.launch.py        │
│        │                 └── upstream mini.launch         │
│        │                      (chassis / lidar / IMU / URDF)│
│        │                                                  │
│        └── include → nav2_mppi_controller.launch.py       │
│                          ├── loads config/*.yaml          │
│                          ├── starts EKF / AMCL / Nav2 / MPPI│
│                          └── starts safety_gate (custom)  │
└───────────────────────────────────────────────────────────┘
        │
        ▼ runtime topics (result of the graph)
  /scan /imu /wheels → EKF → /odometry/filtered
  RViz Goal → plan → MPPI → /cmd_vel_nav → smoother → safety_gate → /cmd_vel → chassis
```

### 2.2 How launches nest

```text
                    ┌─────────────────────────────┐
                    │ rviz_goal_navigation        │
                    │ (top: hardware+nav+optional RViz)│
                    └─────────────┬───────────────┘
                                  │ include
              ┌───────────────────┴───────────────────┐
              ▼                                       ▼
┌─────────────────────────┐             ┌─────────────────────────┐
│ rover_mini_hardware     │             │ nav2_mppi_controller    │
│ sensors + chassis only  │             │ EKF+loc+MPPI+safety     │
└─────────────────────────┘             └───────────┬─────────────┘
                                                    │ loads
                                                    ▼
                                        config/nav2_pure_mppi_params.yaml
                                        config/robot_localization_ekf.yaml
                                        maps/lab2_new.yaml (default map)

Side paths (do not replace top level):

  follow_predefined_path.launch.py
       │  requires: nav stack already running
       ▼
  path_following/follow_path_client.py  →  FollowPath
       ▲
       │ or directly
  ros2 run mppi_navigation follow_path

  ekf_amcl_localization.launch.py  →  EKF debug only
```

| Relation | Meaning |
|----------|---------|
| Top **includes** hardware + nav | Daily: one `rviz_goal_navigation` |
| Nav layer **loads** `config/` | Edit yaml, restart launch; usually no launch edit |
| Mode 2 launch does **not** start hardware | It only injects a path into the running stack |

### 2.3 Who uses which config

```text
config/nav2_pure_mppi_params.yaml
    └─ passed by nav2_mppi_controller.launch.py to Nav2 / MPPI / smoother / AMCL / safety

config/robot_localization_ekf.yaml
    └─ loaded by ekf_node (robot_localization) in the same launch

config/sensor_topics_frames.yaml
    └─ human docs + preflight reference; does not start nodes

config/diagnostic_recording_topics.yaml
    └─ topic list for scripts/record_navigation_test.sh

maps/lab2_new.yaml (+ .pgm)
    └─ default map for map_server (AMCL mode)

paths/*.yaml
    └─ used by follow_path / follow_predefined_path / generate_test_paths
```

### 2.4 Custom Python: entry points → implementations

```text
ros2 run command                       implementation
─────────────────                      ────────────────
safety_gate          ──nodes/──►     safety/cmd_vel_safety_gate.py
                                       ├─ stop_conditions.py
                                       ├─ pass_or_zero_policy.py
                                       └─ twist_copy.py

follow_path          ──nodes/──►     path_following/follow_path_client.py
                                       ├─ path_io.py
                                       └─ generators.py

cancel_navigation    ──nodes/──►     path_following/cancel_client.py

idle_check           ──nodes/──►     preflight/idle_cmd_vel_check.py

reset_rplidar        ──nodes/──►     tools/reset_rplidar.py
                     (scripts/reset_rplidar.py is the same job)
```

**Remember:** `nodes/` = remote buttons; `safety/`, `path_following/`, etc. = the circuit board.

### 2.5 scripts/ vs source modules

```text
scripts/preflight_check_topics.sh  ──►  preflight/topic_health_check.py
scripts/preflight_check_tf.sh      ──►  preflight/tf_tree_check.py
scripts/record_navigation_test.sh  ──►  diagnostic_recording_topics.yaml + ros2 bag
scripts/analyze_navigation_bag.py  ──►  bag_analysis/* + generate_plots
scripts/generate_test_paths.py     ──►  path_following/generators.py → paths/*.yaml
scripts/reset_rplidar.py           ──►  tools/reset_rplidar.py
```

### 2.6 Runtime data flow

```text
[hardware launch]
  rplidar ──► /scan ─────────────────────┐
  bno055  ──► /imu/data ──┐              │
  driver  ──► /odometry/wheels ──┐       │
                                 ▼       │
                    EKF ← robot_localization_ekf.yaml
                                 │
                                 ▼
                      /odometry/filtered
                      TF: odom→base_link
                                 │
[nav launch]                     │
  map_server ← maps/lab2_new     │
  amcl ← /scan + map             │
         TF: map→odom            │
                                 ▼
  bt_navigator / planner / MPPI ← nav2_pure_mppi_params.yaml
                                 │
                                 ▼ /cmd_vel_nav
                         velocity_smoother
                                 │
                                 ▼ /cmd_vel_smoothed
                    safety_gate ← safety/*.py
                                 │
                                 ▼ /cmd_vel
                               chassis driver
```

### 2.7 Find files by intent

| You want to… | Look at |
|--------------|---------|
| Start / layering | `launch/` + §2.2 + Section 1 |
| Sensors only | `rover_mini_hardware` + commands §4.8 |
| Tune speed / MPPI / AMCL | `config/nav2_pure_mppi_params.yaml` |
| Tune EKF fusion | `config/robot_localization_ekf.yaml` |
| Safety gate logic | `mppi_navigation/safety/` |
| Predefined path tests | `paths/` + `path_following/` + `follow_predefined_path.launch.py` |
| Preflight / bag / analysis | `scripts/` + `preflight/` + `bag_analysis/` |
| One-line file roles | **Section 3 file handbook** |

---

## 3. File handbook (by directory)

> **How to read:** This lists files, not daily run commands.  
> - **Operate the robot:** `launch/`, `scripts/`, Section 4.  
> - **Tune params:** `config/`.  
> - **How files connect:** Section 2.  
> - **`mppi_navigation/` `.py` trees:** mostly implementation; skip for daily driving.

### Must-read vs skip

| Directory | Must-read? | One line |
|-----------|------------|----------|
| `launch/` | Yes | Which command starts what |
| `config/` | When tuning | Nav / EKF params in yaml |
| `scripts/` | Yes | Preflight, bag, lidar reset helpers |
| `paths/` / `maps/` | Mode 2 / map swap | Test paths, lab map |
| `mppi_navigation/safety/` etc. | No (dev) | Source layout, not a start menu |
| `nodes/` | No | Thin `ros2 run` wrappers |

---

### launch/ (start switches)

| File | Role | When you use it |
|------|------|-----------------|
| `rviz_goal_navigation.launch.py` | **Daily one-shot:** hardware + loc + MPPI + safety + RViz | Set pose and Goal in RViz |
| `rover_mini_hardware.launch.py` | **Sensors + chassis only**, no nav | Check lidar / IMU / wheel odom |
| `nav2_mppi_controller.launch.py` | Nav “brain” (EKF, AMCL, MPPI, safety) | **Usually do not run alone**; top level includes it |
| `follow_predefined_path.launch.py` | Follow a yaml path instead of RViz Goal | Fixed tests with full stack already up |
| `ekf_amcl_localization.launch.py` | EKF-only debug | Rarely |

Common top-level args: `amcl:=True` (default), `slam:=True`, `use_rviz:=False`. See Section 1 for layering.

---

### config/ (knobs, not programs)

| File | Role |
|------|------|
| `nav2_pure_mppi_params.yaml` | **Main nav config:** MPPI tracking, limits, smoother, safety timeout, AMCL. Usually no rebuild; restart launch |
| `robot_localization_ekf.yaml` | **EKF fusion:** wheel vx/vy + IMU yaw rate → `/odometry/filtered` and `odom→base_link` |
| `sensor_topics_frames.yaml` | Topic/frame cheat sheet (docs + preflight); does not start nodes |
| `diagnostic_recording_topics.yaml` | Which topics to record (for `record_navigation_test.sh`) |

MPPI baseline numbers live in `nav2_pure_mppi_params.yaml`; open that file when tuning—no need to read Python first.

---

### mppi_navigation/ (Python package)

Most files are used indirectly by launch; you rarely run them with raw `python3`.

#### safety/ — last gate on velocity

> Rule: only **PASS** (unchanged) or **ZERO**; never invent a substitute trajectory.

| File | Role |
|------|------|
| `cmd_vel_safety_gate.py` | Gate node: listens to smoothed cmd + odom + estop; publishes to chassis or zeros; watchdog zeros if cmds vanish while moving |
| `stop_conditions.py` | When to stop: cmd timeout, odom timeout, estop |
| `pass_or_zero_policy.py` | Maps conditions to PASS / ZERO |
| `twist_copy.py` | Copy Twist, make zero, is-zero helpers |

Started automatically by `rviz_goal_navigation`; no need to run by hand.

#### nodes/ — CLI entry points (thin)

| File | Command | Real work lives in |
|------|---------|-------------------|
| `run_cmd_vel_safety_gate.py` | `ros2 run mppi_navigation safety_gate` | → `safety/` |
| `run_follow_path_client.py` | `ros2 run mppi_navigation follow_path` | → `path_following/` |
| `run_cancel_navigation.py` | `ros2 run mppi_navigation cancel_navigation` | → `cancel_client.py` |
| `run_idle_cmd_vel_check.py` | `ros2 run mppi_navigation idle_check` | → `preflight/` |
| `run_reset_rplidar.py` | `ros2 run mppi_navigation reset_rplidar` | → `tools/reset_rplidar.py` |

Think “remote buttons”; no need to read line by line.

#### path_following/ — Mode 2 predefined paths

> Mode 1 = RViz Goal; Mode 2 = follow a prepared yaml path (repeatable straight/turn tests).

| File | Role |
|------|------|
| `generators.py` | Build test geometries (straight, ±90°, S-curve, …)—poses only, no speeds |
| `path_io.py` | Read/write path yaml |
| `follow_path_client.py` | Load yaml → map frame → FollowPath; MPPI still computes velocity online |
| `cancel_client.py` | Cancel active navigation (safety gate then zeros on timeout) |

#### cmd_vel_pipeline/ — topic name constants

| File | Role |
|------|------|
| `topic_names.py` | Constants for `/cmd_vel`, `/scan`, etc. |
| `remap_chain.py` | Documents remaps: MPPI → smoother → safety → chassis |

#### mppi_controller/ — tuning reference code

| File | Role |
|------|------|
| `baseline_params.py` | Mirrors yaml baseline for easy comparison |
| `critics_reference.py` | Text notes on the 8 critics; not the live controller |

#### localization/

| File | Role |
|------|------|
| `tf_frame_names.py` | `map` / `odom` / `base_link` constants |
| `readiness_checks.py` | TF chain ready? (for preflight) |

#### tools/

| File | Role |
|------|------|
| `reset_rplidar.py` | Serial RESET for lidar internal error (no power cycle needed) |

#### preflight/

| File | Role |
|------|------|
| `topic_health_check.py` | Listen briefly: are key topics publishing? |
| `tf_tree_check.py` | Is `map→odom→base_link` complete? |
| `idle_cmd_vel_check.py` | Idle: `/cmd_vel` must stay zero |

Prefer `scripts/preflight_check_*.sh` or `ros2 run … idle_check`.

#### bag_analysis/

| File | Role |
|------|------|
| `load_rosbag.py` | Load rosbag into time series |
| `path_tracking_metrics.py` | Cross-track error |
| `heading_error_metrics.py` | Heading error |
| `velocity_smoothness_metrics.py` | Accel / jerk smoothness |
| `controller_timing_metrics.py` | Control rate / dropped frames |
| `generate_plots.py` | Trajectory / speed / error plots |

Entry: `python3 scripts/analyze_navigation_bag.py <bag_path>`.

#### launch_config/

| File | Role |
|------|------|
| `package_share_paths.py` | Resolve installed config/launch/rviz paths |
| `common_launch_args.py` | Shared launch args (`True`/`False` booleans) |

---

### scripts/

| File | Role |
|------|------|
| `preflight_check_topics.sh` | One-shot topic health |
| `preflight_check_tf.sh` | One-shot TF check |
| `record_navigation_test.sh` | Record bag under `~/RoboGait_ws/bags/` |
| `analyze_navigation_bag.py` | Bag → metrics + plots |
| `generate_test_paths.py` | Regenerate `paths/` test yaml |
| `reset_rplidar.py` | Lidar software reset (or `ros2 run mppi_navigation reset_rplidar`) |

---

### paths/

| File | Role |
|------|------|
| `straight.yaml` | ~3 m straight |
| `left_90.yaml` | Forward + left 90° + forward |
| `right_90.yaml` | Same, right turn |
| `s_curve.yaml` | S-curve |
| `obstacle_avoid.yaml` | Straight (place a box for obstacle tests) |

---

### maps/

| File | Role |
|------|------|
| `lab2_new.yaml` + `lab2_new.pgm` | **Default lab map** (AMCL / map_server) |
| `lab2_new.posegraph` + `lab2_new.data` | Same map for slam_toolbox; **not** used by default nav path yet |

---

### Other package / workspace docs

| File | Role |
|------|------|
| `package.xml` | Package name and dependencies |
| `setup.py` / `setup.cfg` | Install + `ros2 run` entry points |
| `rviz/rviz_goal_navigation.rviz` | Default RViz layout |
| `README.md` | Package usage notes |
| `test/test_safety.py` | Unit tests for safety gate |
| `test/test_path_following.py` | Unit tests for path gen / IO |

| Item | Role |
|------|------|
| `.vscode/settings.json` | Markdown opens in preview |
| `TEST_PLAN.md` | On-robot acceptance: stages A/B + tests T1–T8 |
| `PROJECT_CHANGES.md` | Chinese handbook (keep in sync with this file) |
| `README.md` | GitHub / repo landing summary |
| `.gitignore` | Ignore build/install/log/bags (no build artifacts in git) |
| `src/navigation2/` | Nav2 sources (for custom critics, etc.) |
| `ARCHITECTURE.html` | Optional backup; Section 1 of this handbook is authoritative |

---

## 4. Common commands

> Every new terminal: run §4.1 first. Boolean launch args must be `True`/`False`.

### 4.1 Environment

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Without sourcing you get `Package 'mppi_navigation' not found`.

### 4.2 Build

```bash
colcon build --packages-select mppi_navigation --symlink-install
```

- `--packages-select`: build only this package
- `--symlink-install`: edit yaml/launch without rebuild; restart launch to apply

### 4.3 Mode 1 (RViz goal)

```bash
ros2 launch mppi_navigation rviz_goal_navigation.launch.py

ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False
ros2 launch mppi_navigation rviz_goal_navigation.launch.py use_rviz:=False
```

Default map: `maps/lab2_new.yaml`. Override with `map:=/abs/path/other.yaml` (yaml + pgm same directory).

After start: **2D Pose Estimate**, then **Nav2 Goal**.

### 4.4 Mode 2 (predefined path)

With full stack already running (another terminal):

```bash
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90
# path_name: straight / left_90 / right_90 / s_curve / obstacle_avoid

ros2 launch mppi_navigation follow_predefined_path.launch.py path_file:=/abs/path/my_path.yaml
ros2 run mppi_navigation follow_path --ros-args -p path_file:=/abs/path/my_path.yaml
```

Yaml stores geometry only; MPPI computes velocity. `frame: local` paths are transformed from the robot’s current pose into `map`. Path also published on `/predefined_path`. Log `status 4` = SUCCEEDED.

### 4.5 Stop / estop

```bash
ros2 run mppi_navigation cancel_navigation

ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

### 4.6 Preflight

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/preflight_check_topics.sh
./scripts/preflight_check_tf.sh
ros2 run mppi_navigation idle_check 10
```

Exit 0 = pass.

| Tag | Meaning |
|-----|---------|
| `[OK]` | Topic/TF seen in the window |
| `[FAIL]` | Required item missing |
| `[----] (optional)` | Optional missing is fine (e.g. no `/cmd_vel*` yet) |

`[FAIL] /map` often means map_server published once only—confirm with `ros2 topic echo /map --once`.

### 4.7 Bag record / analysis

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/record_navigation_test.sh straight_run1
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/straight_run1
```

### 4.8 Sensor / hardware check

Order: **devices → can0 up → hardware launch → topic hz** (one topic at a time on Humble).

```bash
source /opt/ros/humble/setup.bash && source ~/RoboGait_ws/install/setup.bash

ls -l /dev/rplidar /dev/bno055
ip link show can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
candump -n 10 -T 3000 can0

ros2 launch mppi_navigation rover_mini_hardware.launch.py

ros2 topic hz /scan                # ~10 Hz
ros2 topic hz /imu/data            # ~50–60 Hz
ros2 topic hz /odometry/wheels     # ~15–30 Hz
```

| Sensor | Device | Topic | Expected rate |
|--------|--------|-------|---------------|
| RPLidar | `/dev/rplidar` | `/scan` | ~10 Hz |
| BNO055 | `/dev/bno055` | `/imu/data` | ~50–60 Hz |
| Chassis | `can0` UP | `/odometry/wheels` | ~15–30 Hz |
| EKF (full stack) | — | `/odometry/filtered` | steady output |

After robot reboot / USB re-enumerate: Ctrl+C old launch, then 0→1→2 again. Stale nodes cause IMU `Errno 5` and dead lidar processes.

### 4.9 Hardware troubleshooting

```bash
# Lidar health status 2 / internal error
ros2 run mppi_navigation reset_rplidar
# wait 2–3 s, then relaunch drivers

# Chassis FATAL "data is stale" → can0 UP? candump? robot powered?
# IMU Errno 5 → stop old bno055, relaunch clean
```

### 4.10 Diagnostics

```bash
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /odometry/wheels
ros2 topic hz /odometry/filtered
ros2 topic hz /cmd_vel

ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom

ros2 topic list | grep cmd_vel
ros2 topic info /cmd_vel --verbose    # only safety_gate should publish
ros2 topic echo /cmd_vel_nav
ros2 lifecycle get /bt_navigator
```

---

## 5. Known issues / TODO

| Item | Status |
|------|--------|
| ~~RPLidar internal error (health 2)~~ | **Fixed (07-21):** serial RESET; see §4.9 / §4.8; repeat reset or power-cycle if it returns |
| ~~BNO055 Payload length mismatch~~ | **Fixed (07-21):** rewired |
| `slam:=True` online mapping on robot | Hardware OK; still to verify |
| Full RViz Goal nav (8-test protocol) | Follow `TEST_PLAN.md`: A/B then T1→T2→T7, then rest |
| AMCL on-robot check | Default on; tick Stage B in TEST_PLAN |
| Mode 2 (5 FollowPath routes) | TEST_PLAN **T3–T6** |
| Full TEST_PLAN T1–T8 | Hardware OK; checklist reorganized; run on robot |
| slam_toolbox pose-graph localization | `lab2_new.posegraph` exists; launch not wired (AMCL OK) |
| Custom critic (M10) | Sources ready; not started |
| gazebo_ros_pkgs missing | `sudo apt install ros-humble-gazebo-ros-pkgs` when needed |
