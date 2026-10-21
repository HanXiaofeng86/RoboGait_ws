# RoboGait_ws Project Handbook (beginner-friendly)

> **English handbook.** Chinese source of truth: `PROJECT_CHANGES.md` (keep in sync).  
> **Chinese-only docs:** `PROJECT_CHANGES.md`, `TEST_PLAN.md`. Package READMEs, launch docs, and `NODES_AND_TOPICS.md` are English.  
> Structure: short overview, then **per package**, every important file explained—**what it does, when you edit it, where to go for a given change**.

---

## 1. Two sentences first

**Package prefix:**

| Prefix | Meaning |
|--------|---------|
| **`robot_*`** | Pure robot (no person) |
| **`robogait_*`** | Person follow |
| Other | Drivers / Nav2 third-party |

**Mode A — pure robot** (`robot_*`)

1. **`robot_mppi_navigation`** = **A1**: RViz goals. Hardware + Nav2 + MPPI + safety (**map localization**; no mapping entry).  
2. **`robot_slam_mapping`** = **mapping**: online SLAM + gamepad; then use A1 with the saved map.  
3. **`robot_simple_commander`** = **A2**: teleop **record** / **follow** (panel **开始 / 回到起点 / 停止**).  
   **No dependency on A1** (own stack copy — duplication OK).

**Mode B — person follow** (`robogait_*`, Python)

4. **`robogait_person_interfaces`** = `User.msg` (detection + xyz).  
5. **`robogait_perception`** = see person (**Nuitrack skeleton for gimbal/gait**; SDK = point cloud demo).  
6. **`robogait_gimbal`** = pan+tilt gimbal (pixel / elevation PID + two Dynamixels).  
7. **`robogait_person_follow`** = two exclusive person lines. **B1/B2**: person → Nav2 goal, chassis velocity from MPPI. **B3**: straight follow, no Nav2, distance controls forward/back only.

```text
Mode A (robot_* pure)               Mode B (robogait_* person)
  A1 robot_mppi_navigation (nav)      B0 gimbal → robogait_gimbal
  mapping robot_slam_mapping          B1 one-shot / B2 person layer (MPPI)
  A2 robot_simple_commander           B3 straight follow (no Nav2, gimbal frozen)
B1/B2 chassis [v,ω] comes from MPPI. B3 sends forward/back only, still through safety_gate. Do not run two lines at once.
```

Every new terminal:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Boolean launch args must be `True` / `False`.

### Folder naming (workspace-wide)

| Folder | Holds | Not for |
|--------|-------|---------|
| **`launch/`** | `ros2 launch` **entries** | Node source, yaml params |
| **`launch/parts/`** | Fragments composed by entries | Daily entry points; Mode B can skip |
| **`config/`** | Param `*.yaml` | Path geometry, map images |
| **`maps/`** | Map yaml/pgm | Trajectory paths |
| **`paths/`** | Built-in test geometry | Manual recordings |
| **`recorded/`** | Teleop recordings | Built-in tests |
| **`scripts/`** | Offline scripts | `ros2 run` nodes |
| **`test/`** | Unit tests | On-robot launches |
| **`<pkg>/nodes/`** | **Only** `ros2 run` nodes | Libraries, launches |
| **`<pkg>/lib/`** | Shared libraries | Node entry points |
| **`<pkg>/safety/`** | Safety-gate logic | Other features |

> Rule: **folder name = file type**. Nodes → `nodes/`; libs → `lib/`; launches → `launch/`; params → `config/`.


---

## 2. Package `robot_mppi_navigation` (nav stack)

Path: `src/robot_mppi_navigation/`.

### 2.1 What this package does

| Does | Does not |
|------|----------|
| Hardware, localization, Nav2, **MPPI**, smoother, safety | **Mapping entry** (see `robot_slam_mapping`) |
| Preflight, diagnostic bags, lidar reset; gamepad via this package’s `mini_teleop` | **Mode 2** record/follow (other package; own Nav2; hardware uses this package’s `rover_mini_hardware` for lidar RESET) |

Daily one-shot:

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

---

### 2.2 `launch/` — start switches

> Beginner tip: a launch file = “which programs to start together.” Memorize only the top one.

| File | What it does | When you use / edit it |
|------|--------------|------------------------|
| **`rviz_goal_navigation.launch.py`** | **Top one-shot**: hardware first, then Nav2/RViz after `/scan` + wheel odom; 2D Pose cancels a running goal | **Daily driving** |
| **`nav2_mppi_controller.launch.py`** | Nav “brain”: EKF, map/AMCL/SLAM, Nav2 (**MPPI**), smoother, safety; loads this package’s BT | Usually **do not run alone**; daily mapping uses light `robot_slam_mapping` |
| **`rover_mini_hardware.launch.py`** | Chassis, lidar, IMU, URDF; reset lidar, then wait for `can0`. IMU from `accessories.launch.py` (`bno055.active: true`, 20 Hz) | Sensors/chassis; `reset_lidar:=False` to skip |
| **`mini_teleop.launch.py`** | That hardware + PS4 pad (RESET first) | **Use this for the gamepad.** Do not launch vendor `roverrobotics_driver mini_teleop` — no RESET, lidar dies at health 2 |
| **`ekf_amcl_localization.launch.py`** | EKF-focused debug | Rarely daily |

Common args (on `rviz_goal_navigation`; **mapping → package `robot_slam_mapping`**):

| Arg | Meaning in plain words |
|-----|------------------------|
| (default) | Map + AMCL + RViz |
| `amcl:=False` | No real localization; pipe check |
| `use_rviz:=False` | No GUI |
| `map:=/abs/path/xxx.yaml` | Use another map |

```text
Daily nav: robot_mppi_navigation / rviz_goal_navigation
Mapping:   robot_slam_mapping / slam_mapping
```

**Pitfall:** do not also start `nav2_mppi_controller` while the top launch is running (two Nav2 stacks).

---

### 2.3 `config/` — knobs (prefer editing here)

> Beginner tip: change “how fast / how it avoids / localization” in yaml—**usually not Python**. Restart launch after edits (with `--symlink-install`).

| File | What it does | Edit it when you want to… |
|------|--------------|---------------------------|
| **`nav2_pure_mppi_params.yaml`** | **Most important.** Nav2 + **MPPI** + AMCL (slam block only as fallback for bare `slam:=True`). Indoor Mini: pure MPPI, `vx_max` **1.00 (ceiling, not cruise)**, no reverse; A* planner, do not plan through unknown; local inflation 0.28; ignore lidar inside 0.40 m (**do not revert to 0**) | Speed, clearance, daily localization (**change `vx_max` together with smoother `max_velocity`/`max_accel`**); **mapping tune → `robot_slam_mapping` mapper yaml** |
| **`robot_localization_ekf.yaml`** | Wheel odom + IMU yaw rate → `/odometry/filtered`. IMU stays on; driver drops garbage yaw rates | Pose spins on the map, crooked lines, or **red laser vanishes after a second 2D Pose** |
| **`sensor_topics_frames.yaml`** | Human reference: topic and frame names | Looking up names; usually not required to run |
| **`diagnostic_recording_topics.yaml`** | Which topics go into test bags | More/fewer topics for analysis |

---

### 2.4 `maps/`

| File | What it does | Beginner note |
|------|--------------|---------------|
| **`name.yaml`** + **`name.pgm`** | **The only pair nav needs.** yaml = resolution/origin; pgm = occupancy image | Same folder, same name; `map:=` points at the yaml |
| `lab2_new` / `mini_lab1`–`3` / `mini_slab1` | Lab maps (one yaml+pgm pair each) | Daily default is `mini_slab1.yaml` |
| ~~`.posegraph` / `.data`~~ | RViz **Serialize Map** for slam_toolbox continue-mapping | **AMCL does not read these**; deleted from `maps/` |

---

### 2.5 `rviz/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`rviz_goal_navigation.rviz`** | Default displays (map, laser, path, Nav2 panel, Goal tool). Left **Speed** slider sets this run’s forward ceiling (0.20–1.00 m/s) when released | Change default topics / Fixed Frame. Laser is flat red, **Decay=0**; TF axes off by default. To see the chassis: Add **RobotModel**, topic `/robot_description` (Mini URDF now includes the camera). Slider code: `robot_simple_commander` `speed_panel.cpp` |

`behavior_trees/navigate_to_pose_real_robot.xml`: after a Goal, plan then FollowPath. On plan failure **wait and retry** — no Spin/BackUp, and **do not clear the entire global costmap** (that hung Humble’s planner). Edit this file to change recoveries.

---

### 2.6 `scripts/` — commands you type

| File | What it does | How to use |
|------|--------------|------------|
| **`preflight_check_topics.sh`** | Check lidar/IMU/wheels/EKF topics | After stack up, before goals |
| **`preflight_check_tf.sh`** | Check `map→odom→base_link` | Pose frozen / jumping |
| **`record_navigation_test.sh`** | Record rosbag per topic list | `./scripts/record_navigation_test.sh name`, Ctrl+C |
| **`analyze_navigation_bag.py`** | Metrics + plots from a bag | `python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/<dir>` |
| **`reset_rplidar.py`** | Serial RESET for lidar | Or `ros2 run robot_mppi_navigation reset_rplidar` |
| **`teleop.sh`** | Keyboard teleop (remap before safety gate) | Or `ros2 run robot_mppi_navigation teleop` |

---

### 2.7 Python package `robot_mppi_navigation/` (by folder)

> `__init__.py` files only mark Python packages—**ignore them**.

#### `nodes/` — thin CLI entry points

> Like remote-control buttons; real logic lives elsewhere.

| File | Command | Real code |
|------|---------|-----------|
| `run_cmd_vel_safety_gate.py` | `ros2 run robot_mppi_navigation safety_gate` | → `safety/` |
| `run_idle_cmd_vel_check.py` | `ros2 run robot_mppi_navigation idle_check` | → `preflight/` |
| `run_reset_rplidar.py` | `ros2 run robot_mppi_navigation reset_rplidar` | → `tools/` |
| `run_wait_for_can.py` | `ros2 run robot_mppi_navigation wait_for_can` | → `tools/` (launch runs it) |
| `run_wait_for_nav_sensors.py` | `ros2 run robot_mppi_navigation wait_for_nav_sensors` | → `tools/` (gate Nav2) |
| `run_scan_viz.py` | `ros2 run robot_mppi_navigation scan_viz` | → `tools/` (`/scan_viz` for RViz) |
| `run_cancel_nav_on_pose.py` | `ros2 run robot_mppi_navigation cancel_nav_on_pose` | → `tools/` (2D Pose cancels old nav goal) |
| `run_teleop.py` | `ros2 run robot_mppi_navigation teleop` | Remapped keyboard teleop |

#### `safety/` — last gate before wheels

> Rule: **pass through unchanged** or **all zeros**. Never invent a new velocity.

| File | What it does | Edit when… |
|------|--------------|------------|
| **`cmd_vel_safety_gate.py`** | Gate **node**: listens to smoothed cmd + odom + estop; publishes `/cmd_vel`; zeros if upstream dies | Topic names, check rate (also params in launch/yaml) |
| **`stop_conditions.py`** | When to stop: cmd timeout, odom timeout, software estop | Timeouts / new stop conditions |
| **`pass_or_zero_policy.py`** | Maps conditions → PASS or ZERO | How conditions combine |
| **`twist_copy.py`** | Helpers: copy Twist, make zero, is-zero | Rarely |

Software estop:

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

#### `preflight/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`topic_health_check.py`** | Listen briefly for key topics | Topic list / timeouts |
| **`tf_tree_check.py`** | TF chain intact? | Frames to check |
| **`idle_cmd_vel_check.py`** | With no goal, `/cmd_vel` must stay 0 | Duration / tolerance |

Prefer `scripts/preflight_check_*.sh` day to day.

#### `bag_analysis/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`load_rosbag.py`** | Load bag into series | Topics / message types |
| **`path_tracking_metrics.py`** | Cross-track error | Algorithm / thresholds |
| **`heading_error_metrics.py`** | Heading error | Same |
| **`velocity_smoothness_metrics.py`** | Accel / jerk | Same |
| **`controller_timing_metrics.py`** | Rate / dropped frames | Target rate |
| **`generate_plots.py`** | Plots | Which plots |

Entry point: `scripts/analyze_navigation_bag.py`.

#### `cmd_vel_pipeline/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`topic_names.py`** | Constants for `/cmd_vel`, `/cmd_vel_nav`, … | Renaming topics (keep launch remaps in sync) |
| **`remap_chain.py`** | Documents MPPI → smoother → gate → driver | Design notes; real remaps are in launch |

#### `mppi_controller/` — tuning notes (not the live controller)

> Live MPPI code: `src/navigation2/nav2_mppi_controller/`. Live params: `nav2_pure_mppi_params.yaml`.

| File | What it does | Edit when… |
|------|--------------|------------|
| **`baseline_params.py`** | Python mirror of baseline numbers | Keep in sync with yaml |
| **`critics_reference.py`** | Text notes for 8 critics (what / failure modes) | Learning; **weights still in yaml** |

#### `localization/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`tf_frame_names.py`** | `map` / `odom` / `base_link` constants | Renaming frames |
| **`readiness_checks.py`** | TF ready checks | Readiness rules |

#### `launch_config/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`package_share_paths.py`** | Find installed config/launch/rviz paths | Rarely |
| **`common_launch_args.py`** | Shared launch arg conventions | Adding shared args |

#### `tools/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`reset_rplidar.py`** | Lidar serial RESET implementation | Device path / reset command |
| **`wait_for_can.py`** | Wait for `can0` frames before the chassis driver | Timeout / warning text |
| **`wait_for_nav_sensors.py`** | Wait for `/scan` + `/odometry/wheels` before Nav2/RViz | Topics / timeout |
| **`scan_viz.py`** | Republish `/scan` as `/scan_viz` with `now()` stamps so RViz keeps the red dots | Rarely edit |
| **`cancel_nav_on_pose.py`** | On `/initialpose` (2D Pose Estimate), cancel any running nav/plan goal | Rarely edit |

#### `bin/` — what `ros2 run` / launch actually executes

Shebang wrappers (`reset_rplidar`, `wait_for_nav_sensors`, …). Rarely edit; logic stays in `nodes/` + `tools/`.

---

### 2.8 Package install metadata

| File | What it does | Beginner |
|------|--------------|----------|
| **`package.xml`** | Package name + ROS deps | Edit when adding deps |
| **`setup.py`** | Install + `ros2 run` entry points | Edit when adding executables |
| **`setup.cfg`** | Install script paths | Usually leave alone |
| **`README.md`** | English package README | Keep updated |
| **`test/test_safety.py`** | Safety unit tests | Run after safety changes |

---

### 2.9 Intent → file (this package)

| I want to… | Go to |
|------------|-------|
| Faster/slower, turning, clearance | `config/nav2_pure_mppi_params.yaml` |
| Change lab map | `maps/` + `map:=` |
| Odom/IMU fusion | `config/robot_localization_ekf.yaml` |
| Estop / timeout stop | `robot_mppi_navigation/safety/` |
| Which nodes start | `launch/` |
| Default RViz layout | `rviz/rviz_goal_navigation.rviz` |
| After Goal: plan / recover | `behavior_trees/navigate_to_pose_real_robot.xml` |
| Change MPPI algorithm code | `src/navigation2/nav2_mppi_controller/` (advanced) |

### 2.10 Commands (this package)

```bash
colcon build --packages-select robot_mppi_navigation --symlink-install
source install/setup.bash

ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py

cd ~/RoboGait_ws/src/robot_mppi_navigation
./scripts/preflight_check_topics.sh
./scripts/preflight_check_tf.sh
ros2 run robot_mppi_navigation idle_check 10
ros2 run robot_mppi_navigation reset_rplidar
ros2 run robot_mppi_navigation teleop
```

---

## 2b. Package `robot_slam_mapping` (mapping)

Path: `src/robot_slam_mapping/`. **Mapping only**, separate from daily nav.

| Does | Does not |
|------|----------|
| **Light** online SLAM: hardware + EKF + slam_toolbox + pad + safety gate + **mapping RViz** | **No** full Nav2 / MPPI |
| PS4 pad while mapping (`mini_teleop`) | Replace daily AMCL nav (`robot_mppi_navigation`) |

Map quality comes from lidar + odometry + this package's mapper YAML; then navigate with the full stack.

| File | What it does | How to use |
|------|--------------|------------|
| **`launch/slam_mapping.launch.py`** | Light mapping one-shot (no Nav2; opens mapping RViz by default). Also starts `flatten_scan`, which draws the scan on the map plane | Build a new map. Park at the experiment start before launch (that pose is the map origin). `use_rviz:=False` skips the window |
| **`robot_slam_mapping/nodes/flatten_scan.py`** | Copies `/scan` to `/scan_flat` in a frame at `base_link` height. Mapping still uses the real `/scan`, so wall positions stay the same | Included by the mapping launch. See the FAQ if the scan is still in the air |
| **`rviz/mapping.rviz`** | Mapping view. **保存** bar on the left. Laser shows the ground-level `/scan_flat` (still 3 px / 0.01 m squares). Other displays follow the indoor file: grid, translucent map, smaller axes without names, short odometry arrow with covariance off. Original orbit camera. Help pane height 0. Name and value columns split the width in half | Restart mapping after a change. Do not save a window whose title has `*` |
| **`launch/mini_teleop.launch.py`** | Pad → `/cmd_vel_smoothed` | Included by mapping |
| **`config/mapper_params_online_async.yaml`** | **Mapping params (this package)**. Map cells are **0.05 m**, same as the old workspace mapper yaml | **Edit here to tune SLAM** |
| **`maps/`** | Save new maps here | **yaml+pgm only.** Use `map_saver_cli`, or the mapping RViz **map path** panel. Do not click Serialize Map |
| **`maps/mini_lab1.yaml`** + **`.pgm`** | Lab map 2026-08-22 | `map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab1.yaml` |
| **`maps/mini_lab2.yaml`** + **`.pgm`** | Lab map 2026-08-23 | Same, change the filename |
| **`maps/mini_lab3.yaml`** + **`.pgm`** | Lab map 2026-08-25 | Same |
| **`maps/mini_slab1.yaml`** + **`.pgm`** | Lab map 2026-08-26 (daily default also in the nav package `maps/`) | Same |

```bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
# Save from the RViz bar, or:
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
```

---

## 3. Package `robot_simple_commander` (record / follow)

Path: `src/robot_simple_commander/`.

### 3.1 What this package does (read this)

| Does | Does not |
|------|----------|
| **Own** hardware + localization + (on follow) Nav2/**MPPI** + safety | Depend on / import `robot_mppi_navigation` |
| **Record** while you teleop | Invent velocities — follow uses **this** package’s MPPI |
| **Follow** recorded / bundled paths | Run alongside Mode 1’s full stack |

| Task | Launch |
|------|--------|
| Record | `record_trajectory.launch.py` (PS4 pad by default); Ctrl+C saves |
| Follow | `follow_trajectory.launch.py` (includes MPPI). After it finishes, **回到起点** drives back along the recorded route to the start |

---

### 3.2 Launches vs nodes

```text
launch/                         ← ros2 launch entries
recorded/                       ← teleop-recorded yaml
paths/                          ← built-in test geometry
rviz/                           ← record/follow view (开始 / 回到起点 / 停止)
src/follow_panel.cpp            ← RViz 开始 / 回到起点 / 停止 (C++)
bin/                            ← ros2 run entries
nodes/ lib/ safety/             ← Python
```

#### `launch/` — entry points

| File | What it does | How to use |
|------|--------------|------------|
| **`record_trajectory.launch.py`** | Record: hardware + EKF + **AMCL** + node `record_path` + RViz + **PS4 pad** | Wait for the red laser, **2D Pose Estimate** on the real spot, then drive; Ctrl+C saves. `use_rviz:=False` skips GUI; `use_gamepad:=False` skips pad. **Do not also** launch `mini_teleop` |
| **`follow_trajectory.launch.py`** | Follow: hardware → wait for `/scan` + wheel odom → then Nav2/MPPI + RViz + `follow_path` | **No extra args = name in `route_name.yaml`**. The robot **does not start by itself**. **No RViz during lidar reset** (~15–20 s; **do not Ctrl+C**) → set **2D Pose Estimate** → **开始**. After the run, **回到起点** drives forward along the recording back to the first pose (no reverse gear). **停止** stops the robot and leaves the launch up. Ctrl+C exits. **Stop daily nav first** |

#### `launch/parts/` — Fragments

| File | What it does | How to use |
|------|--------------|------------|
| **`hardware.launch.py`** | Chassis / lidar / IMU (via daily `rover_mini_hardware`, RESET first) | Included by entry launches only |
| **`nav_stack.launch.py`** | EKF + localization + Nav2/MPPI + smoother + this package’s safety gate | Included by follow entry only |
| **`rviz.launch.py`** | Opens this package’s RViz: map, laser, trail; `rviz_config` picks record vs follow layout | Included by record/follow; `use_rviz:=False` if no display |

#### `config/` — this package’s own MPPI (not the daily-nav file)

| File | What it does | Edit when… |
|------|--------------|------------|
| **`nav2_mppi_params.yaml`** | Nav2 + **MPPI** for follow. Indoor Mini aligned with daily nav: pure MPPI, A*, local inflation 0.28, ignore lidar inside 0.40 m | Follow speed / clearance; **do not revert to 1.2 / min_range 0** |
| **`route_name.yaml`** | **Trajectory name** (one line: `name:`). Record writes `recorded/<name>.yaml`; follow reads the same name | Change this to pick which recording to save/follow |
| **`robot_localization_ekf.yaml`** | Wheel + IMU yaw rate for record/follow (same as daily nav) | Drift on the map, laser not lining up |
| **`rviz/record_follow.rviz`** | **Record** view: map, cyan laser, robot, one green trail. Top-down, follows the robot | No start button (Ctrl+C saves) |
| **`rviz/follow.rviz`** | **Follow** view: blue route, orange return route, green start, orange end. Left panel **开始 / 回到起点 / 停止** plus the speed slider | Buttons: `src/follow_panel.cpp`. Slider: `src/speed_panel.cpp` |

#### Nodes (`ros2 run`, not launch)

| Command | Source | Role |
|---------|--------|------|
| `ros2 run robot_simple_commander record_path` | `nodes/record_path.py` | Recorder node only |
| `ros2 run robot_simple_commander follow_path` | `nodes/follow_path.py` | Follow client only |
| `ros2 run robot_simple_commander cancel_navigation` | `nodes/cancel_task.py` | Cancel goals |
| `ros2 run robot_simple_commander safety_gate` | `safety/` | Safety gate (usually started by nav_stack) |
---

### 3.3 Python sources

> **Convention:** runnable nodes live under **`nodes/`**; libs under **`lib/`**; safety logic under **`safety/`**; runnable nodes under **`nodes/`**.

| File | Plain-language role | Edit when… |
|------|---------------------|------------|
| **`nodes/record_path.py`** | Wait for **2D Pose Estimate**, then sample by distance; green trail in RViz; Ctrl+C writes yaml | Sampling distance, wait-for-pose, trail topics |
| **`nodes/follow_path.py`** | Load yaml → wait for Nav2 → **2D Pose only localizes** → **开始** sends FollowPath and stays up. **回到起点** drives from the nearest recorded pose back to pose 0, ending on the original start yaw. **开始** is refused when farther than 0.8 m from the start | Services `/follow_start` `/follow_return` `/follow_stop` |
| **`lib/return_path.py`** | Pose list for the return trip (not a node). Headings point forward because the Mini does not reverse | Home radius 0.40 m; start gate 0.80 m |
| **`nodes/cancel_task.py`** | Cancel NavigateToPose / FollowPath (robot stops, stack stays up) | Which action servers |
| **`lib/path_io.py`** | Read/write path yaml (shared format) | File format fields (affects record + follow) |
| **`lib/route_name.py`** | Read `config/route_name.yaml`; write recordings under source `recorded/<name>.yaml` (not only install, so colcon does not drop them) | Rarely |
| **`lib/generators.py`** | Math for straight / ±90° / S-curve test shapes (no speeds) | Test path length / shape |
| **`src/follow_panel.cpp`** | RViz **开始 / 回到起点 / 停止** | Button labels, service names |
| **`src/speed_panel.cpp`** | RViz speed slider: on release, writes the forward ceiling into MPPI and the smoother (0.20–1.00 m/s). Mapping has no controller; use the gamepad | Slider range, labels |
| **`src/map_save_panel.cpp`** | Mapping RViz save bar: path field and **保存**. Default `maps/today`. Writes `yaml` + `pgm` | Default path, button label |
| **`safety/`** | Own safety gate (logic copied; no import from robot_mppi_navigation) | `/cmd_vel` gate on follow |

---

### 3.4 `recorded/` and `paths/`

| Folder | Contents | How to use |
|--------|----------|------------|
| **`recorded/`** | **Your** teleop recordings; filename is `name` from `config/route_name.yaml` | Change the name in that yaml, not on the command line |
| **`paths/`** | **Built-in** test geometry | `path_name:=left_90` etc. |

> Poses only—**no speeds**. MPPI always computes velocity online.

| `paths/` file | What it is | How to use |
|------|------------|------------|
| **`straight.yaml`** | ~3 m straight | `path_name:=straight` |
| **`left_90.yaml`** | Forward + left 90° + forward | `path_name:=left_90` |
| **`right_90.yaml`** | Same, right | `path_name:=right_90` |
| **`s_curve.yaml`** | S-curve | `path_name:=s_curve` |
| **`obstacle_avoid.yaml`** | Straight; place a box for obstacle tests | `path_name:=obstacle_avoid` |

Format:

```yaml
frame: map          # or local
poses:
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - {x: 0.05, y: 0.0, yaw: 0.0}
```

---

### 3.5 scripts / install / tests

| File | What it does | Edit when… |
|------|--------------|------------|
| **`scripts/generate_test_paths.py`** | Regenerate `paths/*.yaml` from generators | After editing generators |
| **`package.xml`** | Name + deps | Adding deps |
| **`CMakeLists.txt`** | Builds Python nodes + RViz Follow panel; `bin/` is `ros2 run` | New executables or panel edits |
| **`README.md`** | English package README | Keep updated |
| **`test/test_path_io.py`** | Unit tests for generators / yaml | After path_io changes |

---

### 3.6 Intent → file (this package)

| I want to… | Go to |
|------------|-------|
| Denser / sparser recording | `min_separation` (default **0.05 m**): 20 Hz TF check, keep a pose only after 5 cm of travel; not a timed dump |
| Follow / wait-for-Nav2 / 2D Pose / start / return home | `nodes/follow_path.py` and `lib/return_path.py` |
| Follow button UI | This package `src/follow_panel.cpp` |
| Change built-in test shapes | `generators.py` then `generate_test_paths.py` |
| Pick which trajectory to record/follow | Edit `name:` in `config/route_name.yaml` |
| Hand-edit waypoints | Edit `paths/*.yaml` or `recorded/<name>.yaml` |
| Cancel behavior | `nodes/cancel_task.py` |
| Follow speed / wobble (control) | This package’s `config/nav2_mppi_params.yaml` (**not** the `robot_mppi_navigation` file; indoor Mini already aligned: `vx_max` 1.00 / lidar 0.40 m) |
| RViz during record/follow | Record: `rviz/record_follow.rviz`. Follow: `rviz/follow.rviz` |

### 3.7 Commands (this package — no robot_mppi_navigation)

> **Ctrl+C the daily RViz nav first.** Do not run two Nav2 stacks. The name lives in one file: `src/robot_simple_commander/config/route_name.yaml` (`name: loop1`). Stop record before follow.

```bash
# 1) Set the name in config/route_name.yaml (no .yaml)
# 2) Record
ros2 launch robot_simple_commander record_trajectory.launch.py
# Wait for red laser → 2D Pose Estimate → drive → Ctrl+C here to save

# 3) Follow (new terminal; same name)
ros2 launch robot_simple_commander follow_trajectory.launch.py
# Wait ~15–20 s (lidar RESET + CAN; RViz opens after that; do not Ctrl+C)
# → 2D Pose Estimate at the route start (robot still still) → 开始
# After the run, 回到起点 drives back along the route to the green start
# 停止 only cancels motion. Ctrl+C in this terminal exits

# Built-in exam path (does not change the file)
ros2 launch robot_simple_commander follow_trajectory.launch.py name:=straight
```

---

## 4. Mode B packages (Python, person follow)

> Logic inspired by `~/rover_workspace_final` (Nuitrack skeleton + gimbal); downstream rewritten in Python.  
> External deps: `orbbec_person_publisher` (Nuitrack overlay from the old workspace), `dynamixel_sdk`.  
> **Camera is a Femto Mega.** Gimbal/gait skeleton must use **Nuitrack** (the method that actually locked a person on this robot). SDK (K4A + Azure k4abt) is the teacher **point-cloud** demo; this NUC has Intel Iris Xe and **no CUDA**, so CPU body tracking almost never locks a pelvis.  
> **Do not start Nuitrack and SDK together** (same USB camera).  
> Lock point: Nuitrack **torso** (User.msg: x right, y up, z forward). Straight follow can freeze the gimbal at idle; turn pan on for corridor corners.

### 4.1 `robogait_person_interfaces`

| File | Role |
|------|------|
| **`msg/User.msg`** | `detection` + `x/y/z` (camera frame, z = range) |

### 4.2 `robogait_perception`

Two person front-ends; **only one at a time** (they fight over the same Femto Mega). Downstream topics are the same (gimbal / follow do not care which):

| `person_source` | What it is |
|-----------------|------------|
| **`nuitrack` (gimbal/gait default)** | Third-party Nuitrack (`orbbec_person_publisher` / `nuitrack_person_node`). Publishes torso 3D + pixel — the method that used to lock a skeleton. |
| **`sdk` (point-cloud demo)** | Orbbec K4A wrapper: **point cloud** (person-only by default), color, depth, camera pose marker. Azure body tracking **off** (`enable_body: false`). |

| Node / file | Role | Command |
|-------------|------|---------|
| **`src/person_sdk.cpp`** | Femto Mega SDK: cloud (**person-only by default**) + optional k4abt (unreliable on this CPU, off by default) + **camera pose marker** → `/orbbec/*` | `ros2 run robogait_perception person_sdk` |
| **`nodes/distance_orbbec.py`** | `/orbbec/*` → `user_raw`, `person_center_pixel` (Nuitrack torso or SDK point) | `ros2 run robogait_perception distance_orbbec` |
| **`launch/person_source.launch.py`** | Pick nuitrack / sdk; default **Mini URDF** + optional RViz. Nuitrack also publishes static TF **`orbbec_link` → `rgbd_camera_link_optical`** (Nuitrack cloud frame); without it RViz drops the cloud. Missing Nuitrack prints overlay instructions | `use_robot_model:=False` if Nav2 already publishes the URDF. Do **not** run Nuitrack and SDK together |
| **`rviz/orbbec_sdk.rviz`** | SDK demo: **full RobotModel** + cloud + color + orange look-arrow | `person_source:=sdk` with `use_rviz:=True`; Fixed Frame = `base_link` |
| **`rviz/orbbec_nuitrack.rviz`** | Nuitrack skeleton: same Fixed Frame `base_link`, cloud in `rgbd_camera_link_optical`; **no Color/Depth** (Nuitrack does not publish `/orbbec/color/image_raw`, so no No Image panels) | Used automatically for `person_source:=nuitrack`. Empty RViz while logs already lock a person: FAQ **7.35** |
| **`config/perception.yaml`** | Topics, cloud stride, **person-only cloud** (`cloud_person_only`), **SDK body `enable_body` (default false)** | **Do not turn on SDK body for tracking. Keep `cloud_person_only: true` to show only the person. Set `false` and launch `person_source:=sdk` for the full room. Denser cloud: lower `cloud_stride`** |
| **`urdf/orbbec_gimbal.urdf`** | Dual-axis gimbal on Mini (`gimbal_pan_joint` + `gimbal_tilt_joint`) + cyan camera | **Idle look-down: tilt origin pitch must be positive (~`0.12`). Person flies up in RViz: flip `tilt_urdf_sign` in `gimbal.yaml` (FAQ 7.30). Use `test_gimbal`, not `person_source` alone** |
| **`urdf/mini_with_orbbec.xacro`** | Mini chassis + camera for RViz **RobotModel** | Launch xacros this; usually leave it |

```bash
# Point cloud for the teacher (no Nuitrack, no k4abt)
ros2 launch robogait_perception person_source.launch.py person_source:=sdk
# Skeleton (overlay the old workspace first; see gimbal commands below)
ros2 launch robogait_perception person_source.launch.py person_source:=nuitrack
```

### 4.3 `robogait_gimbal`

| Node / file | Role | Command |
|-------------|------|---------|
| **`nodes/camera_tracker.py`** | Pixel / elevation PID → `camera_position` (pan deg) + `camera_tilt` (tilt deg, clamped). Nuitrack torso (y ≠ 0) tilts; SDK depth blob (y = 0) pans only. Heavy tilt filter + large dead zone so walking does not nod | `ros2 run robogait_gimbal camera_tracker` |
| **`nodes/dynamixel_driver.py`** | Two axes: ID 1 pan, ID 2 tilt; publishes angles for the RViz model (`tilt_urdf_sign`). **`freeze_gimbal` holds idle**. Keeps sending pan steps while the person is off-center | `ros2 run robogait_gimbal dynamixel_driver` |
| **`config/gimbal.yaml`** | U2D2 path (now serial **FT8ISNFV**), IDs, limits, **idle `idle_*_ticks`**, PID, **`freeze_gimbal`**, **`enable_gimbal_pan` / `enable_gimbal_tilt`**, **`tilt_dead_zone_deg`**, **RViz tilt sign `tilt_urdf_sign`**, **too-close ignore `min_track_z_m` (default 1.0 m)** | **New U2D2: edit `device_name`. B3 straight-follow launch freezes the gimbal; do not edit this for that test. Corners: `freeze_gimbal: false` and `enable_gimbal_pan: true`. Person bounces in the image: raise `tilt_dead_zone_deg` / lower `tilt_kp`. Idle look: edit idle. Person flies up in RViz: `tilt_urdf_sign: -1.0`. Pan slams when you walk in close: stand 1.5–3 m, or edit `min_track_z_m`** |
| **`test_gimbal.launch.py`** | B0: perception + **gimbal follows you** (no chassis Nav2). **Default `person_source:=nuitrack`** | **Use this so the camera tracks you**; not `person_source` alone. Overlay the old workspace first |

### 4.4 `robogait_person_follow`

| Node / file | Role | Command |
|-------------|------|---------|
| **`nodes/distance_filter.py`** | `user_raw` → `user_coordinates` | `ros2 run robogait_person_follow distance_filter` |
| **`nodes/person_nav_goal.py`** | Person → standoff `NavigateToPose` (MPPI owns full `[v,ω]`) | `ros2 run robogait_person_follow person_nav_goal` |
| **`nodes/straight_follow.py`** | **B3**: person distance `z` → forward/back only, angular always 0, publishes `cmd_vel_smoothed` | `ros2 run robogait_person_follow straight_follow` |
| **`person_follow.launch.py`** | B1 one-shot: A1 Nav2/MPPI + person layer | See below |
| **`person_layer.launch.py`** | B2 terminal 2 on running Nav2 | See below |
| **`straight_follow.launch.py`** | **B3 alone**: hardware + Nuitrack + frozen gimbal + distance control + safety gate. No Nav2 | See below. Do not run with B1/A1 |
| **`config/person_follow.yaml`** | B1/B2 standoff, dead zone, goal update rate | **Edit for turning follow distance** |
| **`config/straight_follow.yaml`** | B3 distance band, forward/back caps, accel | **Edit for straight gap and speed** |

**Control chain (person follow = pure MPPI; gimbal only aims the camera):**

```text
person → camera_tracker → dynamixel_driver → ID1 pan + ID2 tilt (camera FOV)
person → distance_filter → person_nav_goal → NavigateToPose
                                              ↓
                                     planner → MPPI → smoother → safety_gate → /cmd_vel
```

B1/B2 person does **not** rewrite `/cmd_vel`. Chassis speed **ceiling** is MPPI `vx_max` **1.00 m/s** (daily yaml; keep smoother `max_velocity` / `max_accel` in sync). That is **not a constant cruise**: open space can approach 1.0, clutter slows it. A faster walker will still pull away. **Do not jump to 1.2** (indoor rollouts all hit inflated cells).

**B3 straight follow** is a separate chain. Farther than the band → forward; closer → slow reverse; inside the band → stop; angular stays 0. Caps live in `straight_follow.yaml` (forward **0.70**, reverse **0.30**). Do not raise them past 1.0. `/cmd_vel` is still only `safety_gate`.

```bash
# B0 gimbal: Nuitrack skeleton by default (overlay old workspace, then this repo)
# Stand 1.5–3 m facing the camera; expect Nuitrack "Skeleton detected" and joint=torso
# Do not start Nuitrack and SDK together (same USB camera)
# Corner gimbal: freeze_gimbal: false, enable_gimbal_pan: true in gimbal.yaml
# B3 straight follow freezes the gimbal itself; do not edit gimbal.yaml for that
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
# Teacher point cloud (no skeleton): person_source:=sdk

ros2 launch robogait_person_follow person_follow.launch.py
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py

# B3 straight follow (stop B1/nav first). Person ~3 m in front. Gimbal stays still.
ros2 launch robogait_person_follow straight_follow.launch.py
```

---

## 5. Other workspace files

| Item | What it does | Beginner use |
|------|--------------|--------------|
| **`PROJECT_CHANGES.md`** | Chinese handbook (with `TEST_PLAN.md`: only Chinese docs) | “What file / where to edit” |
| **`PROJECT_CHANGES_EN.md`** | This English handbook | Same |
| **`TEST_PLAN.md`** | Today's on-robot steps (**Chinese**): mapping → navigation → record path → follow path. Person and gimbal are not in this file | Follow it on the robot; one stack at a time, same `today` map |
| **`MEETING_BRIEF.md`** | Chinese meeting brief for advisor | Present / screen-share today |
| **`NODES_AND_TOPICS.md`** | Node↔topic diagrams (**English**) | Who publishes / subscribes what |
| **`README.md`** | Repo landing (**English**) | First link for newcomers |
| **`src/*/README.md`** | Package READMEs (**English**) | Complements §§2–4 |
| **`src/navigation2/`** | Full Nav2 sources (incl. MPPI) | **Advanced** algorithm / critic edits. `nav2_util` retries latest TF if a stamp is already expired |
| **`src/bno055/`** | BNO055 IMU driver (UART `/dev/bno055`) | IMU bring-up: flush leftover UART bytes; retry configure in-process so launch does not double-open the port |
| **`.vscode/settings.json`** | Markdown preview default | Optional |
| **`.gitignore`** | Ignore build/install/log | Leave alone |
| **`scripts/install_can0_autoup.sh`** | Install can0 auto-UP on boot/replug (needs sudo) | Run once if CAN keeps dropping |
| **`scripts/install_vscode_server.sh`** | Install/repair VS Code Remote SSH server (needs internet) | Run on NUC if Remote shows `localDownloadFailed` |

---

## 6. TODO

| Item | Status |
|------|--------|
| ~~RPLidar health 2~~ | Fixed: `ros2 run robot_mppi_navigation reset_rplidar` |
| ~~BNO055 comms / UART port fight~~ | Fixed: flush UART on open; retry configure in-process instead of `sys.exit` |
| Mapping on-robot | Opens `rviz/mapping.rviz`; save-on-robot still pending |
| `TEST_PLAN.md` checkboxes | Today only: mapping, navigation, record path, follow path (person/gimbal are not in this plan) |
| Mode A2 record + follow + return to start | Code is in; NUC run still pending. Return follows the recording forward, it does not reverse |
| Mode B: `test_gimbal` on-robot | Default is now Nuitrack skeleton; confirm torso + pan on-robot |
| Mode B: `person_follow` / `person_layer` on-robot (pure MPPI) | Code updated; test pending |
| Mode B3: `straight_follow` on-robot | Separate line added; test the gap and the 0.70 m/s cap |
| posegraph localization in launch | Not done (AMCL now) |
| Custom MPPI critic | Not started (`navigation2`) |
| Gazebo deps | Install when needed |

---

## 7. FAQ — common problems

> On-robot pitfalls. Check here first; when a new issue is fixed, **add it here** (keep CN/EN in sync).

### 7.1 Chassis FATAL: `Did not receive any data… stale`

**Symptom:** `roverrobotics_driver` FATAL; connected to `can0` but no robot data; process restarts.

**Cause:** `can0` is DOWN, or robot off / e-stop / cabling. ROS does **not** configure CAN.

**Fix:**

```bash
ip link show can0
sudo systemctl start can.service
candump -n 10 -T 3000 can0
ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py
```

**One-time install (recommended):**

```bash
sudo bash ~/RoboGait_ws/scripts/install_can0_autoup.sh
```

Boot + USB-CAN replug auto-UP. `Active: active (exited)` is normal for oneshot.

---

### 7.2 Lidar dies: `RPLidar health status : 2`

**Symptom:** health 2 → internal error → rplidar exits 255; no `/scan`.

**Cause:** You do **not** have to type reset every time. Ctrl+C runs `Stop motor`; this **RPLidar S2** then often reports health 2. The stock driver **exits on purpose** (255). Right after power-on, vendor `mini_teleop` works without a reset.

**Fix:** Workspace `rplidar_ros` now **soft-resets itself** on a bad health (up to twice). Use the vendor command again:

```bash
ros2 launch roverrobotics_driver mini_teleop.launch.py
```

You should see `Soft-reset 1/2 then retry...` then `health status : OK`. If still 2: stop launch → **unplug/replug lidar USB/power** (~5 s) → retry.

Daily nav / `rover_mini_hardware` still reset first as a backup. Skip that layer with `reset_lidar:=False`.

**`SL_RESULT_OPERATION_TIMEOUT` while the port exists (2026-09-27):** `/dev/rplidar` is present and the node opens it, but a health read returns no bytes. This is not health 2; a soft reset has nothing to answer. 115200, 256000, and 1000000 all return 0 bytes, so this is not a wrong baud (a mismatch usually yields garbage). The serial in `/dev/serial/by-id` belongs to the USB chip inside the black SLAMTEC adapter box, not to the spinning lidar head. Same head, only the box was swapped: the old CP2102 box (`a64db3d4…`) enumerates, but lidar data does not pass through. Two CP2102N boxes work, `d4ffe520d382eb11b064968d9693f7bc` and `7ee3445a1d0bee11b58f4002f59e3369`; both return health `OK` at 1000000 baud. The head's own serial stays `71e6ecf8c4e699d7b8eb99f9fd394717` when the box changes. Use either new box for mapping. Do not plug the old box back in. Both boxes show up as `10c4:ea60` in `lsusb`; tell them apart with `ls -l /dev/serial/by-id`.

**Why the command:** `0xA5 0x40` is Slamtec soft-reboot; sometimes one shot is not enough — power-cycle is the reliable fix.

---

### 7.3 `Unknown topic '/scan'`

**Cause:** Hardware launch not running yet.

**Fix:** Start launch, then query the topic.

---

### 7.4 Long bno055 KeyboardInterrupt after Ctrl+C

**Cause:** Interrupted mid serial read. Harmless shutdown noise — ignore.

---

### 7.5 `ttyUSB*` numbers keep changing

**Cause:** Kernel port order changes; use udev aliases `/dev/rplidar`, `/dev/bno055` (`99-rover-mini-sensors.rules`).

**Both names missing (2026-09-26):** check USB before launching.

```bash
lsusb
ls -l /dev/serial/by-id
```

- No `10c4:ea60` (Silicon Labs): the NUC is not receiving the lidar USB data. A spinning head only means the motor has power; the data cable is separate. `/dev/rplidar` will not appear, and the node exits with `80008004`. That hub is two chips. The mouse dongle sits on the USB2 side, so the mouse works. The blue USB3 side stayed empty, so a lidar plugged there never showed `10c4:ea60`. Move the lidar USB plug to a port on the NUC itself, then check `lsusb` for `10c4:ea60` and `ls -l /dev/rplidar`. On 2026-09-26 the NUC-port cable enumerated as a CP2102, serial `a64db3d4e89098459d3372b618827745`, and `/dev/rplidar` came from `55-roverrobotics.rules`. The old serial `060d09eb…` in `99-rover-mini-sensors.rules` does not match this cable.
- FTDI `0403:6014` present but no `/dev/bno055`: the adapter plugged in now, `FT8ISNFV`, is the **gimbal U2D2**, not the IMU. The IMU is a different cable and only then appears as `/dev/bno055`. Do not retarget the udev rule at the gimbal adapter.
- **NUC powered by a power bank, hub still plugged into the NUC (2026-09-26):** the data path is unchanged; the hub is still a NUC USB expander. The robot supply was stiffer. A power bank that cannot hold the NUC plus the hub leaves the hub's downstream ports short of current. The hub itself and the mouse (USB2, low current) still enumerate; lidar/IMU on the blue ports do not. The same lidar cable shows `/dev/rplidar` when plugged into a port on the NUC. For today, put lidar, IMU, and the gamepad receiver on NUC ports. To use the hub ports again, power the NUC from the robot supply.

---

### 7.6 Multiple publishers on `/cmd_vel`

```bash
ros2 topic info /cmd_vel --verbose
```

Only `safety_gate` may publish. Stop duplicate Nav2 / any person node writing `/cmd_vel`.

---

### 7.7 Two Nav2 stacks at once

Do not run full `robot_mppi_navigation` and `robot_simple_commander` stacks together. Ctrl+C one before starting the other.

---

### 7.8 Mapping: no map / TF wants `base_footprint`

**Symptom:** `/map` never grows, or slam_toolbox cannot find `base_footprint`.

**Cause:** Mini uses **`base_link` only**. Use package `robot_slam_mapping`; params in that package’s `config/mapper_params_online_async.yaml`.

**Fix:**

```bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
```

Tune mapping by editing that YAML, then restart the launch.

Mapping still runs without `/dev/bno055`. The `bno055` node exits and respawns; the launch stays up. The EKF uses wheel forward speed only, and takes yaw rate from the IMU alone, so turn slowly and let scan matching fill in heading. Navigation, record, and follow also wait on `/scan` and wheel odometry, not on the IMU.

---

### 7.9 `Failed to load map yaml` / `bad file`

**Symptom:** `map_server` cannot read the yaml; localization does not come up; later logs may show `Invalid frame ID "odom"`.

**Cause:** `map:=` points at a missing file, or the yaml is not next to its `.pgm`. RViz **Save Map** with a bare name writes into the launch cwd (`~/RoboGait_ws/`), while launch looks in `maps/`. **Serialize Map** writes `.posegraph` + `.data` only — that is not the nav pair.

**Fix:**

```bash
ls ~/RoboGait_ws/src/robot_slam_mapping/maps/
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

Nav needs **yaml+pgm**, not the posegraph. `.posegraph` / `.data` were deleted from both packages’ `maps/`. Save with:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/name
```

---

### 7.10 RViz: `could not connect to display` / `xcb`

**Symptom:** `rviz2` dies immediately with `qt.qpa.xcb: could not connect to display`.

**Cause:** This terminal has no desktop display (Cursor / SSH). RViz must run on the NUC **local desktop**.

**Fix:** Launch from a desktop terminal on the NUC, or:

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  use_rviz:=False \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
rviz2 -d $(ros2 pkg prefix robot_mppi_navigation)/share/robot_mppi_navigation/rviz/rviz_goal_navigation.rviz
```

---

### 7.11 Startup spam: `Invalid frame ID "odom"` / `"map"`

**Symptom:** For the first ~8–10 s, costmaps log that `base_link → odom` / `map` does not exist. Then lidar reset finishes, the driver starts, the spam stops, and you see `Managed nodes are active`.

**Cause:** Launch **soft-resets the lidar twice** before chassis/EKF publish `odom`. Nav2 is already waiting on TF. The map is fine.

**Fix:** **Do not Ctrl+C.** Wait until the log shows the map read, lidar health OK, and both lifecycle managers active. In RViz use **2D Pose Estimate** to put the robot on the real spot (align laser with walls), then **2D Goal Pose**. AMCL defaults to `(0, 0, 0)` and will be wrong until you set that pose.

---

### 7.12 Set the initial pose; laser looks sparse in RViz

**Symptom:** Robot is in the wrong room on the map; **red laser does not sit on the black walls** (looks like drift); or only a few laser dots show. Terminal spam: `extrapolation into the future` (`lidar_link` → `map`, ~10–40 ms).

**Cause:** This is not a broken MPPI/map change. AMCL defaults to `(0,0,0)` on the map; a wrong heading rotates the red scan off the walls. IMU UART errors (`0x6` / `multiple access`) do **not** rotate the laser vs walls while the robot is standing still. Nav still uses wheel odom. Align with **2D Pose Estimate**.

**Set pose:**

1. Toolbar **2D Pose Estimate**.
2. Click-drag on the **real room**, arrow = nose direction.
3. After release the red laser should **snap** to the new pose and sit on walls. If it smears for 1–2 s, that is the old RViz config (Decay 1.5 s) stacking old scans — relaunch this stack.
4. Turn slowly in place so AMCL particles collapse.
5. Then **2D Goal Pose**.

**See the laser / no smear:** Large flat-red points, `Decay Time = 0` (current scan only), TF filter 10. EKF `transform_time_offset: 0.05`. **Restart the whole launch** so both RViz and EKF pick this up. Do not Ctrl+C for future-TF lines.

---

### 7.13 `planner_server` spam: `Extrapolation Error` / `into the past`

**Symptom:** Stack is active and you set 2D Pose; after a goal (or several pose clicks) `planner_server` logs every ~1 s: `Requested time … but the earliest data is at time …` (**Requested time never changes**).

**Cause:** One log line per second = the behavior tree is replanning, so **a nav goal is already running** (toolbar **Nav2 Goal** / 2D Goal, not 2D Pose). `Requested time` never changes — it is the stamp from the first goal click. A later **2D Pose Estimate** jumps `map→odom`; the planner still looks up that old time. AMCL `Failed to transform initial pose in time` is only a warning — the pose was still applied.

**Fix:** **Do not Ctrl+C.** Each **2D Pose Estimate** now cancels a running goal. You can still click **Cancel** on the Navigation2 panel.

1. **2D Pose Estimate as many times as you need** (the first click is often off). After each release, wait for the red dots to jump, then nudge again until laser sits on walls  
2. **Then click Nav2 Goal / 2D Goal Pose**  
3. You may keep posing while a goal was running (it auto-Cancels); click a new goal after you are aligned

**Moves a bit then stops:** If the log has no `Begin navigating` and RViz has no green `/plan`, **planning failed and MPPI never got a path**. That twitch was default BT Spin/BackUp, not path tracking. This package’s BT now waits (does not wipe the whole global costmap); the planner uses latest TF. Do not Ctrl+C.

If the terminal is already looping `Waiting for async process to finish` / `missed deadline to stop`, mashing Ctrl+C **does nothing** — see **7.24**.

---

### 7.14 What the red / ERROR lines mean

| Log | Action |
|------|--------|
| `Did not receive any data… stale` + driver exit -11, then respawn | Used to happen when the driver bound CAN before frames. Launch now runs `wait_for_can`. If it still FATAL: release e-stop, `sudo systemctl start can.service` |
| First seconds `Invalid frame ID "odom"` / `"map"` | Old: Nav2 started during lidar reset. Now Nav2 waits for sensors |
| `extrapolation into the future` (`lidar_link`→`map`) | RViz uses `/scan_viz` (current stamp). Occasional lines are harmless |
| `Failed to transform initial pose in time` | AMCL warning; pose was still set |
| `GLSL link result` / `particle_cloud` QoS | RViz cosmetics; ignore |
| `Message Filter dropping message: frame 'odom'` / `queue is full` | RViz cannot keep up with odom. TF axes display is off. Ignore |
| One `Timed out waiting for transform` at activate, then `start` | Planner beat AMCL TF by a fraction of a second |
| `bno055: Data fusion not ready` / `BusOverRunException` / `TransmissionException` / `Wrong READ-request response header` | IMU UART busy or leftover bytes. Driver retries **in the same process** (no crash/respawn fight); nav uses wheel odom. See 7.20 |
| `bno055` KeyboardInterrupt / driver -11 after Ctrl+C | Shutdown noise; ignore |
| `rviz2: process has finished cleanly` while other nodes keep running | You closed the RViz window. Nav is still running; you just lost the display. Do not Ctrl+C for that |

---

### 7.15 `Exec format error` on `reset_rplidar` / `wait_for_nav_sensors`

**Symptom:** Launch dies immediately: `OSError: [Errno 8] Exec format error: '.../lib/robot_mppi_navigation/reset_rplidar'` (same for `wait_for_nav_sensors`).

**Cause:** The installed entry scripts were **0-byte empty files** (this image’s setuptools `console_scripts`). The kernel cannot execute them.

**Fix:** This package now installs real shebang scripts from `bin/`. Clean and rebuild:

```bash
cd ~/RoboGait_ws
rm -rf build/robot_mppi_navigation install/robot_mppi_navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mppi_navigation --symlink-install
source install/setup.bash
```

Launch from the **NUC desktop terminal** (the one with a screen), not SSH/`cursor`. `X11 connection rejected` is a display-permission issue, unrelated to the lidar scripts.

---

### 7.16 Goal set, robot only rotates slowly, never drives forward

**Symptom:** Pose is aligned, Goal shows a green path, but the Mini only turns in place. Log: `Optimizer fail to compute path`, then local costmap clear, then `Received a goal` again.

**Cause:** MPPI scores **every sampled trajectory as a collision**. Two usual reasons together:

1. Costmap `obstacle_min_range: 0` paints the **robot body** as obstacles.  
2. `vx_max: 1.2` looks too far ahead indoors; the control loop also missed 20 Hz.

**Fix (lidar 0.40 m body-filter is still the main fix):** Daily nav `nav2_pure_mppi_params.yaml` **and** follow’s `robot_simple_commander/config/nav2_mppi_params.yaml` (follow reads **that** copy, not the daily-nav file) ignore lidar hits inside **0.40 m** (front-mounted lidar otherwise paints the Mini’s tail as an obstacle; **do not set obstacle/raytrace min_range back to 0**). `vx_max` is now a **1.00 m/s ceiling** (`wz_max` 1.20), not a constant cruise: 0.40 cannot keep up with a person; 1.2 made every rollout hit inflated cells. Keep smoother `max_velocity` matched to `vx_max`/`wz_max`. If `Optimizer fail` returns, first lower `vx_max` toward **0.70–0.80**; do not raise to 1.2. Humble MPPI **must stay 20 Hz with model_dt 0.05** — 10 Hz throws `Controller period more then model dt` and the controller never starts. Restart the whole launch.

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

After the laser sits on walls, click a **1–2 m open goal**. You should see `Begin navigating` then forward motion. `Optimizer fail to compute path` should not spam every second.

To change speed later: `vx_max` / `wz_max` in the same yaml (keep smoother `max_velocity` in sync). `vx_max` is a ceiling — open space may approach it. Do not jump to 1.2.

**After this change, rebuild once on the NUC desktop** (do not start RViz from Cursor SSH):

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_util robot_mppi_navigation --symlink-install
source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

Wait for `Hardware topics are live. Starting Nav2.` and both `Managed nodes are active`, then refine 2D Pose until the laser matches the walls, then click Goal.

---

### 7.17 `Package 'robot_mppi_navigation' not found`, search path is `rover_workspace`

**Symptom:** You are in `~/RoboGait_ws` but ROS searches `~/GUI_ROBOGait/...` and `~/rover_workspace/install/...`, not `~/RoboGait_ws/install`.

**Cause:** `cd` does not source the workspace. New terminals run `.bashrc` → `source ~/GUI_ROBOGait/server/install/setup.bash` (which also overlays old `rover_workspace`). That overlay has **no** `robot_mppi_navigation`.

**Fix:** Source **this** repo last before launch:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source ~/RoboGait_ws/install/setup.bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | head
# first line should be /home/robogait/RoboGait_ws/install/...
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

---

### 7.18 First goal works; return trip (opposite heading) does not move

**Symptom:** Same-heading goal succeeds quickly. Goal back toward the start (arrow opposite the nose) barely moves; ~20 s later `Failed to make progress`, then Wait and retry.

**Cause:** With no reverse, a goal behind the robot should rotate in place then drive. The progress check is XY-only and aborted a turn at 20 s.

**Fix:** **No** second local controller (Rotation Shim removed). Still pure MPPI, `vx_min: 0`. Progress window 45 s; stronger PathAngle so it turns to the path then drives forward. Symlinked yaml — **no colcon**, restart the launch.

---

### 7.19 Long looping path (~14 m remaining), sudden accel, collisions

**Symptom:** In a small lab, Nav2 shows **Distance remaining ~14 m** at the start of a nearby goal. The robot lunges. It clips tables/furniture.

**Cause:**
1. NavFn `allow_unknown: true` plus global `track_unknown_space: true` snakes through unknown cells (path many times longer than the straight line).
2. Smoother `max_accel` 1.0 m/s² reaches 0.80 m/s in ~0.8 s.
3. Local inflation was only 0.22 m and obstacle range 2.5 m. **Do not** set `obstacle_min_range` back to 0 (paints the Mini body as an obstacle).

**Fix:** Daily yaml (follow copy synced): A* planner, no unknown cells; accel now **0.70 m/s²** (0.45 never reached the cap in a short lab; 1.0 felt like a lunge); local inflation 0.28, mark obstacles to 4 m. Still pure MPPI. Symlinked yaml — **no colcon**. Stop, then:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

After a goal, remaining distance should be a few metres, not ~14 m.

---

### 7.20 bno055 crash-loop / RViz gone; looks like nav never started

**Symptom:** IMU errors in a loop: `Wrong READ-request response header`, `Unable to set IMU into config mode`, `Payload length mismatch`, `device disconnected or multiple access on port`, then `bno055 process has died` and it starts again. Lidar may log `RPLidar health not OK … Soft-reset`. Soon after: `rviz2: process has finished cleanly`.

**Cause:** Several things look fatal but are not:

1. **Nav2 was already up.** If the log has both `lifecycle_manager_localization: Managed nodes are active` and `lifecycle_manager_navigation: Managed nodes are active`, you can pose and send a goal. After lidar reset you should also see `RPLidar health status : OK`.
2. **BNO055 UART often has leftover bytes at boot.** A crashed node leaves a 45-byte fusion payload in the serial buffer; the next chip-id read logs `Payload length mismatch … awaited=45`, the node exits, launch respawns, and two processes fight `/dev/bno055` (`multiple access on port`). Nav still runs on wheel odometry; IMU only helps the EKF.
3. **`rviz2 … finished cleanly` means you closed the RViz window.** Other nodes keep running. Closing the window is not a stack crash — do not Ctrl+C.

**Fix (navigation):** Once you have `Managed nodes are active`, **do not Ctrl+C and do not close RViz.** Use **2D Pose Estimate** until the laser sits on the walls (you can pose more than once), then **2D Goal Pose**. `Failed to transform initial pose in time` is harmless; the pose was set.

**Fix (IMU must stay on; UART still flaky):** The IMU **stays in the launch**. It dies because `/dev/bno055` UART desyncs (`error 0x6`, empty reads), not because nav removed it. Node runs at **20 Hz** (do not set 100), logs at most once per 5 s, and does not publish huge yaw rates into the EKF. Rebuild `bno055` to pick up the skip-garbage change.

Do not close RViz and do not Ctrl+C for IMU WARNs. Wait for both `Managed nodes are active`, 2D Pose until the laser sits on the walls, then Goal.

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bno055 --symlink-install
source install/setup.bash
```

---

### 7.21 Always slow / feels like a constant crawl even in open space

**Symptom:** The robot does move after a Goal, but at one slow speed the whole way; hallways/open floor never speed up.

**Cause:** There is no separate cruise-control speed. Humble MPPI samples linear velocity in `0 … vx_max` every cycle: open space can use more of the cap; inflated obstacles slow it. Two earlier indoor-safety edits made that look like a constant crawl:

1. `vx_max: 0.80` (cap too low).
2. Smoother `max_accel: 0.45` (~1.8 s to reach 0.80; a short lab never gets there).

**Fix:** Edit daily **`src/robot_mppi_navigation/config/nav2_pure_mppi_params.yaml`** (keep follow’s `robot_simple_commander/config/nav2_mppi_params.yaml` in sync). Now `vx_max: 1.00` (ceiling), `max_accel: 0.70`, `vx_std: 0.22`. **Do not set 1.2.** Symlinked yaml — **no colcon**; restart the launch and retest an open stretch.

---

### 7.22 Record: `Need at least 2 poses, got 1`, no yaml

**Symptom:** `record_trajectory` and RViz start; log shows `Sampled 1 pose(s)`; then `rviz2: process has finished cleanly`, then Ctrl+C, then `Need at least 2 poses, got 1` and `record_path` exit 1.

**Cause:** Either the robot never moved, or save mixed up ROS `Path` with a filesystem path (`Path.__init__() takes 1 positional argument but 2 were given`). Closing RViz does not save. Pasting record and follow in the same terminal runs follow immediately after Ctrl+C.

**Fix:** Save is patched. Rebuild once. Do not close RViz. If the pad logs `Couldn't open joystick /dev/input/js0`, wait until `Opened joystick`. After the red laser appears, **2D Pose Estimate** then drive until the green trail is a real line. Ctrl+C only on the record terminal, wait for `Saved N poses`, then follow in a **new** terminal. File: `src/robot_simple_commander/recorded/<name>.yaml`.

### 7.23 Follow: robot never moves / `Follow path was rejected` / Ctrl+C during startup

**Symptom:** `follow_trajectory` opens RViz; or the log floods `Waiting for amcl_pose` / `Publishing Initial Pose` / AMCL `Setting pose (0,0,0)` until Ctrl+C. Or right after start you see `Invalid frame ID "odom"`, you hit Ctrl+C, then `controller_server` / `rviz2` die with red errors. Or `Follow path was rejected` and the robot never moves.

**Cause:**
- Old `waitUntilNav2Active()` published a fake origin pose and blocked on `/amcl_pose`. Before lidar is up AMCL never answers, so thousands of lines per second.
- **Lidar soft-reset takes several seconds before the chassis / EKF publish `odom`.** If Nav2 starts immediately, costmaps log missing `base_link → odom`. That is not a crash. Ctrl+C in that window produces shutdown noise (`context is invalid`, `exit code -6`).

**Fix:** Follow launch now matches daily nav: **wait for `/scan` and wheel odom, then start Nav2**. The panel says **正在等底盘（雷达复位和 CAN）。先别按 Ctrl+C。**, then **导航已就绪**.

1. `colcon build --packages-select robot_simple_commander --symlink-install && source install/setup.bash`
2. `ros2 launch robot_simple_commander follow_trajectory.launch.py`
3. **Wait 15–20 s** (RViz is not open yet). After `Hardware topics are live. Starting Nav2`, the GUI appears. Wait until the panel says **导航已就绪** (do not Ctrl+C during lidar RESET)
4. **2D Pose Estimate** at the route start, until the laser sits on the walls
5. **开始** to drive. After it finishes, **回到起点**. **停止** only stops the robot; Ctrl+C in this terminal exits

If the Follow panel is missing: Add New Panel → `robot_simple_commander/Follow`. **开始** is refused until you have used 2D Pose Estimate, and also when the robot is more than 0.8 m from the route start.

If `mini_lab1.yaml` was recorded on an older map, re-record on the current `mini_lab3` map when the path does not match the room.

---

### 7.24 Outbound OK, return trip hangs; Ctrl+C cannot kill `planner_server`

**Symptom:** The first goal drives fine. Clicking back to the start (same path, reverse) freezes the robot. The log loops:

- `[compute_path_to_pose] Waiting for async process to finish`
- `Action callback is still running and missed deadline to stop`
- Extra Ctrl+C only prints `user interrupted with ctrl-c again, ignoring...`
- Then `escalating to SIGKILL`, `planner_server` exit -9
- Sometimes `Extrapolation Error … Requested time … into the past` with a **frozen** Requested time

**Cause:** Not the motors. At the first goal the Mini sits in inflation, so the return plan often fails. The old BT then **cleared the entire global costmap**. Humble `planner_server` waits forever in `waitForCostmap` for `current_==true`; that loop **ignores Ctrl+C**. The outbound goal stamp (~20 s old) is also gone from the TF cache. Mashing Ctrl+C only makes launch wait 5 s then SIGKILL.

**Fix (already stuck):** Stop mashing Ctrl+C. In another terminal:

```bash
pkill -9 -f planner_server
pkill -9 -f rviz_goal_navigation
```

When the prompt returns, launch again.

**Fix (code; rebuild C++ once):** Planner wait times out at 2 s; return trip clears a small disk under the robot; plan failure waits instead of wiping the map; TF lookups use latest time.

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_util nav2_planner nav2_navfn_planner nav2_behavior_tree --symlink-install
source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

Align laser → outbound Goal → after arrival click **Cancel** on the Navigation2 panel (or wait for Goal succeeded) → then the return Goal. Do not fire two goals while the first is still running.

---

### 7.25 Gimbal: `Failed to set position control mode` / `There is no status packet`

**Symptom:** `dynamixel_driver` exits immediately. The USB port opens; setting position mode fails.

**Cause:** The port that opened is the **U2D2 adapter** (data only). A servo never replies. U2D2 does **not** power the chain. On 2026-10-11 the adapter serial is **FT8ISNFV** (the old `FT66WQS9` is no longer plugged in; a config still pointing at it can see a by-id entry and still fail to open). The bus is ID 1 **XL430-W250** (pan) and ID 2 **XC330-T288** (tilt).

**Fix:** 12 V on the Dynamixel bus (LED on each servo), U2D2 switch **TTL**, daisy-chain seated. Use the by-id path, not raw `ttyUSB*`:

```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0
```

Log should show `pan=id1` and `tilt=id2`. Missing ID 2 means the tilt cable is off.

Then `ros2 run robogait_gimbal dynamixel_driver` should log `Dynamixel pan+tilt ready`.

---

### 7.26 Camera SDK: empty RViz cloud / `person_sdk` cannot open device

**Symptom:** Launch runs but `/orbbec/points` is empty, or log says `No K4A/Femto device`.

**Cause:** Femto Mega needs **USB3**. The process must load the **Orbbec K4A wrapper** (`~/orbbec_wrapper_v2/lib`), not Microsoft `libk4a`. Do not start Nuitrack and SDK together (they grab the camera).

**Fix:**

```bash
lsusb | grep -i 2bc5
ros2 launch robogait_perception person_source.launch.py
```

RViz Fixed Frame `base_link`: Mini chassis + cyan Femto Mega on top + coloured depth cloud. Orange arrow = look direction. The person sphere is off by default (RViz Person). Stand 1–4 m in front. **Default cloud is person-only** (not the whole room); full room: **7.31**. Empty cloud with no device: check USB3. **Do not use SDK for tracking/skeleton** — see **7.34**.

If the terminal prints `incompatible QoS` / `RELIABILITY_QOS_POLICY`, see **7.27**.

---

### 7.27 Camera SDK: `incompatible QoS` spam, or “where is the camera?” in RViz

**Symptom:** Color cloud is visible, but the log repeats `requesting incompatible QoS` … `RELIABILITY_QOS_POLICY`. Or only tiny RGB axes at the origin, so the lens is hard to spot. Holding Ctrl+C prints `user interrupted … ignoring` and `person_sdk` dies with `exit code -2`.

**Cause:** The node used Best Effort for the cloud; default RViz subscribes Reliable, so that subscription gets nothing. The small axes *are* `orbbec_link` (the camera). Extra Ctrl+C while the capture thread is closing is treated as a second interrupt and the process is killed.

**Fix:** Rebuild, then launch once (close any old RViz first):

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
```

Enable **CameraPose** in RViz: orange arrow = look direction, white **Camera** label. **RobotModel** is the Mini (chassis + lidar + cyan camera). Stop with **one** Ctrl+C and wait a couple of seconds.

---

### 7.28 Point cloud is only axes — show the whole robot / camera on the Mini

**Symptom:** RViz only shows RGB axes or an arrow, so it is not obvious the camera sits on the rover. Or the robot and the cloud do not line up.

**Cause:** Axes are one drawing. The full robot needs the Mini URDF (`robot_state_publisher`), RViz **RobotModel**, and Fixed Frame `base_link` (not `orbbec_link` as the world).

**Fix:**

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
```

You should see wheels + chassis + cyan camera on top, orange arrow into the cloud. If the camera is too far left/right/high: edit the column / gimbal joint `origin` in `src/robogait_perception/urdf/orbbec_gimbal.urdf`.

If Nav2 is already running (`rviz_goal_navigation`), do not start a second URDF: `person_layer.launch.py` already sets `use_robot_model:=False`. You can also Add → RobotModel → `/robot_description` in the navigation RViz.

---

### 7.29 Gimbal does not follow the person / camera looks welded on in RViz

**Symptom:** Cloud and robot show, but walking around does not turn the lens. Or the real servos move while the cyan camera in RViz stays still.

**Cause:** `person_source.launch.py` is **view-only** — it does not start the gimbal. Tracking needs `camera_tracker` + `dynamixel_driver` (`test_gimbal.launch.py`). The model pan/tilt joints read `/gimbal_joint_states` from `dynamixel_driver`.

**Fix:** Stop the old launch, power the gimbal 12 V, then:

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

Stand 1.5–3 m **facing** the camera. After launch the gimbal holds the idle pose (`idle_pan_ticks=2048` = forward). The log should say `waiting for person`. After Nuitrack lock: `Skeleton detected`, `Person locked joint=torso`, then `Person locked — gimbal tracking`. After ~1.5 s lost it returns to idle.

Wrong idle heading: edit `idle_pan_ticks` / `idle_tilt_ticks` in `src/robogait_gimbal/config/gimbal.yaml` (1024 ≈ 90° right, 3072 ≈ 90° left; tilt **smaller ticks = look up**, default 1550 ≈ standing person's chest). Then restart `test_gimbal`.

If the log stays `Waiting for person`: overlay the old workspace and do not also run `person_sdk`. Stand, face the camera, **1.5–3 m**. If it stays `Person from depth` / `joint=depth` with no torso: that is SDK+k4abt CPU — **7.34**. No servo reply: **7.25**. Tracking on but no motion: **7.33**.

---

### 7.30 Gimbal: Color still shows the room, but the person flies into the sky in RViz

**Symptom:** As soon as a person is detected and the gimbal moves, the RViz cloud/person shoots above the grid. Color still shows the person in the room — the real lens is not pointing at the ceiling.

**Cause:** The cloud lives in `orbbec_link`. The real tilt servo and the URDF tilt joint rotated opposite ways, so RViz pitched the camera X (depth) at the sky and the cloud followed. This is not the chassis `base_link` flying. Depth fallback already zeros `pos.y` (pan only); skeleton frames still command tilt. A wrong model tilt sign is enough for “move gimbal → sky”.

**Fix:** Set **`tilt_urdf_sign: -1.0`** under dynamixel in `src/robogait_gimbal/config/gimbal.yaml` (this is already the default). That flips the RViz model only, not the hardware. If left/right is also mirrored when you walk, set `pan_urdf_sign: -1.0` too. Stop the old process, rebuild, and launch:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

After lock, walking left/right should keep the person in front of the robot near the floor. Still too slow: raise `profile_velocity` in the same file (e.g. 500).

---

### 7.31 RViz cloud should show only the person (no floor / walls / furniture)

**Symptom:** `/orbbec/points` paints the whole room and the person is hard to see; or with person-only on, the cloud is almost empty.

**Cause:** Default `cloud_person_only: true`. With a skeleton, Azure body-index pixels are kept. Without one, a ~0.55 m cylinder around the torso is used and floor points far below the torso are dropped. Color `/orbbec/color/image_raw` and depth are **not** cropped. No detection ⇒ the cloud can be empty.

**Fix:**

```bash
# Stop the old person_source / test_gimbal, rebuild, relaunch
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception --symlink-install
source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
# Tracking demo: ros2 launch robogait_gimbal test_gimbal.launch.py
```

Stand 1.5–3 m facing the camera. You should see a coloured blob of the person, not walls or floor.

For a **full-room** teacher demo: set `cloud_person_only: false` in `src/robogait_perception/config/perception.yaml` and restart the launch. Cylinder too tight/wide (no skeleton): edit `cloud_person_radius_m`.

---

### 7.32 Person bounces in the image / freeze the camera for straight follow

**Symptom:** In Color, the whole person jumps up and down while walking, and the gimbal nods every step. Or a straight-corridor test should keep the lens still.

**Cause:** The old lock was chest or a depth blob (legs / upper mass). Walking bobs the COM and a snappy tilt PID hunts. Gait analysis must lock the **pelvis / navel**. The depth blob is only “someone is there” for pan — **not a gait center, and not used for tilt**.

**Fix:**

1. **Torso / pelvis lock:** gimbal default is **Nuitrack** (`joint=torso`). Do not use SDK `joint=depth` for gait. Stand **1.5–3 m facing** the camera. If it stays `Person from depth` / `joint=depth`: **7.34**.
2. **Same image height:** in `gimbal.yaml` set `target_elevation_deg: 0.0` (optical axis on the waist), `tilt_dead_zone_deg: 5.0`, `tilt_kp: 0.22`, `tilt_filter_window: 18`. Tilt only follows sit/stand or robot pitch, not each gait cycle. Pan `kp: 1.0` stays snappy for corners.
3. **Freeze for straight follow:** use **B3**. That launch sets `freeze_gimbal: true` on both axes; do not edit `gimbal.yaml` for it. The gimbal holds `idle_pan_ticks` / `idle_tilt_ticks`. Edit the yaml only if you want to freeze without B3.
4. **Corridor corners:** `freeze_gimbal: false`, `enable_gimbal_pan: true`. Pan tracks while the robot turns. Leave tilt on (slow) or set `enable_gimbal_tilt: false` for pan-only.

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
# YAML-only: no rebuild. After person_sdk.cpp changes:
# colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
ros2 launch robogait_gimbal test_gimbal.launch.py
```

Still bouncing: raise `tilt_dead_zone_deg` to 6 or lower `tilt_kp`. No skeleton: **7.34**. `joint=depth` but no pan at all: see **7.33**; `freeze_gimbal` must be `false` on both nodes.

---

### 7.33 Log already says `joint=depth` / `Person locked — gimbal tracking`, but the gimbal does not turn

**Symptom:** `test_gimbal` logs `Person from depth (no skeleton, pan only)`, `Person locked joint=depth pixel=…`, then `dynamixel_driver: Person locked — gimbal tracking`. The person sits on one side of Color. The real gimbal and the RViz camera do not yaw. `joint=pelvis` never appears.

**Cause:** With no skeleton yet, the depth blob is **pan-only** (keep the person in Color so body tracking can lock the pelvis) — not “hold still”. Older code published the same pixel error once; the first command often landed in the 0.08 s window before the servo entered tracking, and later identical errors were skipped, so the lens froze. `freeze_gimbal: true` also holds idle. Tilt waits for `joint=pelvis`.

**Fix:**

1. In `src/robogait_gimbal/config/gimbal.yaml`, confirm **`freeze_gimbal` is `false` on both camera_tracker and dynamixel_driver**, and `enable_gimbal_pan: true`.
2. Stop the old launch, rebuild, relaunch:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

3. Stand **1.5–3 m** facing the camera. With Nuitrack you should see `joint=torso` and `Pan skeleton pixel=…`, and the lens should yaw.
4. If it stays `joint=depth` / no pelvis or torso: do not tune k4abt CPU — **7.34**.

---

### 7.34 Always `joint=depth` / no `pelvis` or `torso` — skeleton never appears

**Symptom:** Logs stay `Person from depth (no skeleton)` or `Person locked joint=depth`. Never `Person locked joint=pelvis` / `joint=torso`. Nuitrack user count also stays 0.

**Cause:** The stack used to default to **Orbbec K4A + Azure Kinect Body Tracking (libk4abt) in CPU mode**. This NUC11TNKv7 has Intel Iris Xe and **no NVIDIA CUDA**. CPU k4abt is slow and unreliable, so you only ever get a depth blob. The method that actually produced a skeleton on this robot is **Nuitrack** (`nuitrack_person_node` from `~/rover_workspace_final`). Nuitrack and SDK **must not run together** (same USB camera).

**Fix:** Stop every `person_sdk` / old `test_gimbal`, then:

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
# already defaults to person_source:=nuitrack
```

If `orbbec_person_publisher` is missing: source the old overlay **before** this workspace. Stand **1.5–3 m facing** the camera. Expect Nuitrack `Skeleton detected`, `/orbbec/num_users` ≥ 1, `Person locked joint=torso`, then the gimbal pans.

Logs already show `Skeleton detected` / `joint=torso` but RViz cloud is empty: the skeleton is working; the cloud frame is not connected to the robot — **7.35**. Do not stand closer than ~1 m (Nuitrack drops the skeleton and the gimbal slams).

For a **full-room point cloud** teacher demo use SDK and leave body tracking off:

```bash
ros2 launch robogait_perception person_source.launch.py person_source:=sdk
```

Keep `enable_body: false` in `perception.yaml`.

---

### 7.35 Nuitrack logs already lock a person, but RViz looks empty

**Symptom:** The terminal already has `Person source: Nuitrack (torso)`, `nuitrack_person_node`, `Skeleton detected`, `Person locked joint=torso`, `Pan skeleton` / `Person locked — gimbal tracking`, and the gimbal is moving. RViz point cloud is empty and it spams:

```text
Message Filter dropping message: frame 'rgbd_camera_link_optical' ... queue is full
```

RViz Color / Depth panels say **No Image** (Nuitrack does not publish color). Walking in to 0.7–1.1 m slams pan into the stop, then `Waiting for person`.

**Cause:** Nuitrack publishes the cloud in **`rgbd_camera_link_optical`** (optical: X right, Y down, Z out of the lens). The Mini URDF only has **`orbbec_link`** (X forward, Z up). With no TF between them, RViz Fixed Frame `base_link` cannot transform the cloud, the message filter queue fills, and frames are dropped — **the person was already detected**. Nuitrack **does not** publish `/orbbec/color/image_raw`, so Color is empty by design. Closer than ~1 m, Nuitrack drops the skeleton. `libcusolver.so.9.0` / `libKinect2DepthProvider.so` / GPU-module lines are **not fatal** (Nuitrack still ran on CPU).

**Fix:** Stop the old launch, rebuild perception/gimbal, then **overlay the old workspace before this one**:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

Keep RViz **Fixed Frame = `base_link`** (do not switch it to `rgbd_camera_link_optical`). The cloud should appear in front of the Mini. Nuitrack uses `orbbec_nuitrack.rviz` with Color/Depth removed, so you no longer get No Image. Stand **1.5–3 m** facing the camera; do not walk in close. Too close logs `Person too close` and the gimbal holds (`min_track_z_m` in `gimbal.yaml`).

---

### 7.36 Sharing the robot with `rover_workspace_final`

**Symptom:** Two people launch different packages and the robot jerks, lidar or IMU fails to open, or `/cmd_vel` has two publishers.

**Cause:** This NUC's `.bashrc` sets `ROS_DOMAIN_ID` to **10**. `RoboGait_ws` and `~/rover_workspace_final` share that domain, so topics are visible to both. Both claim the same hardware: `/dev/rplidar`, `can0`, the gamepad, and `/cmd_vel`. Both lidar configs use `/dev/rplidar`. IMU path strings differ (`/dev/bno055` here, `/dev/serial/by-id/usb-0403_6014-if00-port0` there) but it is the same chip and only one process can open it. `.bashrc` sources `GUI_ROBOGait` only (no chassis driver), so their packages are **not** pulled in automatically. The clash is when both launches are running.

**Fix:** One stack at a time. Before starting, confirm nobody else's nodes are up:

```bash
ros2 node list
```

If anything is listed, wait until they stop. Today's mapping / navigation / path follow should not `source ~/rover_workspace_final`.

---

### 7.37 Mapping: the lidar scan floats above the map

**Symptom:** From the side, the colored scan is a line in the air, not on the map.

**Cause:** The lidar sits about **0.33 m** above `base_link` (chassis 0.061 m + payload 0.1198 m + lidar joint 0.145 m). The old workspace mounted it at the same height, so a side view floated there too. The map is drawn on the ground plane.

**Fix:** `flatten_scan` in the mapping launch copies that scan to `/scan_flat` at `base_link` height, which is the map plane. `slam_toolbox` still subscribes to the real `/scan`, so wall positions do not move. Mapping RViz LaserScan uses `/scan_flat`. Restart mapping after the change:

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
```

Do not save an RViz window whose title has `*`, or it writes the old topic back over the file.

---

