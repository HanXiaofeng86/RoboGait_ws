# RoboGait_ws Project Handbook (beginner-friendly)

> **English handbook.** Chinese source of truth: `PROJECT_CHANGES.md` (keep in sync).  
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

1. **`robot_mppi_navigation`** = **A1**: RViz goals. Hardware + Nav2 + MPPI + safety.  
2. **`robot_simple_commander`** = **A2**: teleop **record** / **follow**.  
   **No dependency on `robot_mppi_navigation`** (own stack copy — duplication OK).

**Mode B — person follow** (`robogait_*`, Python)

3. **`robogait_person_interfaces`** = `User.msg` (detection + xyz).  
4. **`robogait_perception`** = Orbbec bridge → `user_raw`.  
5. **`robogait_gimbal`** = gimbal (pixel PID + Dynamixel).  
6. **`robogait_person_follow`** = person → Nav2 goal; **chassis velocity from MPPI only** (no PID blend).

```text
Mode A (robot_* pure)               Mode B (robogait_* person)
  A1 robot_mppi_navigation            B0 gimbal → robogait_gimbal
  A2 robot_simple_commander           B1 one-shot / B2 person layer (person→goal→MPPI)
Chassis [v,ω] from MPPI only; person supplies goals. Do not run two Nav2 stacks.
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
| Hardware, localization, Nav2, **MPPI**, smoother, safety | **Mode 2** record/follow (other package) |
| Preflight, diagnostic bags, lidar reset | Act as a dependency of `robot_simple_commander` |

Daily one-shot:

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

---

### 2.2 `launch/` — start switches

> Beginner tip: a launch file = “which programs to start together.” Memorize only the top one.

| File | What it does | When you use / edit it |
|------|--------------|------------------------|
| **`rviz_goal_navigation.launch.py`** | **Top one-shot**: hardware + nav + optional RViz | **Daily driving.** Edit for default RViz on/off, default map args, etc. |
| **`nav2_mppi_controller.launch.py`** | Nav “brain”: EKF, map/AMCL/SLAM, Nav2 (**MPPI**), smoother, safety; remaps MPPI output to `cmd_vel_nav` | Usually **do not run alone** (included by top). Edit for velocity remaps / lifecycle node list |
| **`rover_mini_hardware.launch.py`** | Chassis driver, lidar, IMU, URDF only | **Sensors/chassis only.** Edit how hardware is started |
| **`ekf_amcl_localization.launch.py`** | EKF-focused debug launch | Rarely for daily use |

Common top-level args:

| Arg | Meaning in plain words |
|-----|------------------------|
| (default) | Map + AMCL + RViz |
| `amcl:=False` | No real localization; software pipe check |
| `slam:=True` | Build map while driving |
| `use_rviz:=False` | No GUI |
| `map:=/abs/path/xxx.yaml` | Use another map |

```text
rviz_goal_navigation          ← the one you type daily
   ├── rover_mini_hardware    robot + sensors
   ├── nav2_mppi_controller   localization + MPPI + safety
   └── rviz2 (optional)
```

**Pitfall:** do not also start `nav2_mppi_controller` while the top launch is running (two Nav2 stacks).

---

### 2.3 `config/` — knobs (prefer editing here)

> Beginner tip: change “how fast / how it avoids / localization” in yaml—**usually not Python**. Restart launch after edits (with `--symlink-install`).

| File | What it does | Edit it when you want to… |
|------|--------------|---------------------------|
| **`nav2_pure_mppi_params.yaml`** | **Most important.** Full Nav2 + **MPPI**: speed limits, samples, 8 critic weights, costmaps, AMCL, smoother, safety timeouts. `FollowPath` plugin = `nav2_mppi_controller::MPPIController` | Speed, turning, clearance, goal approach, control rate… **start here** |
| **`robot_localization_ekf.yaml`** | Fuse wheel odom + IMU → `/odometry/filtered` | Pose drifts / crooked straight lines (odom/IMU fusion) |
| **`sensor_topics_frames.yaml`** | Human reference: topic and frame names | Looking up names; usually not required to run |
| **`diagnostic_recording_topics.yaml`** | Which topics go into test bags | More/fewer topics for analysis |

---

### 2.4 `maps/`

| File | What it does | Beginner note |
|------|--------------|---------------|
| **`lab2_new.yaml`** | Map metadata for `map_server` / AMCL | Swap maps as a **yaml + image pair** |
| **`lab2_new.pgm`** | Occupancy grid image | Must sit **next to** the yaml |
| **`lab2_new.posegraph`** + **`.data`** | slam_toolbox pose-graph format | **Not used** by default nav (AMCL today) |

---

### 2.5 `rviz/`

| File | What it does | Edit when… |
|------|--------------|------------|
| **`rviz_goal_navigation.rviz`** | Default displays (map, laser, path, Nav2 panel, Goal tool) | You want different default displays / Fixed Frame |

---

### 2.6 `scripts/` — commands you type

| File | What it does | How to use |
|------|--------------|------------|
| **`preflight_check_topics.sh`** | Check lidar/IMU/wheels/EKF topics | After stack up, before goals |
| **`preflight_check_tf.sh`** | Check `map→odom→base_link` | Pose frozen / jumping |
| **`record_navigation_test.sh`** | Record rosbag per topic list | `./scripts/record_navigation_test.sh name`, Ctrl+C |
| **`analyze_navigation_bag.py`** | Metrics + plots from a bag | `python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/<dir>` |
| **`reset_rplidar.py`** | Serial RESET for lidar | Or `ros2 run robot_mppi_navigation reset_rplidar` |

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
| Record | `record_trajectory.launch.py` + other terminal `teleop_twist_keyboard`; Ctrl+C saves |
| Follow | `follow_trajectory.launch.py` (includes MPPI) |

---

### 3.2 Launches vs nodes

```text
launch/
  record_trajectory.launch.py   ← ros2 launch entry
  follow_trajectory.launch.py   ← ros2 launch entry
  parts/                        ← fragments (not entry points)
recorded/                       ← your teleop-recorded yaml
paths/                          ← built-in test geometry
nodes/                          ← ros2 run nodes
lib/                            ← libraries (not nodes)
safety/                         ← safety-gate logic
```

#### `launch/` — entry points

| File | What it does | How to use |
|------|--------------|------------|
| **`record_trajectory.launch.py`** | Record: hardware + EKF + map TF + node `record_path` | `ros2 launch robot_simple_commander record_trajectory.launch.py` |
| **`follow_trajectory.launch.py`** | Follow: hardware + `parts/nav_stack` (MPPI) + node `follow_path` | `ros2 launch robot_simple_commander follow_trajectory.launch.py` |

#### `launch/parts/` — Fragments

| File | What it does | How to use |
|------|--------------|------------|
| **`hardware.launch.py`** | Chassis / lidar / IMU | Included by entry launches only |
| **`nav_stack.launch.py`** | EKF + localization + Nav2/MPPI + smoother + this package’s safety gate | Included by follow entry only |

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
| **`nodes/record_path.py`** | While you drive, sample pose in `map`; Ctrl+C writes yaml | Sampling distance, frames, save rules |
| **`nodes/follow_path.py`** | Load yaml → (transform `local` if needed) → Simple Commander FollowPath → **MPPI drives** | Controller id, wait_for_nav2, logging |
| **`nodes/cancel_task.py`** | Cancel NavigateToPose / FollowPath goals | Which action servers |
| **`lib/path_io.py`** | Read/write path yaml (shared format) | File format fields (affects record + follow) |
| **`lib/generators.py`** | Math for straight / ±90° / S-curve test shapes (no speeds) | Test path length / shape |
| **`safety/`** | Own safety gate (logic copied; no import from robot_mppi_navigation) | `/cmd_vel` gate on follow |

---

### 3.4 `recorded/` and `paths/`

| Folder | Contents | How to use |
|--------|----------|------------|
| **`recorded/`** | **Your** teleop recordings (default `lab_route.yaml`) | Record writes here by default; follow with `path_file:=…/recorded/….yaml` |
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
| **`package.xml`** | Name + deps (incl. `nav2_simple_commander`) | Adding deps |
| **`setup.py`** / **`setup.cfg`** | Register `record_path` / `follow_path` / `cancel_navigation` | New executables |
| **`README.md`** | English package README | Keep updated |
| **`test/test_path_io.py`** | Unit tests for generators / yaml | After path_io changes |

---

### 3.6 Intent → file (this package)

| I want to… | Go to |
|------------|-------|
| Denser / sparser recording | `nodes/record_path.py` `min_separation` or launch arg |
| Follow / wait-for-Nav2 behavior | `nodes/follow_path.py` |
| Change built-in test shapes | `generators.py` then `generate_test_paths.py` |
| Hand-edit waypoints | Edit `paths/*.yaml` or your recording |
| Cancel behavior | `nodes/cancel_task.py` |
| Follow speed / wobble (control) | This package’s `config/nav2_mppi_params.yaml` (independent of `robot_mppi_navigation`) |

### 3.7 Commands (this package — no robot_mppi_navigation)

```bash
colcon build --packages-select robot_simple_commander --symlink-install
source install/setup.bash

# Record (default → package recorded/lab_route.yaml)
# Terminal A:
ros2 launch robot_simple_commander record_trajectory.launch.py
# Terminal B:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Ctrl+C on A to save

# Follow the recording
REC=$(ros2 pkg prefix robot_simple_commander)/share/robot_simple_commander/recorded/lab_route.yaml
ros2 launch robot_simple_commander follow_trajectory.launch.py path_file:=$REC

# Follow a built-in test path
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=left_90
ros2 run robot_simple_commander cancel_navigation
```

---

## 4. Mode B packages (Python, person follow)

> Logic inspired by `~/rover_workspace`; **rewritten in Python**, not migrated C++.  
> External deps: `orbbec_person_publisher` (Nuitrack), `dynamixel_sdk`.

### 4.1 `robogait_person_interfaces`

| File | Role |
|------|------|
| **`msg/User.msg`** | `detection` + `x/y/z` (camera frame, z = range) |

### 4.2 `robogait_perception`

| Node / file | Role | Command |
|-------------|------|---------|
| **`nodes/distance_orbbec.py`** | `/orbbec/*` → `user_raw`, `person_center_pixel` | `ros2 run robogait_perception distance_orbbec` |
| **`config/perception.yaml`** | Topics, timeouts | Edit for Orbbec topic names |

### 4.3 `robogait_gimbal`

| Node / file | Role | Command |
|-------------|------|---------|
| **`nodes/camera_tracker.py`** | Pixel PID → `camera_position` (deg) | `ros2 run robogait_gimbal camera_tracker` |
| **`nodes/dynamixel_driver.py`** | Angle → XL430 position | `ros2 run robogait_gimbal dynamixel_driver` |
| **`config/gimbal.yaml`** | Serial device, limits, PID | **Set servo device here** |
| **`test_gimbal.launch.py`** | B0: gimbal + perception only | `ros2 launch robogait_gimbal test_gimbal.launch.py` |

### 4.4 `robogait_person_follow`

| Node / file | Role | Command |
|-------------|------|---------|
| **`nodes/distance_filter.py`** | `user_raw` → `user_coordinates` | `ros2 run robogait_person_follow distance_filter` |
| **`nodes/person_nav_goal.py`** | Person → standoff `NavigateToPose` (MPPI owns full `[v,ω]`) | `ros2 run robogait_person_follow person_nav_goal` |
| **`person_follow.launch.py`** | B1 one-shot: A1 Nav2/MPPI + person layer | See below |
| **`person_layer.launch.py`** | B2 terminal 2 on running Nav2 | See below |
| **`config/person_follow.yaml`** | Standoff distance, dead zone, goal update rate | **Edit for follow distance** |

**Control chain (person follow = pure MPPI):**

```text
person → distance_filter → person_nav_goal → NavigateToPose
                                              ↓
                                     planner → MPPI → smoother → safety_gate → /cmd_vel
```

Person does **not** rewrite `/cmd_vel`. Old `cmd_vel_blend` / `straight_follow` (PID mix) removed.

```bash
ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0
ros2 launch robogait_person_follow person_follow.launch.py
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py
```

---

## 5. Other workspace files

| Item | What it does | Beginner use |
|------|--------------|--------------|
| **`PROJECT_CHANGES.md`** | Chinese handbook | “What file / where to edit” |
| **`PROJECT_CHANGES_EN.md`** | This English handbook | Same |
| **`TEST_PLAN.md`** | On-robot checklist (**Chinese**): prep 0/1 → T1–T8 → B0/B1/B2 | Follow it; **B0 gimbal before person follow** |
| **`NODES_AND_TOPICS.md`** | Node↔topic diagrams (**English**, verified vs launch/code) | Who publishes / subscribes what |
| **`README.md`** | Repo landing | First link for newcomers |
| **`src/robot_mppi_navigation/README.md`** | Stack package README | Complements §2 |
| **`src/robot_simple_commander/README.md`** | Record/follow README | Complements §3 |
| **`src/robogait_perception/README.md`** | Mode B perception | §4 |
| **`src/robogait_gimbal/README.md`** | Mode B gimbal | §4 |
| **`src/robogait_person_follow/README.md`** | Mode B person follow | §4 |
| **`src/navigation2/`** | Full Nav2 sources (incl. MPPI) | **Advanced** algorithm / critic edits |
| **`.vscode/settings.json`** | Markdown preview default | Optional |
| **`.gitignore`** | Ignore build/install/log | Leave alone |
---

## 6. TODO

| Item | Status |
|------|--------|
| ~~RPLidar health 2~~ | Fixed: `ros2 run robot_mppi_navigation reset_rplidar` |
| ~~BNO055 comms~~ | Fixed (07-21) |
| `slam:=True` on-robot | Ready to test |
| `TEST_PLAN.md` checkboxes | Pending (0/1 → T1–T8 → B0→B1) |
| Mode A2 record + follow | §3 commands; T3–T6 |
| Mode B: `test_gimbal` on-robot | Code ready; test pending |
| Mode B: `person_follow` / `person_layer` on-robot (pure MPPI) | Code updated; test pending |
| posegraph localization in launch | Not done (AMCL now) |
| Custom MPPI critic | Not started (`navigation2`) |
| Gazebo deps | Install when needed |
