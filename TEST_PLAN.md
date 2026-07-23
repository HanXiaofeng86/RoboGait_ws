# RoboGait on-robot test plan (pure MPPI navigation)

> **How to use:** Work stage by stage; do not start nav trials until the previous stage passes.  
> **Every new terminal:**
> ```bash
> cd ~/RoboGait_ws
> source /opt/ros/humble/setup.bash && source install/setup.bash
> ```
> Command details: `PROJECT_CHANGES_EN.md` §4 (or Chinese `PROJECT_CHANGES.md` §4).

---

## Overview (8 acceptance tests + prerequisites)

| Stage | Content | Purpose | Done |
|-------|---------|---------|------|
| A | Hardware & environment | Robot, sensors, build OK | ☐ |
| B | Bringup + preflight | Full stack up; topics/TF OK | ☐ |
| **T1** | Idle | Velocity must stay zero with no goal | ☐ |
| **T2** | Straight (Mode 1) | RViz Goal straight run | ☐ |
| **T3** | Left 90° (Mode 2) | Predefined-path turn | ☐ |
| **T4** | Right 90° (Mode 2) | Symmetric to T3 | ☐ |
| **T5** | S-curve (Mode 2) | Continuous curve tracking | ☐ |
| **T6** | Static obstacle (Mode 2) | Avoid obstacle on path | ☐ |
| **T7** | Cancel + estop | Safety gate works | ☐ |
| **T8** | Control rate | ~20 Hz, few dropped frames | ☐ |

**Two ways to drive:**
- **Mode 1:** RViz “Nav2 Goal” (T2)
- **Mode 2:** `follow_predefined_path` with yaml paths (T3–T6); full stack must already be running

---

## Stage A: Hardware & environment

### A.1 Devices & CAN (after boot / robot power cycle)

```bash
ls -l /dev/rplidar /dev/bno055          # symlinks should exist
ip link show can0                       # must be state UP; if DOWN:
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
candump -n 10 -T 3000 can0              # should see frames
```

| # | Action | Pass criteria |
|---|--------|---------------|
| A.1.1 | Chassis powered, e-stop released, CAN OK | can0 UP + candump has data |
| A.1.2 | Lidar health 2 → reset | `ros2 run mppi_navigation reset_rplidar`, then restart driver |
| A.1.3 | IMU Errno 5 after USB re-plug | **Ctrl+C clean stop, then relaunch**—do not reuse stale nodes |

### A.2 Sensors only (optional, for bringup)

```bash
ros2 launch mppi_navigation rover_mini_hardware.launch.py
# Other terminal (Humble: one topic per `hz`)
ros2 topic hz /scan              # ~10 Hz
ros2 topic hz /imu/data          # ~50–60 Hz
ros2 topic hz /odometry/wheels   # ~15–30 Hz
```

### A.3 Build

```bash
cd ~/RoboGait_ws
colcon build --packages-select mppi_navigation --symlink-install
source install/setup.bash
ros2 pkg executables mppi_navigation
```

**Pass:** build OK; executables include  
`safety_gate`, `follow_path`, `cancel_navigation`, `idle_check`, `reset_rplidar`.

---

## Stage B: Full stack + preflight

### B.1 Launch (default: AMCL + `lab2_new` + RViz)

```bash
ros2 launch mppi_navigation rviz_goal_navigation.launch.py
```

Variants:

```bash
# Software pipe only — no real localization
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False

# Online mapping (overrides AMCL)
ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True
```

**Pass:**
- [ ] No red FATAL (chassis stale → check can0 in A.1)
- [ ] Lifecycle / controller (MPPI) active in logs
- [ ] RViz opens; with default AMCL, set pose with **2D Pose Estimate** first

### B.2 Preflight

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/preflight_check_topics.sh
./scripts/preflight_check_tf.sh
ros2 topic list | grep cmd_vel
# expect: /cmd_vel  /cmd_vel_nav  /cmd_vel_smoothed

ros2 topic info /cmd_vel --verbose
# only safety_gate should publish
```

**Pass:** both scripts exit 0; three cmd_vel topics present; `/cmd_vel` publisher is only safety_gate.

> `[FAIL] /map`: map_server often publishes once. If `ros2 topic echo /map --once` works and TF `map→odom` is OK, continue.

**Localization look (default AMCL):**
- [ ] Map visible; scan roughly aligns with walls (better after 2D Pose Estimate)
- [ ] Particle cloud converges

---

## Test T1: Idle (must pass first)

No goals:

```bash
ros2 run mppi_navigation idle_check 10
```

| Pass criteria |
|---------------|
| No non-zero `/cmd_vel` for 10 s; robot still; exit code 0 |

---

## Test T2: Straight (Mode 1 · RViz Goal)

Record a bag (new terminal) for formal runs:

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/record_navigation_test.sh straight_run1
```

In RViz, **Nav2 Goal** ~**3 m** ahead. Prefer 3 repeats. Stop bag with Ctrl+C, then:

```bash
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/straight_run1
```

| Pass (qualitative) | Metrics (reference) |
|--------------------|---------------------|
| Smooth start, straight, decelerate to stop | Cross-track RMSE **&lt; 0.1 m** |
| No oscillation / sudden reverse | Control ~**20 Hz**, no wild jerk spikes |
| `/cmd_vel` zeros at goal | — |

---

## Tests T3 / T4 / T5: Turns & S-curve (Mode 2)

Prerequisite: Stage B stack still running; open start area. Record one bag per path.

```bash
cd ~/RoboGait_ws/src/mppi_navigation

# Example: left 90° (T3)
./scripts/record_navigation_test.sh left90_run1
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

# T4 / T5: change bag name and path_name
# path_name:=right_90
# path_name:=s_curve
```

Paths start from the **current** robot pose into `map`. Show `/predefined_path` in RViz if useful.

| Item | Pass criteria |
|------|---------------|
| T3 left 90° | Sensible corner slowdown; no spin-in-place / large overshoot |
| T4 right 90° | Roughly symmetric to left |
| T5 S-curve | Smooth tracking; no cutting corners into failure |
| All | `follow_path` ends with **status 4 (SUCCEEDED)** |
| Metrics | Cross-track RMSE **&lt; 0.15 m** (curves a bit looser OK); heading error does not diverge |

---

## Test T6: Static obstacle (Mode 2)

Place a box (wider than the robot) mid-path on `obstacle_avoid`:

```bash
./scripts/record_navigation_test.sh obstacle_run1
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=obstacle_avoid
```

| Pass criteria |
|---------------|
| Obstacle visible on local costmap (~0.4 m inflation) |
| Slow, keep clearance, resume path after bypass |
| No collision, no long stuck state |

---

## Test T7: Cancel + estop (safety)

Run while **navigating** (Mode 1 or 2).

### T7.1 Cancel goal

```bash
ros2 run mppi_navigation cancel_navigation
```

| Pass criteria |
|---------------|
| Smooth stop within ~**0.5 s**; `/cmd_vel` all zeros |

### T7.2 Software estop

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
# clear:
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

| Pass criteria |
|---------------|
| Immediate zero `/cmd_vel` on estop; after clear, new goals work again |

### T7.3 Command-stream dropout (watchdog)

```bash
# While navigating — use carefully
ros2 lifecycle set /velocity_smoother 4
```

| Pass criteria |
|---------------|
| Within ~**0.5 s** safety gate publishes zero; robot stops |

---

## Test T8: Control rate (from an existing bag)

Any bag from T2–T6:

```bash
python3 ~/RoboGait_ws/src/mppi_navigation/scripts/analyze_navigation_bag.py \
  ~/RoboGait_ws/bags/<bag_name>
```

| Pass criteria |
|---------------|
| `/cmd_vel` mean period ≈ **50 ms (20 Hz)** |
| Max period **&lt; 100 ms**; share of periods &gt;75 ms **&lt; 5%** |
| If not met: try lower MPPI `batch_size` (1200→800) or `time_steps` (44→32) |

---

## Wrap-up

- [ ] All bags under `~/RoboGait_ws/bags/`; run analysis; keep `metrics.json` + plots
- [ ] Failures noted in `PROJECT_CHANGES.md` §5 TODO (Chinese handbook)
- [ ] After all pass: tune in order (exploration noise → horizon → weights → path/obstacle critics)

---

## Progress notes (for reports)

| Already in place (code / tools) | Still to tick on robot |
|--------------------------------|------------------------|
| Mode 1/2 launches, safety gate, EKF, default AMCL | T1–T8 checkboxes |
| Preflight / bag / analysis / idle_check / reset_rplidar | Bags or screenshots per test help |
| Sensor check flow validated | Re-check rates on the day of the trial |

**Suggested order:** A → B → **T1** → **T2** → **T7** (safety) → T3/T4 → T5 → T6 → T8.

---

## Troubleshooting quick list

| Symptom | Check first |
|---------|-------------|
| Chassis FATAL / data stale | can0 UP? candump? robot powered? (not lidar/IMU) |
| Robot never moves | Trace `/cmd_vel_nav` → `/cmd_vel_smoothed` → `/cmd_vel` |
| Safety gate always ZERO | `/odometry/filtered` timeout? estop left true? |
| TF / map issues | AMCL up? 2D Pose Estimate done? `/scan` live? |
| Goal rejected immediately | `ros2 lifecycle get /bt_navigator` active? |
| IMU Errno 5 / no `/scan` | Stop old launch; `reset_rplidar` if needed; relaunch |
| Large curve overshoot | Log + bag; tuning territory—don’t block the whole session |
