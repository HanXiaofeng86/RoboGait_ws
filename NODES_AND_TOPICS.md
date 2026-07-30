# Nodes and Topics (English)

Verified against current `setup.py` entry points and launch files.  
Arrows: `publisher --topic--> subscriber`.

**Important:** MPPI is **not** a ROS node. It is a **plugin** loaded by `controller_server`.

---

## 1. Complete inventory of *our* ROS executables

These are all custom `ros2 run` entry points in this workspace (11 unique names; `safety_gate` exists in two packages):

| Executable (node name) | Package | Role |
|------------------------|---------|------|
| `safety_gate` | `robot_mppi_navigation` | Gate: `cmd_vel_smoothed` → `/cmd_vel` |
| `idle_check` | `robot_mppi_navigation` | Idle `/cmd_vel` checker (tool) |
| `reset_rplidar` | `robot_mppi_navigation` | Lidar serial RESET (tool) |
| `record_path` | `robot_simple_commander` | Record map→base_link poses → yaml |
| `follow_path` | `robot_simple_commander` | Send FollowPath action |
| `cancel_navigation` | `robot_simple_commander` | Cancel Nav2 goals |
| `safety_gate` | `robot_simple_commander` | Same gate logic (A2 stack copy) |
| `distance_orbbec` | `robogait_perception` | Orbbec topics → `user_raw` |
| `camera_tracker` | `robogait_gimbal` | Pixel error → `camera_position` |
| `dynamixel_driver` | `robogait_gimbal` | `camera_position` → Dynamixel |
| `distance_filter` | `robogait_person_follow` | `user_raw` → `user_coordinates` |
| `person_nav_goal` | `robogait_person_follow` | Person → NavigateToPose |

`robogait_person_interfaces` has **no** nodes (messages only).

Utility nodes `idle_check` / `reset_rplidar` / `cancel_navigation` are **not** in the continuous control graphs below.

---

## 2. Mode A1 — RViz goal (full launch node names)

Launch: `ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py`

### 2.1 Velocity chain (exact names)

```text
rviz2  (or other client)
   |  action: navigate_to_pose
   v
bt_navigator
   |  (asks planner + controller via Nav2 internals)
   v
planner_server  ---- global path ---->
                                          \
                                           --> controller_server  [plugin: MPPI]
                                          /
behavior_server  (recovery also remapped to cmd_vel_nav)

controller_server
   |  topic: /cmd_vel_nav          (Twist)
   v
velocity_smoother
   |  topic: /cmd_vel_smoothed     (Twist)
   v
safety_gate                        [ours]
   |  also subs: /odometry/filtered, /safety_estop
   |  topic: /cmd_vel              (Twist)
   v
roverrobotics_driver               [external, from mini.launch]
   |
   v
chassis
```

### 2.2 Other nodes started with A1 (localization / lifecycle)

Typical names from our launches + `roverrobotics_driver/mini.launch.py`:

```text
Hardware / model (via mini.launch):
  roverrobotics_driver
  robot_state_publisher
  joint_state_publisher   (if included by mini)
  rplidar                 (or similar lidar node name from rplidar_ros)
  bno055

Localization:
  ekf_filter_node
  map_server              (when slam:=False)
  amcl                    (default)
  static_map_to_odom      (only if amcl:=False)
  lifecycle_manager_localization

Navigation lifecycle:
  smoother_server         (path smoother — not the velocity smoother)
  waypoint_follower
  lifecycle_manager_navigation

Optional UI:
  rviz2
```

Sensor → localization sketch:

```text
rplidar -------------- /scan -------------> amcl / costmaps
bno055 --------------- /imu/data ---------> ekf_filter_node
roverrobotics_driver - /odometry/wheels --> ekf_filter_node
                                              |
                                              |  /odometry/filtered
                                              |  TF: odom->base_link
                                              v
                                         amcl / controller_server / safety_gate
```

---

## 3. Mode A2 — record / follow

### 3.1 Record

```text
TF: map -> base_link
        |
        v
   record_path ---- writes yaml (no cmd_vel)
```

### 3.2 Follow

```text
path yaml
   |
   v
follow_path ---- action: follow_path ----> bt_navigator / controller_server (MPPI)
   |                                         |
   | topic: /predefined_path (Path, viz)     |  then same velocity chain as A1:
   v                                         |  cmd_vel_nav -> cmd_vel_smoothed
 (RViz optional)                             |  -> safety_gate -> /cmd_vel
```

A2 uses **its own** stack copy (`robot_simple_commander` `safety_gate`), not `robot_mppi_navigation`.

---

## 4. Mode B0 — gimbal only

Launch: `ros2 launch robogait_gimbal test_gimbal.launch.py`

```text
nuitrack_person_node                    [external: orbbec_person_publisher]
   |  /orbbec/person_position           (geometry_msgs/Point)
   |  /orbbec/person_center_pixel       (std_msgs/Int32)
   |  /orbbec/num_users                 (std_msgs/Int32)
   v
distance_orbbec
   |  /user_raw                         (robogait_person_interfaces/User)
   |  /person_center_pixel              (std_msgs/Int32)
   +------------+----------------------+
   |            |                      |
   v            v                      |
distance_filter                        |
   |  /user_coordinates                |
   |            |                      |
   |            v                      |
   |      camera_tracker <-------------+  (also subs /person_center_pixel)
   |            |
   |            |  /camera_position     (std_msgs/Float32, degrees)
   |            v
   |      dynamixel_driver <--- also subs /user_raw (lost-person handling)
   |            |
   |            v
   |         Dynamixel servo (camera pan)
   v
 (no /cmd_vel — chassis not driven in B0)
```

---

## 5. Mode B1 / B2 — person follow (pure MPPI)

Person supplies a **goal**; chassis `[v, ω]` still comes only from MPPI.

```text
[same perception + gimbal graph as B0]
                |
                |  /user_coordinates
                v
         person_nav_goal
                |
    +-----------+---------------------------+
    |                                       |
    |  /person_nav_goal                     |  action: navigate_to_pose
    |  (geometry_msgs/PoseStamped, debug)   v
    v                                 bt_navigator
 (RViz optional)                            |
                                            v
                                  planner_server / controller_server (MPPI)
                                            |
                                            |  /cmd_vel_nav
                                            v
                                    velocity_smoother
                                            |
                                            |  /cmd_vel_smoothed
                                            v
                                      safety_gate
                                            |
                                            |  /cmd_vel
                                            v
                                   roverrobotics_driver
```

---

## 6. Our nodes — subscribe / publish table

| Node | Subscribes | Publishes / actions |
|------|------------|---------------------|
| `safety_gate` | `/cmd_vel_smoothed`, `/odometry/filtered`, `/safety_estop` | `/cmd_vel` |
| `idle_check` | `/cmd_vel` | — |
| `reset_rplidar` | — | serial only |
| `record_path` | TF `map`→`base_link` | yaml file |
| `follow_path` | — | action `follow_path`; topic `/predefined_path` |
| `cancel_navigation` | — | cancel Nav2 actions |
| `distance_orbbec` | `/orbbec/person_position`, `/orbbec/person_center_pixel`, `/orbbec/num_users` | `/user_raw`, `/person_center_pixel` |
| `distance_filter` | `/user_raw` | `/user_coordinates` |
| `camera_tracker` | `/user_coordinates`, `/person_center_pixel` | `/camera_position` |
| `dynamixel_driver` | `/camera_position`, `/user_raw` | (Dynamixel serial) |
| `person_nav_goal` | `/user_coordinates` | `/person_nav_goal`; action `navigate_to_pose` |

---

## 7. What the previous Chinese diagram got wrong / incomplete

| Issue | Correction |
|-------|------------|
| Implied “MPPI” as a node | Use `controller_server` (MPPI is the plugin) |
| Incomplete A1 node list | Added lifecycle, smoother_server, waypoint_follower, hardware names |
| `record_path` shown vaguely | It uses **TF**, not odometry topics |
| Missing `nuitrack_person_node` | External, but started by our B launches |
| Not listing all 11 executables | Section 1 is the complete custom list |

When A1 is running, `ros2 node list` will show **many more** names (costmap nodes under namespaces, BT internal nodes, etc.). Section 2 lists the **launch-level** names we start; it is not every subprocess name Nav2 may spawn.

---

## 8. Quick checks

```bash
ros2 pkg executables robot_mppi_navigation
ros2 pkg executables robot_simple_commander
ros2 pkg executables robogait_perception
ros2 pkg executables robogait_gimbal
ros2 pkg executables robogait_person_follow

ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 action list
```
