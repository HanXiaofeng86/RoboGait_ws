# 节点与话题订阅图

> 只画本仓库**自研节点**和主链路话题。Nav2 / 驱动等系统节点用 `[系统]` 标出。  
> 箭头：`发布者 --话题--> 订阅者`

---

## 1. 模式 A1：RViz 点目标（纯 MPPI）

入口：`ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py`

```text
[RViz / 用户]
      |  (Nav2 Goal / NavigateToPose action)
      v
[系统] bt_navigator / planner_server
      |
      |  FollowPath（局部路径）
      v
[系统] controller_server (MPPI)
      |
      |  /cmd_vel_nav          (Twist)
      v
[系统] velocity_smoother
      |
      |  /cmd_vel_smoothed     (Twist)
      v
[自研] safety_gate
      |  另订: /odometry/filtered, /safety_estop
      |
      |  /cmd_vel              (Twist)
      v
[系统] roverrobotics_driver
      |
      v
    底盘轮子
```

传感器 / 定位（给 MPPI 用，不直接进 `/cmd_vel`）：

```text
[系统] rplidar -------- /scan ------------> [系统] amcl / costmap
[系统] bno055 --------- /imu/data --------> [系统] ekf_filter_node
[系统] rover 驱动 ----- /odometry/wheels -> [系统] ekf_filter_node
                                              |
                                              |  /odometry/filtered
                                              |  TF: map->odom->base_link
                                              v
                                         MPPI / safety_gate
```

---

## 2. 模式 A2：录轨迹 / 跟轨迹

### 2.1 录轨迹

入口：`ros2 launch robot_simple_commander record_trajectory.launch.py`

```text
[系统] TF map→base_link
      |
      v
[自研] record_path -------- 写入 yaml（recorded/*.yaml）
```

（录制时一般不起 MPPI 跟线；遥控把车开过即可。）

### 2.2 跟轨迹

入口：`ros2 launch robot_simple_commander follow_trajectory.launch.py`

```text
yaml 路径文件
      |
      v
[自研] follow_path ---- FollowPath action ----> [系统] Nav2 + MPPI
                                                      |
                                                      |  之后速度链同 A1：
                                                      |  cmd_vel_nav → smoothed → safety_gate → /cmd_vel
                                                      v
                                                   底盘

[自研] cancel_navigation ---- 取消 NavigateToPose / FollowPath
```

---

## 3. 模式 B0：只测云台（车不动）

入口：`ros2 launch robogait_gimbal test_gimbal.launch.py`

```text
[外部] nuitrack_person_node
      |  /orbbec/person_position      (Point)
      |  /orbbec/person_center_pixel  (Int32)
      |  /orbbec/num_users            (Int32)
      v
[自研] distance_orbbec
      |  /user_raw                 (User)
      |  /person_center_pixel      (Int32)
      +----------+----------------+
      |          |
      v          v
[自研] distance_filter     [自研] camera_tracker
      |  /user_coordinates      |  订 /user_coordinates
      |  (User)                 |  订 /person_center_pixel
      |                         |
      |                         |  /camera_position   (Float32, 度)
      |                         v
      |                   [自研] dynamixel_driver
      |                         |  订 /camera_position
      |                         |  订 /user_raw（丢人时可停）
      |                         v
      |                       Dynamixel 舵机（转相机）
      v
   （B0 不发 /cmd_vel，底盘不动）
```

---

## 4. 模式 B1 / B2：跟人（人→目标→纯 MPPI）

- B1：`ros2 launch robogait_person_follow person_follow.launch.py`（Nav + 人层一起）
- B2：终端1 起 A1 Nav；终端2 `person_layer.launch.py`

```text
                    【感知 + 云台】同 B0
                              |
                              |  /user_coordinates
                              v
                    [自研] person_nav_goal
                              |
              +---------------+----------------+
              |                                |
              |  /person_nav_goal              |  NavigateToPose action
              |  (PoseStamped，调试用)          v
              v                      [系统] bt_navigator
         (RViz 可看)                          |
                                              |  规划路径
                                              v
                                    [系统] MPPI → smoother → safety_gate → /cmd_vel
                                              |
                                              v
                                           底盘

说明：人只提供目标；[v, ω] 全部由 MPPI 计算，无 PID 改速度。
```

合并总览（B 叠在 A 上）：

```text
人/相机 ──> distance_orbbec ──> distance_filter ──> person_nav_goal ──action──> Nav2
                |                      |
                |                      +──> camera_tracker ──> dynamixel_driver ──> 舵机
                +──> (user_raw 也可给舵机看是否丢人)

Nav2/MPPI ──> cmd_vel_nav ──> cmd_vel_smoothed ──> safety_gate ──> /cmd_vel ──> 驱动
```

---

## 5. 自研节点速查表

| 节点 | 包 | 主要订阅 | 主要发布 / 动作 |
|------|-----|----------|-----------------|
| `safety_gate` | `robot_mppi_navigation`（A2 包也有副本） | `cmd_vel_smoothed`, `odometry/filtered`, `safety_estop` | `/cmd_vel` |
| `idle_check` | `robot_mppi_navigation` | `/cmd_vel` | （检查用，不控制车） |
| `reset_rplidar` | `robot_mppi_navigation` | — | 串口复位雷达 |
| `record_path` | `robot_simple_commander` | TF `map→base_link` | 写 yaml |
| `follow_path` | `robot_simple_commander` | — | FollowPath **action** |
| `cancel_navigation` | `robot_simple_commander` | — | 取消导航 **action** |
| `distance_orbbec` | `robogait_perception` | `/orbbec/*` | `user_raw`, `person_center_pixel` |
| `distance_filter` | `robogait_person_follow` | `user_raw` | `user_coordinates` |
| `camera_tracker` | `robogait_gimbal` | `user_coordinates`, `person_center_pixel` | `camera_position` |
| `dynamixel_driver` | `robogait_gimbal` | `camera_position`, `user_raw` | （串口到舵机） |
| `person_nav_goal` | `robogait_person_follow` | `user_coordinates` | `person_nav_goal`；NavigateToPose **action** |

---

## 6. 常用查看命令

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic echo /user_coordinates
ros2 action list
```
