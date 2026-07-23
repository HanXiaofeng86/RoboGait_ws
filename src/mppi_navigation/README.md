# mppi_navigation

Rover Robotics Mini 的纯 MPPI 导航包（模式 1：RViz 目标；模式 2：预定义路径）。
MPPI 生成完整速度对 (v, ω)，其它节点只允许原样透传或整组置零。

## 控制链路

```text
RViz 2D Goal
    ↓
bt_navigator → planner_server → /plan
    ↓
controller_server (nav2_mppi_controller)  →  /cmd_vel_nav
    ↓
velocity_smoother                          →  /cmd_vel_smoothed
    ↓
safety_gate（仅 透传 / 置零）               →  /cmd_vel
    ↓
roverrobotics_driver → 底盘
    ↓
轮式里程计 + IMU → EKF → /odometry/filtered → MPPI 下一周期
```

## 启动（模式 1）

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 默认：map_server + AMCL + lab2_new（启动后用 2D Pose Estimate 校正初始位姿）
ros2 launch mppi_navigation rviz_goal_navigation.launch.py

# 在线 SLAM（无需已有地图）：
ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True

# 仅验证软件链路时关掉 AMCL（静态 map->odom）：
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False

# 不开 RViz：
ros2 launch mppi_navigation rviz_goal_navigation.launch.py use_rviz:=False
```

注意：布尔参数必须写 `True`/`False`（首字母大写）。

## 启动（模式 2：预定义路径）

先按上面启动整个导航栈，再发送路径（路径只存几何，速度全部由 MPPI 在线计算）：

```bash
# 内置测试路径：straight / left_90 / right_90 / s_curve / obstacle_avoid
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

# 或者直接 ros2 run：
ros2 run mppi_navigation follow_path --ros-args -p path_file:=/绝对路径/my_path.yaml
```

- `frame: local` 的路径以机器人当前位姿为起点自动变换到 map 系
- 路径文件在 `paths/*.yaml`，可用 `python3 scripts/generate_test_paths.py` 重新生成
- 发送的路径同时发布到 `/predefined_path`（RViz 可视化）

取消当前导航（测试 7）：

```bash
ros2 run mppi_navigation cancel_navigation
```

## Launch 文件

| 文件 | 作用 |
|------|------|
| `rviz_goal_navigation.launch.py` | 顶层一键启动（硬件 + 导航 + RViz） |
| `rover_mini_hardware.launch.py` | 底盘驱动 + URDF + 激光 + IMU（Include 上游 mini.launch.py） |
| `nav2_mppi_controller.launch.py` | EKF + 定位（SLAM/AMCL/静态 TF 三选一）+ Nav2 全栈 + 速度链 + 安全闸 |
| `follow_predefined_path.launch.py` | 模式 2：向已运行的栈发送预定义路径（FollowPath/MPPI） |
| `ekf_amcl_localization.launch.py` | 单独启动 EKF（调试用） |

## 配置文件

| 文件 | 作用 |
|------|------|
| `config/nav2_pure_mppi_params.yaml` | Nav2 全部参数；MPPI 基线：20 Hz、time_steps 44、batch 1200、vx_std 0.25、wz_std 0.35、temperature 0.24、gamma 0.022、vx∈[0,1.2]、wz≤2.0，8 个 critics |
| `config/robot_localization_ekf.yaml` | EKF：轮式里程计(vx,vy) + IMU(yaw rate) |
| `config/sensor_topics_frames.yaml` | 话题与 TF frame 对照表 |
| `config/diagnostic_recording_topics.yaml` | 录包话题列表 |

## 安全闸（safety_gate）

- 输入 `cmd_vel_smoothed`，输出 `/cmd_vel`
- 只有两种行为：**原样透传** 或 **输出零速**；永不生成替代指令
- 置零条件：指令超时（0.5 s）、里程计超时（0.5 s）、`/safety_estop` 为 true
- 软件急停：

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"   # 停
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"  # 恢复
```

## 预检与测试工具

```bash
# 话题是否有数据（默认观察 5 秒）
./scripts/preflight_check_topics.sh

# TF 链 map->odom->base_link 是否完整
./scripts/preflight_check_tf.sh

# 测试 1（系统空闲）：无目标时 /cmd_vel 必须保持零
ros2 run mppi_navigation idle_check 10
# 或: python3 -m mppi_navigation.preflight.idle_cmd_vel_check 10

# 雷达 internal error 时软件复位（无需断电）
ros2 run mppi_navigation reset_rplidar

# 录制导航测试包
./scripts/record_navigation_test.sh straight_test

# 分析录包：指标 JSON + 图
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/<bag目录>
```

## MPPI 调参入口

- 基线常量说明：`mppi_navigation/mppi_controller/baseline_params.py`
- 8 个 critic 文档：`mppi_navigation/mppi_controller/critics_reference.py`
- 实际生效参数：`config/nav2_pure_mppi_params.yaml`（改完 `colcon build` 后生效，
  symlink 安装下改 yaml 无需重新编译，重启 launch 即可）

## 自定义 critic（M10）

Nav2 源码已在本工作空间：`src/navigation2/nav2_mppi_controller/`。
新 critic 参考 `src/critics/*.cpp`，在 `critics.xml` 注册，然后：

```bash
colcon build --packages-select nav2_mppi_controller --symlink-install
```

## 与旧 rover_workspace 的区别

- 不迁入人体检测 / Orbbec / 云台 / `robot_controller_remapping`
- 最终 `/cmd_vel` 的 `linear.x` 与 `angular.z` 都来自 MPPI（旧架构中线速度被人距 PID 覆盖）
- 不要同时 source 两个工作空间
