# RoboGait 实机测试计划（纯 MPPI）

> **用法：** 按编号顺序做；前一步不过，不要跳到后面。  
> **每个新终端先执行：**
> ```bash
> cd ~/RoboGait_ws
> source /opt/ros/humble/setup.bash && source install/setup.bash
> ```
> 命令细节见手册：`PROJECT_CHANGES.md`。  
> **模式 A**（纯机器人）：`robot_mppi_navigation`、`robot_simple_commander` —— 速度由 **MPPI** 出。  
> **模式 B**（跟人）：`robogait_gimbal`、`robogait_perception`、`robogait_person_follow` —— 人只发目标，速度仍全是 **MPPI**。

---

## 编号说明

| 前缀 | 含义 |
|------|------|
| **0.x / 1.x** | 准备阶段（硬件、起栈） |
| **T1–T8** | 模式 A：导航验收 |
| **B0 / B1 / B2** | 模式 B：云台 → 跟人（B = 模式 B，不是「阶段 B」） |

**建议顺序：** 0 → 1 → T1 → T2 → T7 → T3/T4 → T5 → T6 → T8 → **B0** → **B1 或 B2**。

---

## 总览

| 编号 | 内容 | 目的 | 完成 |
|------|------|------|------|
| **0** | 硬件与环境 | 车、传感器、编译 OK | ☐ |
| **1** | 起栈 + 预检 | 导航栈起来；话题/TF OK | ☐ |
| **T1** | 空闲 | 没目标时速度必须为 0 | ☐ |
| **T2** | 直线（模式 A1） | RViz 点目标直行 | ☐ |
| **T3** | 左 90°（模式 A2） | 跟路径（MPPI） | ☐ |
| **T4** | 右 90°（模式 A2） | 与 T3 对称 | ☐ |
| **T5** | S 弯（模式 A2） | 连续弯道跟踪 | ☐ |
| **T6** | 静态障碍（模式 A2） | 路径上避障 | ☐ |
| **T7** | 取消 + 急停 | 安全闸有效 | ☐ |
| **T8** | 控制频率 | 约 20 Hz，少丢帧 | ☐ |
| **B0** | 只测云台 | 舵机跟人像素，**车不动** | ☐ |
| **B1** | 跟人（一站） | 人→目标→**纯 MPPI** | ☐ |
| **B2** | 跟人（两终端） | 同 B1，分层调试 | ☐ |

---

## 代码是否准备好？

| 项 | 状态 |
|----|------|
| 模式 A 导航 / 录跟路径 | 已有 launch、安全闸、预检脚本 |
| B0 `test_gimbal` | **已就绪**（感知 + 云台，无 Nav2） |
| B1/B2 `person_nav_goal` | **已就绪**（人→`NavigateToPose`，无 PID 混速） |
| 外部：`orbbec_person_publisher`（Nuitrack） | 常在 `~/rover_workspace`，测 B 前需 `source` |
| 外部：Dynamixel 串口 | 改 `src/robogait_gimbal/config/gimbal.yaml` 的 `device_name` |

测模式 B 前编译：

```bash
cd ~/RoboGait_ws
colcon build --packages-select \
  robogait_person_interfaces robogait_perception robogait_gimbal robogait_person_follow \
  robot_mppi_navigation --symlink-install
source install/setup.bash
# 若 Nuitrack 在旧工作空间：
# source ~/rover_workspace/install/setup.bash
```

---

## 0 — 硬件与环境

### 0.1 设备与 CAN（开机 / 上电后）

```bash
ls -l /dev/rplidar /dev/bno055          # 符号链接应存在
ip link show can0                       # 须为 UP；若 DOWN：
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
candump -n 10 -T 3000 can0              # 应有帧
```

| 编号 | 操作 | 通过标准 |
|------|------|----------|
| 0.1.1 | 底盘上电、急停松开、CAN 正常 | can0 UP + candump 有数据 |
| 0.1.2 | 雷达 health 2 → 复位 | `ros2 run robot_mppi_navigation reset_rplidar`，再重启驱动 |
| 0.1.3 | 拔插 USB 后 IMU Errno 5 | **Ctrl+C 干净停掉再起**，勿留僵尸节点 |

### 0.2 只开传感器（可选）

```bash
ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py
# 另一终端（Humble：一次 hz 一个话题）
ros2 topic hz /scan              # ~10 Hz
ros2 topic hz /imu/data          # ~50–60 Hz
ros2 topic hz /odometry/wheels   # ~15–30 Hz
```

### 0.3 编译（模式 A）

```bash
cd ~/RoboGait_ws
colcon build --packages-select robot_mppi_navigation robot_simple_commander --symlink-install
source install/setup.bash
ros2 pkg executables robot_mppi_navigation
ros2 pkg executables robot_simple_commander
```

**通过：** 编译成功；`safety_gate` / `idle_check` / `reset_rplidar` 与 `record_path` / `follow_path` / `cancel_navigation` 都在。

---

## 1 — 起栈 + 预检（模式 A）

### 1.1 启动（默认 AMCL + `lab2_new` + RViz）

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

变体：

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py amcl:=False
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py slam:=True
```

**通过：**
- [ ] 无红色 FATAL（底盘 stale → 查 0.1 的 can0）
- [ ] 日志里 lifecycle / controller（MPPI）已 active
- [ ] RViz 打开；默认 AMCL 时先用 **2D Pose Estimate** 标位姿

### 1.2 预检

```bash
cd ~/RoboGait_ws/src/robot_mppi_navigation
./scripts/preflight_check_topics.sh
./scripts/preflight_check_tf.sh
ros2 topic list | grep cmd_vel
# 期望：/cmd_vel  /cmd_vel_nav  /cmd_vel_smoothed

ros2 topic info /cmd_vel --verbose
# 发布者应只有 safety_gate
```

**通过：** 两脚本退出码 0；三条 cmd_vel 都在；`/cmd_vel` 只有 safety_gate 发布。

> `[FAIL] /map`：map_server 常只发一次。若 `ros2 topic echo /map --once` 有数据且 TF `map→odom` 正常，可继续。

**定位观感（默认 AMCL）：**
- [ ] 能看到地图；点云大致贴墙（2D Pose Estimate 后更好）
- [ ] 粒子云收敛

---

## T1 — 空闲（必须先过）

不起目标：

```bash
ros2 run robot_mppi_navigation idle_check 10
```

| 通过标准 |
|----------|
| 10 秒内 `/cmd_vel` 无非零；车不动；退出码 0 |

---

## T2 — 直线（模式 A1 · RViz Goal）

正式跑可另开终端录 bag：

```bash
cd ~/RoboGait_ws/src/robot_mppi_navigation
./scripts/record_navigation_test.sh straight_run1
```

RViz 里 **Nav2 Goal** 指前方约 **3 m**。建议重复 3 次。Ctrl+C 停 bag 后：

```bash
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/straight_run1
```

| 通过（观感） | 指标（参考） |
|--------------|--------------|
| 起步平滑、直行、减速停车 | 横向 RMSE **&lt; 0.1 m** |
| 无振荡 / 突然倒车 | 控制约 **20 Hz**，无狂抖 |
| 到点后 `/cmd_vel` 归零 | — |

---

## T3 / T4 / T5 — 转弯与 S 弯（模式 A2）

前提：阶段 1 的栈仍在跑；前方空旷。每条路径录一个 bag。

```bash
cd ~/RoboGait_ws/src/robot_mppi_navigation

# 例：左 90°（T3）
./scripts/record_navigation_test.sh left90_run1
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=left_90

# T4 / T5：改 bag 名和 path_name
# path_name:=right_90
# path_name:=s_curve
```

路径从**当前**车位进 `map`。RViz 可显示 `/predefined_path`。

| 编号 | 通过标准 |
|------|----------|
| T3 左 90° | 弯前合理减速；无原地空转 / 大超调 |
| T4 右 90° | 大致与左对称 |
| T5 S 弯 | 跟踪平滑；不切弯失败 |
| 全部 | `follow_path` 结束为 **status 4 (SUCCEEDED)** |
| 指标 | 横向 RMSE **&lt; 0.15 m**（弯道可略松）；航向误差不发散 |

---

## T6 — 静态障碍（模式 A2）

在 `obstacle_avoid` 路径中途放箱子（宽于车）：

```bash
./scripts/record_navigation_test.sh obstacle_run1
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=obstacle_avoid
```

| 通过标准 |
|----------|
| 局部代价地图能看到障碍（约 0.4 m 膨胀） |
| 减速、留间隙、绕过后续走 |
| 不撞、不长时间卡死 |

---

## T7 — 取消 + 急停（安全）

在**正在导航**时做（A1 或 A2）。

### T7.1 取消目标

```bash
ros2 run robot_simple_commander cancel_navigation
```

| 通过标准 |
|----------|
| 约 **0.5 s** 内平滑停下；`/cmd_vel` 全零 |

### T7.2 软件急停

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
# 解除：
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

| 通过标准 |
|----------|
| 急停立刻 `/cmd_vel` 归零；解除后新目标仍可用 |

### T7.3 指令流中断（看门狗）

```bash
# 导航中 — 谨慎使用
ros2 lifecycle set /velocity_smoother 4
```

| 通过标准 |
|----------|
| 约 **0.5 s** 内安全闸发零；车停 |

---

## T8 — 控制频率（用已有 bag）

任取 T2–T6 的 bag：

```bash
python3 ~/RoboGait_ws/src/robot_mppi_navigation/scripts/analyze_navigation_bag.py \
  ~/RoboGait_ws/bags/<bag_name>
```

| 通过标准 |
|----------|
| `/cmd_vel` 平均周期约 **50 ms（20 Hz）** |
| 最大周期 **&lt; 100 ms**；周期 &gt;75 ms 的占比 **&lt; 5%** |
| 不达标：可试降低 MPPI `batch_size`（1200→800）或 `time_steps`（44→32） |

---

## B0 / B1 / B2 — 模式 B（云台与跟人）

### 测前准备（B0～B2 共用）

1. Orbbec + Nuitrack 可用（常需）：
   ```bash
   source ~/rover_workspace/install/setup.bash
   source ~/RoboGait_ws/install/setup.bash
   ```
2. 舵机串口（按实际改）：
   - 文件：`src/robogait_gimbal/config/gimbal.yaml` → `device_name`
   - 或 launch：`device_name:=/dev/ttyUSB0`（或你的 `/dev/serial/by-id/...`）
3. 权限：串口用户在 `dialout` 组；舵机供电正常。

---

### B0 — 只测云台（先做）

**不起底盘 Nav2**，只验证：看见人 → 像素 PID → 舵机转，把人拉回画面中心。

```bash
ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0
```

另开终端看话题：

```bash
ros2 topic echo /orbbec/num_users --once
ros2 topic hz /person_center_pixel
ros2 topic echo /camera_position
ros2 topic echo /user_raw
```

| 编号 | 操作 | 通过标准 |
|------|------|----------|
| B0.1 | 人站在镜头前 | `num_users`≥1；`user_raw.detection=1`；`z` 合理（约 1–5 m） |
| B0.2 | 人左右移动 | `person_center_pixel` 变化；`camera_position` 有角度 |
| B0.3 | 舵机响应 | 相机跟着人转，人回到画面中心附近；无狂抖 |
| B0.4 | 人离开 | 一段时间后云台不再猛追（或停在限位内） |

**失败先查：** Nuitrack 是否 source；串口是否对；`dxl_id` / 波特率；舵机是否上电。

---

### B1 — 跟人一站式（人→目标→MPPI）

前提：**B0 已通过**；建议 T1、T2 已通过（定位与 MPPI 可靠）。  
空旷场地；先 **2D Pose Estimate**（若用 AMCL）。

```bash
ros2 launch robogait_person_follow person_follow.launch.py
# 不要 RViz 时可： use_rviz:=False
```

观察：

```bash
ros2 topic echo /person_nav_goal
ros2 topic hz /cmd_vel
# /cmd_vel 发布者仍应是 safety_gate；速度来自 MPPI 链路
```

| 编号 | 操作 | 通过标准 |
|------|------|----------|
| B1.1 | 人站在目标间距附近（默认约 **2.4 m**） | 有 `person_nav_goal`；车不无意义乱窜 |
| B1.2 | 人远离 | 车前进跟上，大致保持间距；**线速度+角速度都由 MPPI** |
| B1.3 | 人靠近 | 车后退或停下，不贴脸撞人 |
| B1.4 | 人左右走 | 车转向跟随目标；云台可同时转（若 `use_gimbal:=true`） |
| B1.5 | 人消失 | 目标取消；`/cmd_vel` 归零，车停 |

间距参数：`src/robogait_person_follow/config/person_follow.yaml` → `target_distance_m`。

---

### B2 — 两终端跟人（分层调试）

与 B1 行为相同，方便单独重启人层：

```bash
# 终端 1
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py

# 终端 2（source 同上，含 rover_workspace 若需要）
ros2 launch robogait_person_follow person_layer.launch.py device_name:=/dev/ttyUSB0
```

**通过标准：** 与 B1.1–B1.5 相同。

---

## 收尾

- [ ] bag 放在 `~/RoboGait_ws/bags/`；分析保留 `metrics.json` / 图
- [ ] 失败项记到 `PROJECT_CHANGES.md` 第五节待办
- [ ] 模式 A 全过后：再调 MPPI（探索噪声 → 时域 → 权重 → critic）
- [ ] 模式 B：先 B0，再 B1 或 B2；保距以后再考虑做成 MPPI critic

---

## 进度备忘（汇报用）

| 已有（代码 / 工具） | 仍需实机勾选 |
|--------------------|--------------|
| 模式 A1/A2 launch、安全闸、EKF、默认 AMCL | T1–T8 |
| 预检 / bag / 分析 / idle_check / reset_rplidar | 每次试验的 bag 或截图 |
| B0 `test_gimbal`、B1/B2 `person_nav_goal`（纯 MPPI） | B0 → B1/B2 |

**顺序：** 0 → 1 → T1 → T2 → T7 → T3/T4 → T5 → T6 → T8 → B0 → B1/B2。

---

## 故障速查

| 现象 | 先查 |
|------|------|
| 底盘 FATAL / data stale | can0 UP？candump？车上电？（不是雷达/IMU） |
| 车一直不动 | 跟 `/cmd_vel_nav` → `/cmd_vel_smoothed` → `/cmd_vel` |
| 安全闸一直 ZERO | `/odometry/filtered` 超时？急停忘关？ |
| TF / 地图问题 | AMCL 起了？2D Pose Estimate？`/scan` 有数据？ |
| 目标立刻被拒 | `ros2 lifecycle get /bt_navigator` 是否 active？ |
| IMU Errno 5 / 无 `/scan` | 停旧 launch；必要时 `reset_rplidar`；再起 |
| 弯道大超调 | 录 bag 再调参——别卡死整场试验 |
| B0 看不见人 | `source rover_workspace`？Nuitrack？Orbbec USB？ |
| B0 舵机不动 | `device_name`、供电、`dxl_id`、波特率 |
| B1 有人但不走 | Nav2 是否 active？`person_nav_goal` 有没有发？map→base_link TF？ |
