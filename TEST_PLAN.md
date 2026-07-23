# RoboGait 实机测试清单（纯 MPPI 导航）

> **怎么用：** 按阶段往下做；前一阶段不通过，不要进入导航实测。  
> **每个新终端先：**
> ```bash
> cd ~/RoboGait_ws
> source /opt/ros/humble/setup.bash && source install/setup.bash
> ```
> 详细命令解释见 `PROJECT_CHANGES.md` 第四节。

---

## 总览（8 项验收 + 前置）

| 阶段 | 内容 | 目的 | 状态（自行勾） |
|------|------|------|----------------|
| A | 硬件与环境 | 车、传感器、编译可用 | ☐ |
| B | 启动 + 预检 | 全栈起来、话题/TF 正常 | ☐ |
| **T1** | 系统空闲 | 没目标时速度必须为 0 | ☐ |
| **T2** | 直线（模式 1） | RViz Goal 跟直线 | ☐ |
| **T3** | 左 90°（模式 2） | 预定义路径转弯 | ☐ |
| **T4** | 右 90°（模式 2） | 与 T3 对称 | ☐ |
| **T5** | S 曲线（模式 2） | 连续弯道跟踪 | ☐ |
| **T6** | 静态避障（模式 2） | 路径上有障碍能绕开 | ☐ |
| **T7** | 取消 + 急停 | 安全闸真正起作用 | ☐ |
| **T8** | 控制频率 | 约 20 Hz、少丢帧 | ☐ |

**两种导航测法：**
- **模式 1**：RViz「Nav2 Goal」点目标（T2）
- **模式 2**：`follow_predefined_path` 发 yaml 路径（T3–T6）；全栈须已在跑

---

## 阶段 A：硬件与环境（前置）

### A.1 设备与 CAN（开机 / 重启机器人后必做）

```bash
ls -l /dev/rplidar /dev/bno055          # 应有软链接
ip link show can0                       # 须 state UP；若 DOWN：
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
candump -n 10 -T 3000 can0              # 应有帧
```

| # | 操作 | 通过标准 |
|---|------|---------|
| A.1.1 | 底盘供电、急停松开、CAN 线正常 | can0 UP + candump 有数据 |
| A.1.2 | 雷达异常（health 2）时复位 | `ros2 run mppi_navigation reset_rplidar` 后重启驱动 |
| A.1.3 | IMU / USB 重插后旧节点报 Errno 5 | **先 Ctrl+C 停干净再 launch**，不要沿用旧进程 |

### A.2 只测传感器（可选，排障时用）

```bash
ros2 launch mppi_navigation rover_mini_hardware.launch.py
# 另开终端（Humble：hz 一次只能一个话题）
ros2 topic hz /scan              # ~10 Hz
ros2 topic hz /imu/data          # ~50–60 Hz
ros2 topic hz /odometry/wheels   # ~15–30 Hz
```

### A.3 编译

```bash
cd ~/RoboGait_ws
colcon build --packages-select mppi_navigation --symlink-install
source install/setup.bash
ros2 pkg executables mppi_navigation
```

**通过标准：** 编译无错；可执行程序含  
`safety_gate`、`follow_path`、`cancel_navigation`、`idle_check`、`reset_rplidar`。

---

## 阶段 B：启动全栈 + 预检

### B.1 启动（默认：AMCL + 地图 lab2_new + RViz）

```bash
ros2 launch mppi_navigation rviz_goal_navigation.launch.py
```

常用变体：

```bash
# 仅验证软件、不要真实定位时
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False

# 边走边建图（覆盖 AMCL）
ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True
```

**启动通过标准：**
- [ ] 无明显红色 FATAL（底盘 stale → 回 A.1 查 can0）
- [ ] 日志有 lifecycle / controller（MPPI）激活相关信息
- [ ] RViz 能打开；默认 AMCL 下先用「2D Pose Estimate」标初始位姿

### B.2 预检

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/preflight_check_topics.sh
./scripts/preflight_check_tf.sh
ros2 topic list | grep cmd_vel
# 应有：/cmd_vel  /cmd_vel_nav  /cmd_vel_smoothed

ros2 topic info /cmd_vel --verbose
# Publisher 必须只有 safety_gate 一个
```

**预检通过标准：** 话题脚本与 TF 脚本均为 exit 0；三个 cmd_vel 话题齐全；`/cmd_vel` 仅安全闸发布。

> `[FAIL] /map`：map_server 常只发一次。用 `ros2 topic echo /map --once` 能出数据，且 TF `map→odom` 已 OK，可继续。

**定位观感（默认 AMCL）：**
- [ ] 地图出现；激光与墙大致对齐（标完 2D Pose Estimate 后更好）
- [ ] 粒子云收敛

---

## 测试 T1：系统空闲（必须先过）

不发任何目标：

```bash
ros2 run mppi_navigation idle_check 10
```

| 通过标准 |
|---------|
| 10 s 内 `/cmd_vel` 无非零；车不动；exit code 0 |

---

## 测试 T2：直线（模式 1 · RViz Goal）

每次正式跑建议先录包（新终端）：

```bash
cd ~/RoboGait_ws/src/mppi_navigation
./scripts/record_navigation_test.sh straight_run1
```

在 RViz 用 **Nav2 Goal** 点正前方约 **3 m**。建议重复 3 次。跑完 Ctrl+C 结束录包，再分析：

```bash
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/straight_run1
```

| 通过标准 | 量化（参考） |
|---------|--------------|
| 平滑起步、直线、到点减速停 | 横向误差 RMSE **&lt; 0.1 m** |
| 无来回振荡、无突然倒车 | 控制约 **20 Hz**，jerk 无异常尖峰 |
| 到点后 `/cmd_vel` 归零 | — |

---

## 测试 T3 / T4 / T5：转弯与 S 弯（模式 2）

前提：阶段 B 的全栈仍在跑；车在空旷起点。每条路径建议单独录包。

```bash
cd ~/RoboGait_ws/src/mppi_navigation

# 示例：左 90°（T3）
./scripts/record_navigation_test.sh left90_run1
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

# T4 / T5 换名字与 path_name：
# path_name:=right_90
# path_name:=s_curve
```

路径以**当前车位姿**为起点变到 map 系。RViz 可显示 `/predefined_path`。

| 项 | 通过标准 |
|----|---------|
| T3 左 90° | 转角减速合理；不原地打转 / 大幅超调 |
| T4 右 90° | 与左侧行为大致对称 |
| T5 S 曲线 | 连续平滑；不切内弯失控 |
| 共同 | `follow_path` 结束为 **status 4（SUCCEEDED）** |
| 量化 | 横向 RMSE **&lt; 0.15 m**（弯道可略宽）；航向误差不发散 |

---

## 测试 T6：静态避障（模式 2）

在 `obstacle_avoid` 直线中间放纸箱（比车宽）：

```bash
./scripts/record_navigation_test.sh obstacle_run1
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=obstacle_avoid
```

| 通过标准 |
|---------|
| 局部代价地图能看见障碍（膨胀约 0.4 m） |
| 减速绕开、保持安全距离，绕回后继续 |
| 无碰撞、无长时间卡死 |

---

## 测试 T7：取消导航 + 急停（安全）

在**正在导航**时测（模式 1 或 2 均可）。

### T7.1 取消目标

```bash
ros2 run mppi_navigation cancel_navigation
```

| 通过标准 |
|---------|
| 约 **0.5 s** 内平滑停住；`/cmd_vel` 全零 |

### T7.2 软件急停

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
# 恢复：
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

| 通过标准 |
|---------|
| 急停后 `/cmd_vel` 立即全零；解除后重新发目标仍可导航 |

### T7.3 指令源中断（看门狗）

```bash
# 导航中模拟指令流断开（谨慎操作）
ros2 lifecycle set /velocity_smoother 4
```

| 通过标准 |
|---------|
| 断流后约 **0.5 s** 内安全闸补发零速，车停 |

---

## 测试 T8：控制频率（用已有 bag）

任选 T2–T6 的录包：

```bash
python3 ~/RoboGait_ws/src/mppi_navigation/scripts/analyze_navigation_bag.py \
  ~/RoboGait_ws/bags/<bag目录名>
```

| 通过标准 |
|---------|
| `/cmd_vel` 平均周期 ≈ **50 ms（20 Hz）** |
| 最大周期 **&lt; 100 ms**；周期 &gt;75 ms 的丢帧占比 **&lt; 5%** |
| 不达标可先降 MPPI `batch_size`（1200→800）或 `time_steps`（44→32）再测 |

---

## 收尾

- [ ] 所有 bag 在 `~/RoboGait_ws/bags/`，各跑一遍分析，保留 `metrics.json` 与图
- [ ] 失败现象记到 `PROJECT_CHANGES.md` **第五节待办**
- [ ] 全部通过后：再按调参顺序微调（探索噪声 → 预测域 → 权重 → 路径/障碍 critics）

---

## 汇报用：当前进度怎么说

| 已具备（代码 / 工具） | 本清单待实跑勾选 |
|----------------------|------------------|
| 模式 1/2 launch、安全闸、EKF、默认 AMCL | T1–T8 按表打勾 |
| 预检 / 录包 / 分析 / idle_check / reset_rplidar | 每项附 bag 或截图更有说服力 |
| 传感器检查流程已验证可通 | 以当天复测频率为准 |

**建议实机顺序（省时间）：** A → B → **T1** → **T2** → **T7**（安全）→ T3/T4 → T5 → T6 → T8。

---

## 常见问题速查

| 现象 | 先查 |
|------|------|
| 底盘 FATAL / data stale | `can0` 是否 UP；candump；车是否上电（不是雷达/IMU 问题） |
| 车完全不动 | 依次看 `/cmd_vel_nav` → `/cmd_vel_smoothed` → `/cmd_vel` 断在哪 |
| 安全闸一直 ZERO | `/odometry/filtered` 是否超时；是否忘了把 estop 设回 false |
| TF / map 异常 | AMCL 是否起来；是否先做了 2D Pose Estimate；`/scan` 是否有数据 |
| Goal 一发就被拒 | `ros2 lifecycle get /bt_navigator` 是否 active |
| IMU Errno 5 / 雷达没 `/scan` | 停干净旧 launch，必要时 `reset_rplidar`，再重新启动 |
| 弯道超调大 | 记现象与 bag，属调参范围，不必当场死磕 |
