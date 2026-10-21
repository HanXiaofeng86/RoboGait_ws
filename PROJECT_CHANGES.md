# RoboGait_ws 项目手册（给小白看）

> **中文主文档**（与 `TEST_PLAN.md` 为工作空间仅有的中文手册）。英文版：`PROJECT_CHANGES_EN.md`（须同步）。  
> 包内 `README.md`、launch 说明、`NODES_AND_TOPICS.md` 等其余文档一律用**英文**。  
> 写法：先总览，再按**包**把每个重要文件说清楚——**是干什么的、平时要不要改、想改什么功能去哪个文件**。

---

## 一、先搞懂这两句话

**包名前缀（看名字就分清）：**

| 前缀 | 意思 |
|------|------|
| **`robot_*`** | 纯机器人（没人） |
| **`robogait_*`** | 跟人（有人） |
| 其它 | 驱动 / Nav2 第三方 |

**模式 A — 纯机器人**（`robot_*`）

1. **`robot_mppi_navigation`** = **A1**：RViz 点目标。硬件 + Nav2 + MPPI + 安全闸（**用地图定位**，不做建图入口）。  
2. **`robot_slam_mapping`** = **建图**：在线 SLAM + 手柄；扫完存图后再用 A1。  
3. **`robot_simple_commander`** = **A2**：手柄**录轨迹** / **自动跟随**（跟线时 RViz 左边 **开始 / 回到起点 / 停止**）。  
   **与 A1 无依赖**（自带栈副本，允许代码重复）。

**模式 B — 跟人**（`robogait_*`，Python）

4. **`robogait_person_interfaces`** = 消息 `User.msg`（检测 + xyz）。  
5. **`robogait_perception`** = 看人（**云台/步态骨架用 Nuitrack**；SDK 只给老师看点云）。  
6. **`robogait_gimbal`** = 云台（水平 + 俯仰，像素/高度 PID + 两颗 Dynamixel）。  
7. **`robogait_person_follow`** = 两条互斥的跟人线。**B1/B2**：人 → Nav2 目标，底盘速度仍全由 MPPI。**B3**：直线跟随，不启 Nav2，只用距离控制前后。

```text
模式 A（robot_* 纯机器人）              模式 B（robogait_* 跟人）
  A1 robot_mppi_navigation（导航）         B0 云台 → robogait_gimbal
  建图 robot_slam_mapping                 B1 一站跟人 / B2 叠人层（MPPI）
  A2 robot_simple_commander               B3 直线跟随（无 Nav2，云台冻住）
B1/B2 的底盘 [v,ω] 只来自 MPPI。B3 单独发前后速度，仍经 safety_gate。不要两套同时起。
```

每个新终端先执行：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
```

布尔参数必须写成 `True` / `False`（首字母大写）。

### 文件夹命名约定（全工作空间统一）

| 文件夹名 | 只放什么 | 不要放什么 |
|----------|----------|------------|
| **`launch/`** | `ros2 launch` **入口** `*.launch.py` | 节点源码、yaml 参数 |
| **`launch/parts/`** | 被入口拼装的**片段**（大栈才用） | 日常入口；Mode B 可不用 |
| **`config/`** | 参数 `*.yaml` | 路径几何、地图图片 |
| **`maps/`** | 地图 yaml/pgm | 轨迹路径 |
| **`paths/`** | 内置考试路径几何 | 手动录制文件 |
| **`recorded/`** | 手动录的轨迹 | 内置考试路径 |
| **`scripts/`** | 离线脚本（`python3 scripts/...`） | `ros2 run` 节点 |
| **`test/`** | 单元测试 | 实机 launch |
| **`<pkg>/nodes/`** | **只有** `ros2 run` 的节点 | 库函数、launch |
| **`<pkg>/lib/`** | 共享库（读 yaml、生成路径等） | 节点入口 |
| **`<pkg>/safety/`** | 安全闸业务逻辑 | 其它功能 |

> 记法：**看文件夹名就知道文件类型**。节点 → `nodes/`；库 → `lib/`；启动 → `launch/`；参数 → `config/`。

---

## 二、包 `robot_mppi_navigation`（导航栈）

目录：`src/robot_mppi_navigation/`。

### 2.1 这个包干什么

| 做 | 不做 |
|----|------|
| 开硬件、定位、Nav2、**MPPI**、平滑、安全闸 | **建图入口**（见包 `robot_slam_mapping`） |
| 预检、录诊断 bag、雷达复位；手柄用本包 `mini_teleop` | **模式 2** 录/跟路径（另一个包，Nav2 自带；硬件走本包 `rover_mini_hardware` 做雷达复位） |

日常一键：

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

---

### 2.2 `launch/` — 启动文件（总开关）

> 小白记法：launch = 「一键开哪些程序」。日常只记顶层那一个。

| 文件 | 是干什么的 | 你什么时候用 / 改 |
|------|------------|-------------------|
| **`rviz_goal_navigation.launch.py`** | **顶层一键**：先硬件，等 `/scan`+轮速后再开 Nav2/RViz；摆 2D Pose 会自动取消旧导航目标 | **每天开车用这个** |
| **`nav2_mppi_controller.launch.py`** | 导航「大脑」：EKF、地图/AMCL/SLAM、Nav2（含 **MPPI**）、速度平滑、安全闸；指定本包行为树 | 一般**不要单独开**（顶层已经 Include）；日常建图用 `robot_slam_mapping` 轻量栈 |
| **`rover_mini_hardware.launch.py`** | 只开底盘、雷达、IMU、URDF；先软复位雷达，再等 `can0` 有帧。IMU 由厂商 `accessories.launch.py` 拉起（`accessories.yaml` 里 `bno055.active: true`，20 Hz） | **只测传感器/底盘**；`reset_lidar:=False` 可跳过复位 |
| **`mini_teleop.launch.py`** | 上面那套硬件 + PS4 手柄（先复位雷达） | **要用手柄时开这个**；不要直接开厂商 `roverrobotics_driver mini_teleop`（不复位，雷达会 health 2 挂掉） |
| **`ekf_amcl_localization.launch.py`** | 调试用，主要起 EKF | 日常不用 |

顶层常用参数（加在 `rviz_goal_navigation` 后面；**建图请用包 `robot_slam_mapping`**）：

| 参数 | 意思（人话） |
|------|----------------|
| （默认） | 用地图 + AMCL 定位，开 RViz |
| `amcl:=False` | 不做真定位，只测软件通不通 |
| `use_rviz:=False` | 不开界面 |
| `map:=/绝对路径/xxx.yaml` | 换一张地图 |

```text
日常导航: robot_mppi_navigation / rviz_goal_navigation
建图:     robot_slam_mapping / slam_mapping
```

**坑：** 顶层已经开着时，不要再单独开一遍 `nav2_mppi_controller`，会起两套导航。

---

### 2.3 `config/` — 参数旋钮（改行为优先来这里）

> 小白记法：改「车开多快、怎么避障、定位松紧」→ 改 yaml，**通常不用改 Python**；改完**重启 launch** 即可（用了 `--symlink-install` 时）。

| 文件 | 是干什么的 | 想改什么就打开它 |
|------|------------|------------------|
| **`nav2_pure_mppi_params.yaml`** | **最重要**。Nav2 + **MPPI** + AMCL 等（内含 slam 段仅作 `slam:=True` 备用）。室内 Mini：纯 MPPI、`vx_max` **1.00（上限不是恒速）**、禁止倒车；规划用 A*、不穿未知格；局部膨胀 0.28、雷达忽略 0.40 m 内（**不要改回 0**） | 车速、避障、日常定位……**先改这个**（`vx_max` + 平滑器 `max_velocity`/`max_accel` 要一起改）；**日常建图参数改 `robot_slam_mapping` 的 mapper yaml** |
| **`robot_localization_ekf.yaml`** | 轮速 + IMU 角速度融合成 `/odometry/filtered`。IMU 必须在；驱动丢掉乱的角速度，避免把车拧转。`transform_time_offset` 让 `odom` TF 稍提前 | 车在图上「自己转」、直线走歪，或**再摆 2D Pose 后红激光消失**时 |
| **`sensor_topics_frames.yaml`** | 对照表：哪个话题、哪个坐标系叫什么（给人看 + 预检用） | 对不上话题名/坐标系名时查阅；一般**不改也能跑** |
| **`diagnostic_recording_topics.yaml`** | 录测试 bag 时要录哪些话题 | 想多录/少录一些话题做分析时 |

---

### 2.4 `maps/` — 地图

| 文件 | 是干什么的 | 小白注意 |
|------|------------|----------|
| **`名字.yaml`** + **`名字.pgm`** | **导航只要这一对**。yaml 写分辨率/原点，pgm 是黑白格子图 | 必须同目录、同名；launch 的 `map:=` 指 yaml |
| `lab2_new` / `mini_lab1`–`3` / `mini_slab1` | 实验室扫过的图（各一对 yaml+pgm） | 日常默认是 `mini_slab1.yaml`；换图改 `map:=` |
| ~~`.posegraph` / `.data`~~ | RViz **Serialize Map** 才写出，给 slam_toolbox 续扫用 | **AMCL 导航不用**；已从本目录删掉，以后也不要再存 |

---

### 2.5 `rviz/` — 可视化界面配置

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`rviz_goal_navigation.rviz`** | 打开 RViz 时显示哪些东西（地图、激光、路径、Nav2 面板、Goal 工具等）。左边有 **Speed** 滑条，松手改本次导航的最高前进速度（0.20–1.00 m/s） | 想默认多显示一条话题、改 Fixed Frame 等。激光大红点、**Decay=0**；默认关掉 TF 坐标轴以免刷屏。要看车身：左边加 **RobotModel**，话题 `/robot_description`（Mini URDF 已含相机）。滑条逻辑在 `robot_simple_commander` 的 `speed_panel.cpp` |

`behavior_trees/navigate_to_pose_real_robot.xml`：点了 Goal 之后「先规划再跟路径」。规划失败时**等待再试**，不先原地转/倒车，也**不要每次清空整张全局地图**（会把 Humble 规划器卡死）。想改恢复动作就改这个文件。

---

### 2.6 `scripts/` — 给你直接敲的脚本

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`preflight_check_topics.sh`** | 查雷达、IMU、轮速、EKF 等话题有没有数据 | 栈起来后、发目标前跑一遍 |
| **`preflight_check_tf.sh`** | 查 `map→odom→base_link` 坐标变换通不通 | 车在图上不动/乱跳时先查 |
| **`record_navigation_test.sh`** | 按 `diagnostic_recording_topics.yaml` 录 rosbag | `./scripts/record_navigation_test.sh 测试名`，Ctrl+C 结束 |
| **`analyze_navigation_bag.py`** | 读 bag，算误差/平滑度/频率并出图 | `python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/某目录` |
| **`reset_rplidar.py`** | 给雷达发串口复位（health 报错时） | 也可用 `ros2 run robot_mppi_navigation reset_rplidar` |
| **`teleop.sh`** | 键盘遥控（自动 remap 到安全闸前） | 也可用 `ros2 run robot_mppi_navigation teleop` |

---

### 2.7 Python 源码 `robot_mppi_navigation/`（按文件夹）

> `__init__.py`：只是告诉 Python「这是一个包」，**不用改**。下面不逐个解释。

#### `nodes/` — 命令行入口（很薄）

> 小白记法：这里像「遥控器按键」，真正逻辑在别的文件夹。

| 文件 | 对应命令 | 真正干活的地方 |
|------|----------|----------------|
| `run_cmd_vel_safety_gate.py` | `ros2 run robot_mppi_navigation safety_gate` | → `safety/` |
| `run_idle_cmd_vel_check.py` | `ros2 run robot_mppi_navigation idle_check` | → `preflight/` |
| `run_reset_rplidar.py` | `ros2 run robot_mppi_navigation reset_rplidar` | → `tools/` |
| `run_wait_for_can.py` | `ros2 run robot_mppi_navigation wait_for_can` | → `tools/`（launch 里自动跑） |
| `run_wait_for_nav_sensors.py` | `ros2 run robot_mppi_navigation wait_for_nav_sensors` | → `tools/`（等雷达+轮速再开 Nav2） |
| `run_scan_viz.py` | `ros2 run robot_mppi_navigation scan_viz` | → `tools/`（给 RViz 用的 `/scan_viz`） |
| `run_cancel_nav_on_pose.py` | `ros2 run robot_mppi_navigation cancel_nav_on_pose` | → `tools/`（摆 2D Pose 时取消旧目标） |
| `run_teleop.py` | `ros2 run robot_mppi_navigation teleop` | 封装键盘遥控 remap |

#### `safety/` — 发给轮子前的最后一道闸

> 规矩：**只能原样放行，或改成全零**。不会自己另算一条速度去开。

| 文件 | 是干什么的 | 想改什么就动它 |
|------|------------|----------------|
| **`cmd_vel_safety_gate.py`** | 安全闸**节点本体**：听平滑后的速度 + 里程计 + 急停话题；决定发给底盘还是发 0；导航程序挂了也会补发 0 停车 | 改输入/输出话题名、检查频率等（参数也在 launch/yaml 里） |
| **`stop_conditions.py`** | 定义「什么情况必须停」：指令太久没来、里程计太久没来、按下软件急停 | 改超时时间逻辑、加新的停条件 |
| **`pass_or_zero_policy.py`** | 把上面的条件变成两种决定：PASS（放行）或 ZERO（置零） | 改「多种条件怎么组合」的策略 |
| **`twist_copy.py`** | 小工具：复制速度消息、造一个全零速度、判断是不是零 | 一般不用改 |

软件急停（不关导航程序，只让闸门输出 0）：

```bash
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"   # 停
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"  # 解除
```

#### `preflight/` — 开车前自动检查

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`topic_health_check.py`** | 听几秒：关键话题有没有在发 | 要检查的话题列表、超时时间 |
| **`tf_tree_check.py`** | 查坐标变换树断没断 | 要检查的 frame 链 |
| **`idle_cmd_vel_check.py`** | 没发目标时，`/cmd_vel` 必须一直是 0 | 检查时长、判定阈值 |

日常请用 `scripts/preflight_check_*.sh`，不必直接 `python3` 跑这些模块。

#### `bag_analysis/` — 录完包算成绩

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`load_rosbag.py`** | 把 rosbag 读成时间序列 | 支持的话题/消息类型 |
| **`path_tracking_metrics.py`** | 算离路径偏了多远（横向误差） | 误差算法、合格阈值 |
| **`heading_error_metrics.py`** | 算朝向偏了多少 | 同上 |
| **`velocity_smoothness_metrics.py`** | 算速度抖不抖（加速度、jerk） | 同上 |
| **`controller_timing_metrics.py`** | 算控制频率够不够、有没有丢帧 | 目标频率等 |
| **`generate_plots.py`** | 画出轨迹、速度、误差图 | 出哪些图、样式 |

入口一般是 `scripts/analyze_navigation_bag.py`，它会调用上面这些。

#### `cmd_vel_pipeline/` — 速度话题名字说明

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`topic_names.py`** | 把 `/cmd_vel`、`/cmd_vel_nav` 等写成常量，避免写错字符串 | 若全局改话题名，常量与 launch remap 要一起改 |
| **`remap_chain.py`** | **说明书式代码**：写明 MPPI → 平滑器 → 安全闸 → 底盘 怎么改名串起来 | 改链路设计时对照；真正 remap 在 `nav2_mppi_controller.launch.py` |

#### `mppi_controller/` — 调 MPPI 时的「说明书」（不是控制器本身）

> 真正跑的 MPPI 在 `src/navigation2/nav2_mppi_controller/`，参数在 `nav2_pure_mppi_params.yaml`。

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`baseline_params.py`** | 和 yaml 里基线数字对照的一份 Python 表，方便文档/测试引用 | 改基线时与 yaml **一起改**，别只改一边 |
| **`critics_reference.py`** | 用文字说明 8 个 critic（障碍、跟路径、朝向……）各管什么、权重大致作用、调坏了会怎样 | 学习调参；改权重仍以 yaml 为准 |

#### `localization/` — 坐标系名字

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`tf_frame_names.py`** | `map` / `odom` / `base_link` 等名字常量 | 若项目改坐标系命名 |
| **`readiness_checks.py`** | 查 TF 是否就绪（给预检等调用） | 就绪条件 |

#### `launch_config/` — launch 用的小工具

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`package_share_paths.py`** | 找到本包装好的 config/launch/rviz 路径 | 一般不用改 |
| **`common_launch_args.py`** | 公共启动参数约定（如布尔写法提醒） | 加公共参数时 |

#### `tools/`

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`reset_rplidar.py`** | 雷达串口复位实现 | 串口设备名、复位命令 |
| **`wait_for_can.py`** | 等到 `can0` 有帧再让底盘驱动上线 | 超时时间；没帧时的提示 |
| **`wait_for_nav_sensors.py`** | 等到 `/scan` + `/odometry/wheels` 再开 Nav2/RViz | 要等的话题、超时 |
| **`scan_viz.py`** | 把 `/scan` 改成当前时间戳发到 `/scan_viz`，RViz 红点才不会丢 | 一般不用改 |
| **`cancel_nav_on_pose.py`** | 收到 `/initialpose`（2D Pose Estimate）就取消正在跑的导航/规划 | 一般不用改 |

#### `bin/` — 真正被 `ros2 run` / launch 执行的脚本

带 `#!/usr/bin/env python3` 的入口（`reset_rplidar`、`wait_for_nav_sensors` 等）。一般不用改；逻辑仍在 `nodes/` + `tools/`。

---

### 2.8 包安装相关文件

| 文件 | 是干什么的 | 小白要不要动 |
|------|------------|--------------|
| **`package.xml`** | 包名、依赖哪些 ROS 包 | 加依赖时改 |
| **`setup.py`** | 安装方式、注册 `ros2 run` 有哪些命令 | 新增可执行入口时改 |
| **`setup.cfg`** | 安装脚本路径等 | 一般不动 |
| **`README.md`** | 本包英文说明 | 可同步更新 |
| **`test/test_safety.py`** | 自动测安全闸逻辑 | 改安全闸后跑测试 |

---

### 2.9 本包：想改什么 → 去哪（速查）

| 我想… | 去哪个文件 |
|-------|------------|
| 车更快/更慢、转弯、避障松紧 | `config/nav2_pure_mppi_params.yaml` |
| 换实验室地图 | `maps/` + 启动时 `map:=` |
| 里程计 / IMU 融合不对 | `config/robot_localization_ekf.yaml` |
| 急停逻辑 / 超时停车 | `robot_mppi_navigation/safety/` |
| 启动时多开/少开节点 | `launch/` |
| RViz 默认显示 | `rviz/rviz_goal_navigation.rviz` |
| 点 Goal 后怎么规划/恢复 | `behavior_trees/navigate_to_pose_real_robot.xml` |
| 改 MPPI 算法源码本身 | `src/navigation2/nav2_mppi_controller/`（进阶） |

### 2.10 本包常用命令

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

## 二附、包 `robot_slam_mapping`（建图）

目录：`src/robot_slam_mapping/`。**建图专用**，与日常导航包分开。

### 做什么

| 做 | 不做 |
|----|------|
| **轻量**在线 SLAM：硬件 + EKF + slam_toolbox + 手柄 + 安全闸 + **建图 RViz** | **不起**完整 Nav2 / MPPI |
| PS4 手柄遥控扫图（`mini_teleop`） | 不替代日常 AMCL 导航（那是 `robot_mppi_navigation`） |

建图精度主要靠雷达 + 里程计 + 本包 mapper 参数；扫完存图后再用导航包开车。

### 文件

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`launch/slam_mapping.launch.py`** | **轻量建图一键**（无 Nav2；默认打开建图 RViz）。同时起 `flatten_scan`，把扫描贴到地图那一层给窗口看 | 扫新地图只开它。车先停在实验起点再启动（这个位置就是地图原点）。`use_rviz:=False` 可关窗口 |
| **`robot_slam_mapping/nodes/flatten_scan.py`** | 把 `/scan` 原样复制成 `/scan_flat`，坐标系降到 `base_link` 高度。建图仍用原来的 `/scan`，不改墙的位置 | 已含在建图 launch 里。扫描线还悬空时看第七节 |
| **`rviz/mapping.rviz`** | 建图界面。左边是 **保存**。激光看的是贴地的 `/scan_flat`（方块仍是 3 像素 / 0.01 米）。其余按室内那份：网格、半透明地图、缩小后的坐标轴（不写名字）、短的里程计箭头（紫圈关掉）。视角是原来的斜视。说明栏高度 0。属性名和数值各占一半宽 | 改完要重新开建图，不要在带 `*` 的窗口里保存 |
| **`launch/mini_teleop.launch.py`** | 只起手柄 → `/cmd_vel_smoothed` | 一般已含在建图里 |
| **`config/mapper_params_online_async.yaml`** | **建图参数（本包自己的）**：分辨率、闭环、`base_link` 等。地图格子 **0.05 m**，与旧工作空间 `mapper_params_online_async.yaml` 相同 | **改建图调这个文件** |
| **`maps/`** | 建议把新图存这里 | 只要 **yaml+pgm**。用 `map_saver_cli`，或建图 RViz 的 **存图路径** 面板。不要点 Serialize Map |
| **`maps/mini_lab1.yaml`** + **`.pgm`** | 2026-08-22 扫的实验室图 | `map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab1.yaml` |
| **`maps/mini_lab2.yaml`** + **`.pgm`** | 2026-08-23 再扫的实验室图 | 同上换文件名 |
| **`maps/mini_lab3.yaml`** + **`.pgm`** | 2026-08-25 再扫 | 同上 |
| **`maps/mini_slab1.yaml`** + **`.pgm`** | 2026-08-26 再扫（日常默认也在导航包 `maps/`） | 同上 |

### 常用命令

```bash
colcon build --packages-select robot_slam_mapping --symlink-install
source install/setup.bash

ros2 launch robot_slam_mapping slam_mapping.launch.py
# 左上「保存」或下面这条命令。文件名自己定，不要带 .yaml
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
```

---

## 三、包 `robot_simple_commander`（录路径 / 跟路径）

目录：`src/robot_simple_commander/`。

### 3.1 这个包干什么（务必读）

| 做 | 不做 |
|----|------|
| **自己的**硬件 + 定位 +（跟线时）Nav2/**MPPI** + 安全闸 | 不调用、不依赖 `robot_mppi_navigation` |
| 键盘遥控时**录**轨迹 yaml | 不算速度数字——跟线时用**本包**拉起的 MPPI |
| 自动**跟随**录好的 / 内置路径 | 不要和模式 1 同时开两套 Nav2 |

推荐入口（一站式）：

| 场景 | 命令 |
|------|------|
| 录轨迹 | `record_trajectory.launch.py`（默认带 PS4 手柄）+ Ctrl+C 保存 |
| 跟轨迹 | `follow_trajectory.launch.py`（自带 MPPI）。跟完点 **回到起点**，车沿原路开回录制起点 |

---

### 3.2 启动文件 vs 节点（分类）

```text
launch/                         ← ros2 launch 入口
recorded/                       ← 你手动录的轨迹 yaml
paths/                          ← 内置考试路径
rviz/                           ← 录/跟界面（跟线：开始 / 回到起点 / 停止）
src/follow_panel.cpp            ← RViz 开始 / 回到起点 / 停止（C++）
bin/                            ← ros2 run 入口
nodes/ lib/ safety/             ← Python
```

#### `launch/` — 启动入口

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`record_trajectory.launch.py`** | 录轨迹：硬件 + EKF + **AMCL** + 节点 `record_path` + RViz + **PS4 手柄** | 先等红激光，RViz **2D Pose Estimate** 摆到真实位置，再开车；Ctrl+C 保存。`use_rviz:=False` 关界面；`use_gamepad:=False` 关手柄。**不要再开** `mini_teleop` |
| **`follow_trajectory.launch.py`** | 跟轨迹：硬件 → 等 `/scan`+轮速 → 再开 Nav2/MPPI + RViz + 节点 `follow_path`（**开始 / 回到起点 / 停止**） | **不写参数 = 跟 `route_name.yaml` 那个名字**。车**不会自己开**。雷达复位期间**没有 RViz**（约 15–20 秒，**不要 Ctrl+C**）→ 面板说导航就绪 → **2D Pose Estimate 只摆位姿** → **开始** 才开车。跟完点 **回到起点**（沿录下的路线正向开回去，因为车不能倒车）。**停止**只停车，程序还在，可以再开始。退出：本终端 Ctrl+C。**先停掉日常导航** |

#### `launch/parts/` — 被拼装的片段

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`hardware.launch.py`** | 底盘 / 雷达 / IMU（走日常包的 `rover_mini_hardware`，先复位雷达） | 只被入口 Include |
| **`nav_stack.launch.py`** | EKF + 定位 + Nav2/MPPI + 平滑 + 本包安全闸 | 只被 follow 入口 Include |
| **`rviz.launch.py`** | 打开本包 RViz：地图、激光、痕迹；可用 `rviz_config` 选录/跟布局 | 被录/跟入口 Include；不想看图就 `use_rviz:=False` |

#### `config/` — 本包自己的 MPPI（跟日常导航包不是同一份）

| 文件 | 是干什么的 | 想改什么就打开它 |
|------|------------|------------------|
| **`nav2_mppi_params.yaml`** | 跟线时的 Nav2 + **MPPI**。室内 Mini 已与日常导航对齐：纯 MPPI、A*、局部膨胀 0.28、雷达忽略 0.40 m 内 | 跟线快慢、避障；**不要改回 1.2 / min_range 0** |
| **`route_name.yaml`** | **轨迹名字**（就改 `name:` 这一行）。录制存 `recorded/<name>.yaml`，跟随读同一个名字 | 换一条录的/跟的路线时改这里，不用记长命令 |
| **`robot_localization_ekf.yaml`** | 本包录/跟时的轮速 + IMU（与日常包一样） | 跟线时图上漂、激光对不齐 |
| **`rviz/record_follow.rviz`** | **录制**界面：只留地图、青色激光、车体、一条绿色轨迹。俯视并跟着车 | 没有开始按钮（录完在终端 Ctrl+C 保存） |
| **`rviz/follow.rviz`** | **跟线**界面：蓝色轨迹、回起点时的橙色轨迹、绿点起点、橙点终点。左边 **开始 / 回到起点 / 停止** 和速度滑条 | 改按钮去 `src/follow_panel.cpp`；改速度滑条去 `src/speed_panel.cpp` |

#### 节点（`ros2 run`，不是 launch）

| 命令 | 源码 | 作用 |
|------|------|------|
| `ros2 run robot_simple_commander record_path` | `nodes/record_path.py` | 只跑录制节点 |
| `ros2 run robot_simple_commander follow_path` | `nodes/follow_path.py` | 只跑跟随客户端 |
| `ros2 run robot_simple_commander cancel_navigation` | `nodes/cancel_task.py` | 取消目标 |
| `ros2 run robot_simple_commander safety_gate` | `safety/` | 安全闸（一般由 nav_stack 拉起） |

---

### 3.3 Python 源码 `robot_simple_commander/`

> **约定：** 能 `ros2 run` 的节点放在 **`nodes/`**；库函数放在 **`lib/`**；安全闸逻辑在 **`safety/`**。

| 文件 | 是干什么的（人话） | 想改什么就动它 |
|------|--------------------|----------------|
| **`nodes/record_path.py`** | 等 **2D Pose Estimate** 后再按距离采点；RViz 画绿色痕迹；Ctrl+C 写成 yaml | 采样间距、是否等初始位姿、痕迹话题 |
| **`nodes/follow_path.py`** | 读 yaml → 等底盘和 Nav2 → **2D Pose 只定位** → **开始** 才发 FollowPath。走完不退出。**回到起点** 从车上最近点沿录制点开回第一个点，最后朝向与录制起点相同。离起点超过 0.8 m 时不允许再点开始 | 服务名 `/follow_start` `/follow_return` `/follow_stop` |
| **`lib/return_path.py`** | 算回起点的点列（**不是节点**）。车不能倒车，所以每段都朝向下一个点 | 判定「已经在起点」的距离（0.40 m）、允不允许开始的距离（0.80 m） |
| **`nodes/cancel_task.py`** | 告诉 Nav2：把 NavigateToPose / FollowPath 目标都取消（车停、栈还在） | 要取消的 action 列表 |
| **`nodes/run_safety_gate.py`** | 安全闸的 `ros2 run` 入口（薄封装） | 一般不用改；逻辑在 `safety/` |
| **`lib/path_io.py`** | 路径 yaml 的读和写（**不是节点**） | 文件格式字段（会连带跟/录都受影响） |
| **`lib/route_name.py`** | 读 `config/route_name.yaml`，把录制文件写到源码 `recorded/<name>.yaml`（避免只写在 install 里一编译就丢） | 一般不用改 |
| **`lib/generators.py`** | 用数学生成考试路线（**不是节点**） | 测试路径长度、弯道形状 |
| **`src/follow_panel.cpp`** | RViz 左边 **开始 / 回到起点 / 停止** | 按钮文字、调用的服务名 |
| **`src/speed_panel.cpp`** | RViz **速度**滑条：松手把本次前进上限写进 MPPI 和平滑器（0.20–1.00 m/s）。建图没有这个控制器，仍用手柄 | 改滑条范围、提示文字 |
| **`src/map_save_panel.cpp`** | 建图 RViz 里的存图条：一行路径加 **保存**。默认 `maps/today`，写出 `yaml` + `pgm` | 改默认路径、按钮文字 |
| **`safety/`** | 本包自己的安全闸业务逻辑 | 跟线时 `/cmd_vel` 闸门 |

---

### 3.4 `recorded/` 与 `paths/`

| 文件夹 | 放什么 | 怎么用 |
|--------|--------|--------|
| **`recorded/`** | **你手动录的**轨迹，文件名是 `config/route_name.yaml` 里的 `name` | 改名字去 `config/route_name.yaml`，不要手改命令行 |
| **`paths/`** | **内置考试**几何路径 | `path_name:=left_90` 等 |

> 里面只有位置和朝向，**没有速度**。速度永远现场由 MPPI 算。

| `paths/` 文件 | 是干什么的 | 小白怎么用 |
|------|------------|------------|
| **`straight.yaml`** | 约 3 m 直线 | `path_name:=straight` |
| **`left_90.yaml`** | 前进 + 左转 90° + 再前进 | `path_name:=left_90` |
| **`right_90.yaml`** | 同上，右转 | `path_name:=right_90` |
| **`s_curve.yaml`** | S 形弯 | `path_name:=s_curve` |
| **`obstacle_avoid.yaml`** | 直线；可在路上摆箱子测避障 | `path_name:=obstacle_avoid` |

格式示例：

```yaml
frame: map          # map=地图坐标；local=相对录制/发送时车头
poses:
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - {x: 0.05, y: 0.0, yaw: 0.0}
```

---

### 3.5 `scripts/` / 安装 / 测试

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`scripts/generate_test_paths.py`** | 用 `generators.py` 重新生成 `paths/*.yaml` | 改完 generators 后跑它更新文件 |
| **`package.xml`** | 包名与依赖 | 加依赖 |
| **`CMakeLists.txt`** | 编 Python 节点 + RViz Follow 面板；`bin/` 里是 `ros2 run` 入口 | 新增加命令或改面板 |
| **`README.md`** | 本包英文说明 | 可同步更新 |
| **`test/test_path_io.py`** | 自动测路径生成和 yaml 读写 | 改 path_io/generators 后跑测试 |

---

### 3.6 本包：想改什么 → 去哪（速查）

| 我想… | 去哪个文件 |
|-------|------------|
| 录密一点 / 稀一点 | `min_separation`（默认 **0.05 m**）：内部 20 Hz 查 TF，车走出 5 cm 才记一个点；不是按时间存 |
| 跟线逻辑、等不等 Nav2 / 等 2D Pose / 开始 / 回到起点 | `nodes/follow_path.py` 和 `lib/return_path.py` |
| 跟线按钮外观 | 本包 `src/follow_panel.cpp` |
| 换内置考试路线形状 | `generators.py` → 再跑 `generate_test_paths.py` |
| 换一条轨迹的名字（录/跟都看这个） | 只改 `config/route_name.yaml` 里的 `name:` |
| 手改一条路径点 | 直接编辑 `paths/*.yaml` 或 `recorded/<name>.yaml` |
| 取消键行为 | `nodes/cancel_task.py` |
| 车跟线时快慢/晃（控制） | 本包 `config/nav2_mppi_params.yaml`（跟 `robot_mppi_navigation` **不是同一文件**；室内已对齐 `vx_max` 1.00 / 雷达 0.40 m） |
| 录/跟时 RViz 显示 | 录：`rviz/record_follow.rviz`；跟：`rviz/follow.rviz`（开始 / 回到起点 / 停止） |

### 3.7 本包常用命令（与 robot_mppi_navigation 无关）

> **先 Ctrl+C 停掉日常 RViz 导航。** 两套 Nav2 不能同时开。名字只改一个文件：`src/robot_simple_commander/config/route_name.yaml`（`name: 走廊1`）。录完停掉再 follow。

```bash
# 1) 改名字：打开 config/route_name.yaml，把 name: 改成你要的（不要写 .yaml）
# 2) 录
ros2 launch robot_simple_commander record_trajectory.launch.py
# 等红激光 → 2D Pose Estimate 摆准 → 手柄开 → 这个终端 Ctrl+C 保存

# 3) 跟（另开终端；读同一个 name）
ros2 launch robot_simple_commander follow_trajectory.launch.py
# 等约 15–20 秒（雷达复位 + CAN；RViz 随后才开；不要 Ctrl+C）→ 面板说导航就绪
# → 2D Pose Estimate 摆到轨迹起点（车还不动）→ 点「开始」
# 跟完点「回到起点」：沿蓝线原路开回绿点，并转到录制时的朝向
# 「停止」只停车。退出这个终端才是 Ctrl+C

# 跟内置考试路线（这次不用改文件）
ros2 launch robot_simple_commander follow_trajectory.launch.py name:=straight
```

---

## 四、模式 B 包（Python，跟人）

> 逻辑参考 `~/rover_workspace_final`（Nuitrack 骨架 + 云台），下游用 Python 重写。  
> 依赖外部：`orbbec_person_publisher`（Nuitrack，旧工作空间 overlay）、`dynamixel_sdk`。  
> **相机是 Femto Mega**。给人跟/步态锁骨架必须用 **Nuitrack（以前能认出人的那套）**。SDK（K4A + Azure k4abt）只给老师看点云；这台 NUC 是 Intel Iris Xe、**没有 CUDA**，CPU 人体跟踪几乎锁不上骨盆。  
> **Nuitrack 和 SDK 不能同时开**（同一台 USB 相机）。  
> 锁点：Nuitrack 发 **躯干 torso**（对应 User.msg 的腰/质心附近，x 右 y 上 z 前）。直线跟随可把云台冻在初始角；拐角走廊再打开水平跟踪。

### 4.1 `robogait_person_interfaces`

| 文件 | 作用 |
|------|------|
| **`msg/User.msg`** | `detection` + `x/y/z`（相机坐标系，z=距离） |

### 4.2 `robogait_perception`

两套看人前端，**同一时间只能开一套**（抢同一台 Femto Mega）。下游话题相同（云台/跟人不关心用哪套）：

| `person_source` | 是什么 |
|-----------------|--------|
| **`nuitrack`（云台/步态默认）** | 第三方 Nuitrack（旧包 `orbbec_person_publisher` / `nuitrack_person_node`）。发躯干 3D + 像素，这是以前能认出骨架的方法。 |
| **`sdk`（点云演示）** | Orbbec K4A 包装：发**点云**（默认只留人体）、彩图、深度、相机位置标记。Azure 人体跟踪 **默认关**（`enable_body: false`）。 |

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`src/person_sdk.cpp`** | Femto Mega SDK：点云（**默认只留人体**）+ 可选 k4abt（这台机器 CPU 上不可靠，默认关）+ **相机位置标记** → `/orbbec/*` | `ros2 run robogait_perception person_sdk` |
| **`nodes/distance_orbbec.py`** | `/orbbec/*` → `user_raw`、`person_center_pixel`（Nuitrack 躯干或 SDK 点） | `ros2 run robogait_perception distance_orbbec` |
| **`launch/person_source.launch.py`** | 选 nuitrack / sdk；默认带 **Mini 整车 URDF** + 可选 RViz。Nuitrack 时额外发静态 TF **`orbbec_link` → `rgbd_camera_link_optical`**（Nuitrack 点云坐标系），否则 RViz 丢点云。找不到 Nuitrack 会提示先 overlay 旧空间 | `use_robot_model:=False` 若导航已经发了 URDF。Nuitrack 和 SDK **不要同时开** |
| **`rviz/orbbec_sdk.rviz`** | SDK 演示：**整车 RobotModel** + 点云 + 彩图 + 橙色镜头箭头 | `person_source:=sdk` 且 `use_rviz:=True`；Fixed Frame = `base_link` |
| **`rviz/orbbec_nuitrack.rviz`** | Nuitrack 骨架：同样 Fixed Frame `base_link`，点云走 `rgbd_camera_link_optical`；**没有彩图/深度图**（Nuitrack 不发 `/orbbec/color/image_raw`，避免再出现 No Image） | `person_source:=nuitrack` 自动用这份。RViz 空但日志已检出人：见 **7.35** |
| **`config/perception.yaml`** | 话题名、点云抽稀（`cloud_stride`）、**只显示人**（`cloud_person_only`）、**SDK 人体 `enable_body`（默认 false）** | **跟人/骨架不要开 SDK body。RViz 只想看人：保持 `cloud_person_only: true`。给老师看整间房：改成 `false` 后用 `person_source:=sdk` 重启。点云太稀改 stride** |
| **`urdf/orbbec_gimbal.urdf`** | Mini 顶上的双轴云台（左右 `gimbal_pan_joint` + 俯仰 `gimbal_tilt_joint`）+ 青色相机 | **镜头默认略低头：tilt 的 origin pitch 用正数（约 `0.12`）。RViz 人飞上天：改 `gimbal.yaml` 的 `tilt_urdf_sign`（见 7.30），不要只用 `person_source`** |
| **`urdf/mini_with_orbbec.xacro`** | Mini 底盘 + 相机，给 RViz **RobotModel** 用 | launch 会自动 xacro；一般不用手改 |

```bash
# 点云给老师看（不开 Nuitrack、不开 k4abt）
ros2 launch robogait_perception person_source.launch.py person_source:=sdk
# 骨架（必须先 overlay 旧空间，见下方云台命令）
ros2 launch robogait_perception person_source.launch.py person_source:=nuitrack
```

### 4.3 `robogait_gimbal`（云台）

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`nodes/camera_tracker.py`** | 人在画面里的偏差 → `camera_position`（左右，度）+ `camera_tilt`（上下，度，有上限）。Nuitrack 躯干有 y 就会俯仰；SDK 深度块（y=0）只水平转。俯仰滤波重、死区大，走路不点头 | `ros2 run robogait_gimbal camera_tracker` |
| **`nodes/dynamixel_driver.py`** | 双轴：ID1 水平、ID2 俯仰；并把角度发到 RViz 车模（`tilt_urdf_sign` 对俯仰方向）。**`freeze_gimbal` 冻在初始角**。人还偏在画面外时会继续发水平步进 | `ros2 run robogait_gimbal dynamixel_driver` |
| **`config/gimbal.yaml`** | U2D2 口（现在是序列号 **FT8ISNFV**）、两轴 ID/限位、**初始角 `idle_*_ticks`**、PID、**冻云台 `freeze_gimbal`**、**水平/俯仰开关 `enable_gimbal_pan` / `enable_gimbal_tilt`**、**俯仰死区 `tilt_dead_zone_deg`**、**RViz 俯仰符号 `tilt_urdf_sign`**、**过近不跟 `min_track_z_m`（默认 1.0 m）** | **换了 U2D2 就改 `device_name`。B3 直线跟随 launch 会自己冻住，不用改这里。拐角：`freeze_gimbal: false` 且 `enable_gimbal_pan: true`。人在画面里上下跳：加大 `tilt_dead_zone_deg` / 减小 `tilt_kp`。没人时朝哪看改 idle；RViz 人飞上天改 `tilt_urdf_sign: -1.0`。贴太近云台猛转：站远到 1.5–3 m，或改 `min_track_z_m`** |
| **`test_gimbal.launch.py`** | B0：感知 + **双轴云台跟人**（不起底盘 Nav2）。**默认 `person_source:=nuitrack`** | **想让镜头跟着你转，用这个**；不要只开 `person_source`。先 overlay 旧空间 |

### 4.4 `robogait_person_follow`

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`nodes/distance_filter.py`** | `user_raw` → `user_coordinates`（平滑） | `ros2 run robogait_person_follow distance_filter` |
| **`nodes/person_nav_goal.py`** | 人坐标 → 保持间距的 `NavigateToPose`（MPPI 算完整 `[v,ω]`） | `ros2 run robogait_person_follow person_nav_goal` |
| **`nodes/straight_follow.py`** | **B3**：只用人体距离 `z` 算前进/后退，角速度恒为 0，发到 `cmd_vel_smoothed` | `ros2 run robogait_person_follow straight_follow` |
| **`person_follow.launch.py`** | B1 一站：A1 Nav2/MPPI + 人层 | 见下 |
| **`person_layer.launch.py`** | B2 终端 2：叠在已起的 Nav2 上 | 见下 |
| **`straight_follow.launch.py`** | **B3 单独一条**：硬件 + Nuitrack + 云台冻住 + 距离控制 + 安全闸。不起 Nav2 | 见下。不要和 B1/A1 同时开 |
| **`config/person_follow.yaml`** | B1/B2 目标间距、死区、目标更新频率 | **调拐弯跟人距离动它** |
| **`config/straight_follow.yaml`** | B3 距离带、前进/后退上限、加速 | **调直线跟距和快慢动它** |

**控制链（跟人 = 纯 MPPI；云台只对准相机）：**

```text
人检测 → camera_tracker → dynamixel_driver → ID1 左右 + ID2 上下（相机 FOV）
人检测 → distance_filter → person_nav_goal → NavigateToPose
                                              ↓
                                     规划器 → MPPI → 平滑 → safety_gate → /cmd_vel
```

B1/B2 人**不**改 `/cmd_vel`。底盘速度上限是 MPPI `vx_max` **1.00 m/s**（日常 yaml；平滑器 `max_velocity` / `max_accel` 要一起对上）。这是**上限不是恒速**：空地可以接近 1.0，靠近障碍会自己慢。人走得比这快会拉开距离。**不要直接改到 1.2**（室内采样会全撞膨胀格）。

**B3 直线跟随**不走上面这条链。人远于距离带就向前，近了就慢退，带内停车；角速度一直是 0。速度上限在 `straight_follow.yaml`（前进 **0.70**、后退 **0.30**），不要超过 1.0。最终 `/cmd_vel` 仍只有 `safety_gate`。

**常用命令：**

```bash
# B0 云台跟人：默认 Nuitrack 骨架（必须先 overlay 旧工作空间，再 source 本仓库）
# 站到镜头正前方 1.5–3 m、身体对着相机；终端应有 Nuitrack「Skeleton detected」和 joint=torso
# Nuitrack 和 SDK 不能同时开（同一台 USB 相机）
# 拐角走廊云台：gimbal.yaml 里 freeze_gimbal: false，enable_gimbal_pan: true（俯仰保持慢跟）
# B3 直线跟随自己会把云台冻住，不用改 gimbal.yaml
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
# 给老师看点云（不开骨架）：person_source:=sdk

# B1 一站跟人（Nav2 + MPPI + 人→目标；同样默认 Nuitrack）
ros2 launch robogait_person_follow person_follow.launch.py

# B2 终端1 Nav2 + 终端2 人层
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py

# B3 直线跟随（先停掉 B1/导航。人站在镜头正前约 3 m。云台不动，车只前后）
ros2 launch robogait_person_follow straight_follow.launch.py
```

---

## 五、工作空间其它文件

| 文件 / 目录 | 是干什么的 | 小白怎么用 |
|-------------|------------|------------|
| **`PROJECT_CHANGES.md`** | 本中文手册（与 `TEST_PLAN.md` 为仅有的中文文档） | 查「文件干什么、去哪改」 |
| **`PROJECT_CHANGES_EN.md`** | 英文手册（与中文对齐） | 给英文读者 |
| **`TEST_PLAN.md`** | 今天的实机步骤（中文）：建图 → 导航 → 录路径 → 跟路径。人、云台不在这份里 | **上机按它做**；一次只开一套，全程同一张 `today` 图 |
| **`MEETING_BRIEF.md`** | 开会给老师看的中文汇报提纲 | **今天汇报投屏/照着讲** |
| **`NODES_AND_TOPICS.md`** | 节点↔话题图（英文） | 搞不清谁订谁发时看它 |
| **`README.md`** | 仓库首页简介（英文） | GitHub / 新人入口 |
| **`src/*/README.md`** | 各包说明（英文） | 与第二～四节互补 |
| **`src/navigation2/`** | 整份 Nav2 源码（含 MPPI 实现） | **进阶**：改控制器算法。`nav2_util` 里过期 TF 时间戳会改用最新变换 |
| **`src/bno055/`** | BNO055 IMU 驱动（串口 `/dev/bno055`） | IMU 起不来时改这里：打开串口会清缓冲，配置失败在进程内重试，避免反复抢口 |
| **`.vscode/settings.json`** | 打开 md 默认预览 | 可不管 |
| **`.gitignore`** | 不把 build/install/log 等提交进 git | 可不管 |
| **`scripts/install_can0_autoup.sh`** | 安装 can0 开机/插拔自动 UP（需 sudo） | CAN 总掉线时跑一次 |
| **`scripts/install_vscode_server.sh`** | 手动安装/修复 VS Code Remote SSH server（需能上网） | Remote 报 `localDownloadFailed` 时在 NUC 上跑 |

---

## 六、待办

| 项 | 状态 |
|----|------|
| ~~RPLidar health 2~~ | 已解决：`ros2 run robot_mppi_navigation reset_rplidar` |
| ~~BNO055 通信失败 / 启动抢串口~~ | 已解决：UART 清缓冲 + 进程内重试配置，不再立刻退出让 launch 抢口 |
| `slam_mapping` / 建图实测 | 建图会打开 `rviz/mapping.rviz`；待实机扫图并点保存 |
| 按 `TEST_PLAN.md` 实机勾选 | 今天只测建图、导航、录路径、跟路径（人/云台不在这份计划里） |
| 模式 A2 录制 + 跟随 + 回到起点 | 代码已接上；待 NUC 按第三节走一遍。回到起点是沿录制路线正向开回，不是倒车 |
| Mode B：`robogait_gimbal` + `test_gimbal` 实机 | 已改默认 Nuitrack 骨架；待实机确认 torso + 云台转 |
| Mode B：`person_follow` / `person_layer` 实机（纯 MPPI） | 代码已改，待测 |
| Mode B3：`straight_follow` 直线跟随实机 | 单独一条已加；待测跟距和 0.70 m/s 上限 |
| posegraph 定位接进 launch | 未做（现用 AMCL） |
| 自定义 MPPI critic | 未开始（在 `navigation2`） |
| Gazebo 仿真依赖 | 需要时再装 |

---

## 七、常见问题怎么解决

> 实机调试里踩过的坑。以后再遇到同类问题，**先查本节**；新坑解决后也要**补进本节**（中英文同步）。

### 7.1 底盘驱动 FATAL：`Did not receive any data… stale`

**现象：** `roverrobotics_driver` 报 FATAL，连上 `can0` 但收不到车数据，进程反复重启。

**原因：** `can0` 是 DOWN，或车上没电 / 急停没松 / CAN 线问题。ROS **不会**自动配 CAN。

**处理：**

```bash
ip link show can0                    # 是否 UP
sudo systemctl start can.service     # 或：sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
candump -n 10 -T 3000 can0           # 必须有帧；仍空 → 查车上电、急停、线
# 再 launch 硬件
ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py
```

**一劳永逸（推荐做过一次）：**

```bash
sudo bash ~/RoboGait_ws/scripts/install_can0_autoup.sh
```

会改 `can.service` + udev：开机和 USB-CAN 重新插入时自动把 `can0` UP。装完后 `Active: active (exited)` 对 oneshot 服务是**正常**的。

---

### 7.2 雷达挂掉：`RPLidar health status : 2` / process died

**现象：** 日志 `health status : 2` → `internal error` → `rplidar` 进程 exit 255；`/scan` 没有。

**原因：** 不是“必须每次手打复位”。Ctrl+C 时驱动会 `Stop motor`，这块 **RPLidar S2** 停转后固件常把 health 打成 2。官方驱动一看到 2 就**自己退出**（exit 255），并不是厂商 launch 坏了。刚上电、雷达没被掐过时，厂商 `mini_teleop` 可以直接开。

**处理：** 已改工作空间里的 `rplidar_ros`：启动时若 health 不是 OK，**驱动自己软复位再查**（最多两次）。之后可以直接用厂商命令，不必先 `reset_rplidar`：

```bash
ros2 launch roverrobotics_driver mini_teleop.launch.py
```

日志里应看到 `Soft-reset 1/2 then retry...` 然后 `health status : OK`。若仍是 2：停 launch → **拔插雷达电源/USB**（等 5 秒）→ 再开。

日常导航 / `rover_mini_hardware` 仍会先复位，多一道保险。跳过那一层：`reset_lidar:=False`。

**为什么要发复位：** `0xA5 0x40` 是思岚设备软重启；有时一次不够，硬断电最稳。电脑侧重启节点清不掉 health=2。

**`SL_RESULT_OPERATION_TIMEOUT`、口已经在（2026-09-27）：** `/dev/rplidar` 存在，节点能打开串口，但读健康没有任何字节。这不是 health 2，软复位也没东西可应答。115200、256000、1000000 都是 0 字节，不是波特率设错（设错通常会有乱码）。`/dev/serial/by-id` 里的序列号是黑色 SLAMTEC 转接盒里的 USB 芯片，不是会转的雷达头。同一颗雷达头只换盒子：旧盒 CP2102（`a64db3d4…`）电脑认得到，但雷达数据过不去。能用的 CP2102N 有两只，`d4ffe520d382eb11b064968d9693f7bc` 和 `7ee3445a1d0bee11b58f4002f59e3369`，1000000 波特下健康都是 `OK`。雷达头自身序列号一直是 `71e6ecf8c4e699d7b8eb99f9fd394717`，换盒子不会变。建图用这两只新盒子，旧盒子不要再插。两只盒子在 `lsusb` 里都是 `10c4:ea60`，用 `ls -l /dev/serial/by-id` 区分。

---

### 7.3 `Unknown topic '/scan'`（或别的话题不存在）

**原因：** 对应节点还没起来（没 launch 硬件/导航），不是设备坏了。

**处理：** 先起 launch，再 `ros2 topic info /scan --verbose`。

---

### 7.4 Ctrl+C 后 bno055 一长串 KeyboardInterrupt / RCLError

**原因：** 打断 IMU 读串口时的正常关机噪音。

**处理：** **可忽略**。下次干净启动即可。

---

### 7.5 `/dev/ttyUSB0` 和 `/dev/rplidar` 对不上、数字总变

**原因：** `ttyUSB*` 随插拔顺序变；`/dev/rplidar`、`/dev/bno055` 是 udev 按序列号起的固定名。

**处理：** launch / 参数里用 **`/dev/rplidar`、`/dev/bno055`**，不要写死 `ttyUSB0`。规则文件：`/etc/udev/rules.d/99-rover-mini-sensors.rules`。

**两个名字都不存在时（2026-09-26）：** 先看 USB，不要接着 launch。

```bash
lsusb
ls -l /dev/serial/by-id
```

- 没有 `10c4:ea60`（Silicon Labs）：NUC 没收到雷达的 USB 数据。雷达头在转只说明电机有电，数据口是另一根线。`/dev/rplidar` 不会出现，节点会报 `80008004` 后退出。这个 Hub 其实是两颗芯片：USB2 那侧有鼠标接收器，所以鼠标正常；蓝色 USB3 那侧四个口一直是空的，雷达插在那边时内核根本看不到 `10c4:ea60`。把雷达 USB 换插到 NUC 机身口，再查 `lsusb` 里有没有 `10c4:ea60`，以及 `ls -l /dev/rplidar`。2026-09-26 插到机身口后认到的是 CP2102，序列号 `a64db3d4e89098459d3372b618827745`，`/dev/rplidar` 由 `55-roverrobotics.rules` 生成。`99-rover-mini-sensors.rules` 里写的旧序列号 `060d09eb…` 对不上这根线，不要靠那一条。
- 有 FTDI `0403:6014` 但没有 `/dev/bno055`：现在插着的 `FT8ISNFV` 是**云台 U2D2**，不是 IMU。IMU 是另一根线，对上之后才会出现 `/dev/bno055`。不要把云台这根改成 IMU 规则。
- **NUC 改由充电宝供电、Hub 仍插在 NUC 上时（2026-09-26）：** 数据路没变，Hub 还是 NUC 的 USB 口扩展。机器人原来的电源更稳；充电宝带不动 NUC 再加 Hub 时，Hub 下游口电流不够。表现就是：Hub 自己和鼠标（USB2、耗电小）还在，蓝色口上的雷达/IMU 不枚举；同一根雷达线插到 NUC 机身口就能出现 `/dev/rplidar`。今天雷达、IMU、手柄接收器都插 NUC 机身口。要再用 Hub 的口，先把 NUC 改回机器人供电。

---

### 7.6 `/cmd_vel` 有多个 Publisher，或跟人不走 MPPI 链

**处理：**

```bash
ros2 topic info /cmd_vel --verbose
```

发布者**只能是** `safety_gate`。若出现其它节点，停掉多余 launch（不要两套 Nav2 同时开；人层不得直接发 `/cmd_vel`）。

---

### 7.7 两套导航同时起、行为混乱

**原因：** `robot_mppi_navigation` 与 `robot_simple_commander` 完整栈各起一套 Nav2。

**处理：** 同一时间只开一套；换模式前 Ctrl+C 停干净。

---

### 7.8 在线建图：没图 / TF 报 `base_footprint`

**现象：** 建图时 `/map` 不增长，或 slam_toolbox 找不到 `base_footprint`。

**原因：** Mini 只有 **`base_link`**。建图应走包 `robot_slam_mapping`，参数在该包 `config/mapper_params_online_async.yaml`。

**处理：**

```bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
# ros2 topic hz /scan   # ~10 Hz
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
```

改建图：编辑 `src/robot_slam_mapping/config/mapper_params_online_async.yaml` 后重启 launch。

没有 `/dev/bno055` 也可以建图。`bno055` 会反复退出，launch 不会因此停。EKF 只用轮子的前后速度，转弯角速度只信 IMU，所以没 IMU 时转弯要慢，让激光匹配把朝向补上。导航、录路径、跟路径同样只等 `/scan` 和轮速，不等 IMU。

---

### 7.9 导航报 `Failed to load map yaml` / `bad file`

**现象：** `map_server` 读 yaml 失败，定位整组起不来；后面还会刷 `Invalid frame ID "odom"`。

**原因：** `map:=` 指的文件不存在，或 yaml 和 `.pgm` 不在同一目录。常见：RViz **Save Map** 只写了名字没写完整路径，yaml 落在 `~/RoboGait_ws/`，而 launch 指的是 `maps/`。只点 **Serialize Map** 会得到 `.posegraph` + `.data`，**没有**导航用的 yaml。

**处理：**

```bash
ls ~/RoboGait_ws/src/robot_slam_mapping/maps/
# 必须同时有 名字.yaml 和 名字.pgm
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

导航读的是 **yaml+pgm**，不是 posegraph。`.posegraph` / `.data` 已从两个包的 `maps/` 删掉；以后存图只用：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/RoboGait_ws/src/robot_slam_mapping/maps/名字
```

---

### 7.10 RViz：`could not connect to display` / `xcb`

**现象：** 导航 launch 里 `rviz2` 立刻挂掉，日志有 `qt.qpa.xcb: could not connect to display`。

**原因：** 这个终端没有图形桌面（常见：Cursor / SSH 远程终端）。RViz 必须在 NUC **本机桌面**的终端里开。

**处理：** 在 NUC 桌面打开终端再 launch；或先关 RViz、本机另开：

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  use_rviz:=False \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
# 本机桌面另开：
rviz2 -d $(ros2 pkg prefix robot_mppi_navigation)/share/robot_mppi_navigation/rviz/rviz_goal_navigation.rviz
```

---

### 7.11 启动前几秒狂刷 `Invalid frame ID "odom"` / `"map"`

**现象：** 导航刚起来时 costmap 每隔 0.5 s 报 `base_link → odom` / `map` 不存在；大约 8–10 秒后雷达复位结束、底盘起来，刷屏停止，出现 `Managed nodes are active`。

**原因：** launch **先**对雷达做两次软复位（要等几秒），**底盘 / EKF 还没发 `odom`**；Nav2 已经在等 TF。这不是地图坏了。

**处理：** **不要 Ctrl+C**。等到日志里同时有：

- `Read map ... mini_lab2.pgm`
- `RPLidar health status : OK`
- `lifecycle_manager_localization: Managed nodes are active`
- `lifecycle_manager_navigation: Managed nodes are active`

然后在 RViz 用 **2D Pose Estimate** 把机器人摆到图上真实位置（激光和墙对齐），再点 **2D Goal Pose**。AMCL 默认初值是 `(0, 0, 0)`，不摆初值会定错位。

---

### 7.12 怎么摆初始位姿；RViz 里雷达点很少

**现象：** 车在图上偏了；**红激光和黑墙对不齐**（看起来像漂移）；或激光只有稀稀拉拉几点。终端狂刷 `extrapolation into the future`（`lidar_link` → `map`，差约 10–40 ms）。

**原因：** 这不是 MPPI / 地图改坏了。AMCL 默认在图上 `(0,0,0)`，朝向不对时红激光就会相对墙转开。IMU 串口失败（`0x6` / `multiple access`）**不会**让静止时激光相对墙转；导航用轮速也能定。要靠 **2D Pose Estimate** 把朝向摆对。

**处理（摆位）：**

1. 工具栏点 **2D Pose Estimate**（绿色箭头）。
2. 在地图上**车实际所在的房间**按住鼠标，拖出车头朝向，松开。
3. 松开后红激光应**马上**出现在新位置并贴墙。若还在旧位置拖一两秒，是旧 RViz 配置（Decay 1.5 s）在叠历史扫描，重启本 launch 即可。
4. 原地慢慢转一下车，让 AMCL 粒子收成一团。
5. 对齐后再用 **2D Goal Pose** 点目标。

**处理（看清激光 / 不拖影）：** 大红点、`Decay Time = 0`（只画当前一圈，摆位立刻跳）、TF 队列 10。EKF `transform_time_offset: 0.05`。必须**重启整个 launch**（RViz 配置和 EKF 才会一起更新）。终端里 future TF **不要 Ctrl+C**。

---

### 7.13 `planner_server` 狂刷 `Extrapolation Error` / `into the past`

**现象：** 已经 `Managed nodes are active`，也摆过 2D Pose；点目标后（或连摆几次位姿后）`planner_server` 每隔约 1 s 报：`Requested time … but the earliest data is at time …`（**Requested time 一直不变**）。

**原因：** 每隔约 1 秒刷一次 = 行为树在重规划，说明 **已经有一条导航目标在跑**（工具栏 **Nav2 Goal** / 2D Goal，不是 2D Pose）。`Requested time` 一直不变，是第一次点目标时记下的旧位姿。你后来再摆 2D Pose，`map→odom` 跳了，规划器还拿着那个旧时刻查 TF。AMCL 的 `Failed to transform initial pose in time` 只是警告，位姿其实已经设上了。

**处理：** **不要 Ctrl+C。** 现在每摆一次 **2D Pose Estimate** 会自动取消旧目标。仍可在 Navigation2 面板点 **Cancel**。

1. **2D Pose Estimate 可以对齐很多次**（第一下不准很正常）。每次松开后等红点跳到新位置再摆下一次，直到激光贴墙  
2. **对齐后再点 Nav2 Goal / 2D Goal Pose**  
3. 已经在跑目标时也可以继续摆 Pose（会自动 Cancel），对齐后再点新目标

**点了目标只走一下就停：** 日志里没有 `Begin navigating` / 没有绿色 `/plan`，说明 **规划没成功，MPPI 没接到路径**。车那一下是默认行为树的 Spin/BackUp 恢复，不是在跟踪路径。已改成本包行为树（失败只等待，不清整张全局地图），规划用最新 TF。不要 Ctrl+C。

若终端已经在刷 `Waiting for async process to finish` / `missed deadline to stop`，狂按 Ctrl+C **没用**，见 **7.24**。

---

### 7.14 终端里那些红色 / ERROR 分别是什么

| 日志 | 要不要管 |
|------|----------|
| `Did not receive any data… stale` + 驱动 exit -11，然后自己又起来 | 以前常见：CAN 还没帧驱动就上了。现在 launch 会先 `wait_for_can`。若仍 FATAL：急停松开、`sudo systemctl start can.service` |
| 前几秒 `Invalid frame ID "odom"` / `"map"` | 旧现象：Nav2 比底盘先起。现在等传感器好了再起 Nav2，不应再刷 |
| `extrapolation into the future`（`lidar_link`→`map`） | RViz 显示用 `/scan_viz`（当前时间戳），不应再狂刷。偶发可忽略 |
| `Failed to transform initial pose in time` | AMCL 警告，位姿已经设上 |
| `GLSL link result` / `particle_cloud` QoS | RViz 外观噪音，可忽略 |
| `Message Filter dropping message: frame 'odom'` / `queue is full` | RViz 显示跟不上里程计。已关掉 TF 坐标轴显示。可忽略 |
| 刚激活时 `Timed out waiting for transform` 一下，随后 `start` | 规划器比 AMCL 的 TF 早半拍，后面会好 |
| `bno055: Data fusion not ready` / `BusOverRunException` / `TransmissionException` / `Wrong READ-request response header` | IMU 串口忙或读脏。驱动会在**同一进程里重试**（不再立刻死掉抢口）；导航仍可用轮速。见 7.20 |
| Ctrl+C 后 `bno055` KeyboardInterrupt、驱动 -11 | 关机噪音，可忽略 |
| `rviz2: process has finished cleanly`（其它节点还在） | 你关了 RViz 窗口。导航还在跑，只是看不见图。不要关窗口，也不要因此 Ctrl+C |

---

### 7.15 `Exec format error`：`reset_rplidar` / `wait_for_nav_sensors` 起不来

**现象：** launch 立刻失败：`OSError: [Errno 8] Exec format error: '.../lib/robot_mppi_navigation/reset_rplidar'`（`wait_for_nav_sensors` 同样）。

**原因：** `install/` 里的入口脚本被写成了 **0 字节空文件**（本机 setuptools 的 `console_scripts` 会这样），内核没法当程序执行。

**处理：** 已改成本包 `bin/` 下带 shebang 的真实脚本。清掉旧安装再编一次：

```bash
cd ~/RoboGait_ws
rm -rf build/robot_mppi_navigation install/robot_mppi_navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mppi_navigation --symlink-install
source install/setup.bash
```

然后在 **NUC 桌面终端**（有屏幕的那个，不要用 SSH/`cursor` 远程）再 launch。`X11 connection rejected` 是远程没显示权限，和雷达脚本无关。

---

### 7.16 点了目标只原地慢转、不前进

**现象：** 2D Pose 对齐后点 Goal，RViz 有绿线，但车只慢慢转、几乎不往前。日志：`Optimizer fail to compute path`，然后清 local costmap，再 `Received a goal`，循环。

**原因：** MPPI 认为**所有采样轨迹都会撞上**。常见两件事叠在一起：

1. 代价地图 `obstacle_min_range: 0`，雷达把**车身**扫成障碍，车被自己围住。  
2. `vx_max: 1.2` 前瞻太远，室内一采样就进膨胀层。控制环还经常 `missed 20 Hz`。

**处理（主修复仍是雷达滤车身）：** 已改日常导航 `nav2_pure_mppi_params.yaml`，**本包跟线**同样改了 `robot_simple_commander/config/nav2_mppi_params.yaml`（launch 读的是这一份，不是日常那份）：雷达忽略 **0.40 m 内**（Mini 雷达在车头，车尾约 0.45 m 会被扫成障碍；**不要把 obstacle/raytrace min_range 改回 0**）。`vx_max` 现为 **1.00 m/s 上限**（`wz_max` 1.20），不是恒速巡航：0.40 跟不上人，1.2 会让每个采样都撞膨胀格。平滑器 `max_velocity` 必须与 `vx_max`/`wz_max` 一致。若再出现 `Optimizer fail`，先把 `vx_max` **降回 0.70–0.80**，不要往 1.2 加。Humble MPPI 必须 **20 Hz 且 model_dt=0.05**，改成 10 Hz 会直接报 `Controller period more then model dt` 起不来。重启整栈（yaml 要重启 launch）：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

对齐激光后，先点 **1–2 米外白色空地**。成功时应：`Begin navigating` → 车先略转再**往前走**。终端里不应再每隔一两秒刷 `Optimizer fail to compute path`。

想再改车速：同一文件里 `vx_max` / `wz_max`（平滑器 `max_velocity` 必须一起改）。`vx_max` 是上限，空地可以接近它；不要直接改到 1.2。

**处理：** 改完后必须编译一次再在 **NUC 桌面终端** launch（不要用 Cursor 远程开 RViz）：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_util robot_mppi_navigation --symlink-install
source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

等日志出现 `Hardware topics are live. Starting Nav2.` 和两组 `Managed nodes are active`，再反复摆 2D Pose 直到激光贴墙，然后再点 Goal。

---

### 7.17 `Package 'robot_mppi_navigation' not found`，搜的是 `rover_workspace`

**现象：** 人已经在 `~/RoboGait_ws` 里敲 launch，但报 package not found，搜索路径是 `~/GUI_ROBOGait/...`、`~/rover_workspace/install/...`，没有 `~/RoboGait_ws/install`。

**原因：** `cd` 进目录**不会**自动加载这个工作空间。新开的终端 `.bashrc` 会 `source ~/GUI_ROBOGait/server/install/setup.bash`（里面还叠了旧的 `rover_workspace`）。那套里**没有** `robot_mppi_navigation`。

**处理：** 每次开这个车，在 launch **之前**再 source 本仓库（要放在最后，才能盖过 GUI/旧仓库）：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source ~/RoboGait_ws/install/setup.bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | head
# 第一行应是 /home/robogait/RoboGait_ws/install/...
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

---

### 7.18 第一个目标能到，往回走（朝向相反）车不动

**现象：** 同向目标几秒内 `Goal succeeded`。再点回原来的点（箭头和车头几乎相反），车几乎不动；约 20 秒后 `Failed to make progress`，然后 `Wait` 再试。

**原因：** 差速车禁止倒车时，目标在后方应先原地转再前进。进度检查只看平面位移，转的时候容易被 20 秒掐掉。

**处理：** **没有**第二套局部控制器（Rotation Shim 已去掉）。仍是纯 MPPI，`vx_min: 0` 不倒车；进度放到 45 秒，PathAngle 加重让它先转到路径朝向再往前开。yaml 软链，**不用 colcon**，重启整栈。

---

### 7.19 规划绕远（剩余十几米）、突然加速、容易撞

**现象：** 小实验室里 Nav2 面板一开始 **Distance remaining ~14 m**；绿线绕远或穿未知区。起步一下子冲很快。贴桌子/家具容易蹭。

**原因：**
1. NavFn `allow_unknown: true` 且全局代价图 `track_unknown_space: true`，路径会绕未知格子，长度比直线远好几倍。
2. 平滑器 `max_accel` 1.0 m/s²，0.80 限速大约 0.8 秒就拉满，感觉像猛冲。
3. 局部膨胀只有 0.22 m，雷达看障碍最远 2.5 m，贴家具时余量不够。**不要**把 `obstacle_min_range` 改回 0（会把车身画成障碍）。

**处理：** 已改日常 yaml（跟线包同步）：规划改 A*、不穿未知；加速现为 **0.70 m/s²**（0.45 空地爬不起来，1.0 又像猛冲）；局部膨胀 0.28、障碍看到 4 m。仍是纯 MPPI。yaml 软链，**不用 colcon**，先停掉再：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

点目标后看剩余距离应接近直线（几米），不应一上来十几米。

---

### 7.20 启动后 bno055 反复挂、RViz 没了，以为导航没起来

**现象：** 终端里 IMU 红字连刷：`Wrong READ-request response header`、`Unable to set IMU into config mode`、`Payload length mismatch`、`device disconnected or multiple access on port`，然后 `bno055 process has died` 又马上自己起来。同时可能看到 `RPLidar health not OK … Soft-reset`。过几秒又出现 `rviz2: process has finished cleanly`。

**原因：** 这几件事叠在一起，看起来像整栈坏了，其实不是：

1. **导航已经起来了。** 日志里只要有 `lifecycle_manager_localization: Managed nodes are active` 和 `lifecycle_manager_navigation: Managed nodes are active`，就可以摆位、点目标。雷达软复位成功后会有 `RPLidar health status : OK`。
2. **BNO055 串口启动时经常读脏。** 上次进程没读完的 45 字节还堵在口里，下一次读 chip-id 会报 `Payload length mismatch … awaited=45`，驱动一退 launch 再拉，两套抢 `/dev/bno055`（`multiple access on port`）。导航用轮速也能走，IMU 只是辅助 EKF。
3. **`rviz2 … finished cleanly` = 你把 RViz 窗口叉掉了。** 导航节点还在。关窗口后看不见地图，不要因此 Ctrl+C。

**处理（导航）：** 已经 `Managed nodes are active` 时 **不要 Ctrl+C，也不要关 RViz**。用 **2D Pose Estimate** 把激光贴到墙上（可以摆多次），再点 **2D Goal Pose**。`Failed to transform initial pose in time` 可忽略，位姿已经设上了。

**处理（IMU 刷屏 / 车自己转 / 必须带 IMU）：** IMU **要开**。挂掉是因为 `/dev/bno055` 串口协议乱了（`error 0x6`、读空），不是导航把 IMU 删了。现仍启动 bno055，但 **20 Hz**（不要改回 100）、失败最多 5 秒打一条、乱的角速度不发给 EKF。要吃到驱动修改需编一次 `bno055`。

不要关 RViz，也不要因为 IMU WARN 就 Ctrl+C。等到两行 `Managed nodes are active`，2D Pose 贴墙，再点 Goal。

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bno055 --symlink-install
source install/setup.bash
```

---

### 7.21 车一直很慢、像匀速爬，空地也不加快

**现象：** 点 Goal 后车能走，但全程差不多一个慢速；走廊/空地也不加速。

**原因：** 没有单独的「恒速巡航」。Humble MPPI 每个周期在 `0 … vx_max` 里采样，空地本来可以用更高的线速度，靠近膨胀障碍会自己减速。之前为了防撞/防猛冲把两处都压低了：

1. `vx_max: 0.80`（上限太低，室内看起来像恒速）。
2. 平滑器 `max_accel: 0.45`（到 0.80 大约要 1.8 秒，实验室短距离根本拉不满）。

**处理：** 改的是日常文件 **`src/robot_mppi_navigation/config/nav2_pure_mppi_params.yaml`**（跟人/跟线要同步改 `robot_simple_commander/config/nav2_mppi_params.yaml`）。现为 `vx_max: 1.00`（上限）、`max_accel: 0.70`、`vx_std: 0.22`。**不要改到 1.2**。yaml 软链，**不用 colcon**，重启整栈后再测一段空地。

---

### 7.22 录轨迹：`Need at least 2 poses, got 1`，没有 yaml

**现象：** `record_trajectory` 起来了，RViz 也开了；终端有 `Sampled 1 pose(s)`；随后 `rviz2: process has finished cleanly`，再 Ctrl+C，报 `Need at least 2 poses, got 1`，`record_path` exit 1。

**原因：** 还没开车，或保存代码把 ROS 的 `Path` 消息和文件路径搞混了（日志 `Path.__init__() takes 1 positional argument but 2 were given`）。关 RViz 窗口**不会保存**。把录和跟两条命令贴在同一个终端会连着跑：Ctrl+C 结束录制后马上又开 follow。

**处理：** 已修好保存。先编一次。RViz 不要叉。手柄若先报 `Couldn't open joystick /dev/input/js0` 等几秒，出现 `Opened joystick` 再开。等红激光后用 **2D Pose Estimate** 摆准再开，绿色痕迹变长后，只在**录制终端** Ctrl+C，看到 `Saved N poses` 再另开终端 follow。成功文件在 `src/robot_simple_commander/recorded/<name>.yaml`。

### 7.23 跟轨迹：车不动 / `Follow path was rejected` / 启动时按了 Ctrl+C

**现象：** `follow_trajectory` 开了，RViz 有图；或日志疯狂刷 `Waiting for amcl_pose` / `Publishing Initial Pose` / AMCL `Setting pose (0,0,0)`，Ctrl+C 才停。或刚起来就刷 `Invalid frame ID "odom"`，你按了 Ctrl+C，随后 `controller_server` / `rviz2` 红字退出。或 `Follow path was rejected`，车不动。

**原因：**
- 旧版 `waitUntilNav2Active()` 会自己往图上原点发假位姿，并死等 `/amcl_pose`。雷达还没起来时 AMCL 不回报，于是一秒刷几千行。
- **雷达软复位要几秒，底盘 / EKF 才发 `odom`。** Nav2 若立刻起来，costmap 会报 `base_link → odom` 不存在。这不是坏了。这时 Ctrl+C，关机红字（`context is invalid`、`exit code -6`）是正常噪音。

**处理：** 跟线 launch 已改成和日常导航一样：**先等 `/scan` 和轮速，再开 Nav2**。面板会写 **正在等底盘（雷达复位和 CAN）。先别按 Ctrl+C。**，好了才写 **导航已就绪**。

1. `colcon build --packages-select robot_simple_commander --symlink-install && source install/setup.bash`
2. `ros2 launch robot_simple_commander follow_trajectory.launch.py`
3. **等 15–20 秒**（这段时间 RViz 还没开）。终端出现 `Hardware topics are live. Starting Nav2` 后才会出图。直到面板写 **导航已就绪**（不要在雷达 RESET 期间按 Ctrl+C）
4. **2D Pose Estimate** 摆到轨迹起点（激光贴墙）
5. **开始** 开车。跟完点 **回到起点**。**停止** 只停车，程序还在；退出才是这个终端 Ctrl+C

若没有跟随面板：Add New Panel → `robot_simple_commander/Follow`。开始提示先摆位姿 = 还没用过 2D Pose Estimate。离起点超过 0.8 m 时开始会被拒绝，先点回到起点。

`mini_lab1.yaml` 若是旧地图上录的，跟 `mini_lab3` 对不齐时，用当前图重新录一条。

---

### 7.24 去程能到，原路返回卡住；Ctrl+C 杀不掉 `planner_server`

**现象：** 第一个目标能开到。再点回原来的位置（或原路返回），车不动、像死机。终端刷：

- `[compute_path_to_pose] Waiting for async process to finish`
- `Action callback is still running and missed deadline to stop`
- 狂按 Ctrl+C 只看到 `user interrupted with ctrl-c again, ignoring...`
- 最后 `escalating to SIGKILL`，`planner_server` exit -9
- 中间可能夹一行 `Extrapolation Error … Requested time … into the past`（`Requested time` 一直不变）

**原因：** 不是电机坏了。去程结束时车停在目标附近，四周是膨胀障碍，回程规划容易失败。旧行为树失败就 **清空整张全局代价地图**，Humble 的 `planner_server` 会在 `waitForCostmap` 里**死等**地图变 current；这个循环**不看 Ctrl+C**。去程用的旧目标时间戳（约 20 秒前）也不在 TF 缓存里了。你狂按 Ctrl+C，launch 只能等 5 秒再 SIGKILL。

**处理（已经卡死时）：** 不要再按 Ctrl+C。另开一个终端：

```bash
pkill -9 -f planner_server
pkill -9 -f rviz_goal_navigation
```

提示符回来后再 launch。

**处理（代码已改，需编一次 C++）：** 规划等待最多 2 秒；回程会在车脚下清开一小圈；失败改为等待而不是清空整图；TF 一律用最新时刻。然后：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_util nav2_planner nav2_navfn_planner nav2_behavior_tree --symlink-install
source install/setup.bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py \
  map:=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/mini_lab2.yaml
```

对齐激光 → 去程 Goal → 到了之后在 **Navigation2 面板点 Cancel**（或等 Goal succeeded）→ 再点返回的 Goal。不要在还没到的时候连续点两个目标。

---

### 7.25 云台：`Failed to set position control mode` / `There is no status packet`

**现象：** `ros2 run robogait_gimbal dynamixel_driver` 立刻退出。USB 口能打开，但切位置模式失败。

**原因：** 打开的是 **U2D2 适配器**，适配器本身是通的，但舵机没有回包。U2D2 **只传数据，不给舵机供电**。2026-10-11 这根适配器序列号是 **FT8ISNFV**（旧的 `FT66WQS9` 已经不在机器上；配置还写旧号时，口能在列表里看到，驱动却打不开）。总线上是 ID 1 **XL430-W250**（水平）和 ID 2 **XC330-T288**（俯仰）。

**处理：**

1. 给舵机 **12 V**（Dynamixel 电源口 / Power Hub，不是只插 USB）。舵机上的 LED 应亮或闪。
2. U2D2 拨动开关拨到 **TTL**（XL430 是 TTL，不是 RS-485）。
3. 3 针 / 4 针舵机线插紧（U2D2 ↔ XL430）。
4. 不要用裸的 `ttyUSB*`（雷达/IMU 也会占号）。默认口：

```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0
```

5. 两颗舵机在同一根总线：启动日志应有 `pan=id1` 和 `tilt=id2`。缺 ID2 就是俯仰线没接好。

供电和线都好了再：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run robogait_gimbal dynamixel_driver
```

日志应有 `Dynamixel pan+tilt ready`。然后再分别点左右 / 上下：

```bash
ros2 topic pub --once /camera_position std_msgs/msg/Float32 "{data: 20.0}"
ros2 topic pub --once /camera_tilt std_msgs/msg/Float32 "{data: 15.0}"
```

---

### 7.26 相机 SDK：RViz 没有点云 / `person_sdk` 打不开设备

**现象：** `test_gimbal` 起来了，但 RViz 里 `/orbbec/points` 是空的，或日志 `No K4A/Femto device`。

**原因：** Femto Mega 必须走 **USB3**；驱动要用 **Orbbec K4A 包装库**（`~/orbbec_wrapper_v2/lib`），不能用微软自带的 `libk4a`。Nuitrack 和 SDK **不要同时开**（会抢相机）。

**处理：**

```bash
lsusb | grep -i 2bc5
ros2 launch robogait_perception person_source.launch.py
# RViz Fixed Frame = base_link；Mini 车身 + 顶上青色相机 + 彩色点云。
# 橙色箭头 = 镜头朝向。绿球是人体跟踪，默认关掉。
```

人体球没有、只有点云：演示给老师看时默认关掉绿球（RViz 里 Person）。站到镜头 1–4 m 内。**默认点云只显示人体**（不是整间房）；要看房间见 **7.31**。设备都没开时点云全空：查 USB3。**跟人/骨架不要用 SDK**，见 **7.34**。

终端若出现 `incompatible QoS` / `RELIABILITY_QOS_POLICY`：见 **7.27**。

---

### 7.27 相机 SDK：终端刷 `incompatible QoS`，或 RViz 里找不到相机在哪

**现象：** 点云/彩图其实有了，但终端反复 `requesting incompatible QoS` … `RELIABILITY_QOS_POLICY`；或者 3D 里只有点和三根小轴，不知道哪是镜头。狂按 Ctrl+C 时 `user interrupted … ignoring`，最后 `person_sdk` `exit code -2`。

**原因：** 节点以前用 Best Effort 发点云，RViz 默认用 Reliable 订，对不上就不给这个订阅发。原点三色轴就是 `orbbec_link`（相机），太小不容易认。Ctrl+C 要等相机采集线程退出，连按会被 launch 当成二次中断后强杀。

**处理：** 重新编译后再开一次（关干净旧 RViz）：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
```

RViz 左边勾上 **CameraPose**：橙色箭头 = 镜头朝向，白字 Camera。Displays 里 **RobotModel** 是整车（底盘 + 雷达 + 青色相机）。关程序按 **一次** Ctrl+C 等几秒即可。

---

### 7.28 点云只有坐标系，想看到整台车 / 相机装在车上

**现象：** RViz 里只有三根轴或一个箭头，老师看不出相机装在 Mini 上面；或者车和点云不在一起。

**原因：** 坐标系只是一种画法。要看「整车」，必须发 Mini 的 URDF（`robot_state_publisher`），RViz 开 **RobotModel**，Fixed Frame 用 `base_link`（不要再用单独的 `orbbec_link` 当世界）。

**处理：**

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
```

应看到：轮子+底盘+顶上青色相机，橙色箭头指向点云。相机左右/高低不对：改 `src/robogait_perception/urdf/orbbec_gimbal.urdf` 里柱子/云台关节的 `origin`。

已经开着导航（`rviz_goal_navigation`）时不要再起第二套 URDF：`person_layer.launch.py` 已设 `use_robot_model:=False`。导航 RViz 里也可 Add → RobotModel → `/robot_description`。

---

### 7.29 云台不跟着人转 / RViz 里相机焊死在车上

**现象：** 只看到点云和车，人走了镜头不转；或实物舵机在转，RViz 里青色相机不动。

**原因：** `person_source.launch.py` **只看相机、不开云台**。跟人要 `camera_tracker` + `dynamixel_driver`（`test_gimbal.launch.py`）。车模上的左右/俯仰关节要订阅 `/gimbal_joint_states`，这个话题由 `dynamixel_driver` 发。

**处理：** 先停掉旧 launch，云台 12 V 上电，再：

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

站到镜头正前方 1.5–3 m、**身体对着相机**。启动后云台先停在初始角（`idle_pan_ticks=2048` 正前方），终端应出现 `waiting for person`。Nuitrack 认出人后：`Skeleton detected`、`Person locked joint=torso`，然后 `Person locked — gimbal tracking`。跟丢约 1.5 s 回到初始角。

初始朝向不对：改 `src/robogait_gimbal/config/gimbal.yaml` 的 `idle_pan_ticks` / `idle_tilt_ticks`（1024≈右 90°，3072≈左 90°；俯仰 **数字越小越抬头**，默认 1550≈看站立的人上半身）。然后重启 `test_gimbal`。

终端一直 `Waiting for person`：先确认 overlay 了旧空间、且没有同时开 `person_sdk`。站直、正面、离镜头 **1.5–3 m**。若日志一直是 `Person from depth` / `joint=depth` 而没有 torso：那是 SDK+k4abt CPU，见 **7.34**。舵机无应答见 **7.25**。跟踪已开但完全不转见 **7.33**。

---

### 7.30 云台跟人：Color 里人还在房间，RViz 里人飞上天

**现象：** 检测到人、云台一动，RViz 里的人和点云飞到格子上方；Color 画面里人仍在房间里（实物镜头没有对着天花板）。

**原因：** 点云在相机坐标系 `orbbec_link` 上。实物舵机转了，但 URDF 俯仰关节转的方向反了，RViz 把镜头的 X（深度）转到朝天，点云就跟着喷上天。不是车体 `base_link` 飞了。深度备用已经不抬头（`pos.y = 0`），但骨架帧仍会发俯仰；只要模型俯仰符号反了，云台一动就会上天。

**处理：** 把 `src/robogait_gimbal/config/gimbal.yaml` 里 dynamixel 的 **`tilt_urdf_sign` 设成 `-1.0`**（默认已是 -1）。这只改 RViz 模型，不改实物舵机。若人左右走时 RViz 左右也反了，再把 `pan_urdf_sign` 改成 `-1.0`。停掉旧进程后重新编译再开：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

检测后左右走，人应留在车前方地面附近。还觉得慢：把同一文件里的 `profile_velocity` 再加大（例如 500）。

---

### 7.31 RViz 点云只想看人（不要地板、墙、家具）

**现象：** `/orbbec/points` 把整间房都画出来了，人看不清；或者默认开了「只显示人」后点云几乎是空的。

**原因：** 默认 `cloud_person_only: true`：有骨架时用 Azure 人体分割图，只留人身上的点；没有骨架时用躯干周围约 0.55 m 的圆柱，并丢掉比躯干低太多的地面点。彩图 `/orbbec/color/image_raw` 和深度图 **不会** 被裁切。没检测到人时，点云可以是空的。

**处理：**

```bash
# 停掉旧的 person_source / test_gimbal，重新编译后再开
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception --symlink-install
source install/setup.bash
ros2 launch robogait_perception person_source.launch.py
# 跟人演示也可用：ros2 launch robogait_gimbal test_gimbal.launch.py
```

站到镜头正前方 1.5–3 m。人应是一团彩色点；墙和地没有。

给老师看**整间房**：把 `src/robogait_perception/config/perception.yaml` 里 `cloud_person_only` 改成 `false`，保存后重启上面的 launch。圆柱太瘦/太胖（无骨架时）：改 `cloud_person_radius_m`。

---

### 7.32 画面里人上下跳 / 直线跟随要把镜头冻住

**现象：** Color 里人走路时整个人忽高忽低，云台跟着每一步点头；或走廊直线测试时不希望镜头动。

**原因：** 以前锁的是胸口或深度块（腿/上半身一团），走路质心上下晃，俯仰 PID 太灵敏就会猎振。步态分析要锁**骨盆/肚脐**。深度块只给人“在不在”的水平跟踪，**不能当步态中心、也不用来俯仰**。

**处理：**

1. **锁躯干/骨盆：** 云台默认走 **Nuitrack**（`joint=torso`），不要用 SDK 的 `joint=depth`。站到镜头**正前方 1.5–3 m**、身体对着相机。若一直是 `Person from depth` / `joint=depth`：见 **7.34**。
2. **画面同一高度：** `gimbal.yaml` 里 `target_elevation_deg: 0.0`（光轴对着腰）、`tilt_dead_zone_deg: 5.0`、`tilt_kp: 0.22`、`tilt_filter_window: 18`。俯仰只在坐下/站起或车抬头时慢跟，不跟步伐。水平 `kp: 1.0` 仍然快，拐角能盯住人。
3. **直线跟随冻镜头：** 用 **B3**，launch 已经把两轴 `freeze_gimbal` 设为 true，不用改 `gimbal.yaml`。云台停在 `idle_pan_ticks` / `idle_tilt_ticks`。手动冻（不跑 B3）才去 yaml 里改。
4. **拐角走廊：** `freeze_gimbal: false`，`enable_gimbal_pan: true`。车转弯时镜头先水平跟上；俯仰可继续开（慢），或设 `enable_gimbal_tilt: false` 只转左右。

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
# 改完 yaml 后不必重编 Python；改了 person_sdk.cpp 才需要：
# colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
ros2 launch robogait_gimbal test_gimbal.launch.py
```

人仍上下跳：把 `tilt_dead_zone_deg` 再加大到 6，或把 `tilt_kp` 再减小。一直没有骨架：见 **7.34**。`joint=depth` 却完全不转：见 **7.33**；两边的 `freeze_gimbal` 都必须是 `false`。

---

### 7.33 终端已是 `joint=depth` / `Person locked — gimbal tracking`，但云台不转

**现象：** `test_gimbal` 日志有 `Person from depth (no skeleton, pan only)`、`Person locked joint=depth pixel=…`，接着 `dynamixel_driver: Person locked — gimbal tracking`，Color 里人偏在画面一侧，实物云台和 RViz 相机都不左右转。从未出现 `joint=pelvis`。

**原因：** 还没有骨架时深度块只负责**水平**（把人留在彩图里，骨架才锁得住骨盆），不是“先别动”。旧逻辑会把相同的像素误差只发一次，而第一次指令往往落在舵机尚未进入 tracking 的 0.08 s 窗口里，后面又被“重复误差跳过”，镜头就焊死。`freeze_gimbal: true` 也会冻住。俯仰必须等 `joint=pelvis`。

**处理：**

1. 打开 `src/robogait_gimbal/config/gimbal.yaml`，确认 **camera_tracker 和 dynamixel_driver 的 `freeze_gimbal` 都是 `false`**，且 `enable_gimbal_pan: true`。
2. 停掉旧 launch，重新编译再开：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

3. 站到镜头正前方 **1.5–3 m**、全身对着相机。Nuitrack 下应出现 `joint=torso` 和 `Pan skeleton pixel=…`，镜头左右跟上。
4. 若一直 `joint=depth` / 没有 pelvis/torso：不要再调 k4abt CPU，见 **7.34**。

---

### 7.34 一直是 `joint=depth` / 没有 `pelvis` 或 `torso`，认不出骨架

**现象：** 日志永远是 `Person from depth (no skeleton)` 或 `Person locked joint=depth`，从来没有 `Person locked joint=pelvis` / `joint=torso`。Nuitrack 人数也不涨。

**原因：** 默认曾经走 **Orbbec K4A + Azure Kinect Body Tracking（libk4abt）CPU 模式**。这台 NUC11TNKv7 只有 Intel Iris Xe，**没有 NVIDIA CUDA**。CPU k4abt 又慢又不可靠，所以骨架出不来，只能用深度团块凑合。以前在这台车上真正能出骨架的是 **Nuitrack**（`~/rover_workspace_final` 的 `nuitrack_person_node`）。Nuitrack 和 SDK **不能同时开**（抢同一台 USB 相机）。

**处理：** 停掉所有 `person_sdk` / 旧 `test_gimbal`，然后：

```bash
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
# 默认已经是 person_source:=nuitrack
```

找不到 `orbbec_person_publisher`：必须先 source 旧空间再 source 本仓库（顺序不要反）。站到镜头正前方 **1.5–3 m**、身体对着相机。期望：Nuitrack `Skeleton detected`、`/orbbec/num_users` ≥ 1、`Person locked joint=torso`，然后云台左右转。

日志已经 `Skeleton detected` / `joint=torso`，但 RViz 点云是空的：骨架在工作，只是坐标系没接到车上，见 **7.35**。不要贴到 1 m 以内（Nuitrack 会丢骨架，云台会猛转）。

给老师看**整房点云**才用 SDK，且不要开人体跟踪：

```bash
ros2 launch robogait_perception person_source.launch.py person_source:=sdk
```

`perception.yaml` 里保持 `enable_body: false`。

---

### 7.35 Nuitrack 日志已检出人，但 RViz 什么也没有

**现象：** 终端已经有 `Person source: Nuitrack (torso)`、`nuitrack_person_node`、`Skeleton detected`、`Person locked joint=torso`、`Pan skeleton` / `Person locked — gimbal tracking`，云台也在转。但 RViz 点云是空的，还刷：

```text
Message Filter dropping message: frame 'rgbd_camera_link_optical' ... queue is full
```

RViz 左边 Color / Depth 写着 **No Image**（Nuitrack 不发彩图）。走近到 0.7–1.1 m 时水平舵机会猛打到限位，随后 `Waiting for person`。

**原因：** Nuitrack 点云发在坐标系 **`rgbd_camera_link_optical`**（镜头光学系：X 右、Y 下、Z 朝外）。车上 URDF 只有 **`orbbec_link`**（X 朝前、Z 朝上）。中间没有 TF 时，RViz Fixed Frame 是 `base_link`，点云变换失败，队列塞满就被丢掉——**不是没检测到人**。Nuitrack **不发** `/orbbec/color/image_raw`，所以彩图本来就是空的。贴太近（小于约 1 m）Nuitrack 会丢掉骨架。`libcusolver.so.9.0` / `libKinect2DepthProvider.so` / GPU 模块那两行 **可以忽略**（Nuitrack 已在 CPU 上跑）。

**处理：** 停掉旧 launch，重新编译感知/云台后，**先 overlay 旧空间再 source 本仓库**：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robogait_perception robogait_gimbal --symlink-install
source ~/rover_workspace_final/install/setup.bash
source ~/RoboGait_ws/install/setup.bash
ros2 launch robogait_gimbal test_gimbal.launch.py
```

RViz **Fixed Frame 保持 `base_link`**（不要改成 `rgbd_camera_link_optical`）。应看到点云在 Mini 车前方；Nuitrack 用的 `orbbec_nuitrack.rviz` 不再开 Color/Depth，不会再出现 No Image。站到镜头正前方 **1.5–3 m**，不要贴太近。过近时日志会写 `Person too close`，云台不再往限位猛转（`gimbal.yaml` 的 `min_track_z_m`）。

---

### 7.36 和 `rover_workspace_final` 同时用这台车

**现象：** 两个人各开各的包，车乱走、雷达或 IMU 打不开、`/cmd_vel` 有两个发布者。

**原因：** 这台 NUC 的 `.bashrc` 把 `ROS_DOMAIN_ID` 固定成 **10**。`RoboGait_ws` 和 `~/rover_workspace_final` 都开在这个域里，话题是通的。两边都占同一套硬件：`/dev/rplidar`、`can0`、手柄、`/cmd_vel`。雷达口两边都写 `/dev/rplidar`。IMU 名字不同（我们是 `/dev/bno055`，他们是 `/dev/serial/by-id/usb-0403_6014-if00-port0`），但是同一颗芯片，只能有一个进程打开。`.bashrc` 只 source 了 `GUI_ROBOGait`（没有底盘驱动），**不会**自动把他们的包拉进来。冲突发生在两边同时 launch 的时候。

**处理：** 同一时间只开一边。开之前先看没有别人的节点：

```bash
ros2 node list
```

有输出就先等对方停掉。今天的建图 / 导航 / 跟路径不要 `source ~/rover_workspace_final`。

---

### 7.37 建图时雷达扫描线悬在地图上面

**现象：** 侧面看，彩色扫描线是悬空的一条，没有贴在地图上。

**原因：** 雷达装在车身上方约 **0.33 m**（`base_link` → 底盘 0.061 m + 载荷 0.1198 m + 雷达关节 0.145 m）。旧工作空间里雷达也在这个高度，侧面看同样会悬空。地图画在地面这一层。

**处理：** 建图 launch 里的 `flatten_scan` 把同一圈扫描复制到 `/scan_flat`，贴在 `base_link` 高度，也就是地图那一层。`slam_toolbox` 仍订阅原来的 `/scan`，墙的位置不变。建图 RViz 的 LaserScan 话题是 `/scan_flat`。改完要重新开一次建图：

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
```

窗口标题有 `*` 时不要保存，否则会把屏幕上的旧话题写回去。

---

