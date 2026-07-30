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

1. **`robot_mppi_navigation`** = **A1**：RViz 点目标。硬件 + Nav2 + MPPI + 安全闸。  
2. **`robot_simple_commander`** = **A2**：键盘**录轨迹** / **自动跟随**。  
   **与上包无依赖**（自带栈副本，允许代码重复）。

**模式 B — 跟人**（`robogait_*`，Python）

3. **`robogait_person_interfaces`** = 消息 `User.msg`（检测 + xyz）。  
4. **`robogait_perception`** = Orbbec 桥接 → `user_raw`。  
5. **`robogait_gimbal`** = 云台（像素 PID + Dynamixel 舵机）。  
6. **`robogait_person_follow`** = 人 → Nav2 目标；**底盘速度仍全由 MPPI**（无 PID 混速）。

```text
模式 A（robot_* 纯机器人）              模式 B（robogait_* 跟人）
  A1 robot_mppi_navigation                B0 云台 → robogait_gimbal
  A2 robot_simple_commander               B1 一站跟人 / B2 叠人层（人→goal→MPPI）
底盘 [v,ω] 只来自 MPPI；人只提供目标。不要两套 Nav2 同时起。
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
| 开硬件、定位、Nav2、**MPPI**、平滑、安全闸 | **模式 2**（录/跟路径）— 那是另一个独立包 |
| 预检、录诊断 bag、雷达复位 | 不给 `robot_simple_commander` 当依赖 |

日常一键：

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

---

### 2.2 `launch/` — 启动文件（总开关）

> 小白记法：launch = 「一键开哪些程序」。日常只记顶层那一个。

| 文件 | 是干什么的 | 你什么时候用 / 改 |
|------|------------|-------------------|
| **`rviz_goal_navigation.launch.py`** | **顶层一键**：把硬件层 + 导航层 + 可选 RViz 拼在一起 | **每天开车用这个**。改「默认开不开 RViz、默认地图」等顶层参数时看它 |
| **`nav2_mppi_controller.launch.py`** | 导航「大脑」：EKF、地图/AMCL/SLAM、Nav2（含 **MPPI**）、速度平滑、安全闸；并把 MPPI 的速度改名叫 `cmd_vel_nav` | 一般**不要单独开**（顶层已经 Include）。改速度话题怎么串、生命周期节点列表时看它 |
| **`rover_mini_hardware.launch.py`** | 只开底盘驱动、雷达、IMU、机器人模型（URDF） | **只测传感器/底盘**时用。改「硬件从哪启动」时看它 |
| **`ekf_amcl_localization.launch.py`** | 调试用，主要起 EKF | 日常不用 |

顶层常用参数（加在 `rviz_goal_navigation` 后面）：

| 参数 | 意思（人话） |
|------|----------------|
| （默认） | 用地图 + AMCL 定位，开 RViz |
| `amcl:=False` | 不做真定位，只测软件通不通 |
| `slam:=True` | 边走边建图（暂时不用已有地图） |
| `use_rviz:=False` | 不开界面 |
| `map:=/绝对路径/xxx.yaml` | 换一张地图 |

```text
rviz_goal_navigation          ← 你平时只敲这个
   ├── rover_mini_hardware    车 + 传感器
   ├── nav2_mppi_controller   定位 + MPPI + 安全闸
   └── rviz2（可关）
```

**坑：** 顶层已经开着时，不要再单独开一遍 `nav2_mppi_controller`，会起两套导航。

---

### 2.3 `config/` — 参数旋钮（改行为优先来这里）

> 小白记法：改「车开多快、怎么避障、定位松紧」→ 改 yaml，**通常不用改 Python**；改完**重启 launch** 即可（用了 `--symlink-install` 时）。

| 文件 | 是干什么的 | 想改什么就打开它 |
|------|------------|------------------|
| **`nav2_pure_mppi_params.yaml`** | **最重要**。Nav2 + **MPPI** 全套参数：最大速度、采样数量、8 个评分项（critic）权重、代价地图、AMCL、平滑器、安全闸超时等。里面 `FollowPath` 的 `plugin` 指向 `nav2_mppi_controller::MPPIController` | 车太快/太慢、转弯晃、贴障碍太近、到点不正、控制频率……**先改这个文件** |
| **`robot_localization_ekf.yaml`** | 轮速里程计 + IMU 怎么融合成 `/odometry/filtered` | 车在图上「漂」、直线走歪怀疑里程计/IMU 融合时 |
| **`sensor_topics_frames.yaml`** | 对照表：哪个话题、哪个坐标系叫什么（给人看 + 预检用） | 对不上话题名/坐标系名时查阅；一般**不改也能跑** |
| **`diagnostic_recording_topics.yaml`** | 录测试 bag 时要录哪些话题 | 想多录/少录一些话题做分析时 |

---

### 2.4 `maps/` — 地图

| 文件 | 是干什么的 | 小白注意 |
|------|------------|----------|
| **`lab2_new.yaml`** | 地图说明（分辨率、原点等），给 `map_server` / AMCL 用 | 换地图通常换「一对」yaml+图片 |
| **`lab2_new.pgm`** | 地图图片（黑白占用栅格） | 必须和 yaml **在同一目录** |
| **`lab2_new.posegraph`** + **`lab2_new.data`** | slam_toolbox 用的位姿图格式 | **当前默认导航没用上**（现在用 AMCL）；以后接 SLAM 定位才会用 |

---

### 2.5 `rviz/` — 可视化界面配置

| 文件 | 是干什么的 | 想改什么 |
|------|------------|----------|
| **`rviz_goal_navigation.rviz`** | 打开 RViz 时显示哪些东西（地图、激光、路径、Nav2 面板、Goal 工具等） | 想默认多显示一条话题、改 Fixed Frame 等，可改这个；也可在 RViz 里改完另存 |

---

### 2.6 `scripts/` — 给你直接敲的脚本

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`preflight_check_topics.sh`** | 查雷达、IMU、轮速、EKF 等话题有没有数据 | 栈起来后、发目标前跑一遍 |
| **`preflight_check_tf.sh`** | 查 `map→odom→base_link` 坐标变换通不通 | 车在图上不动/乱跳时先查 |
| **`record_navigation_test.sh`** | 按 `diagnostic_recording_topics.yaml` 录 rosbag | `./scripts/record_navigation_test.sh 测试名`，Ctrl+C 结束 |
| **`analyze_navigation_bag.py`** | 读 bag，算误差/平滑度/频率并出图 | `python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/某目录` |
| **`reset_rplidar.py`** | 给雷达发串口复位（health 报错时） | 也可用 `ros2 run robot_mppi_navigation reset_rplidar` |

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
| 里程计和 IMU 融合不对 | `config/robot_localization_ekf.yaml` |
| 急停逻辑 / 超时停车 | `robot_mppi_navigation/safety/` |
| 启动时多开/少开节点 | `launch/` |
| RViz 默认显示 | `rviz/rviz_goal_navigation.rviz` |
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
| 录轨迹 | `record_trajectory.launch.py` + 另开终端 `teleop_twist_keyboard`，Ctrl+C 保存 |
| 跟轨迹 | `follow_trajectory.launch.py`（自带 MPPI） |

---

### 3.2 启动文件 vs 节点（分类）

```text
launch/
  record_trajectory.launch.py   ← ros2 launch 入口
  follow_trajectory.launch.py   ← ros2 launch 入口
  parts/                        ← 片段，只被入口拼装
recorded/                       ← 你手动录的轨迹 yaml
paths/                          ← 内置考试路径
nodes/                          ← ros2 run 节点
lib/                            ← 库（path_io、generators），不是节点
safety/                         ← 安全闸逻辑
```

#### `launch/` — 启动入口

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`record_trajectory.launch.py`** | 录轨迹：硬件 + EKF + 地图 TF + 节点 `record_path` | `ros2 launch robot_simple_commander record_trajectory.launch.py` |
| **`follow_trajectory.launch.py`** | 跟轨迹：硬件 + `parts/nav_stack`（MPPI）+ 节点 `follow_path` | `ros2 launch robot_simple_commander follow_trajectory.launch.py` |

#### `launch/parts/` — 被拼装的片段

| 文件 | 是干什么的 | 怎么用 |
|------|------------|--------|
| **`hardware.launch.py`** | 底盘 / 雷达 / IMU | 只被入口 Include |
| **`nav_stack.launch.py`** | EKF + 定位 + Nav2/MPPI + 平滑 + 本包安全闸 | 只被 follow 入口 Include |

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
| **`nodes/record_path.py`** | 一边开，一边按距离采 `map` 里车的位置和朝向；你按 Ctrl+C 时写成 yaml | 默认采样间距、父/子坐标系、保存失败条件 |
| **`nodes/follow_path.py`** | 读 yaml →（若是 `local` 帧就先变到地图）→ 用 Simple Commander 发 FollowPath → **等 MPPI 把车开过去** | 默认 controller 名、是否 `wait_for_nav2`、反馈打印 |
| **`nodes/cancel_task.py`** | 告诉 Nav2：把 NavigateToPose / FollowPath 目标都取消 | 要取消的 action 列表 |
| **`nodes/run_safety_gate.py`** | 安全闸的 `ros2 run` 入口（薄封装） | 一般不用改；逻辑在 `safety/` |
| **`lib/path_io.py`** | 路径 yaml 的读和写（**不是节点**） | 文件格式字段（会连带跟/录都受影响） |
| **`lib/generators.py`** | 用数学生成考试路线（**不是节点**） | 测试路径长度、弯道形状 |
| **`safety/`** | 本包自己的安全闸业务逻辑 | 跟线时 `/cmd_vel` 闸门 |

---

### 3.4 `recorded/` 与 `paths/`

| 文件夹 | 放什么 | 怎么用 |
|--------|--------|--------|
| **`recorded/`** | **你手动录的**轨迹（默认 `lab_route.yaml`） | 录制默认写这里；跟线用 `path_file:=…/recorded/….yaml` |
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
| **`package.xml`** | 包名与依赖（含 `nav2_simple_commander`） | 加依赖 |
| **`setup.py`** / **`setup.cfg`** | 注册 `record_path` / `follow_path` / `cancel_navigation` | 新增加命令入口 |
| **`README.md`** | 本包英文说明 | 可同步更新 |
| **`test/test_path_io.py`** | 自动测路径生成和 yaml 读写 | 改 path_io/generators 后跑测试 |

---

### 3.6 本包：想改什么 → 去哪（速查）

| 我想… | 去哪个文件 |
|-------|------------|
| 录密一点 / 稀一点 | `nodes/record_path.py` 的 `min_separation`，或 launch 参数 |
| 跟线逻辑、等不等 Nav2 | `nodes/follow_path.py` |
| 换内置考试路线形状 | `generators.py` → 再跑 `generate_test_paths.py` |
| 手改一条路径点 | 直接编辑 `paths/*.yaml` 或你的录制文件 |
| 取消键行为 | `nodes/cancel_task.py` |
| 车跟线时快慢/晃（控制） | 本包 `config/nav2_mppi_params.yaml`（跟 `robot_mppi_navigation` 无关） |

### 3.7 本包常用命令（与 robot_mppi_navigation 无关）

```bash
colcon build --packages-select robot_simple_commander --symlink-install
source install/setup.bash

# ——— 录制（手动开车；默认写入包内 recorded/lab_route.yaml）———
# 终端 A
ros2 launch robot_simple_commander record_trajectory.launch.py
# 终端 B
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 开完车 → 终端 A 按 Ctrl+C 保存

# ——— 跟随刚录的 ———
REC=$(ros2 pkg prefix robot_simple_commander)/share/robot_simple_commander/recorded/lab_route.yaml
ros2 launch robot_simple_commander follow_trajectory.launch.py path_file:=$REC

# ——— 跟随内置考试路径 ———
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=left_90
ros2 run robot_simple_commander cancel_navigation
```

---

## 四、模式 B 包（Python，跟人）

> 逻辑参考 `~/rover_workspace`，**全部用 Python 重写**，不迁移 C++。  
> 依赖外部：`orbbec_person_publisher`（Nuitrack）、`dynamixel_sdk`。

### 4.1 `robogait_person_interfaces`

| 文件 | 作用 |
|------|------|
| **`msg/User.msg`** | `detection` + `x/y/z`（相机坐标系，z=距离） |

### 4.2 `robogait_perception`

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`nodes/distance_orbbec.py`** | 订阅 `/orbbec/*` → 发布 `user_raw`、`person_center_pixel` | `ros2 run robogait_perception distance_orbbec` |
| **`config/perception.yaml`** | 话题名、超时、hold | 改 Orbbec 话题时动它 |

### 4.3 `robogait_gimbal`（云台）

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`nodes/camera_tracker.py`** | 像素偏差 PID → `camera_position`（度） | `ros2 run robogait_gimbal camera_tracker` |
| **`nodes/dynamixel_driver.py`** | 角度 → XL430 位置 | `ros2 run robogait_gimbal dynamixel_driver` |
| **`config/gimbal.yaml`** | 串口、角度限位、PID | **改舵机设备路径动它** |
| **`test_gimbal.launch.py`** | B0：只测云台+感知（不起底盘 Nav2） | `ros2 launch robogait_gimbal test_gimbal.launch.py` |

### 4.4 `robogait_person_follow`

| 节点 / 文件 | 作用 | 命令 |
|-------------|------|------|
| **`nodes/distance_filter.py`** | `user_raw` → `user_coordinates`（平滑） | `ros2 run robogait_person_follow distance_filter` |
| **`nodes/person_nav_goal.py`** | 人坐标 → 保持间距的 `NavigateToPose`（MPPI 算完整 `[v,ω]`） | `ros2 run robogait_person_follow person_nav_goal` |
| **`person_follow.launch.py`** | B1 一站：A1 Nav2/MPPI + 人层 | 见下 |
| **`person_layer.launch.py`** | B2 终端 2：叠在已起的 Nav2 上 | 见下 |
| **`config/person_follow.yaml`** | 目标间距、死区、目标更新频率 | **调跟人距离动它** |

**控制链（跟人 = 纯 MPPI）：**

```text
人检测 → distance_filter → person_nav_goal → NavigateToPose
                                              ↓
                                     规划器 → MPPI → 平滑 → safety_gate → /cmd_vel
```

人**不**改 `/cmd_vel`。已删除旧的 `cmd_vel_blend` / `straight_follow`（PID 混控）。

**常用命令：**

```bash
# B0 只测云台
ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0

# B1 一站跟人（Nav2 + MPPI + 人→目标）
ros2 launch robogait_person_follow person_follow.launch.py

# B2 终端1 Nav2 + 终端2 人层
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py
```

---

## 五、工作空间其它文件

| 文件 / 目录 | 是干什么的 | 小白怎么用 |
|-------------|------------|------------|
| **`PROJECT_CHANGES.md`** | 本中文手册（与 `TEST_PLAN.md` 为仅有的中文文档） | 查「文件干什么、去哪改」 |
| **`PROJECT_CHANGES_EN.md`** | 英文手册（与中文对齐） | 给英文读者 |
| **`TEST_PLAN.md`** | 实机验收（中文）：准备 0/1 → T1–T8 → B0/B1/B2 | **上机按它做**；先 B0 云台再跟人 |
| **`NODES_AND_TOPICS.md`** | 节点↔话题图（英文） | 搞不清谁订谁发时看它 |
| **`README.md`** | 仓库首页简介（英文） | GitHub / 新人入口 |
| **`src/*/README.md`** | 各包说明（英文） | 与第二～四节互补 |
| **`src/navigation2/`** | 整份 Nav2 源码（含 MPPI 实现） | **进阶**：改控制器算法、critic 实现 |
| **`.vscode/settings.json`** | 打开 md 默认预览 | 可不管 |
| **`.gitignore`** | 不把 build/install/log 等提交进 git | 可不管 |

---

## 六、待办

| 项 | 状态 |
|----|------|
| ~~RPLidar health 2~~ | 已解决：`ros2 run robot_mppi_navigation reset_rplidar` |
| ~~BNO055 通信失败~~ | 已解决（07-21） |
| `slam:=True` 实测 | 可测 |
| 按 `TEST_PLAN.md` 实机勾选 | 待做（0/1 → T1–T8 → B0→B1） |
| 模式 A2 录制 + 跟随实测 | 用第三节命令；对应 T3–T6 |
| Mode B：`robogait_gimbal` + `test_gimbal` 实机 | 代码已就绪，待测 |
| Mode B：`person_follow` / `person_layer` 实机（纯 MPPI） | 代码已改，待测 |
| posegraph 定位接进 launch | 未做（现用 AMCL） |
| 自定义 MPPI critic | 未开始（在 `navigation2`） |
| Gazebo 仿真依赖 | 需要时再装 |
