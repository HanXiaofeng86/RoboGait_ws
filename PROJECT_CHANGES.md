# RoboGait_ws 项目手册

> 本文件记录 `mppi_navigation` 包（纯 MPPI 导航）的架构、文件作用与常用命令。  
> **英文版：** `PROJECT_CHANGES_EN.md` — **中英文须同步更新**（见 `.cursor/rules/update-changelog.mdc`）。  
> **语言约定：** 仅本文件（`PROJECT_CHANGES.md`）保持中文；包 README、`TEST_PLAN.md`、仓库根 `README.md` 等其它文档用英文。  
> **维护规则**：AI 每次修改/新增/删除代码文件后，必须同步更新本文件（及英文版）的文件关系、文件手册与命令手册。

---

## 一、项目概览

- 工作空间：`~/RoboGait_ws`
- 核心包：`src/mppi_navigation`（ament_python）
- 目标：Rover Mini 仅用 Nav2 MPPI 控制器导航；MPPI 生成完整速度对 (v, ω)，其它节点只允许透传或整组置零
- 当前阶段：模式 1 + 模式 2 代码全部完成；**硬件已恢复（2026-07-21）**；AMCL 默认开启 + 默认地图 `lab2_new`

> 本工作区打开 `.md` 默认进预览（`.vscode/settings.json`）。表格是标准 Markdown，预览里是正常表格，不会出现 HTML 标签。  
> **文件之间谁调用谁、关系图：见第二节。**

---

### 1.1 为什么这样分 Launch

和 Nav2 / Rover 常见做法一致：三层分离，顶层只做拼装。

| 层 | 职责单一 | 好处 |
|----|----------|------|
| `rover_mini_hardware` | 只起传感器和底盘 | 单独调试硬件，不碰导航 |
| `nav2_mppi_controller` | 只起定位 + Nav2 + 速度链 | 换地图 / AMCL / SLAM 时不用重起硬件 |
| `rviz_goal_navigation` | 一键把上面两层 + RViz 拼起来 | 日常只需记一条命令 |

**这样分配的优点**

- **顶层薄、下层可复用**：模式 2 只需再起 `follow_predefined_path`
- **RViz 可选**：`use_rviz:=False`
- **速度链 remap 集中**：全在 `nav2_mppi_controller` 里

**注意**：日常只跑顶层；不要同时再跑一遍 `nav2_mppi_controller`，否则会起两套 Nav2。

---

### 1.2 Launch 包含关系

```text
                    rviz_goal_navigation.launch.py
                           （日常只跑这个）
                                  │
          ┌───────────────────────┼───────────────────────┐
          ▼                       ▼                       ▼
   rover_mini_hardware            nav2_mppi_controller                 rviz2
      （硬件层）               （导航层）                （可选）
          │                       │
          │                       ├── EKF
          │                       ├── map + AMCL（默认）
          ├── driver              ├── planner / BT / MPPI
          ├── rplidar             └── smoother → safety_gate
          ├── bno055
          └── robot_state_publisher

                    follow_predefined_path.launch.py
                         （模式2：另开终端）
                                  │
                                  └──► 向已运行的导航层发 FollowPath
```

---

### 1.3 谁起什么 / 什么时候用

| 脚本 | 角色 | 硬件 | Nav2/MPPI | RViz |
|------|------|:----:|:---------:|:----:|
| `rviz_goal_navigation.launch.py` | **顶层一键入口** | Y | Y | Y（可关） |
| `nav2_mppi_controller.launch.py` | **导航栈本体** | N | Y | N |
| `rover_mini_hardware.launch.py` | **硬件层** | Y | N | N |
| `follow_predefined_path.launch.py` | **模式 2 发路径** | N | N（依赖已运行的栈） | N |
| `ekf_amcl_localization.launch.py` | 调试用 EKF | N | N | N |

| 脚本 | 什么时候用 |
|------|-----------|
| `rviz_goal_navigation` | 日常导航、设位姿、发 Goal |
| `nav2_mppi_controller` | 一般**不单独跑**；被顶层 Include |
| `rover_mini_hardware` | 只测雷达 / IMU / 底盘 |
| `follow_predefined_path` | 全栈已起后，另开终端发预定义路径 |
| `ekf_amcl_localization` | 日常不用 |

**一句话**：`rviz_goal_navigation` = 硬件 + 导航 + RViz；RViz 只负责看和点，不算导航。

---

### 1.4 定位模式

| 启动参数 | map→odom 谁提供 | 需要地图？ | 2D Pose Estimate | 适用场景 |
|----------|-----------------|------------|------------------|----------|
| **默认** `amcl:=True` | AMCL | 要（lab2_new） | 要（必须先标） | **真实导航** |
| `amcl:=False` | 静态恒等 TF | 有图但不定位 | 无效（没人订阅） | 只验证软件链路 |
| `slam:=True` | SLAM Toolbox | 不要（边建） | 一般不用 | 还没有地图时 |

---

### 1.5 控制链路

```text
RViz (Goal / Pose)
        │
        ▼
  bt_navigator ──► planner ──► /plan ──► MPPI
                                           │
                              /cmd_vel_nav ▼
                              velocity_smoother
                                           │
                         /cmd_vel_smoothed ▼
                              safety_gate（透传或置零）
                                           │
                                /cmd_vel ▼
                              driver → 底盘
                                           │
                    wheels + IMU ──► EKF ──┘
                    /scan + map ──► AMCL ──┘（回到 MPPI）
```

| 话题 | 发布者 | 订阅者 | 含义 |
|------|--------|--------|------|
| `/cmd_vel_nav` | MPPI / behavior | velocity_smoother | MPPI 原始输出 |
| `/cmd_vel_smoothed` | velocity_smoother | safety_gate | 限幅 / 加速度平滑后 |
| `/cmd_vel` | **仅** safety_gate | driver | 最终下发；只透传或全零 |

---

### 1.6 默认启动后节点分组

| 分组 | 节点 | 作用 |
|------|------|------|
| 硬件 | driver / rplidar / bno055 / rsp | 底盘、激光、IMU、TF 模型 |
| 定位 | map_server + amcl + ekf | 地图、粒子定位、里程计融合 |
| Nav2 | controller(MPPI) / planner / bt / smoother / … | 规划与跟踪 |
| 自研 | safety_gate | 最终 `/cmd_vel` 闸门 |
| 界面 | rviz2 | 可视化与手动设目标 |

---

## 二、文件关系（谁调用谁）

> 第二节讲「文件之间怎么连」；第三节才是「每个文件干什么」。  
> 图是纯文本，预览里直接能看（不用装 Mermaid 插件）。

### 2.1 一张总图：从启动到轮子

```text
你敲的命令
    │
    ▼
┌───────────────────────────────────────────────────────────┐
│  launch/                                                  │
│   rviz_goal_navigation.launch.py     ← 日常一键入口        │
│        │                                                  │
│        ├── include → rover_mini_hardware.launch.py        │
│        │                 └── 上游 mini.launch             │
│        │                      (底盘 / 雷达 / IMU / URDF)   │
│        │                                                  │
│        └── include → nav2_mppi_controller.launch.py       │
│                          ├── 读 config/*.yaml             │
│                          ├── 起 EKF / AMCL / Nav2 / MPPI  │
│                          └── 起 safety_gate（自研）        │
└───────────────────────────────────────────────────────────┘
        │
        ▼ 运行时话题（不是文件，但是关系的结果）
  /scan /imu /wheels → EKF → /odometry/filtered
  RViz Goal → 规划 → MPPI → /cmd_vel_nav → 平滑 → 安全闸 → /cmd_vel → 底盘
```

### 2.2 Launch 互相怎么套

```text
                    ┌─────────────────────────────┐
                    │ rviz_goal_navigation        │
                    │ （顶层：硬件+导航+可选RViz）  │
                    └─────────────┬───────────────┘
                                  │ include
              ┌───────────────────┴───────────────────┐
              ▼                                       ▼
┌─────────────────────────┐             ┌─────────────────────────┐
│ rover_mini_hardware     │             │ nav2_mppi_controller    │
│ 只传感器+底盘            │             │ EKF+定位+MPPI+安全闸     │
└─────────────────────────┘             └───────────┬─────────────┘
                                                    │ 读
                                                    ▼
                                        config/nav2_pure_mppi_params.yaml
                                        config/robot_localization_ekf.yaml
                                        maps/lab2_new.yaml（默认地图）

另两条「旁路」，不替代顶层：

  follow_predefined_path.launch.py
       │  要求：上面导航栈已经在跑
       ▼
  path_following/follow_path_client.py  →  发 FollowPath
       ▲
       │ 也可直接
  ros2 run mppi_navigation follow_path

  ekf_amcl_localization.launch.py  →  只调试 EKF，日常不用
```

| 关系 | 说明 |
|------|------|
| 顶层 **include** 硬件层 + 导航层 | 所以日常只需开一个 `rviz_goal_navigation` |
| 导航层 **加载** `config/` | 改 yaml 后重启 launch 即可，一般不用改 launch |
| 模式 2 launch **不**再起硬件 | 它只是往已运行的栈里「塞一条路径」 |

### 2.3 配置文件被谁用

```text
config/nav2_pure_mppi_params.yaml
    └─ 被 nav2_mppi_controller.launch.py 交给 Nav2 / MPPI / smoother / AMCL / safety 参数

config/robot_localization_ekf.yaml
    └─ 被同一 launch 里的 ekf_node（robot_localization）加载

config/sensor_topics_frames.yaml
    └─ 主要给人看 + 预检对照；节点不靠它启动

config/diagnostic_recording_topics.yaml
    └─ 被 scripts/record_navigation_test.sh 用来决定录哪些话题

maps/lab2_new.yaml (+ .pgm)
    └─ map_server（amcl 模式）默认地图

paths/*.yaml
    └─ 被 follow_path / follow_predefined_path / generate_test_paths 使用
```

### 2.4 自研 Python：入口 → 实现

```text
你敲的 ros2 run                      真正干活的代码
─────────────────                    ────────────────
safety_gate          ──nodes/──►     safety/cmd_vel_safety_gate.py
                                       ├─ stop_conditions.py
                                       ├─ pass_or_zero_policy.py
                                       └─ twist_copy.py

follow_path          ──nodes/──►     path_following/follow_path_client.py
                                       ├─ path_io.py（读写 yaml）
                                       └─ generators.py（生成 paths/*.yaml 的内容）

cancel_navigation    ──nodes/──►     path_following/cancel_client.py

idle_check           ──nodes/──►     preflight/idle_cmd_vel_check.py

reset_rplidar        ──nodes/──►     tools/reset_rplidar.py
                     （scripts/reset_rplidar.py 也是同一件事）
```

**记一句：** `nodes/` = 遥控器按键；`safety/`、`path_following/` 等 = 电路板。

### 2.5 scripts/ 和源码模块的关系

```text
scripts/preflight_check_topics.sh  ──►  preflight/topic_health_check.py
scripts/preflight_check_tf.sh      ──►  preflight/tf_tree_check.py
scripts/record_navigation_test.sh  ──►  读 diagnostic_recording_topics.yaml，调用 ros2 bag
scripts/analyze_navigation_bag.py  ──►  bag_analysis/ 下各个 metrics + generate_plots
scripts/generate_test_paths.py     ──►  path_following/generators.py  →  写出 paths/*.yaml
scripts/reset_rplidar.py           ──►  tools/reset_rplidar.py
```

### 2.6 运行时数据流（文件如何「接到一起」）

```text
[硬件层 launch]
  rplidar ──► /scan ─────────────────────┐
  bno055  ──► /imu/data ──┐              │
  driver  ──► /odometry/wheels ──┐       │
                                 ▼       │
                    EKF ← robot_localization_ekf.yaml
                                 │
                                 ▼
                      /odometry/filtered
                      TF: odom→base_link
                                 │
[导航层 launch]                   │
  map_server ← maps/lab2_new     │
  amcl ← /scan + 地图            │
         TF: map→odom            │
                                 ▼
  bt_navigator / planner / MPPI ← nav2_pure_mppi_params.yaml
                                 │
                                 ▼ /cmd_vel_nav
                         velocity_smoother
                                 │
                                 ▼ /cmd_vel_smoothed
                    safety_gate ← safety/*.py
                                 │
                                 ▼ /cmd_vel
                               底盘 driver
```

### 2.7 按「你想干什么」找文件

| 你想… | 先看这些 |
|-------|----------|
| 启动 / 分层 | `launch/` + 本节 2.2 + 第一节 |
| 只测传感器 | `rover_mini_hardware` + 第四节命令 4.8 |
| 改限速、MPPI、AMCL | `config/nav2_pure_mppi_params.yaml` |
| 改 EKF 融哪些量 | `config/robot_localization_ekf.yaml` |
| 安全闸逻辑 | `mppi_navigation/safety/` |
| 预定义路径测试 | `paths/` + `path_following/` + `follow_predefined_path.launch.py` |
| 预检 / 录包 / 分析 | `scripts/` + `preflight/` + `bag_analysis/` |
| 每个文件一句说明 | **第三节 文件手册** |

---

## 三、文件手册（按目录）

> **这段怎么读：** 下面是「文件夹里有什么文件」，不是「每天要运行的命令」。  
> - **操作机器人**：优先看 **launch/**、**scripts/**、第四节命令手册。  
> - **改参数**：看 **config/**。  
> - **文件怎么连在一起**：看 **第二节**。  
> - **`mppi_navigation/` 下面一堆 `.py`**：多半是源码实现；日常开车可以跳过，需要改逻辑或查 bug 再翻。

### 先记这张「必看 / 可跳过」

| 目录 | 必看？ | 一句话 |
|------|--------|--------|
| `launch/` | 是 | 用哪条命令启动什么 |
| `config/` | 调参时 | 导航/EKF 等参数写在 yaml 里 |
| `scripts/` | 是 | 预检、录包、雷达复位等小工具 |
| `paths/` / `maps/` | 测模式 2 / 换地图时 | 测试路线、实验室地图 |
| `mppi_navigation/safety/` 等 | 否（开发用） | 源码怎么拆的，不是启动清单 |
| `nodes/` | 否 | 只是 `ros2 run` 的薄入口 |

---

### launch/（启动文件 = 总开关）

| 文件 | 作用 | 你什么时候用 |
|------|--------------|--------------|
| `rviz_goal_navigation.launch.py` | **日常导航一键开**：硬件 + 定位 + MPPI + 安全闸 + RViz | 要在 RViz 标位姿、点目标 |
| `rover_mini_hardware.launch.py` | **只开传感器和底盘**，不开导航 | 查雷达 / IMU / 轮速是否正常 |
| `nav2_mppi_controller.launch.py` | 导航「大脑」（EKF、AMCL、MPPI、安全闸） | **一般别单独开**；顶层会自动带上 |
| `follow_predefined_path.launch.py` | 不点 Goal，改让车跟一条写好的路径 | 全栈已开时，做直线/转弯等固定测试 |
| `ekf_amcl_localization.launch.py` | 单独起 EKF 方便调试 | 日常不用 |

常用参数（顶层）：`amcl:=True`（默认，用地图定位）、`slam:=True`（边走边建图）、`use_rviz:=False`（不开界面）。分层关系见第一节。

---

### config/（参数文件 = 调旋钮，不是程序）

| 文件 | 作用 |
|------|--------------|
| `nav2_pure_mppi_params.yaml` | **导航总配置**：MPPI 怎么跟路径、限速多少、平滑器、安全闸超时、AMCL 等。改完一般不用重编译，重启 launch 即可 |
| `robot_localization_ekf.yaml` | **EKF 怎么融合**：轮速给 vx/vy，IMU 给转向角速度 → 输出 `/odometry/filtered` 和 `odom→base_link` |
| `sensor_topics_frames.yaml` | **对照表**：哪个话题、哪个坐标系叫什么（给人看 + 给预检用），本身不启动节点 |
| `diagnostic_recording_topics.yaml` | **录包要录哪些话题**的清单（给 `record_navigation_test.sh` 用） |

MPPI 基线数值（频率 20Hz、batch、critics 等）都在 `nav2_pure_mppi_params.yaml` 里；需要抠参数时打开该文件，不必先读 Python。

---

### mppi_navigation/（Python 源码包）

下面按**文件夹**说明。多数文件 launch 会间接用到；你很少需要 `python3` 直接跑它们。

#### safety/ —— 速度最后一道闸

> 规矩：只能 **原样放行（PASS）** 或 **改成全零（ZERO）**，不会自己另算一条速度去开。

| 文件 | 作用 |
|------|--------------|
| `cmd_vel_safety_gate.py` | **闸门本体**：听平滑后的速度 + 里程计 + 急停；决定发给底盘还是发 0；指令突然没了车还在动，也会补发 0 |
| `stop_conditions.py` | **什么情况必须停**：指令超时、里程计超时、按下急停 |
| `pass_or_zero_policy.py` | 把上面的条件落成「放行 / 置零」两种决定 |
| `twist_copy.py` | 小工具：复制速度消息、造零速、判断是不是零 |

平时不用手开；`rviz_goal_navigation` 会自动带上安全闸。

#### nodes/ —— 命令行入口（很薄，真正逻辑在别的文件夹）

| 文件 | 对应你敲的命令 | 实际干活的地方 |
|------|----------------|----------------|
| `run_cmd_vel_safety_gate.py` | `ros2 run mppi_navigation safety_gate` | → `safety/` |
| `run_follow_path_client.py` | `ros2 run mppi_navigation follow_path` | → `path_following/` |
| `run_cancel_navigation.py` | `ros2 run mppi_navigation cancel_navigation` | → `cancel_client.py` |
| `run_idle_cmd_vel_check.py` | `ros2 run mppi_navigation idle_check` | → `preflight/`（空闲时速度该为 0） |
| `run_reset_rplidar.py` | `ros2 run mppi_navigation reset_rplidar` | → `tools/reset_rplidar.py` |

把这里理解成「遥控器按键」即可，不必逐行读。

#### path_following/ —— 模式 2：跟固定测试路径

> 模式 1 = RViz 点目标；模式 2 = 事先画好一条线（yaml），让车去跟（方便重复测直线、转弯）。

| 文件 | 作用 |
|------|--------------|
| `generators.py` | **画考题路线**：直线、左/右转 90°、S 弯等（只有位置和朝向，没有速度） |
| `path_io.py` | 路径文件的读和写（yaml） |
| `follow_path_client.py` | **发任务**：读 yaml → 变到地图坐标 → 告诉 Nav2「跟这条路」；速度仍由 MPPI 在线算 |
| `cancel_client.py` | **取消当前导航**（取消后安全闸超时会置零停车） |

#### cmd_vel_pipeline/ —— 速度话题名字（给代码用的常量）

| 文件 | 作用 |
|------|--------------|
| `topic_names.py` | 把 `/cmd_vel`、`/scan` 等名字写成常量，避免代码里写错字符串 |
| `remap_chain.py` | 说明速度怎么改名串起来：MPPI → 平滑器 → 安全闸 → 底盘 |

#### mppi_controller/ —— MPPI 调参时的「说明书式」代码

| 文件 | 作用 |
|------|--------------|
| `baseline_params.py` | 和 yaml 里基线参数一一对应，改参时方便对照 |
| `critics_reference.py` | **文字说明** 8 个评分项（critic）各管什么；不是运行中的控制器本身 |

#### localization/ —— 坐标系名字与是否就绪

| 文件 | 作用 |
|------|--------------|
| `tf_frame_names.py` | `map` / `odom` / `base_link` 等名字常量 |
| `readiness_checks.py` | 查 TF 链通不通（给预检等调用） |

#### tools/

| 文件 | 作用 |
|------|--------------|
| `reset_rplidar.py` | 给雷达发串口复位命令；雷达报 internal error 时用（不必断电） |

#### preflight/ —— 启动后、发目标前的自动检查

| 文件 | 作用 |
|------|--------------|
| `topic_health_check.py` | 听几秒：关键话题有没有在发（雷达、IMU、轮速、EKF…） |
| `tf_tree_check.py` | 查 `map→odom→base_link` 断没断 |
| `idle_cmd_vel_check.py` | 空闲时 `/cmd_vel` 必须一直是 0（测试「没目标不该自己跑」） |

日常请用 `scripts/preflight_check_*.sh` 或 `ros2 run … idle_check`，不必直接记模块路径。

#### bag_analysis/ —— 录完包之后算成绩、出图

| 文件 | 作用 |
|------|--------------|
| `load_rosbag.py` | 把 rosbag 读成时间序列数据 |
| `path_tracking_metrics.py` | 算「偏航多远」（路径横向误差） |
| `heading_error_metrics.py` | 算「朝向偏多少」 |
| `velocity_smoothness_metrics.py` | 算速度是否抖（加速度、jerk） |
| `controller_timing_metrics.py` | 算控制频率够不够、有没有丢帧 |
| `generate_plots.py` | 画出轨迹、速度、误差等图 |

入口一般是：`python3 scripts/analyze_navigation_bag.py <包路径>`。

#### launch_config/ —— launch 用的路径小工具

| 文件 | 作用 |
|------|--------------|
| `package_share_paths.py` | 找到本包装好的 config/launch/rviz 路径 |
| `common_launch_args.py` | 公共启动参数（提醒布尔值要写 `True`/`False`） |

---

### scripts/（给你直接敲的辅助脚本）

| 文件 | 作用 |
|------|--------------|
| `preflight_check_topics.sh` | 一键查关键话题有没有数据 |
| `preflight_check_tf.sh` | 一键查 TF 链 |
| `record_navigation_test.sh` | 按清单录包到 `~/RoboGait_ws/bags/` |
| `analyze_navigation_bag.py` | 分析一个 bag → 指标文件 + 图 |
| `generate_test_paths.py` | 重新生成 `paths/` 里 5 条测试路线 |
| `reset_rplidar.py` | 雷达软件复位（也可用 `ros2 run mppi_navigation reset_rplidar`） |

---

### paths/（模式 2 的测试路线，装进包里）

| 文件 | 作用 |
|------|--------------|
| `straight.yaml` | 直线约 3 m |
| `left_90.yaml` | 前进 + 左转 90° + 再前进 |
| `right_90.yaml` | 同上，右转 |
| `s_curve.yaml` | S 弯 |
| `obstacle_avoid.yaml` | 直线（路上可摆实物障碍做避障测） |

---

### maps/（地图）

| 文件 | 作用 |
|------|--------------|
| `lab2_new.yaml` + `lab2_new.pgm` | **默认实验室地图**（AMCL / map_server 用） |
| `lab2_new.posegraph` + `lab2_new.data` | 同一张图的 slam_toolbox 格式；当前导航默认路径**还没用上** |

---

### 其它（包安装与文档）

| 文件 | 作用 |
|------|--------------|
| `package.xml` | 包叫什么、依赖哪些 ROS 包 |
| `setup.py` / `setup.cfg` | 怎么安装，以及注册 `ros2 run` 那几个命令名 |
| `rviz/rviz_goal_navigation.rviz` | RViz 打开时用哪套显示配置 |
| `README.md` | 包的使用说明（英文/摘要向） |
| `test/test_safety.py` | 自动测安全闸逻辑对不对 |
| `test/test_path_following.py` | 自动测路径生成/读写对不对 |

### 工作空间级

| 项 | 作用 |
|----|--------------|
| `.vscode/settings.json` | 本工作区打开 `.md` 默认进预览 |
| `PROJECT_CHANGES.md` | 中文项目手册（本文件） |
| `PROJECT_CHANGES_EN.md` | 英文项目手册（与本文件同步维护） |
| `README.md` | GitHub / 仓库入口说明（英文摘要） |
| `.gitignore` | 忽略 build/install/log/bags 等，不上传编译产物 |
| `TEST_PLAN.md` | 实机验收清单（英文）：阶段 A/B + 测试 T1–T8 |
| `src/mppi_navigation/README.md` | 包说明（英文） |
| `src/navigation2/` | Nav2 源码（要改 MPPI critic 等时才深入） |
| `ARCHITECTURE.html` | 可选备份；结构以本文件第一节为准 |

---

## 四、常用命令手册

> 所有命令的前提：先在该终端执行环境准备（见 4.1）。布尔 launch 参数必须写 `True`/`False`（首字母大写）。

### 4.1 环境准备（每个新终端都要执行）

```bash
cd ~/RoboGait_ws
source /opt/ros/humble/setup.bash   # 加载 ROS 2 Humble 系统环境
source install/setup.bash           # 加载本工作空间（mppi_navigation、nav2 源码版等）
```

不 source 会报 `Package 'mppi_navigation' not found`。

### 4.2 编译

```bash
colcon build --packages-select mppi_navigation --symlink-install
```

- `--packages-select`：只编译本包，几秒完成（不加则重编全部 50+ 包）
- `--symlink-install`：yaml/launch 用软链接安装，改配置文件后**不用重新编译**，重启 launch 即生效

### 4.3 模式 1 启动（RViz 目标导航）

```bash
# 默认启动：硬件 + EKF + Nav2 全栈 + 安全闸 + RViz + AMCL（lab2_new 地图）
# 启动后在 RViz 用「2D Pose Estimate」校正初始位姿，再用「Nav2 Goal」发目标
ros2 launch mppi_navigation rviz_goal_navigation.launch.py

# 在线 SLAM：slam_toolbox 边走边建图（无需已有地图；会覆盖 AMCL）
ros2 launch mppi_navigation rviz_goal_navigation.launch.py slam:=True

# 仅验证软件链路、不要真实定位时，关掉 AMCL（退回静态 map→odom）
ros2 launch mppi_navigation rviz_goal_navigation.launch.py amcl:=False

# 不开 RViz（机器人上无显示器/远程跑时）
ros2 launch mppi_navigation rviz_goal_navigation.launch.py use_rviz:=False
```

`slam:=False` 时 map_server 默认加载本包 `maps/lab2_new.yaml`（实验室地图）；
换图用 `map:=/绝对路径/其它地图.yaml` 覆盖（注意 yaml 和 pgm 要在同一目录）。

一条命令内部依次拉起：`rover_mini_hardware.launch.py`（底盘 CAN 驱动 + URDF + 雷达 + IMU）
→ EKF → 定位（三选一）→ Nav2 各节点（含 MPPI）→ velocity_smoother → safety_gate → 可选 RViz。
发目标：RViz 工具栏「Nav2 Goal / 2D Goal Pose」在地图上点目标点。

### 4.4 模式 2 启动（预定义路径）

先按 4.3 把全栈跑起来，再在**另一个终端**发送路径：

```bash
# path_name 从内置 5 条测试路径里选：straight / left_90 / right_90 / s_curve / obstacle_avoid
ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

# 或用自定义路径文件（yaml 格式见 paths/ 里任意一个）
ros2 launch mppi_navigation follow_predefined_path.launch.py path_file:=/绝对路径/my_path.yaml

# 等价的 ros2 run 写法：
ros2 run mppi_navigation follow_path --ros-args -p path_file:=/绝对路径/my_path.yaml
```

路径 yaml 只存几何（x, y, yaw），速度全部由 MPPI 在线计算；`frame: local` 的路径会
以机器人**当前位姿**为起点自动变换到 map 系。下发的路径同时发布在 `/predefined_path`
（RViz 加个 Path 显示即可看到）。跑完日志显示 `status 4` 表示成功（SUCCEEDED）。

### 4.5 停止 / 急停

```bash
# 取消当前导航目标（NavigateToPose 和 FollowPath 全部取消；安全闸 0.5s 内置零停车）
ros2 run mppi_navigation cancel_navigation

# 软件急停：安全闸立即输出零速（导航目标还在，但轮子不动）
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: true}"
# 解除急停：
ros2 topic pub -1 /safety_estop std_msgs/msg/Bool "{data: false}"
```

### 4.6 预检（启动全栈后、发目标前）

```bash
cd ~/RoboGait_ws/src/mppi_navigation

./scripts/preflight_check_topics.sh    # /scan /imu/data /odometry/wheels /odometry/filtered 是否有数据
./scripts/preflight_check_tf.sh        # TF 链 map→odom→base_link 是否完整

# 测试 1（系统空闲）：10 秒内 /cmd_vel 必须保持零
ros2 run mppi_navigation idle_check 10
# 或: python3 -m mppi_navigation.preflight.idle_cmd_vel_check 10
```

三个检查 exit code 0 = 通过，非 0 = 失败（终端会打印具体缺什么）。

**预检输出怎么读：**

| 标记 | 含义 |
|------|------|
| `[OK]` | 检查窗口内收到该话题 / TF 可查 |
| `[FAIL]` | 必需项缺失（如传感器、EKF、或缺 TF） |
| `[----] ... (optional)` | 可选项未出现也正常（如尚未导航时 `/cmd_vel*` 不发） |

`[FAIL] /map` 常见于 map_server **只发一次** OccupancyGrid，预检几秒内没听到。用下面确认实际有没有图：

```bash
ros2 topic echo /map --once    # 能打印地图数据即可；TF 的 map→odom 若已 OK，多半可忽略预检误报
```

### 4.7 录包与分析

```bash
cd ~/RoboGait_ws/src/mppi_navigation

# 按 diagnostic_recording_topics.yaml 的话题清单录包，保存到 ~/RoboGait_ws/bags/<名字>
./scripts/record_navigation_test.sh straight_run1     # Ctrl+C 结束录制

# 分析：输出 metrics.json（路径误差/航向误差/平滑度/控制频率）+ 5 张图
python3 scripts/analyze_navigation_bag.py ~/RoboGait_ws/bags/straight_run1
```

### 4.8 传感器 / 硬件逐项检查（开机或重插线后）

推荐顺序：**设备层 → 拉起 can0 → 只起硬件 → 逐话题测频率**。不要在 USB/CAN 未稳定时就起全栈。

```bash
source /opt/ros/humble/setup.bash && source ~/RoboGait_ws/install/setup.bash

# ---------- 0) 设备是否在 ----------
ls -l /dev/rplidar /dev/bno055     # 应指向 /dev/ttyUSBx（重启后编号可能变，软链接会更新）
ip link show can0                  # 接口存在；还须 state UP（见下一步）

# ---------- 1) 拉起 can0（重启后常为 DOWN，不会自动 up）----------
# 未 up 时驱动会：Connected to can0 → 随即 FATAL data is stale（这是底盘问题，不是雷达/IMU）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip link show can0                  # 应为 state UP
candump -n 10 -T 3000 can0         # 应看到帧；全空或 Network is down → 查车电 / CAN 线 / USB2CAN

# ---------- 2) 只起硬件（不必先起导航）----------
ros2 launch mppi_navigation rover_mini_hardware.launch.py

# ---------- 3) 测频率（Humble：ros2 topic hz 一次只能一个话题）----------
ros2 topic hz /scan                # 正常 ~10 Hz
ros2 topic hz /imu/data            # 正常 ~50–60 Hz
ros2 topic hz /odometry/wheels     # 正常 ~15–30 Hz

# 一行串测（各约 3 秒）：
timeout 3 ros2 topic hz /scan; timeout 3 ros2 topic hz /imu/data; timeout 3 ros2 topic hz /odometry/wheels
```

| 传感器 | 设备 | 话题 | 正常频率 |
|--------|------|------|----------|
| 雷达 RPLidar | `/dev/rplidar` | `/scan` | ~10 Hz |
| IMU BNO055 | `/dev/bno055` | `/imu/data` | ~50–60 Hz |
| 底盘 | `can0` UP | `/odometry/wheels` | ~15–30 Hz |
| EKF（全栈后） | — | `/odometry/filtered` | 有稳定输出 |

**机器人重启 / USB 重枚举后：** 先 Ctrl+C 停掉旧 launch，再执行上面 0→1→2。旧节点会占着失效串口句柄，IMU 狂刷 `Errno 5 Input/output error`，雷达进程也可能已挂掉却仍显示半残节点列表。

### 4.9 硬件排障命令

```bash
# 雷达 internal error（health status 2）时：软件复位，不用断电
ros2 run mppi_navigation reset_rplidar
# 等价：python3 scripts/reset_rplidar.py
# 原理：向串口发 SLAMTEC 协议 RESET（A5 40）；复位后等 2~3 秒再**重新**启动驱动/launch

# 底盘 FATAL「Did not receive any data ... stale」
# → 查 can0 是否 UP、candump 有无帧、车是否上电；偶发可停掉 launch 再启一次

# IMU TransmissionException / Errno 5
# → USB 重插或机器人重启后旧 bno055 节点未退出；停干净后重 launch
```

### 4.10 常用诊断命令

```bash
ros2 topic hz /scan                    # 雷达应 ~10 Hz（一次一个话题）
ros2 topic hz /imu/data               # IMU 应 ~50–60 Hz
ros2 topic hz /odometry/wheels        # 轮式里程计应 ~15–30 Hz
ros2 topic hz /odometry/filtered      # EKF 输出（全栈后）
ros2 topic hz /cmd_vel               # 导航中应 ~20 Hz（MPPI 控制频率）

ros2 topic echo /map --once           # 确认 map_server 已发地图
ros2 run tf2_ros tf2_echo odom base_link   # EKF 的 odom→base_link
ros2 run tf2_ros tf2_echo map odom         # AMCL 的 map→odom

ros2 topic list | grep cmd_vel        # 应有 /cmd_vel /cmd_vel_nav /cmd_vel_smoothed 三个
ros2 topic info /cmd_vel --verbose    # Publisher 必须只有 safety_gate 一个
ros2 topic echo /cmd_vel_nav          # 看 MPPI 原始输出（排查"机器人不动"从这里开始）

ros2 lifecycle get /bt_navigator      # Nav2 节点状态，应为 active
```

---

## 五、已知问题 / 待办

| 项 | 状态 |
|----|------|
| ~~RPLidar internal error（health status 2）~~ | **已解决（07-21）**：串口 RESET 命令复位，命令见 4.9；开机检查流程见 4.8；若复发可再复位或断电 |
| ~~BNO055 通信失败（Payload length mismatch）~~ | **已解决（07-21）**：重新接线后正常 |
| `slam:=True` 在线建图实测 | 硬件已恢复，可以测 |
| RViz 发 Goal 的完整导航实测（8 项测试协议） | 按 `TEST_PLAN.md`：先 A/B，再 T1→T2→T7，其余按表 |
| AMCL 定位实测（`amcl:=True`） | 默认已开；待按 TEST_PLAN 阶段 B 勾选观感项 |
| 模式 2 实测（FollowPath 5 条路径） | 对应 TEST_PLAN **T3–T6** |
| 完整导航实测（TEST_PLAN 8 项） | 硬件已通；清单已重梳为 A/B + T1–T8，待实机打勾 |
| slam_toolbox 位姿图定位模式 | `lab2_new.posegraph` 已有，launch 未接（可用 AMCL） |
| 自定义 critic（M10） | 源码已就绪，未开始 |
| gazebo_ros_pkgs 未安装 | 需要时 `sudo apt install ros-humble-gazebo-ros-pkgs` |
