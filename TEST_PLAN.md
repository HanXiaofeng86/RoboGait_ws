# 今天实机测试：建图、导航、录路径、跟路径

今天只做下面四步，从上到下。前一步没过，不要开下一步。  
**今天不测：** 跟人、云台、直线跟随、开机自动手柄。

一次只开一套程序。开下一套之前，先在上一套的终端按 **Ctrl+C**，等到命令提示符回来。

每个新终端都先执行（让本工作空间盖过 `.bashrc` 里的旧空间，也盖过 `/opt/ros/jazzy` 里的 Nav2）。导航跑的是仓库里的 `src/navigation2`，改这份源码后要重新 `colcon build` 再 source。

```bash
cd ~/RoboGait_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**不要** `source ~/rover_workspace_final`。  
RViz 开在这台 NUC 的桌面上。布尔参数写成 `True` / `False`。

本机和 `rover_workspace_final` 同在 `ROS_DOMAIN_ID=10`。每次开 launch 前先看有没有别人的节点：

```bash
ros2 node list
```

有输出就先等对方停掉，再开我们的。

全程用同一张图（第 1 步扫出来）：

```bash
export MAP=$HOME/RoboGait_ws/src/robot_slam_mapping/maps/today
```

| 步骤 | 做什么 | 完成 |
|------|--------|------|
| **0** | 设备检查 | ☐ |
| **1** | 建图，存下 `today.yaml` + `today.pgm` | ☐ |
| **2** | 用这张图做 RViz 导航，并拖速度滑条 | ☐ |
| **3** | 在这张图上录路径 `name:=today` | ☐ |
| **4** | 跟上这条路径，并再拖一次速度滑条 | ☐ |

---

## 0 — 设备

车上电，松开急停。

```bash
ip link show can0
ls -l /dev/input/js0 /dev/rplidar
ls -l /dev/bno055
systemctl is-active roverrobotics.service
```

通过：

- `can0` 是 `UP`
- `/dev/input/js0` 在（手柄）
- `/dev/rplidar` 在（雷达）

`systemctl` 打出 `failed` 可以继续——旧开机服务没在跑就行。**不要改 systemd**，也不要去开厂商的 `mini_teleop`。

`/dev/bno055` 今天可以没有，是可选的。后面日志里若有 bno055 反复 respawn，**忽略**即可。  
注意：偏航角速度目前只来自 IMU，转弯要**慢**；没有倒车（`vx_min` 为 0），车会原地转。**不要**把前进上限 `vx_max` 调到超过 `1.00`。

### 雷达 USB 转接盒（重要）

好用的是 **CP2102N**，序列号二选一：

- `d4ffe520d382eb11b064968d9693f7bc`
- `7ee3445a1d0bee11b58f4002f59e3369`

**不要插**旧盒子 `a64db3d4`：系统能认出设备，但**没有数据**。

怎么区分：

```bash
ls -l /dev/serial/by-id
```

- 转接盒那一行会随盒子变（看上面两个 CP2102N）
- 雷达头本身那行序列号 `71e6ecf8c4e699d7b8eb99f9fd394717` **不随盒子变**，不用靠它挑盒子

`can0` 不是 `UP` 时（只启动服务，不改配置）：

```bash
sudo systemctl start can.service
```

---

## 1 — 建图并保存为 today

建图时速度滑条**无效**。左手柄前后走，右手柄左右转。十字键上下改前进油门，一档 **0.25**，先用小油门。

雷达正在 `RESET` 的那几秒**不要按 Ctrl+C**。

**终端 A：**

```bash
ros2 launch robot_slam_mapping slam_mapping.launch.py
```

等雷达复位结束、车能被手柄推动，再开 **终端 B** 看图：

```bash
rviz2 -d ~/.rviz2/rviz_indoor_slam.rviz
```

- Fixed Frame 选 `map`
- 只看地图和红色激光，**不要点 Nav2 Goal**
- 左边 Speed 滑条在建图时无效，不用管

开车绕房间一圈，让激光贴上墙。图在变、闭环后墙没有明显重影，再存图（**建图 launch 保持开着**）。

### 存图（二选一）

**方式 A — RViz 存图面板（推荐）**

面板默认路径就是 `maps/today`。点保存，会写出 `.yaml` 和 `.pgm`。

没有面板时：菜单 **Panels → Add New Panel → `robot_simple_commander/MapSave`**。

**方式 B — 命令行**

**终端 C：**

```bash
ros2 run nav2_map_server map_saver_cli -f "$MAP"
ls -l "${MAP}.yaml" "${MAP}.pgm"
```

**不要用** Slam Toolbox 面板里的 **Save Map** 或 **Serialize Map**（那不是导航要的 yaml/pgm）。

通过：`today.yaml` 和 `today.pgm` 都在，pgm 不是空的。  
然后 Ctrl+C 关掉终端 A（建图）和终端 B（RViz），再做第 2 步。

另开终端可核对速度只有安全闸在发：

```bash
ros2 topic info /cmd_vel --verbose
```

Publisher 应只有 `safety_gate`。手柄发的是 `/cmd_vel_smoothed`。

---

## 2 — RViz 导航 + 速度滑条

**终端 A：**

```bash
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py map:="${MAP}.yaml"
```

等看到 managed nodes 已 active，并且 RViz 里有红激光之后：

1. 工具栏 **2D Pose Estimate**，绿箭头摆到车真实的位置和朝向，**摆一次**。激光（红）应贴上墙。
2. 先点一个近的、空地上的 **Nav2 Goal**（前方一两米，不要穿墙）。车要往前走，不要只原地转。
3. **下一个目标**：等车到了，或 Navigation 2 面板显示成功，再点下一个 Nav2 Goal。还在走的时候又点新目标，会抢占旧目标，容易走出坏路径。想提前换目标：先点 Navigation 2 的 **Cancel**，再点新 Goal。
4. 导航过程中再点一次 **2D Pose Estimate**，会取消当前目标——一般不要这么做。
5. 左边速度面板英文标签是 **Max forward speed**。拖到例如 `0.30`，**松开**滑条才生效。范围 **0.20～1.00 m/s**，这是上限（天花板），旁边有障碍时会更慢。测完可拖回 `1.00` 并松开。
6. 若窗口是重建插件**之前**开的，标签可能还是旧的：关掉 RViz 再开一次（或重启整个 nav launch）即可看到英文。

RViz 里 **Bumper Hit**、**Realsense** 显示可以忽略。**RobotModel**、**TF** 默认是关的，不用开。

通过后再查：

```bash
ros2 topic info /cmd_vel --verbose
ros2 run robot_mppi_navigation idle_check 10
```

- `/cmd_vel` 的 Publisher 应只有 `safety_gate`
- Cancel 之后、没有 Goal 时，`idle_check` 应看到速度为 0

然后 Ctrl+C 关掉导航，再做第 3 步。

---

## 3 — 录路径 `name:=today`

录路径**没有**速度滑条。这一步手柄**直接发** `/cmd_vel`（没有安全闸），油门保持小。**不要**再开 `mini_teleop`。

**终端 A：**

```bash
ros2 launch robot_simple_commander record_trajectory.launch.py name:=today map:="${MAP}.yaml"
```

1. 等红激光出现。
2. **2D Pose Estimate** 摆到真实位置。没摆之前不要开车。
3. 手柄沿第 2 步走过的空地开一段（至少几米，不要原地转一下就停）。
4. 在**这个 launch 终端**按 Ctrl+C 保存。关掉 RViz 窗口**不会**保存。

检查：

```bash
ls -l ~/RoboGait_ws/src/robot_simple_commander/recorded/today.yaml
grep -c 'yaw:' ~/RoboGait_ws/src/robot_simple_commander/recorded/today.yaml
```

通过：`today.yaml` 里有**好几行** `yaw:`（多于 2）。只有 1 个点就是没录上。  
确认终端已回到提示符，再做第 4 步。

---

## 4 — 跟路径 + 速度滑条

**终端 A：**

```bash
ros2 launch robot_simple_commander follow_trajectory.launch.py name:=today map:="${MAP}.yaml"
```

雷达复位大约 **15～20 秒**，这段时间**没有 RViz**，**不要 Ctrl+C**。等到 Follow 面板写 **Nav2 ready**：

1. **2D Pose Estimate** 只摆位姿，车还不会动。
2. 可选：左边 Speed 拖到 `0.40 m/s` 并**松开**，再点 Follow 面板的 **Start**。
3. 车应沿着刚录的线走，而不是乱转。
4. 停：Follow 面板 **Stop**，或本终端 Ctrl+C。

没有 Follow 面板：Panels → Add New Panel → `robot_simple_commander/Follow`。  
Start 提示要先摆位姿：回去做第 1 步。

```bash
ros2 topic info /cmd_vel --verbose
```

Publisher 应只有 `safety_gate`。

---

## 今天算通过

- 第 1 步留下 `today.yaml` 和 `today.pgm`（存图面板或 `map_saver_cli`，不是 Slam Toolbox 那两个按钮）。
- 第 2 步近目标能走到；滑条松开后空地速度跟着变；Cancel 后车停；`idle_check` 为 0。
- 第 3 步 `recorded/today.yaml` 里有多个 `yaw:`。
- 第 4 步 Start 之后车沿这条线走，Stop 之后车停。
- 第 1、2、4 步的 `/cmd_vel` 只有 `safety_gate` 在发。第 3 步是手柄直接发，这是录路径的正常情况。
