# robogait_person_follow

**Mode B** — 人只提供 Nav2 目标；底盘速度全部由 **MPPI** 计算（无 PID 混速）。

## 文件夹

```text
launch/                 # 入口（平铺）
  person_follow.launch.py   # B1 一站：Nav2/MPPI + 人层
  person_layer.launch.py    # B2 终端2：叠在已起的 Nav 上
config/                 # 人距 / 目标更新参数
robogait_person_follow/
  nodes/                # distance_filter, person_nav_goal
```

```bash
# 一站式
ros2 launch robogait_person_follow person_follow.launch.py

# 或两终端
ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
ros2 launch robogait_person_follow person_layer.launch.py
```
