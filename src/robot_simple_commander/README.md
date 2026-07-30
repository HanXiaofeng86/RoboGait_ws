# robot_simple_commander

**Mode A2** — teleop record / follow. Independent of `robot_mppi_navigation`.

## 文件夹约定（看名字就知道放什么）

```text
launch/                 # 你 ros2 launch 的入口
  record_trajectory.launch.py
  follow_trajectory.launch.py
  parts/                # 被入口拼装的片段（不是入口）
    hardware.launch.py
    nav_stack.launch.py
config/                 # 参数 yaml
maps/                   # 地图
paths/                  # 内置考试路径几何
recorded/               # 手动录的轨迹
scripts/                # 离线脚本（不是 ros2 run 节点）
test/                   # 单元测试
robot_simple_commander/
  nodes/                # 只有 ros2 run 的节点
  lib/                  # 库函数（不是节点）
  safety/               # 安全闸业务逻辑
```

## 日常命令

```bash
ros2 launch robot_simple_commander record_trajectory.launch.py
ros2 launch robot_simple_commander follow_trajectory.launch.py path_name:=left_90
```
