# robogait_gimbal

**Mode B** — 云台/舵机（不发 `/cmd_vel`）。

## 文件夹

```text
launch/                 # 入口
  test_gimbal.launch.py
config/                 # 串口、PID、角度限位
robogait_gimbal/
  nodes/                # camera_tracker, dynamixel_driver
```

```bash
ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0
```
