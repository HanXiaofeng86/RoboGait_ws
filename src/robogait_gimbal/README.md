# robogait_gimbal

**Mode B** — camera pan gimbal / Dynamixel (does **not** publish `/cmd_vel`).

## Layout

```text
launch/                 # entry
  test_gimbal.launch.py
config/                 # serial device, PID, angle limits
robogait_gimbal/
  nodes/                # camera_tracker, dynamixel_driver
```

```bash
ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0
```
