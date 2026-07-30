# robogait_perception

**Mode B** — Orbbec/Nuitrack → `user_raw`.

## Layout

```text
config/                 # parameters
robogait_perception/
  nodes/                # ros2 run nodes (distance_orbbec)
```

No standalone launch; started by `test_gimbal` / `person_layer` / `person_follow`.

```bash
ros2 run robogait_perception distance_orbbec
```
