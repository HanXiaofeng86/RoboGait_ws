# robogait_perception

**Mode B** — Orbbec/Nuitrack → `user_raw`.

## 文件夹

```text
config/                 # 参数
robogait_perception/
  nodes/                # ros2 run 节点（distance_orbbec）
```

无独立 launch：由 `test_gimbal` / `person_layer` / `person_follow` 起节点。

```bash
ros2 run robogait_perception distance_orbbec
```
