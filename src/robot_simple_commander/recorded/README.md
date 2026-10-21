# recorded/

Named recordings: `recorded/<name>.yaml`.

Set `<name>` in `../config/route_name.yaml` (one line: `name: my_loop`).
Record writes that file after **2D Pose Estimate** then driving; follow
reads it (then `../paths/` for built-in names). Follow starts only after
you set 2D Pose Estimate again.
