"""Build a forward path from the robot back to the recorded start.

The Mini controller does not reverse (vx_min is 0), so each pose faces the
next one. The last pose keeps the original start yaw, so the next follow
leaves in the same direction as the recording.
"""

import math

ALREADY_HOME_M = 0.40
START_GATE_M = 0.80


def build_return_poses(poses, robot_xy, already_home_m=ALREADY_HOME_M):
    """Return map-frame (x, y, yaw) poses, or [] if already at the start."""
    if not poses:
        return []
    sx, sy, syaw = poses[0]
    rx, ry = robot_xy
    if math.hypot(sx - rx, sy - ry) <= already_home_m:
        return []

    nearest = min(
        range(len(poses)),
        key=lambda i: (poses[i][0] - rx) ** 2 + (poses[i][1] - ry) ** 2)

    chain = [(rx, ry)]
    for i in range(nearest, -1, -1):
        x, y = poses[i][0], poses[i][1]
        if math.hypot(x - chain[-1][0], y - chain[-1][1]) < 0.05:
            continue
        chain.append((x, y))
    if math.hypot(chain[-1][0] - sx, chain[-1][1] - sy) > 0.02:
        chain.append((sx, sy))
    else:
        chain[-1] = (sx, sy)

    out = []
    last = len(chain) - 1
    for i, (x, y) in enumerate(chain):
        if i == last:
            out.append((x, y, syaw))
            continue
        nx, ny = chain[i + 1]
        out.append((x, y, math.atan2(ny - y, nx - x)))
    if len(out) == 1:
        heading = math.atan2(sy - ry, sx - rx)
        out = [(rx, ry, heading), (sx, sy, syaw)]
    return out
