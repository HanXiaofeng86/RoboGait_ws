"""Read/write geometric path YAML files (paths/*.yaml).

Format:
    frame: local | map
    poses:
      - {x: 0.0, y: 0.0, yaw: 0.0}
      ...
'local' paths are transformed to the robot's current map pose when sent.
"""

import yaml


def save_path_yaml(filename, poses, frame: str = 'local'):
    data = {
        'frame': frame,
        'poses': [{'x': float(x), 'y': float(y), 'yaw': float(yaw)}
                  for x, y, yaw in poses],
    }
    with open(filename, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def load_path_yaml(filename):
    """Return (frame, [(x, y, yaw), ...])."""
    with open(filename) as f:
        data = yaml.safe_load(f)
    poses = [(p['x'], p['y'], p['yaw']) for p in data['poses']]
    return data.get('frame', 'local'), poses
