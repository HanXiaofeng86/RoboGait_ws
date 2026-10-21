"""Active trajectory name: config/route_name.yaml.

Edit that file (one line: name: my_loop). Record writes recorded/<name>.yaml.
Follow reads recorded/<name>.yaml, then paths/<name>.yaml.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def share_root() -> Path:
    return Path(get_package_share_directory('robot_simple_commander'))


def route_name_config() -> Path:
    return share_root() / 'config' / 'route_name.yaml'


def load_route_name(override: str = '') -> str:
    name = 'lab_route'
    cfg = route_name_config()
    if cfg.is_file():
        data = yaml.safe_load(cfg.read_text()) or {}
        raw = data.get('name', name)
        if raw:
            name = str(raw).strip()
    if override and str(override).strip():
        name = str(override).strip()
    name = name.removesuffix('.yaml').strip()
    name = Path(name).name
    if not name:
        raise ValueError('route name is empty')
    return name


def recorded_dir() -> Path:
    """Prefer the source recorded/ folder (symlink-install resolves here)."""
    cfg = route_name_config()
    rec = cfg.resolve().parent.parent / 'recorded'
    rec.mkdir(parents=True, exist_ok=True)
    return rec


def recorded_yaml(name: str) -> Path:
    return recorded_dir() / f'{name}.yaml'


def follow_yaml(name: str, path_file: str = '') -> Path:
    if path_file and str(path_file).strip():
        return Path(path_file).expanduser()
    rec = recorded_yaml(name)
    if rec.is_file():
        return rec
    share_rec = share_root() / 'recorded' / f'{name}.yaml'
    if share_rec.is_file():
        return share_rec
    builtin = share_root() / 'paths' / f'{name}.yaml'
    if builtin.is_file():
        return builtin
    raise FileNotFoundError(
        f'No path yaml named {name!r}. Tried {rec} and {builtin}')
