#!/usr/bin/env python3
"""Generate the 5 standard test path YAMLs into paths/.

Usage: python3 scripts/generate_test_paths.py [output_dir]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_simple_commander.lib.generators import GENERATORS
from robot_simple_commander.lib.path_io import save_path_yaml


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path(__file__).resolve().parent.parent / 'paths')
    out_dir.mkdir(parents=True, exist_ok=True)

    for name, gen in GENERATORS.items():
        poses = gen()
        target = out_dir / f'{name}.yaml'
        save_path_yaml(target, poses, frame='local')
        print(f'{target}  ({len(poses)} poses)')


if __name__ == '__main__':
    main()
