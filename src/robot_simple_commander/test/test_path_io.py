"""Unit tests for path generators and YAML I/O."""

import math
from pathlib import Path

from robot_simple_commander.lib.generators import (
    GENERATORS, make_left_90, make_s_curve, make_straight)
from robot_simple_commander.lib.path_io import load_path_yaml, save_path_yaml


def test_straight_length_and_yaw():
    poses = make_straight(length=3.0, step=0.05)
    assert len(poses) >= 2
    assert abs(poses[0][0]) < 1e-9
    assert abs(poses[-1][0] - 3.0) < 1e-6
    assert all(abs(p[2]) < 1e-9 for p in poses)


def test_left_90_turns_positive_yaw():
    poses = make_left_90(leg=2.0, step=0.1)
    assert abs(poses[-1][2] - math.pi / 2.0) < 1e-6
    assert poses[-1][1] > 1.5


def test_s_curve_ends_near_length():
    poses = make_s_curve(length=4.0, amplitude=1.0, step=0.05)
    assert abs(poses[-1][0] - 4.0) < 1e-6
    assert abs(poses[0][1]) < 1e-6


def test_all_generators_registered():
    assert set(GENERATORS) == {
        'straight', 'left_90', 'right_90', 's_curve', 'obstacle_avoid'}


def test_path_yaml_roundtrip(tmp_path: Path):
    poses = make_straight(length=1.0, step=0.25)
    path = tmp_path / 'p.yaml'
    save_path_yaml(path, poses, frame='local')
    frame, loaded = load_path_yaml(path)
    assert frame == 'local'
    assert len(loaded) == len(poses)
    assert abs(loaded[-1][0] - poses[-1][0]) < 1e-9
