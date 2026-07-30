"""Helpers to resolve files inside this package's share directory."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'robot_mppi_navigation'


def share_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME))


def config_file(name: str) -> str:
    return str(share_dir() / 'config' / name)


def launch_file(name: str) -> str:
    return str(share_dir() / 'launch' / name)


def rviz_file(name: str) -> str:
    return str(share_dir() / 'rviz' / name)
