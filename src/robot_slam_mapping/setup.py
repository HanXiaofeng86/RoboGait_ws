import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_slam_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robogait',
    maintainer_email='robogait@todo.todo',
    description='Lightweight SLAM mapping (hardware+EKF+slam_toolbox+pad) for Rover Mini.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flatten_scan = robot_slam_mapping.nodes.flatten_scan:main',
        ],
    },
)
