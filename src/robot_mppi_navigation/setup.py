import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_mppi_navigation'

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
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robogait',
    maintainer_email='robogait@todo.todo',
    description='Pure Nav2 MPPI navigation stack for Rover Robotics Mini '
                '(bringup, safety gate). Path record/follow: '
                'robot_simple_commander.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Real shebang scripts — setuptools console_scripts on this image
    # wrote 0-byte files (Exec format error on launch).
    scripts=[
        'bin/safety_gate',
        'bin/idle_check',
        'bin/reset_rplidar',
        'bin/wait_for_can',
        'bin/wait_for_nav_sensors',
        'bin/scan_viz',
        'bin/cancel_nav_on_pose',
        'bin/teleop',
    ],
    entry_points={
        'console_scripts': [],
    },
)
