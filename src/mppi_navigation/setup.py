import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mppi_navigation'

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
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'paths'),
            glob('paths/*')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robogait',
    maintainer_email='robogait@todo.todo',
    description='Pure Nav2 MPPI navigation for Rover Robotics Mini '
                '(Mode 1: RViz goal, Mode 2: predefined path).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_gate = mppi_navigation.nodes.run_cmd_vel_safety_gate:main',
            'follow_path = mppi_navigation.nodes.run_follow_path_client:main',
            'cancel_navigation = mppi_navigation.nodes.run_cancel_navigation:main',
            'idle_check = mppi_navigation.nodes.run_idle_cmd_vel_check:main',
            'reset_rplidar = mppi_navigation.nodes.run_reset_rplidar:main',
        ],
    },
)
