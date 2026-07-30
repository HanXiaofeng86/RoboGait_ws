import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_simple_commander'

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
        (os.path.join('share', package_name, 'launch', 'parts'),
            glob('launch/parts/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'paths'),
            glob('paths/*')),
        (os.path.join('share', package_name, 'recorded'),
            glob('recorded/*')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robogait',
    maintainer_email='robogait@todo.todo',
    description='Standalone path record/follow for RoboGait (own Nav2/MPPI bringup).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_path = robot_simple_commander.nodes.record_path:main',
            'follow_path = robot_simple_commander.nodes.follow_path:main',
            'cancel_navigation = robot_simple_commander.nodes.cancel_task:main',
            'safety_gate = robot_simple_commander.nodes.run_safety_gate:main',
        ],
    },
)
