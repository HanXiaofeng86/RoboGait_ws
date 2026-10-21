import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robogait_person_follow'

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
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robogait',
    maintainer_email='robogait@todo.todo',
    description='Mode B: person follow. B1/B2 Nav2 goal; B3 straight distance PID (Python).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'distance_filter = robogait_person_follow.nodes.distance_filter:main',
            'person_nav_goal = robogait_person_follow.nodes.person_nav_goal:main',
            'straight_follow = robogait_person_follow.nodes.straight_follow:main',
        ],
    },
)
