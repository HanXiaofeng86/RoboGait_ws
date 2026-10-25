# RoboGait_ws

ROS 2 Jazzy: pure-robot navigation (`robot_*`) + person follow (`robogait_*`).

## Package prefix

| Prefix | Meaning |
|--------|---------|
| **`robot_*`** | Pure robot (no person) |
| **`robogait_*`** | Person follow |
| Other | Drivers / Nav2 |

## Packages

| Package | Role |
|---------|------|
| [`robot_mppi_navigation`](src/robot_mppi_navigation/README.md) | **A1** RViz Goal |
| [`robot_simple_commander`](src/robot_simple_commander/README.md) | **A2** record + follow |
| [`robogait_perception`](src/robogait_perception/README.md) | **B** person (SDK default / Nuitrack) |
| [`robogait_gimbal`](src/robogait_gimbal/README.md) | **B** pan+tilt gimbal |
| [`robogait_person_follow`](src/robogait_person_follow/README.md) | **B** person follow: B1/B2 Nav goal (MPPI), B3 straight distance |
| [`robogait_person_interfaces`](src/robogait_person_interfaces) | **B** `User.msg` |

## Docs language

| File | Language |
|------|----------|
| [`PROJECT_CHANGES.md`](PROJECT_CHANGES.md) | **Chinese** (main handbook) |
| [`TEST_PLAN.md`](TEST_PLAN.md) | **Chinese** — today's on-robot steps: mapping, navigation, record/follow |
| [`PROJECT_CHANGES_EN.md`](PROJECT_CHANGES_EN.md) | English handbook |
| [`NODES_AND_TOPICS.md`](NODES_AND_TOPICS.md) | English node/topic graphs |
| Package `README.md` files | English |

## Quick start

Navigation uses the Nav2 source in `src/navigation2`, not the Nav2 debs under `/opt/ros/jazzy`. Change that tree, rebuild, then source `install/setup.bash` so the workspace copy is the one that runs. `colcon_defaults.yaml` lets that source override the same package names already installed with Jazzy.

```bash
cd ~/RoboGait_ws
source /opt/ros/jazzy/setup.bash
export CC=gcc CXX=g++
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
```

Mode A2 / Mode B: see package READMEs.  
Node/topic graph: [`NODES_AND_TOPICS.md`](NODES_AND_TOPICS.md).
