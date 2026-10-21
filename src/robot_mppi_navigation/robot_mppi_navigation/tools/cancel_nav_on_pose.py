"""Cancel running Nav2 goals when the operator sets 2D Pose Estimate.

RViz 2D Pose Estimate publishes /initialpose and jumps map->odom. If a
NavigateToPose / ComputePathToPose is already running, the planner keeps
retrying the old stamped pose (~1 Hz Extrapolation Error) and the robot
stalls. Cancel first so the operator can refine pose, then click a new goal.
"""

from __future__ import annotations

from action_msgs.srv import CancelGoal
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


_CANCEL_SRVS = (
    'navigate_to_pose/_action/cancel_goal',
    'navigate_through_poses/_action/cancel_goal',
    'compute_path_to_pose/_action/cancel_goal',
)


class CancelNavOnPose(Node):

    def __init__(self) -> None:
        super().__init__('cancel_nav_on_pose')
        self._clients = [
            self.create_client(CancelGoal, name) for name in _CANCEL_SRVS
        ]
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_pose, 10)
        self.get_logger().info(
            'Listening on /initialpose — 2D Pose Estimate cancels any running nav goal')

    def _on_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        req = CancelGoal.Request()
        n = 0
        for client in self._clients:
            if client.service_is_ready():
                client.call_async(req)
                n += 1
        self.get_logger().info(
            '2D Pose Estimate: cancelled running nav/plan goals (%d servers)' % n)


def main(argv=None) -> int:
    del argv
    rclpy.init()
    node = CancelNavOnPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
