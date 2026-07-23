"""Test 7 helper: cancel all active navigation goals; the robot must stop.

    ros2 run mppi_navigation cancel_navigation

Sends a cancel-all request to the NavigateToPose and FollowPath action
servers. The safety gate then sees the command stream stop and outputs
zero velocity within its cmd_timeout.
"""

import rclpy
from rclpy.node import Node

from action_msgs.srv import CancelGoal

ACTION_SERVERS = [
    '/navigate_to_pose/_action/cancel_goal',
    '/follow_path/_action/cancel_goal',
]


def main(args=None):
    rclpy.init(args=args)
    node = Node('cancel_navigation')

    any_cancelled = False
    for srv_name in ACTION_SERVERS:
        client = node.create_client(CancelGoal, srv_name)
        if not client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn(f'{srv_name}: not available')
            continue
        # Empty request = cancel all goals on this server.
        future = client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None:
            node.get_logger().warn(f'{srv_name}: no response')
            continue
        n = len(resp.goals_canceling)
        any_cancelled = any_cancelled or n > 0
        node.get_logger().info(f'{srv_name}: canceling {n} goal(s)')

    node.get_logger().info(
        'Cancel requests sent. Verify the robot stops (safety gate zeroes '
        'cmd_vel after its timeout).' if any_cancelled
        else 'No active goals found.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
