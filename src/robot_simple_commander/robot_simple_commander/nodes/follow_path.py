"""Follow a geometric path yaml via nav2_simple_commander.BasicNavigator.

Requires an already-running Nav2 stack. Path yaml stores geometry only;
MPPI computes velocities online.

Does not start driving until Nav2 is active. 2D Pose Estimate only
localizes. The RViz panel stays up after each run:

  /follow_start   drive the recorded route (robot must be near the start)
  /follow_return  drive forward along that route back to the start pose
  /follow_stop    cancel the current motion; the node keeps running

    ros2 run robot_simple_commander follow_path --ros-args \\
      -p path_file:=/path/to/route.yaml
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_simple_commander.lib.path_io import load_path_yaml
from robot_simple_commander.lib.return_path import (
    ALREADY_HOME_M, START_GATE_M, build_return_poses)


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _to_map_frame(navigator, tf_buffer, poses):
    try:
        tf = tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(),
            timeout=Duration(seconds=5.0))
    except Exception as exc:
        navigator.get_logger().error(f'TF map->base_link unavailable: {exc}')
        return None

    rx = tf.transform.translation.x
    ry = tf.transform.translation.y
    ryaw = _quat_to_yaw(tf.transform.rotation)
    c, s = math.cos(ryaw), math.sin(ryaw)
    return [(rx + x * c - y * s, ry + x * s + y * c, ryaw + yaw)
            for x, y, yaw in poses]


def _build_path(navigator, poses):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()
    for x, y, yaw in poses:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
    return path


def _empty_path(navigator):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()
    return path


def _route_markers(poses):
    markers = MarkerArray()
    if not poses:
        return markers

    def _add(marker_id, kind, x, y, yaw, r, g, b, scale):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'route'
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.05
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.95
        markers.markers.append(marker)

    sx, sy, syaw = poses[0]
    ex, ey, _ = poses[-1]
    _add(0, Marker.SPHERE, sx, sy, 0.0, 0.18, 0.80, 0.32, (0.28, 0.28, 0.28))
    _add(1, Marker.ARROW, sx, sy, syaw, 0.18, 0.80, 0.32, (0.55, 0.12, 0.12))
    _add(2, Marker.SPHERE, ex, ey, 0.0, 0.95, 0.55, 0.15, (0.28, 0.28, 0.28))
    return markers


def _robot_xy(tf_buffer):
    try:
        tf = tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(),
            timeout=Duration(seconds=0.3))
    except Exception:
        return None
    return tf.transform.translation.x, tf.transform.translation.y


def _wait_tf(navigator, tf_buffer, timeout_sec=30.0):
    deadline = navigator.get_clock().now() + Duration(seconds=timeout_sec)
    while rclpy.ok() and navigator.get_clock().now() < deadline:
        try:
            tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            return True
        except Exception:
            rclpy.spin_once(navigator, timeout_sec=0.2)
    navigator.get_logger().error('Timed out waiting for TF map→base_link')
    return False


def _lifecycle_active(navigator, client):
    if not client.service_is_ready():
        return False
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=1.0)
    try:
        result = future.result()
    except Exception:
        return False
    return result is not None and result.current_state.label == 'active'


def _wait_nav2_ready(navigator, ui, wait_for_amcl):
    """Wait until Nav2 is active. Do not inject AMCL origin pose."""
    amcl = navigator.create_client(GetState, 'amcl/get_state')
    bt = navigator.create_client(GetState, 'bt_navigator/get_state')
    last = navigator.get_clock().now()
    ui.set_status('正在等底盘（雷达复位和 CAN）。先别按 Ctrl+C。')
    while rclpy.ok() and ui.pending != 'stop':
        bt_ok = _lifecycle_active(navigator, bt)
        amcl_ok = (not wait_for_amcl) or _lifecycle_active(navigator, amcl)
        if bt_ok and amcl_ok:
            return True
        now = navigator.get_clock().now()
        if (now - last).nanoseconds > 5e9:
            navigator.get_logger().info(
                'Still waiting for lidar RESET + CAN + Nav2. Do not Ctrl+C.')
            ui.set_status('正在等底盘（雷达复位和 CAN）。先别按 Ctrl+C。')
            last = now
        rclpy.spin_once(navigator, timeout_sec=0.2)
    return False


def _send_follow(navigator, path, controller_id, goal_checker_id, tries=15):
    for attempt in range(1, tries + 1):
        if navigator.followPath(path, controller_id, goal_checker_id):
            return True
        navigator.get_logger().warn(
            f'FollowPath rejected ({attempt}/{tries}); retrying...')
        for _ in range(10):
            rclpy.spin_once(navigator, timeout_sec=0.2)
    navigator.get_logger().error('FollowPath rejected after retries')
    return False


def _cancel_and_wait(navigator):
    try:
        navigator.cancelTask()
    except Exception as exc:
        navigator.get_logger().warn(f'cancel failed: {exc}')
    deadline = navigator.get_clock().now() + Duration(seconds=5.0)
    while rclpy.ok() and not navigator.isTaskComplete():
        if navigator.get_clock().now() > deadline:
            break
        rclpy.spin_once(navigator, timeout_sec=0.1)


class _FollowUi:
    """RViz panel talks to start / return / stop and /follow_ui_status."""

    def __init__(self, node, wait_for_pose):
        self.node = node
        self.wait_for_pose = wait_for_pose
        self.nav2_ready = False
        self.pose_ok = not wait_for_pose
        self.busy = False
        self.task = None
        self.pending = None
        self.status_pub = node.create_publisher(
            String, '/follow_ui_status', _LATCH)
        node.create_service(Trigger, '/follow_start', self._on_start)
        node.create_service(Trigger, '/follow_return', self._on_return)
        node.create_service(Trigger, '/follow_stop', self._on_stop)
        node.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._on_pose, 10)
        self.set_status('正在等导航（底盘和定位）…')

    def set_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.node.get_logger().info(text)

    def _on_pose(self, _msg):
        self.pose_ok = True
        if self.nav2_ready and not self.busy and self.pending is None:
            self.set_status(
                '位置已摆好。点「开始」出发。跟完再点「回到起点」。')

    def _reject(self, response, message):
        response.success = False
        response.message = message
        self.set_status(message)
        return response

    def _on_start(self, _request, response):
        if self.busy or self.pending in ('follow', 'return'):
            return self._reject(response, '正在行驶。要回家请点「回到起点」，要停下请点「停止」。')
        if not self.nav2_ready:
            return self._reject(response, '导航还没就绪。')
        if self.wait_for_pose and not self.pose_ok:
            return self._reject(response, '先用 2D Pose Estimate 把车摆到地图上。')
        self.pending = 'follow'
        response.success = True
        response.message = '开始'
        return response

    def _on_return(self, _request, response):
        if self.task == 'return':
            return self._reject(response, '已经在回起点。')
        if not self.nav2_ready:
            return self._reject(response, '导航还没就绪。')
        if self.wait_for_pose and not self.pose_ok:
            return self._reject(response, '先用 2D Pose Estimate 把车摆到地图上。')
        self.pending = 'return'
        response.success = True
        response.message = '回到起点'
        if self.busy:
            self.set_status('先停下，再沿原路回到起点。')
        return response

    def _on_stop(self, _request, response):
        self.pending = 'stop'
        response.success = True
        response.message = '停止'
        self.set_status('正在停下。')
        return response


def _run_task(navigator, ui, path, controller_id, goal_checker_id, kind):
    ui.task = kind
    ui.set_status(
        '正在跟随轨迹…' if kind == 'follow' else '正在沿原路回到起点…')
    if not _send_follow(navigator, path, controller_id, goal_checker_id):
        ui.task = None
        ui.set_status('导航拒绝了这条路径。')
        return 'failed'
    step = 0
    while rclpy.ok() and not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if ui.pending == 'stop':
            ui.pending = None
            _cancel_and_wait(navigator)
            ui.task = None
            if ui.pending is None:
                ui.set_status('已停下。可以点「开始」或「回到起点」。')
            return 'canceled'
        if kind == 'follow' and ui.pending == 'return':
            _cancel_and_wait(navigator)
            ui.task = None
            return 'switch'
        step += 1
        feedback = navigator.getFeedback()
        if feedback is not None and step % 20 == 0:
            navigator.get_logger().info(
                f'Distance to goal: {feedback.distance_to_goal:.2f} m')
    result = navigator.getResult()
    ui.task = None
    if result == TaskResult.SUCCEEDED:
        if kind == 'follow':
            ui.set_status('跟随完成。点「回到起点」，车会沿录制路线开回去。')
        else:
            ui.set_status('已回到起点，朝向与录制时一致。点「开始」再走一遍。')
        return 'ok'
    if result == TaskResult.CANCELED:
        ui.set_status('已取消。可以点「开始」或「回到起点」。')
        return 'canceled'
    ui.set_status('这次没走完。可以再点一次，或点「回到起点」。')
    return 'failed'


def _exit(navigator, code):
    navigator.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(code)


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='robogait_follow_path')

    navigator.declare_parameter('path_file', '')
    navigator.declare_parameter('controller_id', 'FollowPath')
    navigator.declare_parameter('goal_checker_id', 'general_goal_checker')
    navigator.declare_parameter('wait_for_nav2', True)
    navigator.declare_parameter('wait_for_initial_pose', True)
    navigator.declare_parameter('wait_for_start', True)

    path_file = navigator.get_parameter('path_file').value
    if not path_file:
        navigator.get_logger().error('path_file parameter is required')
        _exit(navigator, 1)

    wait_pose = bool(navigator.get_parameter('wait_for_initial_pose').value)
    wait_start = bool(navigator.get_parameter('wait_for_start').value)
    if wait_start and not navigator.get_parameter('wait_for_nav2').value:
        navigator.get_logger().error(
            'wait_for_start requires wait_for_nav2:=true')
        _exit(navigator, 1)

    frame, poses = load_path_yaml(path_file)
    navigator.get_logger().info(
        f'Loaded {len(poses)} poses from {path_file} (frame={frame})')
    if len(poses) < 2:
        navigator.get_logger().error('Path needs at least 2 poses')
        _exit(navigator, 1)

    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)
    ui = _FollowUi(navigator, wait_pose)

    path_pub = navigator.create_publisher(Path, 'predefined_path', _LATCH)
    return_pub = navigator.create_publisher(Path, 'return_path', _LATCH)
    marker_pub = navigator.create_publisher(
        MarkerArray, 'route_markers', _LATCH)
    map_poses = list(poses) if frame == 'map' else None

    def publish_route(route):
        path_pub.publish(_build_path(navigator, route))
        marker_pub.publish(_route_markers(route))
        return_pub.publish(_empty_path(navigator))

    if map_poses is not None:
        publish_route(map_poses)

    if navigator.get_parameter('wait_for_nav2').value:
        navigator.get_logger().info(
            'Waiting for Nav2. Lidar reset then CAN — do not Ctrl+C yet.')
        if not _wait_nav2_ready(navigator, ui, wait_pose):
            ui.set_status('等待导航时被停下了。')
            _exit(navigator, 1)
        ui.nav2_ready = True
        if ui.pose_ok:
            ui.set_status('导航已就绪。点「开始」出发。跟完再点「回到起点」。')
        else:
            ui.set_status(
                '导航已就绪。先用 2D Pose Estimate 对准激光，再点「开始」。')

    controller_id = navigator.get_parameter('controller_id').value
    goal_checker_id = navigator.get_parameter('goal_checker_id').value

    def ensure_map_poses():
        nonlocal map_poses
        if map_poses is not None:
            return map_poses
        if not _wait_tf(navigator, tf_buffer):
            return None
        map_poses = _to_map_frame(navigator, tf_buffer, poses)
        if map_poses is not None:
            publish_route(map_poses)
        return map_poses

    if not wait_start:
        _run_once(
            navigator, ui, tf_buffer, ensure_map_poses, wait_pose,
            controller_id, goal_checker_id, return_pub)
        return

    try:
        while rclpy.ok():
            while rclpy.ok() and ui.pending is None:
                rclpy.spin_once(navigator, timeout_sec=0.2)
            if not rclpy.ok() or ui.pending is None:
                break
            cmd = ui.pending
            ui.pending = None
            if cmd == 'stop':
                ui.set_status('已停下。可以点「开始」或「回到起点」。')
                continue
            ui.busy = True
            try:
                _handle_command(
                    navigator, ui, tf_buffer, ensure_map_poses,
                    controller_id, goal_checker_id, return_pub, cmd)
            finally:
                ui.busy = False
                ui.task = None
        _exit(navigator, 0)
    except KeyboardInterrupt:
        navigator.get_logger().warn('Ctrl+C — canceling FollowPath')
        try:
            navigator.cancelTask()
        except Exception:
            pass
        _exit(navigator, 1)


def _handle_command(
        navigator, ui, tf_buffer, ensure_map_poses,
        controller_id, goal_checker_id, return_pub, cmd):
    route = ensure_map_poses()
    if not route:
        ui.set_status('读不到车在地图上的位置。等激光和定位后再点。')
        return
    xy = _robot_xy(tf_buffer)
    if xy is None:
        ui.set_status('暂时读不到车的位置，等一两秒再点。')
        return
    if cmd == 'follow':
        dist = math.hypot(xy[0] - route[0][0], xy[1] - route[0][1])
        if dist > START_GATE_M:
            ui.set_status(
                f'离起点还有 {dist:.1f} 米。请先点「回到起点」。')
            return
        _run_task(
            navigator, ui, _build_path(navigator, route),
            controller_id, goal_checker_id, 'follow')
        return
    if cmd != 'return':
        return
    back = build_return_poses(route, xy, ALREADY_HOME_M)
    if not back:
        ui.set_status('已经在起点。点「开始」再走一遍。')
        return
    return_pub.publish(_build_path(navigator, back))
    _run_task(
        navigator, ui, _build_path(navigator, back),
        controller_id, goal_checker_id, 'return')
    return_pub.publish(_empty_path(navigator))


def _run_once(
        navigator, ui, tf_buffer, ensure_map_poses, wait_pose,
        controller_id, goal_checker_id, return_pub):
    """One follow, then exit. Used only when wait_for_start is false."""
    if wait_pose and not ui.pose_ok:
        while rclpy.ok() and not ui.pose_ok and ui.pending != 'stop':
            rclpy.spin_once(navigator, timeout_sec=0.2)
        if ui.pending == 'stop' or not ui.pose_ok:
            ui.set_status('还没摆位姿就停了。')
            _exit(navigator, 1)
    route = ensure_map_poses()
    if not route:
        _exit(navigator, 1)
    ui.busy = True
    code = 0 if _run_task(
        navigator, ui, _build_path(navigator, route),
        controller_id, goal_checker_id, 'follow') == 'ok' else 1
    return_pub.publish(_empty_path(navigator))
    _ = tf_buffer
    _exit(navigator, code)


if __name__ == '__main__':
    main()
