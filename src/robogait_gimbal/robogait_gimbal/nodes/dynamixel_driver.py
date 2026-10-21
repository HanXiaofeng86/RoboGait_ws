"""Pan + tilt Dynamixel driver — keep the camera pointed at the person.

Two XL-series servos on one U2D2 bus (same as rover_workspace_final tower_controller):
  ID 1 = pan (yaw), ID 2 = tilt (pitch).
Topics ``camera_position`` / ``camera_tilt`` are relative degree errors from
camera_tracker; this node converts them to ticks and clamps to mechanical limits.
Does not publish ``/cmd_vel``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from rclpy.node import Node
from robogait_person_interfaces.msg import User
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
POSITION_CONTROL_MODE = 3
TICKS_PER_REV = 4096
DEFAULT_DEVICE = (
    '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0')


@dataclass
class Joint:
    name: str
    urdf_joint: str
    dxl_id: int
    min_pos: int
    max_pos: int
    home: int
    invert: bool
    urdf_sign: float
    tf_zero: int = 0
    urdf_max_rad: float = 0.0
    last_goal: int = -1
    last_ticks: int = -1
    last_cmd_ticks: int = -1


class DynamixelDriver(Node):
    """Drive pan (ID 1) and tilt (ID 2) from camera_position / camera_tilt."""

    def __init__(self) -> None:
        super().__init__('dynamixel_driver')
        self.declare_parameter('device_name', DEFAULT_DEVICE)
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter('dxl_id', 1)  # alias for pan_id
        self.declare_parameter('pan_min_ticks', 1024)
        self.declare_parameter('pan_max_ticks', 3072)
        self.declare_parameter('pan_home_ticks', 2048)
        self.declare_parameter('pan_invert', True)
        self.declare_parameter('tilt_min_ticks', 700)
        self.declare_parameter('tilt_max_ticks', 2220)
        self.declare_parameter('tilt_home_ticks', 1550)
        self.declare_parameter('tilt_invert', False)
        self.declare_parameter('enable_tilt', True)
        self.declare_parameter('enable_gimbal_pan', True)
        self.declare_parameter('freeze_gimbal', False)
        self.declare_parameter('goto_home_on_start', True)
        self.declare_parameter('idle_pan_ticks', 2048)
        self.declare_parameter('idle_tilt_ticks', 1550)
        self.declare_parameter('lock_sec', 0.08)
        self.declare_parameter('lost_return_idle_sec', 1.5)
        self.declare_parameter('profile_velocity', 360)
        self.declare_parameter('profile_acceleration', 50)
        self.declare_parameter('home_profile_velocity', 80)
        self.declare_parameter('dead_zone_deg', 1.5)
        self.declare_parameter('min_goal_change_deg', 0.5)
        self.declare_parameter('max_step_deg', 45.0)
        self.declare_parameter('error_repeat_deg', 2.5)
        self.declare_parameter('pan_urdf_joint', 'gimbal_pan_joint')
        self.declare_parameter('tilt_urdf_joint', 'gimbal_tilt_joint')
        self.declare_parameter('pan_urdf_sign', 1.0)
        self.declare_parameter('tilt_urdf_sign', -1.0)
        self.declare_parameter('tilt_urdf_max_deg', 40.0)
        self.declare_parameter('joint_state_topic', 'gimbal_joint_states')
        self.declare_parameter('joint_state_hz', 20.0)

        device = str(self.get_parameter('device_name').value)
        baud = int(self.get_parameter('baudrate').value)
        self._dead_zone = float(self.get_parameter('dead_zone_deg').value)
        self._min_goal_change_deg = float(self.get_parameter('min_goal_change_deg').value)
        self._max_step_deg = max(1.0, float(self.get_parameter('max_step_deg').value))
        self._error_repeat_deg = max(0.5, float(self.get_parameter('error_repeat_deg').value))
        self._track_vel = int(self.get_parameter('profile_velocity').value)
        self._track_acc = int(self.get_parameter('profile_acceleration').value)
        self._home_vel = int(self.get_parameter('home_profile_velocity').value)
        self._lock_sec = float(self.get_parameter('lock_sec').value)
        self._lost_return_idle_sec = float(self.get_parameter('lost_return_idle_sec').value)
        self._idle_pan = int(self.get_parameter('idle_pan_ticks').value)
        self._idle_tilt = int(self.get_parameter('idle_tilt_ticks').value)
        self._enable_pan = bool(self.get_parameter('enable_gimbal_pan').value)
        self._freeze_gimbal = bool(self.get_parameter('freeze_gimbal').value)
        self._tracking = False
        self._seen_since = None
        self._lost_since = None
        self._last_rel: dict[str, float] = {}
        self._pending: dict[str, float] = {}

        pan_id = int(self.get_parameter('pan_id').value)
        if int(self.get_parameter('dxl_id').value) != 1:
            pan_id = int(self.get_parameter('dxl_id').value)

        tilt_urdf_max = math.radians(
            max(1.0, float(self.get_parameter('tilt_urdf_max_deg').value)))
        self._pan = Joint(
            'pan', str(self.get_parameter('pan_urdf_joint').value), pan_id,
            int(self.get_parameter('pan_min_ticks').value),
            int(self.get_parameter('pan_max_ticks').value),
            int(self.get_parameter('pan_home_ticks').value),
            bool(self.get_parameter('pan_invert').value),
            float(self.get_parameter('pan_urdf_sign').value),
            tf_zero=self._idle_pan,
        )
        self._tilt: Joint | None = None
        if bool(self.get_parameter('enable_tilt').value):
            self._tilt = Joint(
                'tilt', str(self.get_parameter('tilt_urdf_joint').value),
                int(self.get_parameter('tilt_id').value),
                int(self.get_parameter('tilt_min_ticks').value),
                int(self.get_parameter('tilt_max_ticks').value),
                int(self.get_parameter('tilt_home_ticks').value),
                bool(self.get_parameter('tilt_invert').value),
                float(self.get_parameter('tilt_urdf_sign').value),
                tf_zero=self._idle_tilt,
                urdf_max_rad=tilt_urdf_max,
            )

        self._port = PortHandler(device)
        self._packet = PacketHandler(PROTOCOL_VERSION)
        if not self._port.openPort():
            raise RuntimeError(f'Failed to open Dynamixel port: {device}')
        if not self._port.setBaudRate(baud):
            raise RuntimeError(f'Failed to set baudrate {baud}')

        joints = [self._pan] + ([self._tilt] if self._tilt else [])
        for joint in joints:
            self._setup_joint(joint, device, baud)

        goto_home = bool(self.get_parameter('goto_home_on_start').value)
        for joint in joints:
            self._write4(joint, ADDR_PROFILE_VELOCITY, self._home_vel)
            self._write4(joint, ADDR_PROFILE_ACCELERATION, self._track_acc)
            self._write1(joint, ADDR_TORQUE_ENABLE, 1)
        if goto_home:
            self._go_idle()
        else:
            for joint in joints:
                pos = self._read_pos(joint)
                if pos is not None:
                    joint.last_goal = pos

        self.create_subscription(Float32, 'camera_position', self._on_pan, 10)
        self.create_subscription(Float32, 'camera_tilt', self._on_tilt, 10)
        self.create_subscription(User, 'user_raw', self._on_user, 10)
        js_topic = str(self.get_parameter('joint_state_topic').value)
        js_hz = max(2.0, float(self.get_parameter('joint_state_hz').value))
        self._js_pub = self.create_publisher(JointState, js_topic, 10)
        self._js_timer = self.create_timer(1.0 / js_hz, self._publish_joint_states)
        ids = ', '.join(f'{j.name}=id{j.dxl_id}' for j in joints)
        tilt_sign = self._tilt.urdf_sign if self._tilt else 0.0
        if self._freeze_gimbal:
            self.get_logger().info(
                f'Dynamixel frozen at idle on {device} ({ids}) '
                f'pan={self._idle_pan} tilt={self._idle_tilt} — straight-follow / no tracking')
        else:
            self.get_logger().info(
                f'Dynamixel pan+tilt ready on {device} ({ids}) '
                f'idle pan={self._idle_pan} tilt={self._idle_tilt} '
                f'enable_pan={self._enable_pan} enable_tilt={self._tilt is not None} '
                f'urdf_sign pan={self._pan.urdf_sign} tilt={tilt_sign} — waiting for person')

    def _restore_track_profile(self) -> None:
        for joint in [self._pan] + ([self._tilt] if self._tilt else []):
            self._write4(joint, ADDR_PROFILE_VELOCITY, self._track_vel)

    def _go_idle(self) -> None:
        for joint in [self._pan] + ([self._tilt] if self._tilt else []):
            self._write4(joint, ADDR_PROFILE_VELOCITY, self._home_vel)
        self._last_rel.clear()
        self._pending.clear()
        self._goto(self._pan, self._idle_pan, force=True)
        if self._tilt is not None:
            self._goto(self._tilt, self._idle_tilt, force=True)
        self.get_logger().info(
            f'Idle pose pan={self._idle_pan} tilt={self._idle_tilt} — waiting for person')

    def _setup_joint(self, joint: Joint, device: str, baud: int) -> None:
        _model, comm, err = self._packet.ping(self._port, joint.dxl_id)
        self._require(
            f'Ping {joint.name} id={joint.dxl_id} on {device} @ {baud}',
            comm, err, allow_hw_err=True)
        comm, err = self._packet.write1ByteTxRx(
            self._port, joint.dxl_id, ADDR_TORQUE_ENABLE, 0)
        self._require(f'{joint.name}: disable torque', comm, err)
        comm, err = self._packet.write1ByteTxRx(
            self._port, joint.dxl_id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        self._require(f'{joint.name}: set position mode', comm, err)
        self._write4(joint, ADDR_MIN_POSITION_LIMIT, joint.min_pos)
        self._write4(joint, ADDR_MAX_POSITION_LIMIT, joint.max_pos)

    def _require(
        self, what: str, comm: int, err: int, *, allow_hw_err: bool = False,
    ) -> None:
        if comm == COMM_SUCCESS and (not err or allow_hw_err):
            return
        detail = self._packet.getTxRxResult(comm)
        if err:
            detail += f'; {self._packet.getRxPacketError(err) or f"err={err}"}'
        hint = ''
        if comm != COMM_SUCCESS:
            hint = (
                ' USB adapter opened but a servo sent no reply. '
                'U2D2 is data-only: 12 V on the Dynamixel chain, TTL switch, '
                'and both ID 1 (pan) and ID 2 (tilt) on the same cable.')
        raise RuntimeError(f'{what}: {detail}.{hint}')

    def _write1(self, joint: Joint, addr: int, val: int) -> None:
        comm, err = self._packet.write1ByteTxRx(
            self._port, joint.dxl_id, addr, val)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error(
                f'{joint.name} write1 addr={addr} failed err={err}',
                throttle_duration_sec=2.0)

    def _write4(self, joint: Joint, addr: int, val: int) -> None:
        comm, err = self._packet.write4ByteTxRx(
            self._port, joint.dxl_id, addr, val)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error(
                f'{joint.name} write4 addr={addr} failed err={err}')

    def _read_pos(self, joint: Joint) -> int | None:
        pos, comm, err = self._packet.read4ByteTxRx(
            self._port, joint.dxl_id, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error(
                f'{joint.name}: failed to read position',
                throttle_duration_sec=2.0)
            return None
        ticks = int(pos)
        if ticks < 0 or ticks >= TICKS_PER_REV:
            self.get_logger().warn(
                f'{joint.name}: present position {ticks} out of range',
                throttle_duration_sec=2.0)
            return None
        joint.last_ticks = ticks
        return ticks

    def _ticks_to_rad(self, joint: Joint, ticks: int) -> float:
        # Joint 0 in the URDF is the idle look, not a wild present-position glitch.
        angle = (
            (ticks - joint.tf_zero) * (2.0 * math.pi / TICKS_PER_REV)
            * joint.urdf_sign)
        if joint.urdf_max_rad > 0.0:
            angle = max(-joint.urdf_max_rad, min(joint.urdf_max_rad, angle))
        return angle

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint in [self._pan] + ([self._tilt] if self._tilt else []):
            ticks = self._read_pos(joint)
            if ticks is None:
                ticks = joint.last_ticks if joint.last_ticks >= 0 else joint.tf_zero
            msg.name.append(joint.urdf_joint)
            msg.position.append(self._ticks_to_rad(joint, ticks))
        self._js_pub.publish(msg)

    def _goto(self, joint: Joint, ticks: int, *, force: bool = False) -> None:
        goal = max(joint.min_pos, min(joint.max_pos, int(ticks)))
        ticks_per_deg = TICKS_PER_REV / 360.0
        if (
            not force
            and joint.last_goal >= 0
            and abs(goal - joint.last_goal) < self._min_goal_change_deg * ticks_per_deg
        ):
            return
        joint.last_goal = goal
        self._write4(joint, ADDR_GOAL_POSITION, goal)

    def _on_relative(self, joint: Joint | None, msg: Float32) -> None:
        if joint is None:
            return
        if not self._tracking:
            # First pan often arrives before lock_sec. Keep it so we move
            # as soon as tracking starts (depth lock must still yaw).
            self._pending[joint.name] = msg.data
            return
        self._apply_relative(joint, msg.data)

    def _apply_relative(self, joint: Joint, data: float) -> None:
        raw = -data if joint.invert else data
        if abs(raw) < self._dead_zone:
            return
        current = self._read_pos(joint)
        if current is None:
            self._pending[joint.name] = data
            return
        ticks_per_deg = TICKS_PER_REV / 360.0
        arrived = (
            joint.last_goal < 0
            or abs(current - joint.last_goal) <= max(6.0, 0.8 * ticks_per_deg))
        prev = self._last_rel.get(joint.name)
        same_err = (
            prev is not None and abs(raw - prev) < self._error_repeat_deg)
        # Skip a repeat error only while the last command is still in motion.
        # If the first write never moved the servo, or it already arrived and
        # the person is still off-center (pixel stuck at ~420), keep commanding.
        if same_err and not arrived:
            stalled = (
                joint.last_cmd_ticks >= 0
                and abs(current - joint.last_cmd_ticks) <= 6)
            if not stalled:
                return
        step = max(-self._max_step_deg, min(self._max_step_deg, raw))
        delta = int(step * ticks_per_deg)
        if abs(delta) < 1:
            return
        self._last_rel[joint.name] = raw
        joint.last_cmd_ticks = current
        # If the last goal never moved the servo, rewrite even if ticks match.
        self._goto(
            joint, current - delta,
            force=same_err and not arrived)
        self.get_logger().info(
            f'{joint.name} track step={step:.1f}° {current}->{current - delta}',
            throttle_duration_sec=1.0)

    def _flush_pending(self) -> None:
        pending = dict(self._pending)
        self._pending.clear()
        if self._enable_pan and 'pan' in pending:
            self._apply_relative(self._pan, pending['pan'])
        if self._tilt is not None and 'tilt' in pending:
            self._apply_relative(self._tilt, pending['tilt'])

    def _on_pan(self, msg: Float32) -> None:
        if self._freeze_gimbal or not self._enable_pan:
            return
        self._on_relative(self._pan, msg)

    def _on_tilt(self, msg: Float32) -> None:
        if self._freeze_gimbal:
            return
        self._on_relative(self._tilt, msg)

    def _on_user(self, msg: User) -> None:
        if self._freeze_gimbal:
            return
        now = self.get_clock().now()
        seen = msg.detection == 1 and msg.z >= 0.15
        if seen:
            self._lost_since = None
            if self._seen_since is None:
                self._seen_since = now
            held = (now - self._seen_since).nanoseconds / 1e9
            if not self._tracking and held >= self._lock_sec:
                self._tracking = True
                self._restore_track_profile()
                self.get_logger().info('Person locked — gimbal tracking')
                self._flush_pending()
        else:
            self._seen_since = None
            if self._tracking:
                if self._lost_since is None:
                    self._lost_since = now
                lost = (now - self._lost_since).nanoseconds / 1e9
                if lost >= self._lost_return_idle_sec:
                    self._tracking = False
                    self._go_idle()

    def destroy_node(self) -> bool:
        try:
            if rclpy.ok():
                for joint in [self._pan] + ([self._tilt] if self._tilt else []):
                    self._write1(joint, ADDR_TORQUE_ENABLE, 0)
            self._port.closePort()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamixelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
