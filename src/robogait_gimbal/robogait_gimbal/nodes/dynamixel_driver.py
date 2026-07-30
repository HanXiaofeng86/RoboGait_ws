"""Dynamixel XL430 position driver — pan gimbal from camera_position (degrees)."""

from __future__ import annotations

import rclpy
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from rclpy.node import Node
from robogait_person_interfaces.msg import User
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


class DynamixelDriver(Node):
    """Subscribe to camera_position; move servo relative to current ticks."""

    def __init__(self) -> None:
        super().__init__('dynamixel_driver')
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('min_angle_deg', -80)
        self.declare_parameter('max_angle_deg', 80)
        self.declare_parameter('profile_velocity', 750)
        self.declare_parameter('profile_acceleration', 150)
        self.declare_parameter('dead_zone_deg', 2.5)
        self.declare_parameter('min_goal_change_deg', 1.0)
        self.declare_parameter('center_ticks', -1)

        device = self.get_parameter('device_name').value
        self._dxl_id = int(self.get_parameter('dxl_id').value)
        baud = int(self.get_parameter('baudrate').value)
        self._dead_zone = float(self.get_parameter('dead_zone_deg').value)
        self._min_goal_change_deg = float(self.get_parameter('min_goal_change_deg').value)
        center_ticks = int(self.get_parameter('center_ticks').value)

        self._port = PortHandler(device)
        self._packet = PacketHandler(PROTOCOL_VERSION)
        if not self._port.openPort():
            raise RuntimeError(f'Failed to open Dynamixel port: {device}')
        if not self._port.setBaudRate(baud):
            raise RuntimeError(f'Failed to set baudrate {baud}')

        comm, err = self._packet.write1ByteTxRx(
            self._port, self._dxl_id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        if comm != COMM_SUCCESS or err:
            raise RuntimeError('Failed to set position control mode')

        initial, comm, err = self._packet.read4ByteTxRx(
            self._port, self._dxl_id, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err:
            raise RuntimeError('Failed to read initial servo position')
        if 0 <= center_ticks <= 4095:
            initial = center_ticks
        self._zero_ticks = int(initial)

        min_angle = int(self.get_parameter('min_angle_deg').value)
        max_angle = int(self.get_parameter('max_angle_deg').value)
        self._min_pos = int(self._zero_ticks + min_angle * TICKS_PER_REV / 360)
        self._max_pos = int(self._zero_ticks + max_angle * TICKS_PER_REV / 360)
        self._write4(ADDR_MAX_POSITION_LIMIT, self._max_pos)
        self._write4(ADDR_MIN_POSITION_LIMIT, self._min_pos)
        self._write4(ADDR_PROFILE_VELOCITY, int(self.get_parameter('profile_velocity').value))
        self._write4(
            ADDR_PROFILE_ACCELERATION,
            int(self.get_parameter('profile_acceleration').value),
        )
        self._write1(ADDR_TORQUE_ENABLE, 1)
        if 0 <= center_ticks <= 4095:
            self._write4(ADDR_GOAL_POSITION, center_ticks)

        self._last_goal = -1
        self.create_subscription(Float32, 'camera_position', self._on_angle, 10)
        self.create_subscription(User, 'user_raw', self._on_user, 10)
        self.get_logger().info(
            f'Dynamixel ready on {device} id={self._dxl_id} zero_ticks={self._zero_ticks}'
        )

    def _write1(self, addr: int, val: int) -> None:
        comm, err = self._packet.write1ByteTxRx(self._port, self._dxl_id, addr, val)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error(f'Dynamixel write1 failed addr={addr} err={err}')

    def _write4(self, addr: int, val: int) -> None:
        comm, err = self._packet.write4ByteTxRx(self._port, self._dxl_id, addr, val)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error(f'Dynamixel write4 failed addr={addr} err={err}')

    def _on_user(self, msg: User) -> None:
        pass  # optional: could stop gimbal when person lost

    def _on_angle(self, msg: Float32) -> None:
        angle = -msg.data
        if abs(angle) < self._dead_zone:
            return
        current, comm, err = self._packet.read4ByteTxRx(
            self._port, self._dxl_id, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err:
            self.get_logger().error('Failed to read present position')
            return
        delta_ticks = int(angle * TICKS_PER_REV / 360)
        goal = max(self._min_pos, min(self._max_pos, int(current) - delta_ticks))
        ticks_per_deg = TICKS_PER_REV / 360.0
        if (
            self._last_goal >= 0
            and abs(goal - self._last_goal) < self._min_goal_change_deg * ticks_per_deg
        ):
            return
        self._last_goal = goal
        self._write4(ADDR_GOAL_POSITION, goal)

    def destroy_node(self) -> bool:
        try:
            self._write1(ADDR_TORQUE_ENABLE, 0)
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
