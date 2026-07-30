"""Load navigation test data from a rosbag2 directory.

Extracts time series needed by the metrics modules:
  - cmd velocities (/cmd_vel and the intermediate chain topics)
  - filtered odometry (pose + velocity)
  - global plan (/plan, last received)
"""

from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py


def _reader(bag_path: str):
    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def load_bag(bag_path: str, topics=None):
    """Return {topic: (times[s], [messages])} for the requested topics."""
    bag_path = Path(bag_path)
    reader = _reader(bag_path)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(topics) if topics else set(type_map)

    data = {name: ([], []) for name in wanted if name in type_map}
    msg_classes = {name: get_message(type_map[name]) for name in data}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in data:
            msg = deserialize_message(raw, msg_classes[topic])
            data[topic][0].append(t_ns * 1e-9)
            data[topic][1].append(msg)

    return {name: (np.asarray(times), msgs) for name, (times, msgs) in data.items()}


def odometry_series(times, msgs):
    """(t, x, y, yaw, v, w) arrays from nav_msgs/Odometry messages."""
    xs, ys, yaws, vs, ws = [], [], [], [], []
    for m in msgs:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        xs.append(p.x)
        ys.append(p.y)
        yaws.append(yaw)
        vs.append(m.twist.twist.linear.x)
        ws.append(m.twist.twist.angular.z)
    return (times, np.asarray(xs), np.asarray(ys), np.asarray(yaws),
            np.asarray(vs), np.asarray(ws))


def twist_series(times, msgs):
    """(t, v, w) arrays from geometry_msgs/Twist messages."""
    vs = np.asarray([m.linear.x for m in msgs])
    ws = np.asarray([m.angular.z for m in msgs])
    return times, vs, ws


def last_path_xy(msgs):
    """(x, y) arrays of the final received nav_msgs/Path."""
    if not msgs:
        return np.array([]), np.array([])
    path = msgs[-1]
    xs = np.asarray([p.pose.position.x for p in path.poses])
    ys = np.asarray([p.pose.position.y for p in path.poses])
    return xs, ys
