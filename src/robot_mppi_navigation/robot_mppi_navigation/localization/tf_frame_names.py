"""TF frame names (map -> odom -> base_link -> sensors)."""

MAP = 'map'
ODOM = 'odom'
BASE_LINK = 'base_link'
LASER = 'lidar_link'
IMU = 'imu_link'

# Required chain for navigation, in parent -> child order.
REQUIRED_CHAIN = ((MAP, ODOM), (ODOM, BASE_LINK))
