"""
qos.py

Central source of truth for Quality of Service (QoS) profiles.
"""
from rclpy.qos import QoSProfile

# << FILL IN HERE >>: Define standard QoS profiles
# default_qos = QoSProfile(depth=10)

# Example: Best Effort for sensor data (fire and forget)
# from rclpy.qos import ReliabilityPolicy
# sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)