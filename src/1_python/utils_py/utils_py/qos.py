from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def from_parameters(node) -> QoSProfile:
    # Declare with defaults
    rel = node.declare_parameter("qos.reliability", "reliable").value
    depth = node.declare_parameter("qos.depth", 10).value

    profile = QoSProfile(depth=depth)
    if rel == "best_effort":
        profile.reliability = ReliabilityPolicy.BEST_EFFORT
    elif rel == "reliable":
        profile.reliability = ReliabilityPolicy.RELIABLE
    return profile
