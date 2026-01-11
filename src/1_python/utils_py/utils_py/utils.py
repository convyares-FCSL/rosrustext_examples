from rclpy.node import Node
from typing import Any


def _get_or_declare(node: Node, name: str, default: Any, *, warn_label: str) -> Any:
    """
    Declare the parameter (so ROS overrides become visible), then read it.
    Warn only when the effective value is the default AND it was not provided
    by an override (e.g. --params-file / -p).
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, default)

    value = node.get_parameter(name).value

    # rclpy stores overrides passed at node creation (params files / -p)
    overrides = getattr(node, "_parameter_overrides", {}) or {}
    was_overridden = name in overrides

    if (not was_overridden) and (value == default):
        node.get_logger().warn(
            f"[config] Using default {warn_label}: {name}='{value}' (no external override found)"
        )

    return value
