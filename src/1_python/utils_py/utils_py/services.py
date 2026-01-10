"""
Central definition of service names for the tutorial workspace.
"""

from rclpy.node import Node

def compute_stats(node: Node) -> str:
    """
    Returns the service name for the statistics service.
    Currently returns a hardcoded string, but wrapping it here allows
    for future remapping or parameterization logic if needed.
    """
    return "compute_stats"