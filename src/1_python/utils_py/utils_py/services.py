from rclpy.node import Node
from utils_py import _get_or_declare

DEFAULT_COMPUTE_STATS = "/compute_stats"

def compute_stats(node: Node) -> str:
    return _get_or_declare(node, "services.compute_stats", DEFAULT_COMPUTE_STATS, warn_label="service")
