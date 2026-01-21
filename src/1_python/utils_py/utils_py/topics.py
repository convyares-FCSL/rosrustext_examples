from rclpy.node import Node
from utils_py import _get_or_declare

# Canonical defaults (used only if no --params-file override is provided)
DEFAULT_CHATTER = "/tutorial/chatter"      # Lessons 00–04
DEFAULT_TELEMETRY = "/tutorial/telemetry"  # Lesson 05
DEFAULT_FIBONACCI = "/tutorial/fibonacci"  # Lesson 07/08

def chatter(node: Node) -> str:
    return _get_or_declare(node, "topics.chatter", DEFAULT_CHATTER, warn_label="topic")


def telemetry(node: Node) -> str:
    return _get_or_declare(node, "topics.telemetry", DEFAULT_TELEMETRY, warn_label="topic")


def fibonacci(node: Node) -> str:
    return _get_or_declare(node, "topics.fibonacci", DEFAULT_FIBONACCI, warn_label="action")
