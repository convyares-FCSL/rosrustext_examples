from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from utils_py import _get_or_declare

DEFAULT_PROFILE = "telemetry"

_PROFILES = {
    "telemetry":          {"reliability": "best_effort", "durability": "volatile",        "depth": 10},
    "commands":           {"reliability": "reliable",    "durability": "volatile",        "depth": 1},
    "state_latched":      {"reliability": "reliable",    "durability": "transient_local", "depth": 1},
    "events":             {"reliability": "reliable",    "durability": "volatile",        "depth": 50},
    "reliable_data":      {"reliability": "reliable",    "durability": "volatile",        "depth": 10},
    "static_data_latched":{"reliability": "reliable",    "durability": "transient_local", "depth": 1},
}

def from_parameters(node: Node) -> QoSProfile:
    profile = _get_or_declare(
        node,
        "qos.default_profile",
        DEFAULT_PROFILE,
        warn_label="qos.default_profile",
    )
    return _load_profile(node, profile)


def telemetry(node: Node) -> QoSProfile:
    return _load_profile(node, "telemetry")


def commands(node: Node) -> QoSProfile:
    return _load_profile(node, "commands")


def state_latched(node: Node) -> QoSProfile:
    return _load_profile(node, "state_latched")


def events(node: Node) -> QoSProfile:
    return _load_profile(node, "events")


def reliable_data(node: Node) -> QoSProfile:
    return _load_profile(node, "reliable_data")


def static_data_latched(node: Node) -> QoSProfile:
    return _load_profile(node, "static_data_latched")


def _load_profile(node: Node, name: str) -> QoSProfile:
    key = (name or "").strip().lower()
    if key not in _PROFILES:
        key = DEFAULT_PROFILE

    defaults = _PROFILES[key]
    base = f"qos.profiles.{key}"

    rel = _get_or_declare(node, f"{base}.reliability", defaults["reliability"], warn_label="qos")
    dur = _get_or_declare(node, f"{base}.durability", defaults["durability"], warn_label="qos")
    depth = int(_get_or_declare(node, f"{base}.depth", defaults["depth"], warn_label="qos"))

    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT if rel == "best_effort" else ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL if dur == "transient_local" else DurabilityPolicy.VOLATILE
    return qos
