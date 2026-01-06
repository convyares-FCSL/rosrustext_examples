from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoSProfile from node parameters
def from_parameters(node) -> QoSProfile:
    profile_name = node.declare_parameter("qos.profile", "telemetry").value
    profile_key = _normalize_profile(profile_name)
    defaults_map = _profile_defaults()
    defaults = defaults_map.get(profile_key, defaults_map["telemetry"])
    base = f"qos.profiles.{profile_key}"
    rel = node.declare_parameter(f"{base}.reliability", defaults["reliability"]).value
    durability = node.declare_parameter(f"{base}.durability", defaults["durability"]).value
    depth = node.declare_parameter(f"{base}.depth", defaults["depth"]).value

    profile = QoSProfile(depth=depth)
    if rel == "best_effort":
        profile.reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        profile.reliability = ReliabilityPolicy.RELIABLE

    if durability == "transient_local":
        profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        profile.durability = DurabilityPolicy.VOLATILE
    return profile

# Default QoS profiles
def _profile_defaults():
    return {
        "telemetry": {"reliability": "best_effort", "durability": "volatile", "depth": 10},
        "commands": {"reliability": "reliable", "durability": "volatile", "depth": 1},
        "state_latched": {
            "reliability": "reliable",
            "durability": "transient_local",
            "depth": 1,
        },
        "events": {"reliability": "reliable", "durability": "volatile", "depth": 50},
        "reliable_data": {"reliability": "reliable", "durability": "volatile", "depth": 10},
        "static_data_latched": {
            "reliability": "reliable",
            "durability": "transient_local",
            "depth": 1,
        },
    }


# Helper to normalize profile names and their aliases
def _normalize_profile(name: str) -> str:
    if not name:
        return "telemetry"
    key = name.strip()
    aliases = {
        "stateLatched": "state_latched",
        "statelatched": "state_latched",
        "reliableData": "reliable_data",
        "reliabledata": "reliable_data",
        "staticDataLatched": "static_data_latched",
        "staticdatalatched": "static_data_latched",
    }
    alias = aliases.get(key) or aliases.get(key.lower())
    if alias:
        return alias
    return key.lower()
