from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 1. Generic Loader (Automatic selection via parameter)
def from_parameters(node) -> QoSProfile:
    """Load the QoS profile specified by the 'qos.profile' parameter."""
    """Load the QoS profile specified by the 'qos.profile' parameter."""
    if node.has_parameter("qos.profile"):
        profile_name = node.get_parameter("qos.profile").value
    else:
        profile_name = node.declare_parameter("qos.profile", "telemetry").value

    return _load_profile(node, profile_name)


# 2. Explicit Profile Getters (Manual selection)
def telemetry(node) -> QoSProfile:
    """Best-effort, volatile, depth=10. Good for sensor data."""
    return _load_profile(node, "telemetry")

def commands(node) -> QoSProfile:
    """Reliable, volatile, depth=1. Good for control commands."""
    return _load_profile(node, "commands")

def state_latched(node) -> QoSProfile:
    """Reliable, transient-local, depth=1. Good for state/configuration."""
    return _load_profile(node, "state_latched")

def events(node) -> QoSProfile:
    """Reliable, volatile, depth=50. Good for event logs."""
    return _load_profile(node, "events")

def reliable_data(node) -> QoSProfile:
    """Reliable, volatile, depth=10. Good for critical data (e.g. TF)."""
    return _load_profile(node, "reliable_data")

def static_data_latched(node) -> QoSProfile:
    """Reliable, transient-local, depth=1. Good for maps/static info."""
    return _load_profile(node, "static_data_latched")


# 3. Internal Implementation
def _load_profile(node, profile_name: str) -> QoSProfile:
    """Load a specific profile, allowing parameter overrides."""
    profile_key = _normalize_profile(profile_name)
    defaults_map = _profile_defaults()
    
    # Fallback to telemetry if unknown
    defaults = defaults_map.get(profile_key, defaults_map["telemetry"])
    
    # Use the normalized key for parameter lookups
    base = f"qos.profiles.{profile_key}"
    
    # Declare params with defaults (allows external override)
    # Declare params with defaults (allows external override)
    def get_or_declare(n, name, default):
        if n.has_parameter(name):
            return n.get_parameter(name).value
        return n.declare_parameter(name, default).value

    rel = get_or_declare(node, f"{base}.reliability", defaults["reliability"])
    durability = get_or_declare(node, f"{base}.durability", defaults["durability"])
    depth = get_or_declare(node, f"{base}.depth", defaults["depth"])

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


def _profile_defaults():
    return {
        "telemetry": {"reliability": "best_effort", "durability": "volatile", "depth": 10},
        "commands": {"reliability": "reliable", "durability": "volatile", "depth": 1},
        "state_latched": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        "events": {"reliability": "reliable", "durability": "volatile", "depth": 50},
        "reliable_data": {"reliability": "reliable", "durability": "volatile", "depth": 10},
        "static_data_latched": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
    }


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
