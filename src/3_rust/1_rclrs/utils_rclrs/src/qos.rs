use std::sync::Arc;
use rclrs::{QoSDurabilityPolicy, Node, QoSProfile, QoSReliabilityPolicy, MandatoryParameter, QoSHistoryPolicy};

/// Create a QosProfile from ROS2 node parameters.
pub fn from_parameters(node: &Node) -> QoSProfile {
    let profile = node
        .declare_parameter("qos.profile")
        .default(Arc::from("telemetry"))
        .mandatory()
        .map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| "telemetry".to_string());
    _load_profile_impl(node, &profile)
}

fn _load_profile_impl(node: &Node, profile_name: &str) -> QoSProfile {
   let profile_key = normalize_profile(profile_name);
    let defaults = defaults_for_profile(&profile_key);
    let base = format!("qos.profiles.{}.", profile_key);
    let rel = node
        .declare_parameter((base.clone() + "reliability").as_str())
        .default(Arc::from(defaults.reliability))
        .mandatory()
        .map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| defaults.reliability.to_string());
    let durability = node
        .declare_parameter((base.clone() + "durability").as_str())
        .default(Arc::from(defaults.durability))
        .mandatory()
        .map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| defaults.durability.to_string());
    let depth = node
        .declare_parameter((base + "depth").as_str())
        .default(defaults.depth)
        .mandatory()
        .map(|p: MandatoryParameter<i64>| p.get())
        .unwrap_or(defaults.depth);

    let mut qos = QoSProfile::default();
    qos.history = QoSHistoryPolicy::KeepLast { depth: depth as u32 };
    if rel == "best_effort" {
        qos.reliability = QoSReliabilityPolicy::BestEffort;
    } else {
        qos.reliability = QoSReliabilityPolicy::Reliable;
    }
    if durability == "transient_local" {
        qos.durability = QoSDurabilityPolicy::TransientLocal;
    } else {
        qos.durability = QoSDurabilityPolicy::Volatile;
    }
    qos
}

/// Struct to hold default QoS settings for a profile.
struct ProfileDefaults {
    reliability: &'static str,
    durability: &'static str,
    depth: i64,
}

/// Get default QoS settings for a given profile.
fn defaults_for_profile(profile: &str) -> ProfileDefaults {
    match profile {
        "commands" => ProfileDefaults {
            reliability: "reliable",
            durability: "volatile",
            depth: 1,
        },
        "state_latched" => ProfileDefaults {
            reliability: "reliable",
            durability: "transient_local",
            depth: 1,
        },
        "events" => ProfileDefaults {
            reliability: "reliable",
            durability: "volatile",
            depth: 50,
        },
        "reliable_data" => ProfileDefaults {
            reliability: "reliable",
            durability: "volatile",
            depth: 10,
        },
        "static_data_latched" => ProfileDefaults {
            reliability: "reliable",
            durability: "transient_local",
            depth: 1,
        },
        _ => ProfileDefaults {
            reliability: "best_effort",
            durability: "volatile",
            depth: 10,
        },
    }
}

/// Normalize profile names to standard keys.
fn normalize_profile(profile: &str) -> String {
    let key = profile.trim().to_lowercase();
    match key.as_str() {
        "" => "telemetry".to_string(),
        "statelatched" => "state_latched".to_string(),
        "reliabledata" => "reliable_data".to_string(),
        "staticdatalatched" => "static_data_latched".to_string(),
        _ => key,
    }
}
