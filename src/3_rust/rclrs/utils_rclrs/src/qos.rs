use rclrs::{DurabilityPolicy, Node, QosProfile, ReliabilityPolicy};

pub fn from_parameters(node: &Node) -> QosProfile {
    let profile = node
        .declare_parameter("qos.profile")
        .default("telemetry")
        .get::<String>()
        .unwrap_or_else(|_| "telemetry".to_string());
    let profile_key = normalize_profile(&profile);
    let defaults = defaults_for_profile(&profile_key);
    let base = format!("qos.profiles.{}.", profile_key);
    let rel = node
        .declare_parameter(&(base.clone() + "reliability"))
        .default(defaults.reliability)
        .get::<String>()
        .unwrap_or_else(|_| defaults.reliability.to_string());
    let durability = node
        .declare_parameter(&(base.clone() + "durability"))
        .default(defaults.durability)
        .get::<String>()
        .unwrap_or_else(|_| defaults.durability.to_string());
    let depth = node
        .declare_parameter(&(base + "depth"))
        .default(defaults.depth)
        .get::<i64>()
        .unwrap_or(defaults.depth);

    let mut qos = QosProfile::default();
    qos.depth = depth as usize;
    if rel == "best_effort" {
        qos.reliability = ReliabilityPolicy::BestEffort;
    } else {
        qos.reliability = ReliabilityPolicy::Reliable;
    }
    if durability == "transient_local" {
        qos.durability = DurabilityPolicy::TransientLocal;
    } else {
        qos.durability = DurabilityPolicy::Volatile;
    }
    qos
}

struct ProfileDefaults {
    reliability: &'static str,
    durability: &'static str,
    depth: i64,
}

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

fn normalize_profile(profile: &str) -> String {
    let mut key = profile.trim().to_lowercase();
    if key == "statelatched" {
        key = "state_latched".to_string();
    } else if key == "reliabledata" {
        key = "reliable_data".to_string();
    } else if key == "staticdatalatched" {
        key = "static_data_latched".to_string();
    }
    if key.is_empty() {
        "telemetry".to_string()
    } else {
        key
    }
}
