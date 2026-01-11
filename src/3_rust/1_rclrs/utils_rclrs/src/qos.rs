use crate::utils::get_or_declare_parameter;
use rclrs::{Node, QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy};
use std::sync::Arc;

pub const DEFAULT_PROFILE_NAME: &str = "telemetry";

struct ProfileDefaults {
    reliability: &'static str,
    durability: &'static str,
    depth: i64,
}

fn get_defaults_for_key(key: &str) -> ProfileDefaults {
    match key {
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

fn load_profile(node: &Node, profile_name: &str) -> QoSProfile {
    let key = if profile_name.is_empty() {
        DEFAULT_PROFILE_NAME
    } else {
        profile_name
    };

    let defs = get_defaults_for_key(key);
    let base = format!("qos.profiles.{}.", key);

    // Load reliability (must use Arc<str> for string params)
    let rel_default: Arc<str> = Arc::from(defs.reliability);
    let rel = get_or_declare_parameter(
        node,
        &format!("{}reliability", base),
        rel_default,
        "qos",
    );

    // Load durability
    let dur_default: Arc<str> = Arc::from(defs.durability);
    let dur = get_or_declare_parameter(
        node,
        &format!("{}durability", base),
        dur_default,
        "qos",
    );

    // Load depth (i64 is valid)
    let depth = get_or_declare_parameter(
        node,
        &format!("{}depth", base),
        defs.depth,
        "qos",
    );

    // Construct QoS
    // Note: use .as_ref() to compare Arc<str> to &str
    QoSProfile {
        history: QoSHistoryPolicy::KeepLast { depth: depth as u32 },
        reliability: if rel.as_ref() == "best_effort" {
            QoSReliabilityPolicy::BestEffort
        } else {
            QoSReliabilityPolicy::Reliable
        },
        durability: if dur.as_ref() == "transient_local" {
            QoSDurabilityPolicy::TransientLocal
        } else {
            QoSDurabilityPolicy::Volatile
        },
        ..QoSProfile::default()
    }
}

// --- Public API ---

pub fn from_parameters(node: &Node) -> QoSProfile {
    let default: Arc<str> = Arc::from(DEFAULT_PROFILE_NAME);
    let profile = get_or_declare_parameter(
        node,
        "qos.default_profile",
        default,
        "qos.default_profile",
    );
    // Convert Arc<str> -> &str for internal logic
    load_profile(node, &profile)
}

pub fn telemetry(node: &Node) -> QoSProfile {
    load_profile(node, "telemetry")
}

pub fn commands(node: &Node) -> QoSProfile {
    load_profile(node, "commands")
}

pub fn state_latched(node: &Node) -> QoSProfile {
    load_profile(node, "state_latched")
}

pub fn events(node: &Node) -> QoSProfile {
    load_profile(node, "events")
}

pub fn reliable_data(node: &Node) -> QoSProfile {
    load_profile(node, "reliable_data")
}

pub fn static_data_latched(node: &Node) -> QoSProfile {
    load_profile(node, "static_data_latched")
}