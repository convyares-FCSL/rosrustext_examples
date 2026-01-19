// utils_roslibrust/src/qos.rs
use crate::utils::Config;
use serde::{Deserialize, Serialize};

/// Matches the structure in qos_config.yaml
#[derive(Debug, Deserialize)]
struct QosProfileConfig {
    pub reliability: String,
    pub durability: String,
    pub depth: u32,
}

/// Options supported by roslibrust/rosbridge
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct AdvertiseOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latch: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub queue_size: Option<usize>,
}

// --- Internal Helper: Load defaults matching rclrs qol ---
fn get_defaults_for_key(key: &str) -> QosProfileConfig {
    match key {
        "commands" => QosProfileConfig {
            reliability: "reliable".to_string(),
            durability: "volatile".to_string(),
            depth: 1,
        },
        "state_latched" => QosProfileConfig {
            reliability: "reliable".to_string(),
            durability: "transient_local".to_string(),
            depth: 1,
        },
        "events" => QosProfileConfig {
            reliability: "reliable".to_string(),
            durability: "volatile".to_string(),
            depth: 50,
        },
        "reliable_data" => QosProfileConfig {
            reliability: "reliable".to_string(),
            durability: "volatile".to_string(),
            depth: 10,
        },
        "static_data_latched" => QosProfileConfig {
            reliability: "reliable".to_string(),
            durability: "transient_local".to_string(),
            depth: 1,
        },
        // Default / "telemetry"
        _ => QosProfileConfig {
            reliability: "best_effort".to_string(),
            durability: "volatile".to_string(),
            depth: 10,
        },
    }
}

/// Loads a profile from Config, falling back to defaults if keys are missing.
/// Mirroring logic from utils_rclrs.
pub fn load_profile(cfg: &Config, profile_name: &str) -> AdvertiseOptions {
    let key = if profile_name.is_empty() { "telemetry" } else { profile_name };
    let defaults = get_defaults_for_key(key);
    let base = format!("qos.profiles.{}", key);

    // 1. Fetch values from Config (YAML) or use defaults
    // Note: roslibrust Config.get_or works for individual fields
    let _reliability: String = cfg.get_or(&format!("{}.reliability", base), defaults.reliability);
    let durability: String = cfg.get_or(&format!("{}.durability", base), defaults.durability);
    let depth: u32 = cfg.get_or(&format!("{}.depth", base), defaults.depth);

    // 2. Map to AdvertiseOptions
    // rosbridge v2 protocol maps 'latch' roughly to Transient Local durability
    let latch = durability == "transient_local";
    
    // rosbridge 'queue_size' maps to depth
    let queue_size = depth as usize;

    AdvertiseOptions {
        latch: Some(latch),
        queue_size: Some(queue_size),
    }
}

// --- Public API (Matches rclrs qol signatures) ---

pub fn telemetry(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "telemetry")
}

pub fn commands(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "commands")
}

pub fn state_latched(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "state_latched")
}

pub fn events(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "events")
}

pub fn reliable_data(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "reliable_data")
}

pub fn static_data_latched(cfg: &Config) -> AdvertiseOptions {
    load_profile(cfg, "static_data_latched")
}

/// Helper for when you just want default options without config lookup
pub fn get_options(profile: &str) -> AdvertiseOptions {
    // Create a dummy config (empty) to trigger default fallbacks
    // In a real app, you should pass the loaded config.
    // This helper exists mainly for Lesson 02 compatibility.
    let defaults = get_defaults_for_key(profile);
    AdvertiseOptions {
        latch: Some(defaults.durability == "transient_local"),
        queue_size: Some(defaults.depth as usize),
    }
}