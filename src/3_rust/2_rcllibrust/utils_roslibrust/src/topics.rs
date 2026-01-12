// utils_roslibrust/src/topics.rs
use crate::utils::Config;

// --- CONSTANTS (For Lessons 00-04) ---
// These are used directly by lessons that don't implement the full Config loader.
pub const CHATTER: &str = "/tutorial/chatter";

// --- CONFIGURATION HELPERS (For Lessons 05+) ---
// These are used by lessons that load YAML config.

const DEFAULT_TELEMETRY: &str = "/tutorial/telemetry";

/// Resolves the telemetry topic name from configuration.
/// Key: `topics.telemetry`
pub fn telemetry(cfg: &Config) -> String {
    cfg.get_or("topics.telemetry", DEFAULT_TELEMETRY.to_string())
}

/// Resolves the chatter topic name from configuration.
/// Key: `topics.chatter`
pub fn chatter(cfg: &Config) -> String {
    cfg.get_or("topics.chatter", CHATTER.to_string())
}