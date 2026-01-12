// utils_roslibrust/src/services.rs
use crate::utils::Config;

// --- CONSTANTS ---
// The Source of Truth for service names when not using dynamic config.
pub const COMPUTE_STATS: &str = "/compute_stats";
const DEFAULT_COMPUTE_STATS: &str = "/compute_stats";

// --- PUBLIC API ---

/// Primary accessor for Lesson 04 (Service).
/// Returns the hardcoded service name.
/// Keeps backward compatibility with older lessons.
#[inline]
pub fn compute_stats() -> &'static str {
    COMPUTE_STATS
}

/// Configuration-aware accessor for Lesson 05+ (if needed).
/// Reads 'services.compute_stats' from the YAML config.
pub fn compute_stats_from_config(cfg: &Config) -> String {
    cfg.get_or("services.compute_stats", DEFAULT_COMPUTE_STATS.to_string())
}