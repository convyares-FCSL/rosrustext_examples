/// Service name constants for roslibrust.
///
/// Roslibrust (rosbridge) does not support ROS parameters,
/// so service names are compile-time constants.
///
/// Keep names aligned with utils_rclrs for cross-lesson consistency.
pub const COMPUTE_STATS: &str = "/compute_stats";

/// Helper for symmetry with utils_rclrs.
/// Reads as: `services::compute_stats()`
#[inline]
pub fn compute_stats() -> &'static str {
    COMPUTE_STATS
}
