use crate::utils::get_or_declare_parameter;
use rclrs::Node;
use std::sync::Arc;

pub const DEFAULT_COMPUTE_STATS: &str = "/compute_stats";

pub fn compute_stats(node: &Node) -> String {
    let default: Arc<str> = Arc::from(DEFAULT_COMPUTE_STATS);
    let val = get_or_declare_parameter(
        node,
        "services.compute_stats",
        default,
        "service",
    );
    val.to_string()
}