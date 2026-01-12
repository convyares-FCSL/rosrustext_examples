use crate::utils::declare_parameter;
use rclrs::Node;
use std::sync::Arc;

pub const DEFAULT_COMPUTE_STATS: &str = "/compute_stats";

pub fn compute_stats(node: &Node) -> String {
    let default: Arc<str> = Arc::from(DEFAULT_COMPUTE_STATS);
    let param = declare_parameter(node, "services.compute_stats", default)
        .expect("Failed to declare parameter: services.compute_stats");
    let val: Arc<str> = param.get();
    val.to_string()
}
