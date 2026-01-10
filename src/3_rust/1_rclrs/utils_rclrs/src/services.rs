use std::sync::Arc;
use rclrs::Node;
use rclrs::MandatoryParameter;

/// Service name constants.
pub const COMPUTE_STATS: &str = "compute_stats";

/// Retrieve the service name from parameters with a default fallback.
/// Looks for parameters under "services.<name>"
pub fn from_params(node: &Node, service_name: &str, default_value: &str) -> String {
    let param = node.declare_parameter(format!("services.{}", service_name).as_str())
        .default(Arc::from(default_value))
        .mandatory();
        
    param.map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| default_value.to_string())
}

/// Helper to get the compute_stats service name
pub fn compute_stats(node: &Node) -> String {
    from_params(node, "compute_stats", COMPUTE_STATS)
}