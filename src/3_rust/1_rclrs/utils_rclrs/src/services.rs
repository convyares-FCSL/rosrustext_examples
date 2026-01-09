use std::sync::Arc;
use rclrs::Node;

/// Service name constants.
pub const ADD_TWO_INTS: &str = "add_two_ints";
pub const ADD_TWO_INTS_SERVICE: &str = "add_two_ints";

use rclrs::MandatoryParameter;

/// Retrieve the service name from parameters with a default fallback.
pub fn from_params(node: &Node, service_name: &str, default_value: &str) -> String {
    node.declare_parameter(format!("services.{}", service_name).as_str())
        .default(Arc::from(default_value))
        .mandatory()
        .map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| default_value.to_string())
}
