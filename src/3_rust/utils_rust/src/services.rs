use rclrs::Node;

/// Service name constants.
pub const ADD_TWO_INTS: &str = "add_two_ints";
pub const ADD_TWO_INTS_SERVICE: &str = "add_two_ints";

/// Retrieve the service name from parameters with a default fallback.
pub fn from_params(node: &Node, service_name: &str, default_value: &str) -> String {
    node.declare_parameter(&format!("services.{}", service_name))
        .default(default_value)
        .get::<String>()
        .unwrap_or_else(|_| default_value.to_string())
}
