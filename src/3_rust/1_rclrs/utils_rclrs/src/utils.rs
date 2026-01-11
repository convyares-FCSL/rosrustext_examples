use rclrs::{log_error, log_warn, Node, ParameterVariant};
use std::fmt::Display;

/// Declares a parameter if not present, returns the effective value.
///
/// # Arguments
/// * `node` - Reference to the ROS 2 node.
/// * `name` - The parameter name.
/// * `default_value` - The default value.
/// * `warn_label` - Label for logging.
pub fn get_or_declare_parameter<T>(
    node: &Node,
    name: &str,
    default_value: T,
    warn_label: &str,
) -> T
where
    // T must be a valid ROS parameter type (Arc<str>, i64, bool, etc.)
    T: ParameterVariant + Clone + PartialEq + Display + std::fmt::Debug + 'static,
{
    // 1. Declare and Get
    let param_result = node
        .declare_parameter(name)
        .default(default_value.clone())
        .mandatory();

    match param_result {
        Ok(param) => {
            let value: T = param.get();

            // 2. Warn if using default
            if value == default_value {
                log_warn!(node.logger(), "[config] Using default {}: {}='{:?}' (no external override detected)", warn_label, name, value);
            }
            value
        }
        Err(e) => {
            log_error!(node.logger(),"Failed to declare parameter '{}': {}",name,e);

            default_value
        }
    }
}