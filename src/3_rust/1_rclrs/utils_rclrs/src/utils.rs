use rclrs::{DeclarationError, MandatoryParameter, Node, ParameterVariant};
use std::fmt::Debug;

/// Declare a parameter (or reuse existing declaration) and return a typed handle.
///
/// ROS 2 merges parameter overrides (YAML/CLI) before node creation, so `param.get()`
/// always returns the effective value. We do **not** try to guess whether it came
/// from YAML vs default (that’s not reliably inferable).
pub fn declare_parameter<T>(
    node: &Node,
    name: &str,
    default_value: T,
) -> Result<MandatoryParameter<T>, DeclarationError>
where
    T: ParameterVariant + Clone + Debug + 'static,
{
    let param = node
        .declare_parameter(name)
        .default(default_value)
        .mandatory()?;

    // Optional: log effective value once at startup (boring, truthful).
    // let value: T = param.get();
    // rclrs::log_info!(node.logger(), "[config] {} = {:?}", name, value);

    Ok(param)
}