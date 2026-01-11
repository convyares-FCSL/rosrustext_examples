use crate::utils::get_or_declare_parameter;
use rclrs::Node;
use std::sync::Arc;

pub const DEFAULT_CHATTER: &str = "/tutorial/chatter";
pub const DEFAULT_TELEMETRY: &str = "/tutorial/telemetry";

pub fn chatter(node: &Node) -> String {
    // String params MUST be Arc<str> in rclrs
    let default: Arc<str> = Arc::from(DEFAULT_CHATTER);
    let val = get_or_declare_parameter(node, "topics.chatter", default, "topic");
    val.to_string()
}

pub fn telemetry(node: &Node) -> String {
    let default: Arc<str> = Arc::from(DEFAULT_TELEMETRY);
    let val = get_or_declare_parameter(node, "topics.telemetry", default, "topic");
    val.to_string()
}