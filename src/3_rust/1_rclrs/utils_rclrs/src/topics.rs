use crate::utils::declare_parameter;
use rclrs::Node;
use std::sync::Arc;

pub const DEFAULT_CHATTER: &str = "/tutorial/chatter";
pub const DEFAULT_TELEMETRY: &str = "/tutorial/telemetry";

pub fn chatter(node: &Node) -> String {
    let default: Arc<str> = Arc::from(DEFAULT_CHATTER);
    let param = declare_parameter(node, "topics.chatter", default)
        .expect("Failed to declare parameter: topics.chatter");
    let val: Arc<str> = param.get();
    val.to_string()
}

pub fn telemetry(node: &Node) -> String {
    let default: Arc<str> = Arc::from(DEFAULT_TELEMETRY);
    let param = declare_parameter(node, "topics.telemetry", default)
        .expect("Failed to declare parameter: topics.telemetry");
    let val: Arc<str> = param.get();
    val.to_string()
}
