use std::sync::Arc;
use rclrs::Node;

/// Topic name constants.
pub const ROBOT_NEWS_TOPIC: &str = "robot_news";
pub const NUMBER_TOPIC: &str = "number";
pub const NUMBER_COUNT_TOPIC: &str = "number_count";

use rclrs::MandatoryParameter;

/// Retrieve the topic name from parameters with a default fallback.
pub fn from_params(node: &Node, topic_name: &str, default_value: &str) -> String {
    let param = node.declare_parameter(format!("topics.{}", topic_name).as_str())
        .default(Arc::from(default_value))
        .mandatory();
        
    param.map(|p: MandatoryParameter<Arc<str>>| p.get().to_string())
        .unwrap_or_else(|_| default_value.to_string())
}

pub const CHATTER: &str = "chatter";

pub fn chatter(node: &Node) -> String {
    from_params(node, "chatter", CHATTER)
}

pub fn robot_news(node: &Node) -> String {
    from_params(node, "robot_news", ROBOT_NEWS_TOPIC)
}

pub fn number(node: &Node) -> String {
    from_params(node, "number", NUMBER_TOPIC)
}

pub fn number_count(node: &Node) -> String {
    from_params(node, "number_count", NUMBER_COUNT_TOPIC)
}
