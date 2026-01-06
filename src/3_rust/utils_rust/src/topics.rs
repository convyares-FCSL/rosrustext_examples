use rclrs::Node;

/// Topic name constants.
pub const ROBOT_NEWS_TOPIC: &str = "robot_news";
pub const NUMBER_TOPIC: &str = "number";
pub const NUMBER_COUNT_TOPIC: &str = "number_count";

/// Retrieve the topic name from parameters with a default fallback.
pub fn from_params(node: &Node, topic_name: &str, default_value: &str) -> String {
    node.declare_parameter(&format!("topics.{}", topic_name))
        .default(default_value)
        .get::<String>()
        .unwrap_or_else(|_| default_value.to_string())
}
