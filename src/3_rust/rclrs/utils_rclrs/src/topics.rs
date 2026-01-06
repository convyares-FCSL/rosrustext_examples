use rclrs::Node;

pub fn from_params(node: &Node, topic_name: &str, default_value: &str) -> String {
    node.declare_parameter(&format!("topics.{}", topic_name))
        .default(default_value)
        .get::<String>()
        .unwrap_or_else(|_| default_value.to_string())
}
