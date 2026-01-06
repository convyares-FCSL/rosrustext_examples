use rclrs::{Node, QosProfile, ReliabilityPolicy};

pub fn from_parameters(node: &Node) -> QosProfile {
    let rel = node
        .declare_parameter("qos.reliability")
        .default("reliable")
        .get::<String>()
        .unwrap_or_else(|_| "reliable".to_string());
    let depth = node
        .declare_parameter("qos.depth")
        .default(10)
        .get::<i64>()
        .unwrap_or(10);

    let mut qos = QosProfile::default();
    qos.depth = depth as usize;
    if rel == "best_effort" {
        qos.reliability = ReliabilityPolicy::BestEffort;
    } else {
        qos.reliability = ReliabilityPolicy::Reliable;
    }
    qos
}
