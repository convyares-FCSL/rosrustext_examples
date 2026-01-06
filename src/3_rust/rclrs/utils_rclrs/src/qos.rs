use rclrs::{Node, QOS_PROFILE_DEFAULT};
use rclrs::{QosProfile, ReliabilityPolicy};

pub fn from_parameters(node: &Node) -> QosProfile {
    // Note: rclrs parameter API might differ slightly depending on version.
    // Assuming a common pattern for tutorials if it was recommended.
    
    // In many rclrs versions, you might need to use node.declare_parameter("name").default(val).get()
    // or node.get_parameter("name").unwrap_or_else(...)
    
    let rel = node.declare_parameter("qos.reliability").default("reliable").get::<String>().unwrap_or_else(|_| "reliable".to_string());
    let depth = node.declare_parameter("qos.depth").default(10).get::<i64>().unwrap_or(10);

    let mut qos = QosProfile::default();
    qos.depth = depth as usize;
    if rel == "best_effort" {
        qos.reliability = ReliabilityPolicy::BestEffort;
    } else {
        qos.reliability = ReliabilityPolicy.Reliable;
    }
    qos
}
