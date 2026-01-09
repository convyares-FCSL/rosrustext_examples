use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use rclrs::{
    log_info, Context, CreateBasicExecutor, Executor, MandatoryParameter, Node, RclrsError,
    SpinOptions, Publisher, Logger,
};

// Now use local dependencies.
use lesson_interfaces::msg::MsgCount;
use utils_rust::{qos, topics};


const NODE_NAME: &str = "lesson_02_node";

/// Logic Component: Handles the business logic of counting and publishing.
struct PublisherChatter {
    publisher: Publisher<MsgCount>,
    count: AtomicU64,
    _logger: Logger, //RAII
}

/// Implementation block for the PublisherChatter struct.
impl PublisherChatter {
    // Pure logic function.
    fn on_tick(&self) -> Result<(), RclrsError> {
        // Increment count.
        let n = self.count.fetch_add(1, Ordering::Relaxed) + 1;

        // Build message.
        let msg = MsgCount { count: n as i64 };

        // Publish message.
        self.publisher.publish(&msg)?;

        Ok(())
    }
}

/// Resource Container: Manages the lifecycle of the node and its components.
struct Lesson02Node {
    pub node: Node,
    // The underscore prefix indicates these are held for RAII (keep-alive) purposes.
    _publisher_component: Arc<PublisherChatter>,
    _timer: Arc<rclrs::Timer>, 
}

/// Implementation block for the Lesson02Node struct.
impl Lesson02Node {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        let period_param = node.declare_parameter("timer_period_s").default(1.0).mandatory()?;

        // Instantiate components
        let publisher = Arc::new(Self::build_chatter_publisher(&node)?);
        
        let timer = Self::build_timer(&node, Arc::clone(&publisher), period_param)?;

        log_info!(node.logger(), "Lesson 02 node started...");

        Ok(Self {node, _publisher_component: publisher, _timer: timer})
    }

    // Helper to construct the publisher logic.
    fn build_chatter_publisher(node: &Node) -> Result<PublisherChatter, RclrsError> {
        // Clone logger for RAII.
        let logger = node.logger().clone();

        // Topic name.
        let topic_name = topics::chatter(node);

        // QoS profile.
        let qos_profile = qos::from_parameters(node);

        // Options.
        let mut options = rclrs::PublisherOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        // Publisher.
        let publisher = node.create_publisher(options)?;

        // Return the publisher.
        Ok(PublisherChatter { publisher, count: AtomicU64::new(0), _logger: logger })
    }

    /// Helper to construct the timer logic. Same as in lesson 01 with added publisher struct.
    fn build_timer(node: &Node, chatter: Arc<PublisherChatter>, period_param: MandatoryParameter<f64>) -> Result<Arc<rclrs::Timer>, RclrsError> {
        let mut period_s = period_param.get();
        if !period_s.is_finite() || period_s <= 0.0 {
            period_s = 1.0;
            log_info!(node.logger(), "Invalid period. Defaulting to {}s.", period_s);
        }

        node.create_timer_repeating(
            Duration::from_secs_f64(period_s), 
            move || {chatter.on_tick().expect("Timer callback failed");}
        )
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson02Node::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}