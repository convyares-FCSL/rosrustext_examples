use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use rclrs::{
    log_info, Context, CreateBasicExecutor, Executor, MandatoryParameter, Node, RclrsError,
    SpinOptions, RclrsErrorFilter,
};

const NODE_NAME: &str = "lesson_01_node";

/// Lesson01Node: The container.
struct Lesson01Node {
    pub node: Node,

    // FIX: rclrs::Timer is already an Arc, so we don't need Arc<rclrs::Timer>
    _timer: rclrs::Timer, 
}

/// Implementation block for the Lesson01Node struct.
impl Lesson01Node {
    /// Constructor: Sets up the node and its resources.
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create the node.
        let node = executor.create_node(NODE_NAME)?;

        // "mandatory" ensures the user passed the param or we fail at startup.
        let period_param = node.declare_parameter("timer_period_s").default(1.0).mandatory()?;

        // State: We use AtomicU64 inside an Arc so it can be safely shared 
        // across thread boundaries if the executor becomes multi-threaded.
        let count = Arc::new(AtomicU64::new(0));

        // Build the timer.
        let timer = Self::build_timer(&node, count, period_param)?;

        // Log a message.
        log_info!(node.logger(), "Lesson 01 node started...");

        // Return the node.
        Ok(Self { node, _timer: timer })
    }

    /// Helper to construct the timer logic.
    // FIX: Return type is rclrs::Timer (which is already an Arc wrapper internally)
    fn build_timer(node: &Node, count: Arc<AtomicU64>, period_param: MandatoryParameter<f64>) -> Result<rclrs::Timer, RclrsError> {
        // Get the period.
        let mut period_s = period_param.get();

        // Validate the period.
        if !period_s.is_finite() || period_s <= 0.0 {
            period_s = 1.0;
            log_info!(node.logger(), "Invalid period. Defaulting to {}s.", period_s);
        }

        // Clone handle for the closure capture
        let node_clone = node.clone();
        
        // Build the timer.
        node.create_timer_repeating(
            Duration::from_secs_f64(period_s),  // Period.
            move || { Self::on_tick(&node_clone, &count); }  // Callback.
        )
    }

    // Pure logic function
    fn on_tick(node: &Node, count: &AtomicU64) {
        // Increment the count.
        let n = count.fetch_add(1, Ordering::Relaxed) + 1;

        // Log the count.
        log_info!(node.logger(), "tick {}", n);
    }
}

/// Main function.
fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson01Node::new(&executor)?;

    // Spin indefinitely - keep the node alive to process timer events
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