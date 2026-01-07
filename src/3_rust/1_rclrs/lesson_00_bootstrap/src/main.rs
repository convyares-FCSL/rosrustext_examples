use std::time::Duration;
use rclrs::{log_info, Context, CreateBasicExecutor, Executor, Node, RclrsError, SpinOptions};

const NODE_NAME: &str = "lesson_00_bootstrap";

/// Struct representing the bootstrap node.
struct BootstrapNode {
    node: Node,
}

/// Implementation block for the BootstrapNode struct.
impl BootstrapNode {
    // Constructor: create the node and log startup.
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        /// Create the node using the executor's context.
        let node = executor.create_node(NODE_NAME)?;

        /// Log an informational message indicating the node has started.
        log_info!(node.logger(), "Lesson 00 bootstrap node started...");

        /// Return the constructed node.
        Ok(Self { node })
    }
}

/// Main function: entry point of the program.
fn main() -> Result<(), RclrsError> {
    /// Initialize the ROS 2 context from environment variables.
    let context = Context::default_from_env()?;

    /// Create an executor that will own and spin our callbacks.
    let mut executor = context.create_basic_executor();

    /// Create the node via a constructor, mirroring the rclpy pattern.
    let bootstrap = BootstrapNode::new(&executor)?;

    /// Spin the executor, handling errors appropriately.
    executor
        .spin(SpinOptions::spin_once().timeout(Duration::from_millis(0)))
        .ignore_non_errors()
        .first_error()
        .map(|err| {
            rclrs::log_error!(bootstrap.node.logger(), "Executor stopped with error: {err}");
            err
        })
        .transpose()?; // Option<Result<..>> -> Result<Option<..>> and propagate if Err

    /// Return success.
    Ok(())
}
