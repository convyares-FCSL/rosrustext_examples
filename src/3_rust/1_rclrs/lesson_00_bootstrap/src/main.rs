use std::time::Duration;
use rclrs::{log_info, Context, CreateBasicExecutor, Executor, Node, RclrsError, SpinOptions};

/// Node name.
const NODE_NAME: &str = "lesson_00_bootstrap";

/// Lesson00Node: The container.
/// In ROS 2 Rust, we wrap the Node in a struct to manage its lifetime 
/// and the lifetimes of its resources (timers, publishers, subscribers).
struct Lesson00Node {
    /// Public: accessible for logging in main()
    pub node: Node,
}

/// Implementation block for the Lesson00Node struct.
impl Lesson00Node {
    /// Constructor: Sets up the node and its resources.
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create the node instance within the executor's context
        let node = executor.create_node(NODE_NAME)?;

        // Log startup to console (stdout)
        log_info!(node.logger(), "Lesson 00 bootstrap node started...");

        Ok(Self { node })
    }
}

/// Main function.
fn main() -> Result<(), RclrsError> {
    // 1. Initialize the ROS 2 context (middleware communication)
    let context = Context::default_from_env()?;

    // 2. Create the executor (handles the event loop)
    let mut executor = context.create_basic_executor();

    // 3. Create our Node Container
    let node = Lesson00Node::new(&executor)?;

    // 4. Spin Once: Run the loop one time to check for immediate work, then exit.
    // This confirms the node can bootstrap successfully without hanging.
    executor
        .spin(SpinOptions::spin_once().timeout(Duration::from_millis(0)))
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}