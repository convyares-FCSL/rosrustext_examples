use std::sync::{Arc, Mutex};
use rclrs::{Context, Executor, Node, RclrsError, SpinOptions};

// Helper modules for shared configuration
// mod qos;
// mod topics;
// mod services;

// << FILL IN HERE >>: Define the default node name
const NODE_NAME: &str = "template_node";

/// Configuration: Holds values loaded from ROS parameters
struct NodeParams {
    // << FILL IN HERE >>: Add parameter fields
    // pub timer_period_s: f64,
}

/// State: Holds the mutable business logic variables
struct NodeState {
    // << FILL IN HERE >>: Add state variables
    // pub count: u64,
}

impl NodeState {
    fn new() -> Self {
        Self {
            // << FILL IN HERE >>: Initialize defaults
            // count: 0,
        }
    }
}

/// Container: Holds the Node and RAII handles to keep them alive
struct MyNode {
    pub node: Node,
    // << FILL IN HERE >>: Add RAII handles (Timers, Subscriptions)
    // _timer: Arc<rclrs::Timer>,
}

impl MyNode {
    /// Constructor: Wires up parameters, state, and callbacks
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        // 1. Setup Parameters
        let params = Self::declare_parameters(&node)?;

        // 2. Setup State
        // Wrapped in Arc<Mutex<>> for thread-safe access in callbacks
        let state = Arc::new(Mutex::new(NodeState::new()));

        // << FILL IN HERE >>: Create Publishers/Subscribers
        // let publisher = node.create_publisher::<MsgType>("topic", QOS)?;

        // << FILL IN HERE >>: Create Timers (injecting params and state)
        // let _timer = Self::create_timer(&node, &params, &state)?;

        rclrs::log_info!(node.logger(), "Node '{}' initialized.", NODE_NAME);

        Ok(Self {
            node,
            // << FILL IN HERE >>: Store handles
            // _timer,
        })
    }

    /// Helper: Centralizes all parameter declarations and validation
    fn declare_parameters(node: &Node) -> Result<NodeParams, RclrsError> {
        // << FILL IN HERE >>: Declare and read parameters
        // let period = node.declare_parameter("period_s").default(1.0).mandatory()?;

        Ok(NodeParams {
            // timer_period_s: period.get(),
        })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    
    // Bootstrap the node container
    let node = MyNode::new(&executor)?;

    // Run the event loop
    executor
        .spin(SpinOptions::default())
        .ignore_non_errors() 
        .first_error()       
        .map_err(|err| {
            rclrs::log_error!(node.node.logger(), "Fatal error during spin: {err}");
            err
        })?;

    Ok(())
}