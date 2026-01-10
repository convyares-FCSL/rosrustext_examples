use std::sync::{Arc, Mutex};

use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, Executor, Node, RclrsError,
    RclrsErrorFilter, SpinOptions, Logger,
};

use lesson_interfaces::srv::ComputeStats;
use utils_rclrs::{qos, service};

const NODE_NAME: &str = "lesson_04_service_node";

struct StreamState {
    initialized: bool,
    expected: u64,
    reset_max_value: u64,
}

// Subscriber component: owns the Subscription + stream-validation state.
struct ServiceListener {
    _sub: rclrs::Service<ComputeStats>,
    logger: Logger,
}

impl ServiceListener {
    fn new(node: &Node, reset_max_value: u64) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        let service_name = service::compute_stats(node);

        let sub = node.create_service::<ComputeStats, _>(service_name, move |request, response| {
            Self::handle_request(request, response);
        })?;

        Ok(Self {_sub: sub, logger})
    }


    fn handle_request(request: rclrs::Request<ComputeStats>, response: rclrs::Response<ComputeStats>) {
        log_info!(self.logger, "Incoming request with {} samples.", request.data.len());

        let data = request.data.clone();

        let result = stats_logic::Logic::compute(data);

        response.sum = result.sum;
        response.average = result.average;
        response.status = result.status;

        if (result.status != "Success") {
            log_warn!(self.logger, "Logic Warning: {}", result.status);
        } else {
            log_info!(self.logger, "Computation Complete: Sum={:.2}, Avg={:.2}", result.sum, result.average);
        }
    }
}

struct Lesson04ServiceNode {
    pub node: Node,
    _service_component: Arc<ServiceListener>,
}

impl Lesson04ServiceNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        let sub = Arc::new(ServiceListener::new(&node, 1)?);

        log_info!(node.logger(), "Lesson 04 service node started (server). Ctrl+C to exit.");

        Ok(Self {node, _service_component: sub})
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson04ServiceNode::new(&executor)?;

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