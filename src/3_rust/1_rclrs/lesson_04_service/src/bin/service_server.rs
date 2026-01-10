use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, Node, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

use lesson_interfaces::srv::{ComputeStats, ComputeStats_Request, ComputeStats_Response};
use lesson_04_service_rclrs::compute;
use utils_rclrs::services;

const NODE_NAME: &str = "lesson_04_service_server";

struct ServiceComponent {
    _server: rclrs::Service<ComputeStats>,
    _logger: Logger,
}

impl ServiceComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();
        let service_name = services::compute_stats(node);

        // IMPORTANT: use the 1-arg callback form (request only).
        // This matches the IntoNodeServiceCallback impl in your rclrs checkout.
        let server = node.create_service::<ComputeStats, _>(&service_name, {
            let logger = logger.clone();
            move |request: ComputeStats_Request| -> ComputeStats_Response {
                Self::handle_request(&logger, request)
            }
        })?;

        Ok(Self {
            _server: server,
            _logger: logger,
        })
    }

    fn handle_request(logger: &Logger, request: ComputeStats_Request) -> ComputeStats_Response {
        log_info!(logger, "Incoming request with {} samples.", request.data.len());

        let result = compute(&request.data);

        if result.status != "Success" {
            log_warn!(logger, "Logic warning: {}", result.status);
        } else {
            log_info!(
                logger,
                "Computation complete: Sum={:.2}, Avg={:.2}",
                result.sum,
                result.average
            );
        }

        ComputeStats_Response {
            sum: result.sum,
            average: result.average,
            status: result.status,
        }
    }
}

struct Lesson04Node {
    pub node: Node,
    _service: ServiceComponent,
}

impl Lesson04Node {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let service = ServiceComponent::new(&node)?;

        log_info!(node.logger(), "Lesson 04 service server started. Ctrl+C to exit.");

        Ok(Self {
            node,
            _service: service,
        })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson04Node::new(&executor)?;

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
