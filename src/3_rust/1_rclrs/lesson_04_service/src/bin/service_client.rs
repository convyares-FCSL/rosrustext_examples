use std::time::Duration;

use rclrs::{
    log_info, log_error, Context, CreateBasicExecutor, Executor, Logger, Node, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

use lesson_interfaces::srv::{ComputeStats, ComputeStats_Request, ComputeStats_Response};
use utils_rclrs::services;

const NODE_NAME: &str = "lesson_04_service_client";

struct ClientComponent {
    client: rclrs::Client<ComputeStats>,
    _logger: Logger,
}

impl ClientComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();
        let service_name = services::compute_stats(node);
        let client = node.create_client::<ComputeStats>(&service_name)?;
        Ok(Self {
            client,
            _logger: logger,
        })
    }

    fn wait_for_service(&self, logger: &Logger) -> Result<(), RclrsError> {
        while !self.client.service_is_ready()? {
            log_info!(logger, "Waiting for service...");
            std::thread::sleep(Duration::from_millis(500));
        }
        Ok(())
    }

    fn send_request(&self, logger: &Logger) -> Result<(), RclrsError> {
        let request = ComputeStats_Request {
            data: vec![10.5, 20.2, 30.7],
        };

        log_info!(logger, "Sending request with {} samples...", request.data.len());

        // Type inference fails in your build: annotate response type explicitly.
        let _promise = self.client.call_then(&request, {
            let logger = logger.clone();
            move |response: ComputeStats_Response| {
                log_info!(
                    &logger,
                    "Result -> Sum: {:.2}, Avg: {:.2}, Status: '{}'",
                    response.sum,
                    response.average,
                    response.status
                );
            }
        })?;

        Ok(())
    }
}

struct Lesson04Node {
    pub node: Node,
    _client: ClientComponent,
}

impl Lesson04Node {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let client = ClientComponent::new(&node)?;

        log_info!(node.logger(), "Lesson 04 service client started.");

        Ok(Self { node, _client: client })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson04Node::new(&executor)?;

    // Wait then send
    node._client.wait_for_service(node.node.logger())?;
    node._client.send_request(node.node.logger())?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}
