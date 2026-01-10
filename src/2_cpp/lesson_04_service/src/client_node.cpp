use std::time::Duration;
use rclrs::{
    log_info, log_error, Context, CreateBasicExecutor, Executor, Node, 
    RclrsError, RclrsErrorFilter, SpinOptions
};
use lesson_interfaces::srv::{ComputeStats, ComputeStats_Request};
use utils_rclrs::services;

const NODE_NAME: &str = "lesson_04_service_client";

struct ClientComponent {
    client: rclrs::Client<ComputeStats>,
    logger: rclrs::Logger,
}

impl ClientComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();
        let service_name = services::compute_stats(node);
        let client = node.create_client::<ComputeStats>(&service_name)?;

        Ok(Self { client, logger })
    }

    fn send_sample_request(&self, context: Context) {
        let logger = self.logger.clone();
        let request = ComputeStats_Request {
            data: vec![10.5, 20.2, 30.7],
        };

        log_info!(logger, "Sending request with {} samples...", request.data.len());

        // async_send_request returns a Receiver
        let rx = self.client.async_send_request(&request);

        // Spawn a thread to "wait" for the response without blocking the executor
        // This is the Rust equivalent of the C++ async callback
        std::thread::spawn(move || {
            match rx.recv() {
                Ok(response) => {
                    log_info!(
                        logger,
                        "Result -> Sum: {:.2}, Avg: {:.2}, Status: '{}'",
                        response.sum,
                        response.average,
                        response.status
                    );
                }
                Err(e) => log_error!(logger, "Service call failed: {:?}", e),
            }
            // Shut down after receiving the result (matches C++ logic)
            context.shutdown();
        });
    }
}

struct Lesson04Node {
    pub node: Node,
    pub client_component: ClientComponent,
}

impl Lesson04Node {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let client_component = ClientComponent::new(&node)?;
        Ok(Self { node, client_component })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node_wrapper = Lesson04Node::new(&executor)?;

    // Wait for service (simple block)
    while !node_wrapper.client_component.client.wait_for_service()? {
        log_info!(node_wrapper.node.logger(), "Waiting for service...");
        std::thread::sleep(Duration::from_millis(500));
    }

    // Trigger the request
    node_wrapper.client_component.send_sample_request(context.clone());

    // Spin until context.shutdown() is called in the thread
    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!(node_wrapper.node.logger(), "Executor stopped: {err}");
            err
        })?;

    Ok(())
}