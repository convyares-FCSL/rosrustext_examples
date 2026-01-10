use std::sync::Arc;
use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, Executor, Node, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

// Import structs
use lesson_interfaces::srv::{ComputeStats, ComputeStats_Request, ComputeStats_Response};
use utils_rclrs::services;
use lesson_04_service_rclrs::compute;

const NODE_NAME: &str = "lesson_04_service_server";

struct ServiceComponent {
    // We use Arc<rclrs::Service<T>> to match the actual return type of create_service
    _server: Arc<rclrs::Service<ComputeStats>>,
}

impl ServiceComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();
        let service_name = services::compute_stats(node);

        let server = node.create_service::<ComputeStats, _>(
            &service_name,
            move |header, request: ComputeStats_Request| {
                // The header (rmw_request_id_t) is now explicitly handled by name
                let _ = header; 
                let data = &request.data;
                
                log_info!(&logger, "Incoming request with {} samples.", data.len());

                let result = compute(data);

                if result.status != "Success" {
                    log_warn!(&logger, "Logic Warning: {}", result.status);
                } else {
                    log_info!(&logger, "Computation Complete: Sum={:.2}, Avg={:.2}", 
                        result.sum, result.average);
                }

                ComputeStats_Response {
                    sum: result.sum,
                    average: result.average,
                    status: result.status,
                }
            },
        )?;

        Ok(Self { _server: server })
    }
}

struct Lesson04Node {
    pub node: Node,
    _service_component: ServiceComponent,
}

impl Lesson04Node {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let service_component = ServiceComponent::new(&node)?;

        log_info!(node.logger(), "Lesson 04 service node started (server). Ctrl+C to exit.");

        Ok(Self {
            node,
            _service_component: service_component,
        })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node_wrapper = Lesson04Node::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!(node_wrapper.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}