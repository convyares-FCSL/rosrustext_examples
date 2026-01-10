use roslibrust::rosbridge::ClientHandle;
use roslibrust::{RosMessageType, RosServiceType};
use serde::{Deserialize, Serialize};

use lesson_04_service::compute;
use utils_roslibrust::services;

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_04_service_server";

/// ------------------------------
/// Service Type Definitions
/// ------------------------------

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ComputeStatsRequest {
    pub data: Vec<f64>,
}

impl RosMessageType for ComputeStatsRequest {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/srv/ComputeStats_Request";
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ComputeStatsResponse {
    pub sum: f64,
    pub average: f64,
    pub status: String,
}

impl RosMessageType for ComputeStatsResponse {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/srv/ComputeStats_Response";
}

pub struct ComputeStats;

impl RosServiceType for ComputeStats {
    type Request = ComputeStatsRequest;
    type Response = ComputeStatsResponse;

    const ROS_SERVICE_NAME: &'static str = "lesson_interfaces/srv/ComputeStats";
}

/// ------------------------------
/// Service Component (Adapter)
/// ------------------------------

struct ServiceComponent {
    _handle: roslibrust::rosbridge::ServiceHandle,
}

impl ServiceComponent {
    async fn new(client: &ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        let service_name = services::compute_stats();

        let handle = client
            .advertise_service::<ComputeStats, _>(service_name, move |req: ComputeStatsRequest| {
                log::info!(
                    "[{}] Incoming request on {} with {} samples.",
                    NODE_NAME,
                    service_name,
                    req.data.len()
                );

                let result = compute(&req.data);

                if result.status != "Success" {
                    log::warn!("[{}] Logic warning: {}", NODE_NAME, result.status);
                } else {
                    log::info!(
                        "[{}] Computation complete: Sum={:.2}, Avg={:.2}",
                        NODE_NAME,
                        result.sum,
                        result.average
                    );
                }

                Ok(ComputeStatsResponse {
                    sum: result.sum,
                    average: result.average,
                    status: result.status,
                })
            })
            .await?;

        Ok(Self { _handle: handle })
    }
}

/// ------------------------------
/// Node Wrapper
/// ------------------------------

struct Lesson04ServerNode {
    _client: ClientHandle,
    _service: ServiceComponent,
}

impl Lesson04ServerNode {
    async fn new(client: ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        let service = ServiceComponent::new(&client).await?;

        log::info!("[{}] Started (rosbridge). Ctrl+C to exit.", NODE_NAME);
        log::info!("[{}] Service name: {}", NODE_NAME, services::compute_stats());
        log::info!(
            "[{}] Service type: {}",
            NODE_NAME,
            ComputeStats::ROS_SERVICE_NAME
        );

        Ok(Self {
            _client: client,
            _service: service,
        })
    }
}

/// ------------------------------
/// Entry Point
/// ------------------------------

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let client = ClientHandle::new(ROSBRIDGE_URL).await?;
    log::info!("[{}] Connected to {}", NODE_NAME, ROSBRIDGE_URL);

    let _node = Lesson04ServerNode::new(client).await?;

    match tokio::signal::ctrl_c().await {
        Ok(()) => log::info!("[{}] Shutting down...", NODE_NAME),
        Err(e) => log::error!("[{}] Signal error: {}", NODE_NAME, e),
    }

    Ok(())
}
