use std::time::Duration;

use roslibrust::rosbridge::ClientHandle;
use roslibrust::{RosMessageType, RosServiceType};
use serde::{Deserialize, Serialize};

use utils_roslibrust::services;

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_04_service_client";

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
/// Client Component
/// ------------------------------

struct ClientComponent;

impl ClientComponent {
    async fn call_with_retry(
        client: &ClientHandle,
    ) -> Result<ComputeStatsResponse, Box<dyn std::error::Error>> {
        let service_name = services::compute_stats();

        let request = ComputeStatsRequest {
            data: vec![10.5, 20.2, 30.7],
        };

        loop {
            log::info!(
                "[{}] Calling {} with {} samples...",
                NODE_NAME,
                service_name,
                request.data.len()
            );

            match client
                .call_service::<ComputeStats>(service_name, request.clone())
                .await
            {
                Ok(response) => return Ok(response),
                Err(e) => {
                    log::warn!(
                        "[{}] Service not ready yet: {}. Retrying...",
                        NODE_NAME,
                        e
                    );
                    tokio::time::sleep(Duration::from_millis(500)).await;
                }
            }
        }
    }
}

/// ------------------------------
/// Node Wrapper
/// ------------------------------

struct Lesson04ClientNode {
    _client: ClientHandle,
}

impl Lesson04ClientNode {
    async fn new(client: ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        log::info!("[{}] Started (rosbridge).", NODE_NAME);
        log::info!("[{}] Service name: {}", NODE_NAME, services::compute_stats());
        log::info!(
            "[{}] Service type: {}",
            NODE_NAME,
            ComputeStats::ROS_SERVICE_NAME
        );

        Ok(Self { _client: client })
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

    let node = Lesson04ClientNode::new(client).await?;

    let response = ClientComponent::call_with_retry(&node._client).await?;

    log::info!(
        "[{}] Response received -> Sum: {:.2}, Avg: {:.2}, Status: '{}'",
        NODE_NAME,
        response.sum,
        response.average,
        response.status
    );

    Ok(())
}
