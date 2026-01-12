use roslibrust::rosbridge::{ClientHandle, Publisher};
use roslibrust::RosMessageType;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::{qos, topics};

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_05_publisher";
const DEFAULT_PERIOD_MS: u64 = 1000;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MsgCount {
    pub count: i64,
}

impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
}

struct Parameters {
    topic_name: String,
    publish_period_ms: u64,
}

impl Parameters {
    fn from_config(cfg: &Config) -> Self {
        let topic_name = topics::telemetry(cfg);
        let publish_period_ms = cfg.get_or("publisher.period_ms", DEFAULT_PERIOD_MS);
        log::info!("Configuration: topic='{}', period={}ms", topic_name, publish_period_ms);
        Self { topic_name, publish_period_ms }
    }
}

struct PublisherChatter {
    publisher: Publisher<MsgCount>,
    count: AtomicU64,
}

impl PublisherChatter {
    async fn on_tick(&self) {
        let n = self.count.fetch_add(1, Ordering::Relaxed) + 1;
        let msg = MsgCount { count: n as i64 };
        if let Err(e) = self.publisher.publish(&msg).await {
            log::error!("Failed to publish: {}", e);
        }
    }
}

struct Lesson05Node {
    _client: ClientHandle,
    _timer: tokio::task::JoinHandle<()>,
    _publisher_component: Arc<PublisherChatter>,
}

impl Lesson05Node {
    async fn new(client: ClientHandle, cfg: &Config) -> Result<Self, Box<dyn std::error::Error>> {
        let params = Parameters::from_config(cfg);
        let publisher_component = Arc::new(Self::build_chatter_publisher(&client, &params.topic_name).await?);
        let timer = Self::build_timer(params.publish_period_ms, Arc::clone(&publisher_component));

        log::info!("Lesson 05 node started (publisher)...");

        Ok(Self { _client: client, _timer: timer, _publisher_component: publisher_component })
    }

    async fn build_chatter_publisher(client: &ClientHandle, topic: &str) -> Result<PublisherChatter, Box<dyn std::error::Error>> {
        let _options = qos::get_options("default"); 
        let publisher = client.advertise::<MsgCount>(topic).await?;
        Ok(PublisherChatter { publisher, count: AtomicU64::new(0) })
    }

    fn build_timer(period_ms: u64, chatter: Arc<PublisherChatter>) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(period_ms));
            loop {
                interval.tick().await;
                chatter.on_tick().await;
            }
        })
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let param_files = parse_params_files_from_args();
    if param_files.is_empty() {
        log::error!("No parameter files provided! usage: --params-file <path>");
    }

    let config = Config::from_files(&param_files).expect("Failed to load configuration files");
    log::info!("Loaded configuration from {} files.", param_files.len());

    let client = ClientHandle::new(ROSBRIDGE_URL).await?;
    log::info!("Connected to {}", ROSBRIDGE_URL);

    let _node = Lesson05Node::new(client, &config).await?;

    match tokio::signal::ctrl_c().await {
        Ok(()) => log::info!("Shutting down {}...", NODE_NAME),
        Err(e) => log::error!("Signal error: {}", e),
    }

    Ok(())
}