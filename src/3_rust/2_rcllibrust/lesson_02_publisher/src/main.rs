use roslibrust::rosbridge::{ClientHandle, Publisher};
use roslibrust::RosMessageType;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

// Local modules to mirror the structure of utils_rust
use utils_roslibrust::{qos, topics};

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_02_node";

/// --- Message Definition ---
/// Since we are in "Cargo-only" mode and cannot easily read the compiled
/// C++ interfaces, we define the struct manually to match the JSON schema.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MsgCount {
    pub count: i64,
}

/// We must implement this trait so roslibrust knows the ROS type name.
impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
}

/// --- Logic Component ---
/// Encapsulates the business logic of publishing.
struct PublisherChatter {
    publisher: Publisher<MsgCount>,
    count: AtomicU64,
}

impl PublisherChatter {
    /// Pure logic: Increment and Publish
    async fn on_tick(&self) {
        // 1. Logic
        let n = self.count.fetch_add(1, Ordering::Relaxed) + 1;
        let msg = MsgCount { count: n as i64 };

        // 2. Side Effect (Publish)
        // We 'await' because network operations are async.
        // We only log if something goes WRONG. Success is silent.
        if let Err(e) = self.publisher.publish(&msg).await {
            log::error!("Failed to publish: {}", e);
        }
    }
}

/// --- Resource Container ---
/// Manages the lifecycle of the node and its components.
struct Lesson02Node {
    /// Keep connection alive
    _client: ClientHandle,
    /// Keep timer task alive
    _timer: tokio::task::JoinHandle<()>,
    /// Keep publisher logic alive (shared with timer)
    _publisher_component: Arc<PublisherChatter>,
}

impl Lesson02Node {
    async fn new(client: ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        // 1. Setup Parameters (Defaults for now)
        let period_s = 1.0;
        
        // 2. Build Components
        // We need an Arc because the timer task will need to own a reference to it.
        let publisher_component = Arc::new(Self::build_chatter_publisher(&client).await?);

        // 3. Build Timer
        let timer = Self::build_timer(period_s, Arc::clone(&publisher_component));

        log::info!("Lesson 02 node started...");

        Ok(Self {
            _client: client,
            _timer: timer,
            _publisher_component: publisher_component,
        })
    }

    async fn build_chatter_publisher(client: &ClientHandle) -> Result<PublisherChatter, Box<dyn std::error::Error>> {
        // Use our shared helper modules
        let topic = topics::CHATTER;
        let _options = qos::get_options("default"); // Not fully used in roslibrust 0.6 advertise yet, but good for structure

        // Advertise the topic
        let publisher = client.advertise::<MsgCount>(topic).await?;
        
        Ok(PublisherChatter {
            publisher,
            count: AtomicU64::new(0),
        })
    }

    fn build_timer(period_s: f64, chatter: Arc<PublisherChatter>) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f64(period_s));
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

    // 1. Connect
    let client = ClientHandle::new(ROSBRIDGE_URL).await?;
    log::info!("Connected to {}", ROSBRIDGE_URL);

    // 2. Initialize Node Container
    let _node = Lesson02Node::new(client).await?;

    // 3. Park Main Thread
    match tokio::signal::ctrl_c().await {
        Ok(()) => log::info!("Shutting down {}...", NODE_NAME),
        Err(e) => log::error!("Signal error: {}", e),
    }

    Ok(())
}