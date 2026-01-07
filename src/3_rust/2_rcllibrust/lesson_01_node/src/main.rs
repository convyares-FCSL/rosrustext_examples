use roslibrust::rosbridge::ClientHandle;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_01_node";

struct Lesson01Node {
    /// Client handle (kept alive for communication).
    _client_handle: ClientHandle,
    /// Timer handle (must be kept alive; dropping it aborts the task).
    _timer_handle: tokio::task::JoinHandle<()>,
}

impl Lesson01Node {
    async fn new(client: ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        // For this lesson, we will default to 1.0s to keep it simple.
        let period_s = 1.0;
        log::info!("Using fixed timer period (parameters WIP): timer_period_s = {}", period_s);

        // Tick counter: shared, mutable state for callbacks.
        let tick = Arc::new(AtomicU64::new(0));

        // Create timer from the parameter value.
        let timer = create_timer_from_param(period_s, tick);

        log::info!("Lesson 01 node started (simulated)...");

        Ok(Self {_client_handle: client,_timer_handle: timer })
    }
}

/// Create a repeating timer using the provided period.
fn create_timer_from_param(timer_period_s: f64, tick: Arc<AtomicU64>) -> tokio::task::JoinHandle<()> {
    let mut period_s = timer_period_s;

    if period_s <= 0.0 {
        period_s = 1.0;
        log::warn!("Invalid timer period specified. Defaulting to {} seconds.", period_s);
    }

    // Capture the tick counter for the async task
    let tick_cb = tick.clone();

    // Spawn a Tokio task to act as the timer
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs_f64(period_s));
        loop {
            // Wait for the next tick
            interval.tick().await;
            on_tick(&tick_cb);
        }
    })
}

/// Timer callback helper.
fn on_tick(tick: &Arc<AtomicU64>) {
    let n = tick.fetch_add(1, Ordering::Relaxed) + 1;
    log::info!("tick {}", n);
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::init();

    // Connect to rosbridge
    let client = ClientHandle::new(ROSBRIDGE_URL).await?;
    log::info!("Client connected to {}", ROSBRIDGE_URL);

    // Initialize the Node
    let _node = Lesson01Node::new(client).await?;

    // Keep the program running until Ctrl+C
    match tokio::signal::ctrl_c().await {
        Ok(()) => {
            log::info!("Shutting down {}...", NODE_NAME);
        }
        Err(err) => {
            log::error!("Unable to listen for shutdown signal: {}", err);
        }
    }

    Ok(())
}