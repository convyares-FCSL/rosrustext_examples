use std::time::Duration;
use std::sync::{Arc, Mutex};
use roslibrust::rosbridge::{ClientHandle, Publisher};
use rosrustext_roslibrust::lifecycle::{LifecycleCallbacks, LifecycleNode, CallbackResult};
use rosrustext_roslibrust::transport::roslibrust::register_lifecycle_backend_rosbridge;
use lesson_06_lifecycle_roslibrust::{MsgCount, TelemetryPublisherCore, period_s_to_ms_strict};
use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::topics;
use tokio::sync::mpsc;

const NODE_NAME: &str = "lesson_06_lifecycle_publisher";
const PARAM_PERIOD: &str = "timer_period_s";
const DEFAULT_PERIOD_S: f64 = 1.0;

// --- Architecture Definitions ---

/// Commands sent from Sync Callbacks to Async Engine.
/// Note: No response channel needed as callbacks are non-blocking fire-and-forget.
#[derive(Debug)]
enum LifecycleCommand {
    Configure { topic: String },
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

/// The Sync Shim that gives LifecycleNode a synchronous interface
struct PublisherNode {
    cmd_tx: mpsc::Sender<LifecycleCommand>,
    topic_name: String,
}

impl PublisherNode {
    fn send_command(&self, cmd: LifecycleCommand) -> CallbackResult {
        // Non-blocking send. If queue is full or closed, we return Failure.
        // This prevents deadlocks between the callback thread and the async engine.
        match self.cmd_tx.try_send(cmd) {
            Ok(_) => CallbackResult::Success,
            Err(e) => {
                log::error!("Failed to enqueue lifecycle command: {}", e);
                CallbackResult::Failure
            }
        }
    }
}

impl LifecycleCallbacks for PublisherNode {
    fn on_configure(&mut self) -> CallbackResult {
        log::info!("on_configure: Queuing Configure command...");
        let topic = self.topic_name.clone();
        self.send_command(LifecycleCommand::Configure { topic })
    }

    fn on_activate(&mut self) -> CallbackResult {
        log::info!("on_activate: Queuing Activate command...");
        self.send_command(LifecycleCommand::Activate)
    }

    fn on_deactivate(&mut self) -> CallbackResult {
        log::info!("on_deactivate: Queuing Deactivate command...");
        self.send_command(LifecycleCommand::Deactivate)
    }

    fn on_cleanup(&mut self) -> CallbackResult {
        log::info!("on_cleanup: Queuing Cleanup command...");
        self.send_command(LifecycleCommand::Cleanup)
    }

    fn on_shutdown(&mut self) -> CallbackResult {
        log::info!("on_shutdown: Queuing Shutdown command...");
        self.send_command(LifecycleCommand::Shutdown)
    }

    fn on_error(&mut self) -> CallbackResult {
        log::error!("Lifecycle error occurred!");
        CallbackResult::Failure
    }
}

// --- Main & Async Engine ---

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    // Configuration
    let param_files = parse_params_files_from_args();
    let config = if !param_files.is_empty() {
        Config::from_files(&param_files).map_err(|e| format!("{:?}", e))?
    } else {
        Config::from_files(&[] as &[&str]).map_err(|e| format!("{:?}", e))?
    };

    let topic_name = topics::telemetry(&config);
    let initial_period_s = config.get_or(PARAM_PERIOD, DEFAULT_PERIOD_S);
    let period_ms_val = period_s_to_ms_strict(initial_period_s).unwrap_or(1000);

    log::info!("Starting {} with period={}s", NODE_NAME, initial_period_s);

    // Connection
    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected to rosbridge");
    log::info!("Node started. Current state: Unconfigured");

    // Channels
    let (cmd_tx, mut cmd_rx) = mpsc::channel(32); // Reasonable buffer size

    // Sync Node Implementation
    let node_impl = PublisherNode {
        cmd_tx: cmd_tx.clone(),
        topic_name: topic_name.clone(),
    };

    // Lifecycle Shim
    let lifecycle_node = Arc::new(Mutex::new(LifecycleNode::new(NODE_NAME, Box::new(node_impl))?));
    register_lifecycle_backend_rosbridge(&client, NODE_NAME, Arc::clone(&lifecycle_node)).await?;
    let activation_gate = lifecycle_node.lock().unwrap().activation_gate();

    // Async Engine Task
    let engine_handle = tokio::spawn(async move {
        // Resources
        let mut publisher: Option<Publisher<MsgCount>> = None;
        let mut core = TelemetryPublisherCore::new();
        let mut interval = tokio::time::interval(Duration::from_millis(period_ms_val));

        log::info!("Async Engine started.");

        loop {
            tokio::select! {
                // A. Handle Lifecycle Commands
                Some(cmd) = cmd_rx.recv() => {
                    match cmd {
                        LifecycleCommand::Configure { topic } => {
                            log::info!("Engine: Configuring...");
                            match client.advertise::<MsgCount>(&topic).await {
                                Ok(handle) => {
                                    publisher = Some(handle);
                                    log::info!("Engine: Publisher created.");
                                }
                                Err(e) => log::error!("Engine: Failed to advertise: {}", e),
                            }
                        }
                        LifecycleCommand::Activate => {
                            log::info!("Engine: Activating...");
                            // Resources already created. Gate is managed by LifecycleNode logic externally.
                        }
                        LifecycleCommand::Deactivate => {
                            log::info!("Engine: Deactivating...");
                            // Gate managed externally.
                        }
                        LifecycleCommand::Cleanup => {
                            log::info!("Engine: Cleaning up...");
                            publisher = None;
                            // Reset core state (sequence number) according to lifecycle semantics
                            core = TelemetryPublisherCore::new();
                        }
                        LifecycleCommand::Shutdown => {
                            log::info!("Engine: Shutting down...");
                            break; // Break the loop to exit task
                        }
                    }
                }
                
                // B. Handle Tick
                _ = interval.tick() => {
                    if activation_gate.is_active() {
                        if let Some(pub_handle) = &publisher {
                            let msg = core.next_message();
                            if let Err(e) = pub_handle.publish(&msg).await {
                                log::warn!("Failed to publish: {}", e);
                            }
                        }
                    }
                }

                else => { break; } // Channel closed
            }
        }
        log::info!("Async Engine stopped.");
    });

    // Shutdown Handler
    match tokio::signal::ctrl_c().await {
        Ok(()) => {
            log::info!("Ctrl-C received. Initiating shutdown...");
            // Send explicit Shutdown command to engine to ensure resources are dropped correctly before exit
            let _ = cmd_tx.send(LifecycleCommand::Shutdown).await;
        }
        Err(e) => log::error!("Signal error: {}", e),
    }

    // Wait for Engine
    // We expect the engine to exit now.
    let _ = engine_handle.await;
    log::info!("Node exited cleanly.");

    Ok(())
}


