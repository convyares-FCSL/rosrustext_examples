use std::sync::{Arc, Mutex};
use roslibrust::rosbridge::{ClientHandle, Subscriber};
use rosrustext_roslibrust::lifecycle::{LifecycleCallbacks, LifecycleNode, CallbackResult};
use rosrustext_roslibrust::transport::roslibrust::register_lifecycle_backend_rosbridge;
use lesson_06_lifecycle_roslibrust::{MsgCount, TelemetryStreamValidator, StreamEvent};
use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::topics;
use tokio::sync::mpsc;

const NODE_NAME: &str = "lesson_06_lifecycle_subscriber";
const VALIDATOR_RESET_LIMIT_PARAM: &str = "validator_reset_limit";
const DEFAULT_RESET_LIMIT: i64 = 5;

// --- Architecture Definitions ---

#[derive(Debug)]
enum LifecycleCommand {
    Configure { topic: String, reset_limit: i64 },
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

struct SubscriberNode {
    cmd_tx: mpsc::Sender<LifecycleCommand>,
    topic_name: String,
    initial_reset_limit: i64,
}

impl SubscriberNode {
    fn send_command(&self, cmd: LifecycleCommand) -> CallbackResult {
        // Non-blocking send.
        match self.cmd_tx.try_send(cmd) {
            Ok(_) => CallbackResult::Success,
            Err(e) => {
                log::error!("Failed to enqueue lifecycle command: {}", e);
                CallbackResult::Failure
            }
        }
    }
}

impl LifecycleCallbacks for SubscriberNode {
    fn on_configure(&mut self) -> CallbackResult {
        log::info!("on_configure: Queuing Configure command...");
        self.send_command(LifecycleCommand::Configure { 
            topic: self.topic_name.clone(), 
            reset_limit: self.initial_reset_limit 
        })
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
    let initial_reset_limit = config.get_or(VALIDATOR_RESET_LIMIT_PARAM, DEFAULT_RESET_LIMIT);

    log::info!("Starting {} with topic={}, reset_limit={}", NODE_NAME, topic_name, initial_reset_limit);

    // Connection
    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected to rosbridge");
    log::info!("Node started. Current state: Unconfigured");

    // Channels
    let (cmd_tx, mut cmd_rx) = mpsc::channel(32);

    // Sync Node Implementation
    let node_impl = SubscriberNode {
        cmd_tx: cmd_tx.clone(),
        topic_name: topic_name.clone(),
        initial_reset_limit,
    };

    // Lifecycle Shim
    let lifecycle_node = Arc::new(Mutex::new(LifecycleNode::new(NODE_NAME, Box::new(node_impl))?));
    register_lifecycle_backend_rosbridge(&client, NODE_NAME, Arc::clone(&lifecycle_node)).await?;
    // Note: Subscriber gating is implicitly handled by creation/destruction of the subscription.
    
    // Async Engine Task
    let engine_handle = tokio::spawn(async move {
        // Resources
        let mut subscription: Option<Subscriber<MsgCount>> = None;
        let mut validator = TelemetryStreamValidator::new(initial_reset_limit);
        
        // State
        // State
        let mut target_topic = String::new();
        let mut stored_reset_limit = initial_reset_limit;

        log::info!("Async Engine started.");

        loop {
            if let Some(sub) = &mut subscription {
                tokio::select! {
                    Some(cmd) = cmd_rx.recv() => {
                         let keep_going = handle_sub_command(
                            cmd, 
                            &client, 
                            &mut subscription, 
                            &mut validator, 
                            &mut target_topic,
                            &mut stored_reset_limit
                        ).await;
                        if !keep_going { break; }
                    }
                    // subscriber::next() returns MsgCount directly (inherent method).
                    // It does not implement Stream, so we cannot check for None.
                    msg = sub.next() => {
                        let decision = validator.on_count(msg.count);
                        match decision.event {
                            StreamEvent::Valid => log::info!("[Valid] {}", decision.message),
                            StreamEvent::OutOfOrder => log::warn!("[OutOfOrder] {}", decision.message),
                            StreamEvent::Reset => log::info!("[Reset] {}", decision.message),
                        }
                    }
                    else => { break; } 
                }
            } else {
                match cmd_rx.recv().await {
                    Some(cmd) => {
                         let keep_going = handle_sub_command(
                            cmd, 
                            &client, 
                            &mut subscription, 
                            &mut validator, 
                            &mut target_topic,
                            &mut stored_reset_limit
                        ).await;
                        if !keep_going { break; }
                    }
                    None => { break; }
                }
            }
        }
        log::info!("Async Engine stopped.");
    });

    // Deterministic Shutdown
    match tokio::signal::ctrl_c().await {
        Ok(()) => {
             log::info!("Ctrl-C received. Initiating shutdown...");
             let _ = cmd_tx.send(LifecycleCommand::Shutdown).await;
        }
        Err(e) => log::error!("Signal error: {}", e),
    }

    engine_handle.await?;
    log::info!("Node exited cleanly.");
    Ok(())
}

/// Returns true if engine should continue, false if it should shut down.
async fn handle_sub_command(
    cmd: LifecycleCommand,
    client: &ClientHandle,
    subscription: &mut Option<Subscriber<MsgCount>>,
    validator: &mut TelemetryStreamValidator,
    target_topic: &mut String,
    configured_reset_limit: &mut i64,
) -> bool {
    match cmd {
        LifecycleCommand::Configure { topic, reset_limit } => {
            log::info!("Engine: Configuring with topic={}, limit={}", topic, reset_limit);
            *target_topic = topic;
            *configured_reset_limit = reset_limit;
            // Initialize validator with the configured limit
            validator.set_reset_max_value(reset_limit);
        }
        LifecycleCommand::Activate => {
             log::info!("Engine: Activating...");
             if !target_topic.is_empty() {
                 match client.subscribe::<MsgCount>(target_topic).await {
                     Ok(sub) => {
                         *subscription = Some(sub);
                         log::info!("Engine: Subscribed to {}", target_topic);
                     }
                     Err(e) => log::error!("Engine: Failed to subscribe: {}", e),
                 }
             } else {
                 log::error!("Engine: Activate called but no topic configured.");
             }
        }
        LifecycleCommand::Deactivate => {
            log::info!("Engine: Deactivating...");
            *subscription = None; // Drops subscription, stops data flow
        }
        LifecycleCommand::Cleanup => {
            log::info!("Engine: Cleaning up...");
            *subscription = None;
            // Reset validator to the configured limit to be ready for next cycle
            validator.set_reset_max_value(*configured_reset_limit);
            // Note: We keep target_topic and configured_reset_limit in engine state 
            // even though we are "Unconfigured", effectively caching them. 
            // A real Configure transition would overwrite them anyway.
        }
        LifecycleCommand::Shutdown => {
            log::info!("Engine: Shutting down...");
            *subscription = None;
            return false;
        }
    }
    true
}
