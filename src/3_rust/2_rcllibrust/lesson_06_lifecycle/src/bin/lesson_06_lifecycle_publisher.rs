use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use roslibrust::rosbridge::{ClientHandle, Publisher};
use tokio::sync::Mutex as TokioMutex;
use lesson_06_lifecycle_roslibrust::lifecycle::*;
use lesson_06_lifecycle_roslibrust::*;
use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::topics;

const NODE_NAME: &str = "lesson_06_lifecycle_publisher";
const PARAM_PERIOD: &str = "timer_period_s";
const DEFAULT_PERIOD_S: f64 = 1.0;

/// Shared runtime state
struct RuntimeState {
    period_ms: AtomicU64,
}

/// Resources managed by lifecycle
#[derive(Default)]
struct Resources {
    publisher: Option<Publisher<MsgCount>>,
    // We don't need to store the timer handle if we use a long-running task that checks state
    // But to match "teardown" behavior properly, we might want to kill it. 
    // However, simplest "gating" is a persistent task that checks state.
    // For specific "Cleanup" parity where resources are destroyed, we should drop the publisher.
    // The timer task can remain running but do nothing, or we can use a CancellationToken.
    // For this lesson, we'll use a persistent task for simplicity, but gate on state/resources.
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    
    // 1. Initial Config
    let param_files = parse_params_files_from_args();
    let config = if !param_files.is_empty() {
        Config::from_files(&param_files).map_err(|e| format!("{:?}", e))?
    } else {
        Config::from_files(&[] as &[&str]).map_err(|e| format!("{:?}", e))?
    };     
    
    // Connect
    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected to rosbridge at ws://localhost:9090");

    // Telemetry Topic Name
    let topic_name = topics::telemetry(&config);
    let initial_period_s = config.get_or(PARAM_PERIOD, DEFAULT_PERIOD_S);
    let initial_period_ms = period_s_to_ms_strict(initial_period_s).unwrap_or(1000);

    // Shared State
    let lifecycle_state = Arc::new(Mutex::new(LifecycleState::Unconfigured));
    let resources = Arc::new(TokioMutex::new(Resources::default()));
    let runtime_state = Arc::new(RuntimeState {
        period_ms: AtomicU64::new(initial_period_ms),
    });

    // Core Logic Protected by Mutex
    let core = Arc::new(Mutex::new(TelemetryPublisherCore::new()));

    // 2. Parameter Watcher (Runtime updates)
    let client_clone = client.clone();
    let runtime_state_clone = runtime_state.clone();
    tokio::spawn(async move {
        // Subscribe to parameter events
        // Note: roslibrust doesn't fully support subscribing to arbitrary types easily without strict definitions
        // We use our manual definition from lifecycle.rs
        let mut sub = client_clone.subscribe::<ParameterEvent>("/parameter_events").await.unwrap();
        log::info!("Parameter watcher started on /parameter_events");
        
        loop {
            let event = sub.next().await;
            // Check if it's for our node (roughly match node name or absolute)
            // ros2 param set often targets specific nodes.
            // The `node` field in ParameterEvent is usually fully qualified e.g. /lesson_06_lifecycle_publisher
            if event.node.ends_with(NODE_NAME) {
                 for p in event.changed_parameters.iter().chain(event.new_parameters.iter()) {
                     if p.name == PARAM_PERIOD {
                         if p.value.type_ == 3 { // Double
                             let val = p.value.double_value;
                             match period_s_to_ms_strict(val) {
                                 Ok(ms) => {
                                     runtime_state_clone.period_ms.store(ms, Ordering::Relaxed);
                                     log::info!("Parameter {} updated to {}s ({}ms)", PARAM_PERIOD, val, ms);
                                 },
                                 Err(e) => log::error!("Invalid update for {}: {}", PARAM_PERIOD, e),
                             }
                         } else if p.value.type_ == 2 { // Integer support as fallback
                             let val = p.value.integer_value as f64;
                             match period_s_to_ms_strict(val) {
                                 Ok(ms) => {
                                     runtime_state_clone.period_ms.store(ms, Ordering::Relaxed);
                                     log::info!("Parameter {} updated to {} (int) -> {}ms", PARAM_PERIOD, val, ms);
                                 },
                                 Err(e) => log::error!("Invalid update for {}: {}", PARAM_PERIOD, e),
                             }
                         }
                     }
                 }
            }
        }
    });

    // 3. Timer / Publisher Task
    let resources_clone = resources.clone();
    let lifecycle_state_clone = lifecycle_state.clone();
    let runtime_state_clone_2 = runtime_state.clone();
    let core_clone = core.clone();
    
    tokio::spawn(async move {
        // A loop that sleeps dynamically
        // Note: to handle changing periods immediately, we should ideally use timeout or similar,
        // but simple sleep is okay for "parity" unless strict timing needed.
        loop {
            let ms = runtime_state_clone_2.period_ms.load(Ordering::Relaxed);
            tokio::time::sleep(Duration::from_millis(ms)).await;

            // Check State
            let state = *lifecycle_state_clone.lock().unwrap();
            if state != LifecycleState::Active {
                continue;
            }

            // Publish if resource exists
            let res = resources_clone.lock().await;
            if let Some(pub_handle) = &res.publisher {
                // Generate msg
                let msg = {
                    let mut guard = core_clone.lock().unwrap();
                    guard.next_message()
                };
                
                if let Err(e) = pub_handle.publish(&msg).await {
                    log::error!("Publish failed: {}", e);
                }
            }
        }
    });

    // 4. Lifecycle Services
    
    // GET STATE
    let state_clone_for_get = lifecycle_state.clone();
    // Assuming standard name: /node_name/get_state
    let get_state_topic = format!("{}/get_state", NODE_NAME);
    let _srv_get = client.advertise_service::<GetState, _>(
        &get_state_topic,
        move |_req| {
            let s = *state_clone_for_get.lock().unwrap();
            Ok(GetStateResponse { current_state: s.to_msg() })
        }
    ).await?;
    log::info!("Service advertised: {}", get_state_topic);

    // CHANGE STATE
    let state_clone_for_change = lifecycle_state.clone();
    let resources_clone_for_change = resources.clone();
    let client_clone_for_change = client.clone();
    let change_state_topic = format!("{}/change_state", NODE_NAME);
    let _srv_change = client.advertise_service::<ChangeState, _>(
        &change_state_topic,
        move |req| {
            let transition_id = req.transition.id;
            let mut current_state = state_clone_for_change.lock().unwrap();
            let mut success = false;
            
            match (current_state.clone(), transition_id) {
                 (LifecycleState::Unconfigured, TRANSITION_CONFIGURE) => {
                     let r_clone = resources_clone_for_change.clone();
                     let c_clone = client_clone_for_change.clone();
                     let t_name = topic_name.clone();
                     
                     let res = tokio::task::block_in_place(|| {
                         tokio::runtime::Handle::current().block_on(async move {
                             c_clone.advertise::<MsgCount>(&t_name).await
                         })
                     });
                     
                     match res {
                         Ok(pub_handle) => {
                             let mut r = tokio::task::block_in_place(|| {
                                 tokio::runtime::Handle::current().block_on(async {
                                     r_clone.lock().await
                                 })
                             });
                             r.publisher = Some(pub_handle);
                             *current_state = LifecycleState::Inactive;
                             success = true;
                         }
                         Err(e) => {
                             log::error!("Failed to create publisher: {}", e);
                             success = false;
                         }
                     }
                 },
                 (LifecycleState::Inactive, TRANSITION_ACTIVATE) => {
                     *current_state = LifecycleState::Active;
                     success = true;
                 },
                 (LifecycleState::Active, TRANSITION_DEACTIVATE) => {
                     *current_state = LifecycleState::Inactive;
                     success = true;
                 },
                 (LifecycleState::Inactive, TRANSITION_CLEANUP) => {
                     let mut r = tokio::task::block_in_place(|| {
                         tokio::runtime::Handle::current().block_on(async {
                             resources_clone_for_change.lock().await
                         })
                     });
                     r.publisher = None;
                     *current_state = LifecycleState::Unconfigured;
                     success = true;
                 },
                 (LifecycleState::Unconfigured, TRANSITION_SHUTDOWN) |
                 (LifecycleState::Inactive, TRANSITION_SHUTDOWN) |
                 (LifecycleState::Active, TRANSITION_SHUTDOWN) => {
                      let mut r = tokio::task::block_in_place(|| {
                         tokio::runtime::Handle::current().block_on(async {
                             resources_clone_for_change.lock().await
                         })
                     });
                     r.publisher = None;
                     *current_state = LifecycleState::Finalized;
                     success = true;
                 },
                 _ => {
                     log::warn!("Invalid transition {} from {}", transition_id, current_state.label());
                     success = false;
                 }
            }

            Ok(ChangeStateResponse { success })
        }
    ).await?;
    log::info!("Service advertised: {}", change_state_topic);

    // SET PARAMETERS (Parity with ros2 param set)
    let runtime_state_clone_set = runtime_state.clone();
    let set_param_topic = format!("{}/set_parameters", NODE_NAME);
    let _srv_set_param = client.advertise_service::<SetParameters, _>(
        &set_param_topic,
        move |req| {
             let mut results = Vec::new();
             for p in req.parameters {
                 if p.name == PARAM_PERIOD {
                     let mut success = false;
                     let mut reason = "Invalid Type/Value".to_string();
                     
                     // Try double
                     if p.value.type_ == 3 {
                         let val = p.value.double_value;
                         match period_s_to_ms_strict(val) {
                             Ok(ms) => {
                                 runtime_state_clone_set.period_ms.store(ms, Ordering::Relaxed);
                                 log::info!("SetParameters: Updated {} to {}s", PARAM_PERIOD, val);
                                 success = true;
                                 reason = "".to_string();
                             },
                             Err(e) => reason = e,
                         }
                     } else if p.value.type_ == 2 { // Int
                          let val = p.value.integer_value as f64;
                           match period_s_to_ms_strict(val) {
                             Ok(ms) => {
                                 runtime_state_clone_set.period_ms.store(ms, Ordering::Relaxed);
                                 log::info!("SetParameters: Updated {} to {}s (from int)", PARAM_PERIOD, val);
                                 success = true;
                                 reason = "".to_string();
                             },
                             Err(e) => reason = e,
                         }
                     }
                     
                     results.push(SetParametersResult { successful: success, reason });
                 } else {
                     results.push(SetParametersResult { successful: true, reason: "Ignored unknown param".to_string() });
                 }
             }
             Ok(SetParametersResponse { results })
        }
    ).await?;
    log::info!("Service advertised: {}", set_param_topic);

    log::info!("Node {} started. State: Unconfigured", NODE_NAME);

    // Keep main alive
    tokio::signal::ctrl_c().await?;
    Ok(())
}
