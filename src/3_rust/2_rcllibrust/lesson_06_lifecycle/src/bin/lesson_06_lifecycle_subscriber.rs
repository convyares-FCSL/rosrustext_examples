use std::sync::{Arc, Mutex};
use roslibrust::rosbridge::ClientHandle;
use tokio_util::sync::CancellationToken;
use lesson_06_lifecycle_roslibrust::lifecycle::*;
use lesson_06_lifecycle_roslibrust::*;
use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::topics;
use futures::StreamExt; // For .next()

const NODE_NAME: &str = "lesson_06_lifecycle_subscriber";
const VALIDATOR_RESET_LIMIT_PARAM: &str = "validator_reset_limit";
const DEFAULT_RESET_LIMIT: i64 = 5;

struct SubscriberResources {
    // We control the subscription task via a token
    sub_token: Option<CancellationToken>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    
    // Config
    let param_files = parse_params_files_from_args();
    let config = if !param_files.is_empty() {
        Config::from_files(&param_files).map_err(|e| format!("{:?}", e))?
    } else {
        Config::from_files(&[] as &[&str]).map_err(|e| format!("{:?}", e))?
    };     

    let client = ClientHandle::new("ws://localhost:9090").await?;
    log::info!("Connected to rosbridge at ws://localhost:9090");

    let topic_name = topics::telemetry(&config);
    let initial_reset_limit = config.get_or(VALIDATOR_RESET_LIMIT_PARAM, DEFAULT_RESET_LIMIT);

    // State
    let lifecycle_state = Arc::new(Mutex::new(LifecycleState::Unconfigured));
    let resources = Arc::new(Mutex::new(SubscriberResources { sub_token: None }));
    let validator = Arc::new(Mutex::new(TelemetryStreamValidator::new(initial_reset_limit)));

    // Parameter Watcher for 'validator_reset_limit'
    let client_clone_param = client.clone();
    let validator_clone_param = validator.clone();
    tokio::spawn(async move {
        match client_clone_param.subscribe::<ParameterEvent>("/parameter_events").await {
            Ok(mut sub) => {
                 loop {
                     let event = sub.next().await;
                     if event.node.ends_with(NODE_NAME) {
                         for p in event.changed_parameters.iter().chain(event.new_parameters.iter()) {
                             if p.name == VALIDATOR_RESET_LIMIT_PARAM {
                                 if p.value.type_ == 2 { // Integer
                                     let val = p.value.integer_value;
                                     validator_clone_param.lock().unwrap().set_reset_max_value(val);
                                     log::info!("Validator reset limit updated to {}", val);
                                 }
                             }
                         }
                     }
                 }
            }
            Err(e) => log::warn!("Failed to subscribe to parameter events: {}", e),
        }
    });

    // Lifecycle Services
    
    // GET STATE
    let state_clone_get = lifecycle_state.clone();
    let get_topic = format!("{}/get_state", NODE_NAME);
    let _srv_get = client.advertise_service::<GetState, _>(
        &get_topic,
        move |_| {
            let s = *state_clone_get.lock().unwrap();
            Ok(GetStateResponse { current_state: s.to_msg() })
        }
    ).await?;
    log::info!("Service advertised: {}", get_topic);

    // CHANGE STATE
    let state_clone_change = lifecycle_state.clone();
    let resources_clone_change = resources.clone();
    let client_clone_change = client.clone();
    let validator_clone_sub = validator.clone();
    let change_topic = format!("{}/change_state", NODE_NAME);
    let topic_name_clone = topic_name.clone();

    let _srv_change = client.advertise_service::<ChangeState, _>(
        &change_topic,
        move |req| {
            let transition_id = req.transition.id;
            let mut current_state = state_clone_change.lock().unwrap();
            let mut success = false;

            match (current_state.clone(), transition_id) {
                (LifecycleState::Unconfigured, TRANSITION_CONFIGURE) => {
                    // Start Subscription Task
                    let token = CancellationToken::new();
                    let token_clone = token.clone();
                    let client_c = client_clone_change.clone();
                    let topic_n = topic_name_clone.clone();
                    let valid_c = validator_clone_sub.clone();
                    let state_c = state_clone_change.clone();

                    tokio::spawn(async move {
                        let mut sub = match client_c.subscribe::<MsgCount>(&topic_n).await {
                            Ok(s) => s,
                            Err(e) => {
                                log::error!("Failed to subscribe: {}", e);
                                return;
                            }
                        };
                        
                        loop {
                            tokio::select! {
                                _ = token_clone.cancelled() => {
                                    return;
                                }
                                msg = sub.next() => {
                                    // Gating logic
                                    let s = *state_c.lock().unwrap();
                                    if s == LifecycleState::Active {
                                        let decision = valid_c.lock().unwrap().on_count(msg.count);
                                        match decision.event {
                                            StreamEvent::Valid => log::info!("[Valid] {}", decision.message),
                                            StreamEvent::Reset => log::warn!("[Reset] {}", decision.message),
                                            StreamEvent::OutOfOrder => log::error!("[OutOfOrder] {}", decision.message),
                                        }
                                    } else {
                                        // Inactive
                                    }
                                }
                            }
                        }
                    });

                    resources_clone_change.lock().unwrap().sub_token = Some(token);
                    *current_state = LifecycleState::Inactive;
                    success = true;
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
                    if let Some(token) = resources_clone_change.lock().unwrap().sub_token.take() {
                        token.cancel();
                    }
                    *current_state = LifecycleState::Unconfigured;
                    success = true;
                },
                 (LifecycleState::Unconfigured, TRANSITION_SHUTDOWN) |
                 (LifecycleState::Inactive, TRANSITION_SHUTDOWN) |
                 (LifecycleState::Active, TRANSITION_SHUTDOWN) => {
                    if let Some(token) = resources_clone_change.lock().unwrap().sub_token.take() {
                        token.cancel();
                    }
                    *current_state = LifecycleState::Finalized;
                    success = true;
                },
                _ => {
                    log::warn!("Invalid transition request");
                    success = false;
                }
            }

            Ok(ChangeStateResponse { success })
        }
    ).await?;
    log::info!("Service advertised: {}", change_topic);

    // SET PARAMETERS
    let validator_clone_set = validator.clone();
    let set_param_topic = format!("{}/set_parameters", NODE_NAME);
    let _srv_set_param = client.advertise_service::<SetParameters, _>(
        &set_param_topic,
        move |req| {
             let mut results = Vec::new();
             for p in req.parameters {
                 if p.name == VALIDATOR_RESET_LIMIT_PARAM {
                     let mut success = false;
                     let mut reason = "Invalid Type/Value".to_string();

                     if p.value.type_ == 2 { // Integer
                         let val = p.value.integer_value;
                         validator_clone_set.lock().unwrap().set_reset_max_value(val);
                         log::info!("SetParameters: Updated {} to {}", VALIDATOR_RESET_LIMIT_PARAM, val);
                         success = true;
                         reason = "".to_string();
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

    tokio::signal::ctrl_c().await?;
    Ok(())
}
