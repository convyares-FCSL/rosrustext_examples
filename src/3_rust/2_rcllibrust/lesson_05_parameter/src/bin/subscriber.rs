use roslibrust::rosbridge::ClientHandle;
use roslibrust::RosMessageType;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

//
use utils_roslibrust::utils::{Config, parse_params_files_from_args};
use utils_roslibrust::{qos, topics};

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_05_subscriber";
const DEFAULT_RESET_MAX: u64 = 10;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MsgCount {
    pub count: i64,
}

impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
}

struct Parameters {
    topic_name: String,
    reset_max_value: u64,
}

impl Parameters {
    fn from_config(cfg: &Config) -> Self {
        let topic_name = topics::telemetry(cfg); // Uses topics.rs helper
        let reset_max_value = cfg.get_or("subscriber.reset_max_value", DEFAULT_RESET_MAX);

        log::info!("Configuration: topic='{}', reset_limit={}", topic_name, reset_max_value);

        Self {
            topic_name,
            reset_max_value,
        }
    }
}

struct StreamState {
    initialized: bool,
    expected: u64,
    reset_max_value: u64,
}

struct SubscriberChatter {
    _task: tokio::task::JoinHandle<()>,
    _state: Arc<Mutex<StreamState>>,
}

impl SubscriberChatter {
    async fn new(client: &ClientHandle, cfg: &Config) -> Result<Self, Box<dyn std::error::Error>> {
        let params = Parameters::from_config(cfg);
        let _options = qos::get_options("default");

        let state = Arc::new(Mutex::new(StreamState {
            initialized: false,
            expected: 0,
            reset_max_value: params.reset_max_value,
        }));

        let stream = client.subscribe::<MsgCount>(&params.topic_name).await?;
        let state_cb = Arc::clone(&state);

        let task = tokio::spawn(async move {
            loop {
                // Inherent next() returns MsgCount directly, no Option wrapper
                let msg = stream.next().await;
                
                let mut st = match state_cb.lock() {
                    Ok(g) => g,
                    Err(e) => { log::error!("Mutex poison: {}", e); continue; }
                };
                Self::on_msg(&mut st, &msg);
            }
        });

        Ok(Self {_task: task, _state: state})
    }

    fn on_msg(st: &mut StreamState, msg: &MsgCount) {
        let count = match Self::extract_count(msg) { Some(v) => v, None => return };
        if Self::handle_initial(st, count) { return; }
        if Self::handle_reset(st, count) { return; }
        if Self::handle_out_of_order(st, count) { return; }
        log::info!("Received: {}", count);
        st.expected = count + 1;
    }

    fn extract_count(msg: &MsgCount) -> Option<u64> {
        if msg.count < 0 { None } else { Some(msg.count as u64) }
    }

    fn handle_initial(st: &mut StreamState, c: u64) -> bool {
        if !st.initialized {
            st.initialized = true;
            st.expected = c + 1;
            log::info!("Received (initial): {}", c);
            return true;
        }
        false
    }

    fn handle_reset(st: &mut StreamState, c: u64) -> bool {
        if c <= st.reset_max_value && c < st.expected {
            st.expected = c + 1;
            log::warn!("Detected counter reset. Re-syncing at: {}", c);
            return true;
        }
        false
    }

    fn handle_out_of_order(st: &mut StreamState, c: u64) -> bool {
        if c < st.expected {
            log::warn!("Out-of-order sample: {} < expected {}", c, st.expected);
            return true;
        }
        false
    }
}

struct Lesson05Node {
    _client: ClientHandle,
    _subscriber_component: Arc<SubscriberChatter>,
}

impl Lesson05Node {
    async fn new(client: ClientHandle, cfg: &Config) -> Result<Self, Box<dyn std::error::Error>> {
        let subscriber_component = Arc::new(SubscriberChatter::new(&client, cfg).await?);
        log::info!("Lesson 05 node started (subscriber). Ctrl+C to exit.");
        Ok(Self { _client: client, _subscriber_component: subscriber_component })
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