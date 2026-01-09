use roslibrust::rosbridge::{ClientHandle};
use roslibrust::RosMessageType;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

use utils_roslibrust::{qos, topics};

const ROSBRIDGE_URL: &str = "ws://localhost:9090";
const NODE_NAME: &str = "lesson_03_node";

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MsgCount {
    pub count: i64,
}

impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
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
    pub async fn new(client: &ClientHandle, reset_max_value: u64) -> Result<Self, Box<dyn std::error::Error>> {
        let topic_name = topics::CHATTER;
        let _options = qos::get_options("default");

        let state = Arc::new(Mutex::new(StreamState {
            initialized: false,
            expected: 0,
            reset_max_value,
        }));

        let stream = client.subscribe::<MsgCount>(topic_name).await?;

        let state_cb = Arc::clone(&state);

        let task = tokio::spawn(async move {
            loop {
                let msg = stream.next().await;
                
                let mut st = match state_cb.lock() {
                    Ok(g) => g,
                    Err(e) => {
                        log::error!("StreamState mutex poisoned: {}", e);
                        continue; 
                    }
                };
                
                Self::on_msg(&mut st, &msg);
            }
        });

        Ok(Self {_task: task, _state: state})
    }

    fn on_msg(st: &mut StreamState, msg: &MsgCount) {
        let count = match Self::extract_count(msg) {
            Some(v) => v,
            None => return,
        };

        if Self::handle_initial(st, count) { return; }
        if Self::handle_reset(st, count) { return; }
        if Self::handle_out_of_order(st, count) { return; }

        log::info!("Received: {}", count);
        st.expected = count + 1;
    }

    fn extract_count(msg: &MsgCount) -> Option<u64> {
        if msg.count < 0 {
            log::warn!("Invalid payload: negative count {}", msg.count);
            None
        } else {
            Some(msg.count as u64)
        }
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

struct Lesson03Node {
    _client: ClientHandle,
    _subscriber_component: Arc<SubscriberChatter>,
}

impl Lesson03Node {
    async fn new(client: ClientHandle) -> Result<Self, Box<dyn std::error::Error>> {
        let subscriber_component = Arc::new(SubscriberChatter::new(&client, 1).await?);

        log::info!("Lesson 03 node started (subscriber). Ctrl+C to exit.");

        Ok(Self {
            _client: client,
            _subscriber_component: subscriber_component,
        })
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let client = ClientHandle::new(ROSBRIDGE_URL).await?;
    log::info!("Connected to {}", ROSBRIDGE_URL);

    let _node = Lesson03Node::new(client).await?;

    match tokio::signal::ctrl_c().await {
        Ok(()) => log::info!("Shutting down {}...", NODE_NAME),
        Err(e) => log::error!("Signal error: {}", e),
    }

    Ok(())
}