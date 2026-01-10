use std::sync::{Arc, Mutex};

use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, Executor, Node, RclrsError,
    RclrsErrorFilter, SpinOptions, Logger,
};

use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};

const NODE_NAME: &str = "lesson_03_node";

struct StreamState {
    initialized: bool,
    expected: u64,
    reset_max_value: u64,
}

// Subscriber component: owns the Subscription + stream-validation state.
struct SubscriberChatter {
    _sub: rclrs::Subscription<MsgCount>,
    _logger: Logger,
    _state: Arc<Mutex<StreamState>>, // keep-alive / shared with callback
}

impl SubscriberChatter {
    fn new(node: &Node, reset_max_value: u64) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        let topic_name = topics::chatter(node);
        let qos_profile = qos::from_parameters(node);

        let state = Arc::new(Mutex::new(StreamState {
            initialized: false,
            expected: 0,
            reset_max_value,
        }));

        let mut options = rclrs::SubscriptionOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        let sub = Self::create_subscription(node, options, logger.clone(), Arc::clone(&state))?;

        Ok(Self {_sub: sub, _logger: logger, _state: state})
    }

    fn create_subscription(node: &Node, options: rclrs::SubscriptionOptions, logger: Logger, state: Arc<Mutex<StreamState>>) -> Result<rclrs::Subscription<MsgCount>, RclrsError> {
        node.create_subscription::<MsgCount, _>(options, move |msg: MsgCount| {
            let mut st = match state.lock() {
                Ok(g) => g,
                Err(_) => return, // poisoned mutex: drop sample and continue
            };
            Self::on_msg(&logger, &mut st, &msg);
        })
    }

    fn on_msg(logger: &Logger, st: &mut StreamState, msg: &MsgCount) {
        let count = match Self::extract_count(logger, msg) {
            Some(v) => v,
            None => return,
        };

        if Self::handle_initial(logger, st, count) { return; }

        if Self::handle_reset(logger, st, count) { return; }

        if Self::handle_out_of_order(logger, st, count) { return; }

        log_info!(logger, "Received: {}", count);
        st.expected = count + 1;
    }

    fn extract_count(logger: &Logger, msg: &MsgCount) -> Option<u64> {
        if msg.count < 0 {
            log_warn!(logger, "Out-of-order/invalid: {}", msg.count);
            None
        } else {
            Some(msg.count as u64)
        }
    }

    fn handle_initial(logger: &Logger, st: &mut StreamState, c: u64) -> bool {
        if !st.initialized {
            st.initialized = true;
            st.expected = c + 1;
            log_info!(logger, "Received (initial): {}", c);
            return true;
        }
        false
    }

    fn handle_reset(logger: &Logger, st: &mut StreamState, c: u64) -> bool {
        if c <= st.reset_max_value && c < st.expected {
            st.expected = c + 1;
            log_warn!(logger, "Detected counter reset. Re-syncing at: {}", c);
            return true;
        }
        false
    }

    fn handle_out_of_order(logger: &Logger, st: &mut StreamState, c: u64) -> bool {
        if c < st.expected {
            log_warn!(logger, "Out-of-order/invalid: {} < {}", c, st.expected);
            return true;
        }
        false
    }
}

struct Lesson03Node {
    pub node: Node,
    _subscriber_component: Arc<SubscriberChatter>,
}

impl Lesson03Node {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        let sub = Arc::new(SubscriberChatter::new(&node, 1)?);

        log_info!(node.logger(), "Lesson 03 node started (subscriber). Ctrl+C to exit.");

        Ok(Self {node, _subscriber_component: sub})
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Lesson03Node::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}