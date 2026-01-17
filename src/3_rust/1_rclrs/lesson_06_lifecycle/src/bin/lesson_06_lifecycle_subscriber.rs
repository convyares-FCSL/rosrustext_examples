use std::sync::{Arc, Mutex};

use rclrs::{ log_error, log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, RclrsError, RclrsErrorFilter, SpinOptions, };
use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
use rosrustext_rosrs::parameters::{ Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value, };
use rosrustext_rosrs::State;

use lesson_06_logic::{StreamEvent, TelemetryStreamValidator};
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};
use utils_rclrs::IntoRclrsError;

const NODE_NAME: &str = "lesson_06_lifecycle_subscriber";
const RESET_MAX_PARAM: &str = "reset_max_value";

/// Subscriber component encapsulating the ROS subscription and validation logic.
///
/// The component is created during the Configure lifecycle transition and
/// released during Cleanup/Shutdown.
struct SubscriberComponent {
    _sub: rclrs::Subscription<MsgCount>,
    validator: Arc<Mutex<TelemetryStreamValidator>>,
}

impl SubscriberComponent {
    /// Creates and configures the subscriber component.
    ///
    /// The subscription callback validates incoming messages and emits
    /// classification events via logging.
    fn new(node: &LifecycleNode, initial_reset_max: i64) -> Result<Self, RclrsError> {
        let logger = node.node().logger().clone();

        let raw_node = node.node();
        let topic_name = topics::telemetry(raw_node);
        let qos_profile = qos::telemetry(raw_node);

        let validator = Arc::new(Mutex::new(TelemetryStreamValidator::new(initial_reset_max)));
        let validator_cb = Arc::clone(&validator);

        let mut sub_opts = rclrs::SubscriptionOptions::new(&topic_name);
        sub_opts.qos = qos_profile;

        let sub = node.create_subscription::<MsgCount, _>(sub_opts, move |msg| {
            let Ok(mut v) = validator_cb.lock() else {
                // Validator lock failure is treated as non-fatal.
                return;
            };
            Self::on_msg(&logger, &mut v, &msg);
        })
        .rcl_generic()?;

        log_info!(node.node().logger(), "Component configured: subscriber on '{}'", topic_name);

        Ok(Self { _sub: sub, validator })
    }

    /// Handles a received message by delegating to the stream validator
    /// and emitting an appropriate log entry.
    fn on_msg(logger: &Logger, v: &mut TelemetryStreamValidator, msg: &MsgCount) {
        let decision = v.on_count(msg.count);
        match decision.event {
            StreamEvent::Reset | StreamEvent::OutOfOrder => log_warn!(logger, "{}", decision.message),
            StreamEvent::Valid => log_info!(logger, "{}", decision.message),
        }
    }
}

/// Container for resources managed across lifecycle transitions.
#[derive(Default)]
struct ResourceState {
    component: Option<Arc<SubscriberComponent>>,
    watcher: Option<ParameterWatcher>,
    params: Option<ParameterNode>,
    last_reset_max: i64,
}

/// Lifecycle-managed subscriber node.
struct Lesson06SubscriberNode {
    state: Arc<Mutex<ResourceState>>,
}

impl Lesson06SubscriberNode {
    fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(ResourceState::default())),
        }
    }

    /// Creates the lifecycle node and registers callbacks.
    pub fn create(executor: &Executor) -> Result<LifecycleNode, RclrsError> {
        let callbacks = Box::new(Self::new());
        let ln = LifecycleNode::create_with_callbacks(executor, NODE_NAME, callbacks).rcl_generic()?;

        log_info!(ln.node().logger(), "Node initialized (state: Unconfigured). Awaiting lifecycle manager.");

        Ok(ln)
    }

    /// Resets all dynamically managed resources.
    fn reset_state(state: &Arc<Mutex<ResourceState>>) {
        let Ok(mut guard) = state.lock() else {
            return;
        };
        guard.watcher = None;
        guard.component = None;
        guard.params = None;
        guard.last_reset_max = 0;
    }

    /// Declares parameters and retrieves the initial reset tolerance.
    fn _declare_parameters(params: &ParameterNode, logger: &Logger) -> i64 {
        let desc = Descriptor {
            description: "Maximum tolerated backward counter jump".to_string(),
            ..Descriptor::default()
        };

        if let Err(e) = params.declare(RESET_MAX_PARAM, Type::Integer, Value::Integer(1), desc) {
            log_error!(logger, "Parameter declaration failed: {}. Using default.", e);
            return 1;
        }

        let keys = [RESET_MAX_PARAM.to_string()];
        let store_arc = params.store();
        let store = store_arc.lock().unwrap();

        store.get(&keys).first().and_then(|v| match v { Value::Integer(i) => Some(*i), _ => None, }).unwrap_or(1)
    }

    /// Handles runtime parameter updates.
    fn _on_param_change(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, change: &ParameterChange) {
        if change.name != RESET_MAX_PARAM {
            return;
        }

        let Value::Integer(new_val) = &change.new_value else {
            log_warn!(node.node().logger(), "Ignoring {} update: non-integer value", RESET_MAX_PARAM);
            return;
        };
        let new_val = *new_val;

        if new_val < 0 {
            log_warn!(node.node().logger(), "Ignoring {} update: value must be non-negative", RESET_MAX_PARAM);
            return;
        }

        Self::_on_param_update(node, state, new_val);
    }

    /// Applies a validated parameter update to the active component.
    fn _on_param_update(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, new_val: i64) {
        let Ok(mut guard) = state.lock() else {
            return;
        };

        if guard.last_reset_max == new_val {
            return;
        }

        let Some(comp) = guard.component.clone() else {
            return;
        };

        let lock_result = comp.validator.lock();
        match lock_result {
            Ok(mut v) => {
                v.set_reset_max_value(new_val);
                guard.last_reset_max = new_val;
                log_info!(node.node().logger(), "Updated {} -> {}", RESET_MAX_PARAM, new_val);
            }
            Err(_) => {
                log_error!(node.node().logger(), "Failed to lock validator");
            }
        }
    }
}

impl LifecycleCallbacksWithNode for Lesson06SubscriberNode {
    fn on_configure(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        let logger = node.node().logger();
        log_info!(logger, "Transitioning: Unconfigured -> Inactive");

        let params = match ParameterNode::try_new(node.node().clone()) {
            Ok(p) => p,
            Err(e) => {
                log_error!(logger, "ParameterNode creation failed: {}", e);
                return CallbackResult::Failure;
            }
        };

        let initial_val = Self::_declare_parameters(&params, logger);
        let initial_reset_max = initial_val.max(0);

        let comp = match SubscriberComponent::new(node, initial_reset_max) {
            Ok(c) => Arc::new(c),
            Err(e) => {
                log_error!(logger, "Component creation failed: {}", e);
                return CallbackResult::Failure;
            }
        };

        let watcher = match ParameterWatcher::new(&params) {
            Ok(w) => w,
            Err(e) => {
                log_error!(logger, "ParameterWatcher creation failed: {}", e);
                return CallbackResult::Failure;
            }
        };

        let node_clone = node.clone();
        let state_clone = Arc::clone(&self.state);

        watcher.on_change(RESET_MAX_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, change);
        });

        let Ok(mut guard) = self.state.lock() else {
            log_error!(logger, "State lock failure during configuration");
            return CallbackResult::Failure;
        };

        guard.params = Some(params);
        guard.watcher = Some(watcher);
        guard.component = Some(comp);
        guard.last_reset_max = initial_reset_max;

        CallbackResult::Success
    }

    fn on_activate(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        log_info!(node.node().logger(), "Transitioning: Inactive -> Active");
        CallbackResult::Success
    }

    fn on_deactivate(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        log_info!(node.node().logger(), "Transitioning: Active -> Inactive");
        CallbackResult::Success
    }

    fn on_cleanup(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        log_info!(node.node().logger(), "Transitioning: Inactive -> Unconfigured");
        Self::reset_state(&self.state);
        CallbackResult::Success
    }

    fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        Self::reset_state(&self.state);
        CallbackResult::Success
    }

    fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let lifecycle_node = Lesson06SubscriberNode::create(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            log_error!(lifecycle_node.node().logger(), "Executor error: {err}");
            err
        })?;

    Ok(())
}
