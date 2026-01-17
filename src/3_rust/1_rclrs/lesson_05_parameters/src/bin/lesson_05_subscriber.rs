use std::sync::{Arc, Mutex};

use rclrs::{ log_error, log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, Node, RclrsError, RclrsErrorFilter, SpinOptions, };

use rosrustext_rosrs::parameters::{  Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value, };

use lesson_05_logic::{StreamEvent, TelemetryStreamValidator};
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};
use utils_rclrs::IntoRclrsError;

const NODE_NAME: &str = "lesson_05_subscriber";
const RESET_MAX_PARAM: &str = "reset_max_value";

/// Subscriber component encapsulating the ROS subscription and validation logic.
///
/// The component is created once at startup and lives for the life of the process.
/// It is passive: it owns the subscription and validation state but does not manage
/// parameters or configuration logic.
struct SubscriberComponent {
    _sub: rclrs::Subscription<MsgCount>,
    validator: Arc<Mutex<TelemetryStreamValidator>>,
}

impl SubscriberComponent {
    /// Creates and configures the subscriber component.
    ///
    /// The subscription callback validates incoming messages and emits
    /// classification events via logging.
    fn new(node: &Node, initial_reset_max: i64) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        let topic_name = topics::telemetry(node);
        let qos_profile = qos::telemetry(node);

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
        })?;

        log_info!(node.logger(), "Component configured: subscriber on '{}'", topic_name);

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

/// Container for mutable resources that change at runtime.
#[derive(Default)]
struct ResourceState {
    last_reset_max: i64,
}

/// Parameter-driven subscriber node.
///
/// Uses rosrustext's ParameterWatcher to apply runtime updates to the validator
/// without polling or lifecycle management.
struct Lesson05SubscriberNode {
    /// Shared node handle. Required because rosrustext ParameterNode binds to Arc<Node>.
    node: Arc<Node>,

    _component: Arc<SubscriberComponent>,

    /// Mutable state updated by parameter change callbacks.
    _state: Arc<Mutex<ResourceState>>,

    /// Parameter plumbing and watcher keep-alives.
    _params: ParameterNode,
    _watcher: ParameterWatcher,
}

impl Lesson05SubscriberNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = Arc::new(executor.create_node(NODE_NAME)?);
        let logger = node.logger().clone();

        let params = ParameterNode::try_new(Arc::clone(&node)).rcl_generic()?;

        let initial_val = Self::_declare_parameters(&params, &logger);
        let initial_reset_max = initial_val.max(0);

        let component = Arc::new(SubscriberComponent::new(node.as_ref(), initial_reset_max)?);

        let state = Arc::new(Mutex::new(ResourceState {
            last_reset_max: initial_reset_max,
        }));

        let watcher = ParameterWatcher::new(&params).rcl_generic()?;

        let node_clone = Arc::clone(&node);
        let state_clone = Arc::clone(&state);
        let comp_clone = Arc::clone(&component);

        watcher.on_change(RESET_MAX_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, &comp_clone, change);
        });

        log_info!(&logger, "Lesson 05 Subscriber started.");

        Ok(Self { node, _component : component, _state : state, _params: params, _watcher: watcher, })
    }

    /// Declares parameters and retrieves the initial reset tolerance.
    ///
    /// Mirrors the Lesson 06 `_declare_parameters` pattern.
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

        store
            .get(&keys)
            .first()
            .and_then(|v| match v {
                Value::Integer(i) => Some(*i),
                _ => None,
            })
            .unwrap_or(1)
    }

    /// Validates a parameter change event and applies it if required.
    ///
    /// This function performs:
    /// - name filtering
    /// - type validation
    /// - semantic validation
    /// - "no-op" detection
    ///
    /// If the update is valid and changes effective configuration, it delegates to
    /// `_on_param_update`.
    fn _on_param_change(node: &Arc<Node>, state: &Arc<Mutex<ResourceState>>, comp: &Arc<SubscriberComponent>, change: &ParameterChange,) {
        if change.name != RESET_MAX_PARAM {
            return;
        }

        let Value::Integer(new_val) = &change.new_value else {
            log_warn!(node.logger(), "Ignoring {} update: non-integer value", RESET_MAX_PARAM);
            return;
        };
        let new_val = *new_val;

        if new_val < 0 {
            log_warn!(node.logger(), "Ignoring {} update: value must be non-negative", RESET_MAX_PARAM);
            return;
        }

        Self::_on_param_update(node, state, comp, new_val);
    }

    /// Applies a validated parameter update to the active component.
    fn _on_param_update(node: &Arc<Node>, state: &Arc<Mutex<ResourceState>>, comp: &Arc<SubscriberComponent>, new_val: i64,) {
        let mut guard = match state.lock() {
            Ok(g) => g,
            Err(_) => return,
        };

        // No-op detection: if the value hasn't changed, do nothing.
        if guard.last_reset_max == new_val { return; }

        // Update validator in place.
        match comp.validator.lock() {
            Ok(mut v) => {
                v.set_reset_max_value(new_val);
                guard.last_reset_max = new_val;
                log_info!(node.logger(), "Updated {} -> {}", RESET_MAX_PARAM, new_val);
            }
            Err(_) => {
                log_error!(node.logger(), "Failed to lock validator");
            }
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = Lesson05SubscriberNode::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}
