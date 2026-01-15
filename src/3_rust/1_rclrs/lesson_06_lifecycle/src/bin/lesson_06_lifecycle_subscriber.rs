use std::sync::{
    atomic::{AtomicI64, Ordering},
    Arc, Mutex,
};

use rclrs::{
    log_error, log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, RclrsError,
    SpinOptions,
};
use rosrustext_rosrs::lifecycle::{
    CallbackResult, LifecycleCallbacksWithNode, LifecycleNode, ManagedSubscription, State,
};
use rosrustext_rosrs::parameters::{
    Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value,
};

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};

// Pure logic library
use lesson_06_logic::{StreamEvent, TelemetryStreamValidator};

const NODE_NAME: &str = "lesson_06_lifecycle_subscriber";
const RESET_MAX_PARAM: &str = "reset_max_value";

// ==============================================================================
// 1. Subscriber Component (Passive Resource)
// ==============================================================================

struct SubscriberComponent {
    // ManagedSubscription handles Transport Gating (DDS Lifecycle)
    _sub: ManagedSubscription<MsgCount>,
    
    // Application Logic (Shared with Watcher)
    validator: Arc<Mutex<TelemetryStreamValidator>>,
}

impl SubscriberComponent {
    /// Factory: Acts as the constructor for the Configured state.
    fn new(
        node: &LifecycleNode,
        initial_reset_max: i64,
    ) -> Result<Self, RclrsError> {
        let logger = node.node().logger().clone();
        let topic_name = topics::telemetry(node.node());
        let qos_profile = qos::telemetry(node.node());

        // Shared validator state (updated in-place via watcher)
        let validator = Arc::new(Mutex::new(TelemetryStreamValidator::new(initial_reset_max)));
        let validator_cb = validator.clone();

        // Create Managed Subscription
        // Note: LifecycleNode automatically gates the callback based on state.
        let sub = node.create_subscription_with_qos::<MsgCount, _>(
            &topic_name,
            qos_profile,
            move |msg| {
                // Lock validator to ensure correctness during concurrent updates
                let mut v = match validator_cb.lock() {
                    Ok(g) => g,
                    Err(_) => return, // poisoned: drop sample and continue
                };
                Self::on_msg(&logger, &mut v, &msg);
            },
        )?;

        log_info!(
            node.node().logger(),
            "Component Configured: Subscriber on '{}'",
            topic_name
        );

        Ok(Self {
            _sub: sub,
            validator,
        })
    }

    fn on_msg(logger: &Logger, v: &mut TelemetryStreamValidator, msg: &MsgCount) {
        let decision = v.on_count(msg.count);

        match decision.event {
            StreamEvent::Reset | StreamEvent::OutOfOrder => {
                log_warn!(logger, "{}", decision.message)
            }
            StreamEvent::Valid => log_info!(logger, "{}", decision.message),
        }
    }
}

// ==============================================================================
// 2. Active State Container
// ==============================================================================

/// Holds all lazy-allocated resources.
#[derive(Default)]
struct ResourceState {
    component: Option<Arc<SubscriberComponent>>,
    watcher: Option<ParameterWatcher>,
    params: Option<ParameterNode>,
    last_reset_max: i64,
}

// ==============================================================================
// 3. Node & Callback Implementation
// ==============================================================================

struct Lesson06SubscriberNode {
    state: Arc<Mutex<ResourceState>>,
}

impl Lesson06SubscriberNode {
    fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(ResourceState::default())),
        }
    }

    /// Factory method to create the lifecycle node and inject callbacks.
    pub fn create(executor: &Executor) -> Result<LifecycleNode, RclrsError> {
        let callbacks = Box::new(Self::new());
        let ln = LifecycleNode::create_with_callbacks(executor, NODE_NAME, callbacks)?;

        log_info!(
            ln.node().logger(),
            "Node initialized (State: Unconfigured). Waiting for manager..."
        );

        Ok(ln)
    }

    // --- Internal Helpers ---

    fn _declare_parameters(params: &ParameterNode, logger: &Logger) -> i64 {
        let desc = Descriptor {
            description: "Max allowed backward jump".to_string(),
            ..Descriptor::default()
        };

        if let Err(e) = params.declare(
            RESET_MAX_PARAM,
            Type::Integer,
            Value::Integer(1), // Default 1
            desc,
        ) {
            log_error!(logger, "Failed to declare parameter: {}. Using 1", e);
            return 1;
        }

        // Read effective value
        let store = params.store();
        match store.get(RESET_MAX_PARAM) {
            Some(Value::Integer(v)) => *v,
            _ => 1,
        }
    }

    /// Callback-shaped helper: Extracts, validates, and triggers update.
    fn _on_param_change(
        node: &LifecycleNode,
        state: &Arc<Mutex<ResourceState>>,
        change: &ParameterChange,
    ) {
        // Filter param
        if change.name != RESET_MAX_PARAM {
            return;
        }

        // 1. Extraction
        let new_val = match &change.new_value {
            Value::Integer(v) => *v,
            _ => {
                log_warn!(node.node().logger(), "Ignoring {} update (non-integer)", RESET_MAX_PARAM);
                return;
            }
        };

        // 2. Validation
        if new_val < 0 {
            log_warn!(node.node().logger(), "Ignoring {} update (must be >= 0)", RESET_MAX_PARAM);
            return;
        }

        // 3. Execution Logic
        Self::_on_param_update(node, state, new_val);
    }

    /// Logic helper: Locks state and updates the validator in-place.
    fn _on_param_update(
        node: &LifecycleNode,
        state: &Arc<Mutex<ResourceState>>,
        new_val: i64,
    ) {
        let mut guard = match state.lock() {
            Ok(g) => g,
            Err(_) => return,
        };

        // Deduplicate
        if guard.last_reset_max == new_val {
            return;
        }

        // Update the validator (if component exists)
        if let Some(comp) = &guard.component {
            match comp.validator.lock() {
                Ok(mut v) => {
                    v.set_reset_max_value(new_val);
                    guard.last_reset_max = new_val;
                    log_info!(node.node().logger(), "Updated {} -> {}", RESET_MAX_PARAM, new_val);
                }
                Err(_) => {
                    log_error!(node.node().logger(), "Failed to lock validator for update");
                }
            }
        }
    }
}

impl LifecycleCallbacksWithNode for Lesson06SubscriberNode {
    fn on_configure(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        let logger = node.node().logger();
        log_info!(logger, "Transitioning: Unconfigured -> Inactive");

        // 1. Parameter Stack
        let params = match ParameterNode::try_new(node.node().clone()) {
            Ok(p) => p,
            Err(e) => {
                log_error!(logger, "Failed to create ParameterNode: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 2. Declare & Config
        let initial_val = Self::_declare_parameters(&params, logger);
        
        // Sanitize initial value
        let initial_reset_max = if initial_val < 0 {
            log_error!(logger, "Invalid initial {}='{}'. Defaulting to 0.", RESET_MAX_PARAM, initial_val);
            0
        } else {
            initial_val
        };

        // 3. Allocate Component
        let comp = match SubscriberComponent::new(node, initial_reset_max) {
            Ok(c) => Arc::new(c),
            Err(e) => {
                log_error!(logger, "Failed to create component: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 4. Create Watcher
        let watcher = match ParameterWatcher::new(&params) {
            Ok(w) => w,
            Err(e) => {
                log_error!(logger, "Failed to create watcher: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 5. Register Handler
        let node_clone = node.clone();
        let state_clone = Arc::clone(&self.state);

        watcher.on_change(RESET_MAX_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, change);
        });

        // 6. Store Resources
        let mut guard = self.state.lock().unwrap();
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

        let mut guard = self.state.lock().unwrap();
        
        // Drop watcher first to stop incoming events
        guard.watcher = None;
        // Drop component to release subscription handles
        guard.component = None;
        // Drop params handle (releases adapter-owned parameter surface)
        guard.params = None;
        
        guard.last_reset_max = 0;

        CallbackResult::Success
    }

    fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        let mut guard = self.state.lock().unwrap();
        guard.watcher = None;
        guard.component = None;
        guard.params = None;
        guard.last_reset_max = 0;
        CallbackResult::Success
    }

    fn on_error(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        log_error!(node.node().logger(), "Lifecycle Error Detected");
        CallbackResult::Success
    }
}

// ==============================================================================
// 4. Main Entry
// ==============================================================================

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // Create the Lifecycle Node via factory
    let lifecycle_node = Lesson06SubscriberNode::create(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            log_error!(lifecycle_node.node().logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}