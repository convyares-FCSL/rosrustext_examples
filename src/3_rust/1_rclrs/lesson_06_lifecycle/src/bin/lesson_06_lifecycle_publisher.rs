use std::sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex};
use std::time::Duration;

use rclrs::{
    log_error, log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, RclrsError,
    SpinOptions,
};
use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode, ManagedPublisher, ManagedTimer, State,};
use rosrustext_rosrs::parameters::{Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value,};

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};

// Pure logic library
use lesson_06_logic::{period_s_to_ms_strict, TelemetryPublisherCore};

const NODE_NAME: &str = "lesson_06_lifecycle_publisher";
const PERIOD_PARAM: &str = "timer_period_s";

// ==============================================================================
// 1. Publisher Component (Passive Resource)
// ==============================================================================

struct PublisherComponent {
    publisher: ManagedPublisher<MsgCount>,
    core: Mutex<TelemetryPublisherCore>,
    tick_failed: AtomicBool,
}

impl PublisherComponent {
    /// Factory: Acts as the constructor for the Configured state.
    fn new(node: &LifecycleNode) -> Result<Self, RclrsError> {
        let logger = node.node().logger();
        let topic_name = topics::telemetry(node.node());
        let qos_profile = qos::telemetry(node.node());

        let publisher = node.create_publisher_with_qos::<MsgCount>(&topic_name, qos_profile)?;
        let core = Mutex::new(TelemetryPublisherCore::new());

        log_info!(logger, "Component Configured: Publisher on '{}'", topic_name);

        Ok(Self { publisher, core, tick_failed: AtomicBool::new(false), })
    }

    fn publish(&self) -> Result<(), RclrsError> {
        let mut core = match self.core.lock() {
            Ok(g) => g,
            Err(_) => return Ok(()), // poisoned: skip tick and continue
        };

        let msg = core.next_message();

        // Safety: If Inactive, ManagedPublisher drops this silently.
        self.publisher.publish(msg)?;
        Ok(())
    }
}

// ==============================================================================
// 2. Active State Container
// ==============================================================================

/// Holds all lazy-allocated resources.
/// Must be shared (Arc/Mutex) so the ParameterWatcher closure can update the timer.
#[derive(Default)]
struct ResourceState {
    component: Option<Arc<PublisherComponent>>,
    timer: Option<ManagedTimer>,
    watcher: Option<ParameterWatcher>,
    params: Option<ParameterNode>,
    last_period_ms: u64,
}

// ==============================================================================
// 3. Node + Lifecycle Callbacks (single struct)
// ==============================================================================

struct Lesson06PublisherNode {
    state: Arc<Mutex<ResourceState>>,
}

impl Lesson06PublisherNode {
    fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(ResourceState::default())),
        }
    }

    /// Factory method to create the lifecycle node and inject callbacks.
    /// This keeps 'main' clean while allowing the log to be placed correctly.
    pub fn create(executor: &Executor) -> Result<LifecycleNode, RclrsError> {
        let callbacks = Box::new(Self::new());
        let ln = LifecycleNode::create_with_callbacks(executor, NODE_NAME, callbacks)?;

        log_info!(ln.node().logger(), "Node initialized (State: Unconfigured). Waiting for manager...");

        Ok(ln)
    }

    // --- Internal Helpers ---

    /// Declares the parameter and returns the effective initial value.
    fn _declare_parameters(params: &ParameterNode, logger: &Logger) -> f64 {
        let desc = Descriptor {
            description: "Timer period in seconds".to_string(),
            ..Descriptor::default()
        };

        if let Err(e) = params.declare(PERIOD_PARAM, Type::Double, Value::Double(1.0), desc) {
            log_error!(logger, "Failed to declare parameter: {}. Using 1.0", e);
            return 1.0;
        }

        // Read the effective value from the store (handles overrides if present)
        let store = params.store();
        match store.get(PERIOD_PARAM) {
            Some(Value::Double(v)) => *v,
            Some(Value::Integer(v)) => *v as f64,
            _ => 1.0,
        }
    }

    fn _create_gated_timer(node: &LifecycleNode, comp: &Arc<PublisherComponent>, period_ms: u64,) -> Result<ManagedTimer, RclrsError> {
        let comp_clone = Arc::clone(comp);
        let logger = node.node().logger().clone();

        node.create_timer_repeating_gated(Duration::from_millis(period_ms), move || {
            match comp_clone.publish() {
                Ok(_) => {
                    comp_clone.tick_failed.store(false, Ordering::Relaxed);
                }
                Err(e) => {
                    if !comp_clone.tick_failed.swap(true, Ordering::Relaxed) {
                        log_error!(&logger, "Tick failed: {}", e);
                    }
                }
            }
        })
    }

    /// Callback-shaped helper: Extracts, validates, and triggers update.
    fn _on_param_change(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, change: &ParameterChange,) {
        // 1. Extraction & Type Coercion
        let new_period = match &change.new_value {
            Value::Double(v) => *v,
            Value::Integer(v) => *v as f64,
            _ => {
                log_warn!(node.node().logger(), "Ignoring {} update (non-numeric)", PERIOD_PARAM);
                return;
            },
        };

        // 2. Validation
        if new_period <= 0.0 {
            log_warn!(node.node().logger(), "Ignoring {} update (<= 0.0)", PERIOD_PARAM);
            return;
        }

        // 3. Strict Conversion
        let Ok(ms) = period_s_to_ms_strict(new_period) else {
            log_warn!(node.node().logger(), "Ignoring {} update (conversion failed)", PERIOD_PARAM);
            return;
        };

        // 4. Execution Logic
        Self::_on_param_update(node, state, comp, ms);
    }

    /// Logic helper: Locks state and hot-swaps the timer.
    fn _on_param_update(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, new_ms: u64,) {
        let mut guard = match state.lock() {
            Ok(g) => g,
            Err(_) => return,
        };

        // Deduplicate
        if guard.last_period_ms == new_ms {
            return;
        }

        // Create new timer first, then swap (dropping old one)
        match Self::_create_gated_timer(node, comp, new_ms) {
            Ok(new_timer) => {
                guard.timer = Some(new_timer);
                guard.last_period_ms = new_ms;
                log_info!(node.node().logger(), "Timer updated to {}ms", new_ms);
            }
            Err(e) => {
                log_error!(node.node().logger(), "Failed to update timer: {}", e);
            }
        }
    }
}

// ==============================================================================
// 4. Lifecycle Callbacks Trait
// ==============================================================================

impl LifecycleCallbacksWithNode for Lesson06PublisherNode {
    fn on_configure(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        let logger = node.node().logger();
        log_info!(logger, "Transitioning: Unconfigured -> Inactive");

        // 1. Parameter Stack (v0.2.0: Create wrapper around existing node)
        let params = match ParameterNode::try_new(node.node().clone()) {
            Ok(p) => p,
            Err(e) => {
                log_error!(logger, "Failed to create ParameterNode: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 2. Declare & Config
        let initial_val_s = Self::_declare_parameters(&params, logger);
        let initial_ms = match period_s_to_ms_strict(initial_val_s) {
            Ok(ms) => ms,
            Err(e) => {
                log_error!(logger, "Invalid initial parameter: {}. Defaulting to 1.0s", e);
                1000
            }
        };

        // 3. Allocate Component
        let comp = match PublisherComponent::new(node) {
            Ok(c) => Arc::new(c),
            Err(e) => {
                log_error!(logger, "Failed to create component: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 4. Create Initial Timer
        let timer = match Self::_create_gated_timer(node, &comp, initial_ms) {
            Ok(t) => t,
            Err(e) => {
                log_error!(logger, "Failed to create timer: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 5. Create Watcher
        let watcher = match ParameterWatcher::new(&params) {
            Ok(w) => w,
            Err(e) => {
                log_error!(logger, "Failed to create watcher: {}", e);
                return CallbackResult::Failure;
            }
        };

        // 6. Register Minimal Handler
        let node_clone = node.clone();
        let state_clone = Arc::clone(&self.state);
        let comp_clone = Arc::clone(&comp);

        watcher.on_change(PERIOD_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, &comp_clone, change);
        });

        // 7. Store Resources
        let mut guard = self.state.lock().unwrap();
        guard.params = Some(params);
        guard.watcher = Some(watcher);
        guard.component = Some(comp);
        guard.timer = Some(timer);
        guard.last_period_ms = initial_ms;

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
        // Drop timer and component to release DDS handles
        guard.timer = None;
        guard.component = None;
        
        // Drop params handle (releases adapter-owned parameter surface)
        guard.params = None;
        
        guard.last_period_ms = 0;

        CallbackResult::Success
    }

    fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        let mut guard = self.state.lock().unwrap();
        guard.watcher = None;
        guard.timer = None;
        guard.component = None;
        guard.params = None;
        guard.last_period_ms = 0;
        CallbackResult::Success
    }

    fn on_error(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        log_error!(node.node().logger(), "Lifecycle Error Detected");
        CallbackResult::Success
    }
}

// ==============================================================================
// 5. Main Entry
// ==============================================================================

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // Create the Lifecycle Node via factory (includes logging)
    let lifecycle_node = Lesson06PublisherNode::create(&executor)?;

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