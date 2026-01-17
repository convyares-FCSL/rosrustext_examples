use std::sync::{ atomic::{AtomicBool, Ordering}, Arc, Mutex, };
use std::time::Duration;

use rclrs::{ log_error, log_info, Context, CreateBasicExecutor, Executor, Logger, RclrsError, RclrsErrorFilter, SpinOptions, };
use rosrustext_rosrs::lifecycle::{ CallbackResult, LifecycleCallbacksWithNode, LifecycleNode, ManagedPublisher, ManagedTimer, };
use rosrustext_rosrs::parameters::{ Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value, };
use rosrustext_rosrs::State;

use lesson_06_logic::{period_s_to_ms_strict, TelemetryPublisherCore};
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};
use utils_rclrs::IntoRclrsError;

const NODE_NAME: &str = "lesson_06_lifecycle_publisher";
const PERIOD_PARAM: &str = "timer_period_s";

/// Publishes telemetry and owns the associated business logic state.
///
/// This component is instantiated during Configure and released during Cleanup/Shutdown.
struct PublisherComponent {
    publisher: ManagedPublisher<MsgCount>,
    core: Mutex<TelemetryPublisherCore>,
    tick_failed: AtomicBool,
}

impl PublisherComponent {
    /// Creates and configures the publisher component.
    fn new(node: &LifecycleNode) -> Result<Self, RclrsError> {
        let logger = node.node().logger();

        let raw_node = node.node();
        let topic_name = topics::telemetry(raw_node);
        let qos_profile = qos::telemetry(raw_node);

        let publisher = node.publisher::<MsgCount>(&topic_name).qos(qos_profile).create().rcl_generic()?;

        let core = Mutex::new(TelemetryPublisherCore::new());

        log_info!(logger, "Component configured: publisher on '{}'", topic_name);

        Ok(Self { publisher, core, tick_failed: AtomicBool::new(false), })
    }

    /// Publishes the next message produced by the core.
    ///
    /// Lock failures are treated as non-fatal to avoid terminating the timer callback.
    fn publish(&self) -> Result<(), RclrsError> {
        let Ok(mut core) = self.core.lock() else {
            return Ok(());
        };

        let msg = core.next_message();

        self.publisher.publish(msg).rcl_generic()?;

        Ok(())
    }
}

/// Container for resources managed across lifecycle transitions.
#[derive(Default)]
struct ResourceState {
    component: Option<Arc<PublisherComponent>>,
    timer: Option<ManagedTimer>,
    watcher: Option<ParameterWatcher>,
    params: Option<ParameterNode>,
    last_period_ms: u64,
}

/// Lifecycle-managed publisher node.
struct Lesson06PublisherNode {
    state: Arc<Mutex<ResourceState>>,
}

impl Lesson06PublisherNode {
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

    /// Declares parameters and retrieves the initial timer period (seconds).
    fn _declare_parameters(params: &ParameterNode, logger: &Logger) -> f64 {
        let desc = Descriptor { description: "Timer period in seconds".to_string(), ..Descriptor::default() };

        if let Err(e) = params.declare(PERIOD_PARAM, Type::Double, Value::Double(1.0), desc) {
            log_error!(logger, "Parameter declaration failed: {}. Using default.", e);
            return 1.0;
        }

        let keys = [PERIOD_PARAM.to_string()];
        let store_arc = params.store();
        let store = store_arc.lock().unwrap();

        store.get(&keys).first().and_then(|v| match v { 
            Value::Double(v) => Some(*v), 
            Value::Integer(v) => Some(*v as f64), 
            _ => None, 
        }).unwrap_or(1.0)
    }

    /// Creates a gated repeating timer.
    ///
    /// The callback is executed only when the lifecycle node is Active.
    fn _create_gated_timer(node: &LifecycleNode, comp: &Arc<PublisherComponent>, period_ms: u64) -> Result<ManagedTimer, RclrsError> {
        let comp_clone = Arc::clone(comp);
        let logger = node.node().logger().clone();

        node.timer_repeating(Duration::from_millis(period_ms)).callback(move || {
           if let Err(e) = comp_clone.publish() {
                // Failure is logged once until the first subsequent success.
                if !comp_clone.tick_failed.swap(true, Ordering::Relaxed) {
                    log_error!(&logger, "Tick failed: {}", e);
                }
            } else {
                comp_clone.tick_failed.store(false, Ordering::Relaxed);
            }
        }).create().rcl_generic()
    }

    /// Handles runtime parameter updates for the timer period.
    fn _on_param_change(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, change: &ParameterChange) {
        if change.name != PERIOD_PARAM {
            return;
        }

        let new_period_s = match &change.new_value {
            Value::Double(v) => *v,
            Value::Integer(v) => *v as f64,
            _ => return,
        };

        // Negative or zero periods are invalid.
        if new_period_s <= 0.0 { return; }

        let Ok(ms) = period_s_to_ms_strict(new_period_s) else { return; };

        Self::_on_param_update(node, state, comp, ms);
    }

    /// Applies a validated timer period update.
    fn _on_param_update(node: &LifecycleNode, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, new_ms: u64) {
        let Ok(mut guard) = state.lock() else {
            return;
        };

        // No change.
        if guard.last_period_ms == new_ms { return; }

        match Self::_create_gated_timer(node, comp, new_ms) {
            Ok(timer) => {
                guard.timer = Some(timer);
                guard.last_period_ms = new_ms;
                log_info!(node.node().logger(), "Timer updated to {}ms", new_ms);
            }
            Err(e) => {
                log_error!(node.node().logger(), "Timer update failed: {}", e);
            }
        }
    }

    /// Resets all dynamically managed resources.
    fn reset_state(state: &Arc<Mutex<ResourceState>>) {
        // Lock failure is non-fatal.
        let Ok(mut guard) = state.lock() else { return; };

        guard.watcher = None;
        guard.timer = None;
        guard.component = None;
        guard.params = None;
        guard.last_period_ms = 0;
    }
}

impl LifecycleCallbacksWithNode for Lesson06PublisherNode {
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

        let initial_val_s = Self::_declare_parameters(&params, logger);
        let initial_ms = match period_s_to_ms_strict(initial_val_s) {
            Ok(ms) => ms,
            Err(e) => {
                log_error!(logger, "Invalid {}: {}. Defaulting to 1.0s", PERIOD_PARAM, e);
                1000
            }
        };

        let comp = match PublisherComponent::new(node) {
            Ok(c) => Arc::new(c),
            Err(e) => {
                log_error!(logger, "Component creation failed: {}", e);
                return CallbackResult::Failure;
            }
        };

        let timer = match Self::_create_gated_timer(node, &comp, initial_ms) {
            Ok(t) => t,
            Err(e) => {
                log_error!(logger, "Timer creation failed: {}", e);
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
        let comp_clone = Arc::clone(&comp);

        watcher.on_change(PERIOD_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, &comp_clone, change);
        });

        let Ok(mut guard) = self.state.lock() else {
            log_error!(logger, "State lock failure during configuration");
            return CallbackResult::Failure;
        };

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
    let lifecycle_node = Lesson06PublisherNode::create(&executor)?;

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
