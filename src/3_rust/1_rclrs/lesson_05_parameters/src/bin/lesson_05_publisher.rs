use std::sync::{ atomic::{AtomicBool, Ordering}, Arc, Mutex, };
use std::time::Duration;

use rclrs::{ log_error, log_info, Context, CreateBasicExecutor, Executor, Logger, Node, Publisher, RclrsError, RclrsErrorFilter, SpinOptions, };

use rosrustext_rosrs::parameters::{ Descriptor, ParameterChange, ParameterNode, ParameterWatcher, Type, Value, };

use lesson_05_logic::{period_s_to_ms_strict, TelemetryPublisherCore};
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics};
use utils_rclrs::IntoRclrsError;

const NODE_NAME: &str = "lesson_05_publisher";
const PERIOD_PARAM: &str = "timer_period_s";

/// Publishes telemetry and owns the associated business logic state.
///
/// This component is passive: it provides publish functionality but does not manage timers
/// or configuration. It is referenced by timer callbacks and remains valid for the life
/// of the node process.
struct PublisherComponent {
    publisher: Publisher<MsgCount>,
    core: Mutex<TelemetryPublisherCore>,

    /// Prevents repeated log spam: logs the first tick failure until a success occurs.
    tick_failed: AtomicBool,
}

impl PublisherComponent {
    /// Creates and configures the publisher component.
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let publisher = Self::create_publisher(node)?;
        let core = Mutex::new(TelemetryPublisherCore::new());

        Ok(Self { publisher, core, tick_failed: AtomicBool::new(false), })
    }

    /// Creates the ROS publisher with configured topic name and QoS profile.
    fn create_publisher(node: &Node) -> Result<Publisher<MsgCount>, RclrsError> {
        let topic_name = topics::telemetry(node);
        let qos_profile = qos::telemetry(node);

        let mut options = rclrs::PublisherOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        node.create_publisher::<MsgCount>(options)
    }

    /// Publishes the next message produced by the core.
    ///
    /// Lock failures are treated as non-fatal to keep the timer callback resilient.
    fn publish(&self) -> Result<(), RclrsError> {
        let Ok(mut core) = self.core.lock() else {
            return Ok(());
        };

        let msg = core.next_message();
        self.publisher.publish(&msg)?;
        Ok(())
    }
}

/// Container for resources managed at runtime.
///
/// Lesson 05 replaces the publish timer when the period parameter changes.
#[derive(Default)]
struct ResourceState {
    /// Publish timer; replaced on period updates.
    timer: Option<rclrs::Timer>,

    /// Last applied period in milliseconds.
    last_period_ms: u64,
}

/// Parameter-driven publisher node.
///
/// Provides runtime reconfiguration of publish period using ParameterWatcher,
/// without lifecycle state management.
struct Lesson05PublisherNode {
    /// Shared node handle. Required because rosrustext ParameterNode binds to Arc<Node>.
    node: Arc<Node>,

    _component: Arc<PublisherComponent>,

    /// Mutable state updated by parameter change callbacks.
    _state: Arc<Mutex<ResourceState>>,

    /// Parameter plumbing and watcher keep-alives.
    _params: ParameterNode,
    _watcher: ParameterWatcher,
}

impl Lesson05PublisherNode {
    /// Creates the node, declares parameters, configures publisher, and starts timers.
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = Arc::new(executor.create_node(NODE_NAME)?);
        let logger = node.logger().clone();

        let params = ParameterNode::try_new(Arc::clone(&node)).rcl_generic()?;
        let initial_period_s = Self::_declare_parameters(&params, &logger);

        let initial_ms = match period_s_to_ms_strict(initial_period_s) {
            Ok(ms) => ms,
            Err(e) => {
                log_error!( &logger, "Invalid initial {}='{}': {}. Defaulting to 1.0s", PERIOD_PARAM, initial_period_s, e );
                1000_u64
            }
        };

        let component = Arc::new(PublisherComponent::new(node.as_ref())?);

        let state = Arc::new(Mutex::new(ResourceState { timer: None, last_period_ms: initial_ms, }));

        Self::create_or_replace_timer_ms( Arc::clone(&node), Arc::clone(&state), Arc::clone(&component), initial_ms )?;

        let watcher = ParameterWatcher::new(&params).rcl_generic()?;

        let node_clone = Arc::clone(&node);
        let state_clone = Arc::clone(&state);
        let comp_clone = Arc::clone(&component);

        watcher.on_change(PERIOD_PARAM, move |change| {
            Self::_on_param_change(&node_clone, &state_clone, &comp_clone, change);
        });

        log_info!(&logger, "Lesson 05 Publisher started.");

        Ok(Self { node, _component : component, _state : state, _params: params, _watcher: watcher, })
    }

    /// Declares parameters and retrieves the initial timer period (seconds).
    ///
    /// Mirrors the Lesson 06 `_declare_parameters` pattern.
    fn _declare_parameters(params: &ParameterNode, logger: &Logger) -> f64 {
        let desc = Descriptor {
            description: "Timer period in seconds".to_string(),
            ..Descriptor::default()
        };

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

    /// Creates a repeating publish timer and replaces any existing timer.
    ///
    /// The previous timer is dropped when replaced, stopping the prior schedule.
    fn create_or_replace_timer_ms( node: Arc<Node>, state: Arc<Mutex<ResourceState>>, component: Arc<PublisherComponent>, period_ms: u64, ) -> Result<(), RclrsError> {
        let tick_logger = node.logger().clone();

        let new_timer = node.create_timer_repeating(Duration::from_millis(period_ms), move || {
            match component.publish() {
                Ok(_) => {
                    component.tick_failed.store(false, Ordering::Relaxed);
                }
                Err(e) => {
                    if !component.tick_failed.swap(true, Ordering::Relaxed) {
                        log_error!(&tick_logger, "Tick failed: {}", e);
                    }
                }
            }
        })?;

        let Ok(mut guard) = state.lock() else {
            log_error!(node.logger(), "State lock failure during timer update");
            return Ok(());
        };

        guard.timer = Some(new_timer);
        guard.last_period_ms = period_ms;

        log_info!(node.logger(), "Timer updated to {}ms", period_ms);

        Ok(())
    }

    /// Validates a parameter change event and applies it if required.
    ///
    /// This function performs:
    /// - name filtering
    /// - type conversion
    /// - semantic validation
    /// - "no-op" detection
    ///
    /// If the update is valid and changes effective configuration, it delegates to
    /// `_on_param_update`.
    fn _on_param_change( node: &Arc<Node>, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, change: &ParameterChange, ) {
        if change.name != PERIOD_PARAM {
            return;
        }

        let new_period_s = match &change.new_value {
            Value::Double(v) => *v,
            Value::Integer(v) => *v as f64,
            _ => {
                log_error!( node.logger(), "Ignoring {} update: expected numeric type", PERIOD_PARAM );
                return;
            }
        };

        if new_period_s <= 0.0 {
            log_error!( node.logger(), "Ignoring {} update: value must be > 0.0", PERIOD_PARAM );
            return;
        }

        let Ok(new_ms) = period_s_to_ms_strict(new_period_s) else {
            log_error!(
                node.logger(),
                "Ignoring {} update: failed to convert to milliseconds",
                PERIOD_PARAM
            );
            return;
        };

        Self::_on_param_update(node, state, comp, new_ms);
    }

    /// Applies a validated timer period update.
    ///
    /// This recreates the publish timer and replaces the old one.
    fn _on_param_update( node: &Arc<Node>, state: &Arc<Mutex<ResourceState>>, comp: &Arc<PublisherComponent>, new_ms: u64, ) {
        let current_ms = {
            let Ok(guard) = state.lock() else { return; };
            guard.last_period_ms
        };

        if current_ms == new_ms {
            return;
        }

        if let Err(e) = Self::create_or_replace_timer_ms( Arc::clone(node), Arc::clone(state), Arc::clone(comp), new_ms, ) {
            log_error!(node.logger(), "Timer update failed: {}", e);
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = Lesson05PublisherNode::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            log_error!(node.node.logger(), "Executor error: {err}");
            err
        })?;

    Ok(())
}
