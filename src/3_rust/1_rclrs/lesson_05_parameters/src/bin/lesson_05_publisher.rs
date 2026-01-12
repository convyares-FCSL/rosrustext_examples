use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc, Mutex,
};
use std::time::Duration;

use rclrs::{
    log_error, log_info, Context, CreateBasicExecutor, Executor, Logger, Node, Publisher, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics, utils};

// Pure logic library (shared with subscriber)
use lesson_05_logic::{period_s_to_ms_strict, TelemetryPublisherCore};

const NODE_NAME: &str = "lesson_05_publisher";
const PERIOD_PARAM: &str = "timer_period_s";

/// Hotfix:
/// rclrs v0.6.x does not expose on-set parameter callbacks.
/// Until that lands (or rosrustext bridges it), we poll parameters at a fixed rate.
const PARAM_POLL_PERIOD: Duration = Duration::from_secs(1);

/// Encapsulates the publisher and its business logic.
///
/// Thread-safety: shared between the timer callback (data plane) and the
/// parameter polling loop (control plane), so the core is protected by a Mutex.
struct PublisherComponent {
    publisher: Publisher<MsgCount>,
    core: Mutex<TelemetryPublisherCore>,
    _logger: Logger, // RAII keep-alive

    // Avoid log spam: only emit the first tick error until a success occurs.
    tick_failed: AtomicBool,
}

impl PublisherComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();
        let publisher = Self::create_publisher(node)?;
        let core = Mutex::new(TelemetryPublisherCore::new());

        Ok(Self {
            publisher,
            core,
            _logger: logger,
            tick_failed: AtomicBool::new(false),
        })
    }

    fn create_publisher(node: &Node) -> Result<Publisher<MsgCount>, RclrsError> {
        let topic_name = topics::telemetry(node);
        let qos_profile = qos::telemetry(node);

        let mut options = rclrs::PublisherOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        node.create_publisher::<MsgCount>(options)
    }

    fn on_tick(&self) -> Result<(), RclrsError> {
        let mut core = match self.core.lock() {
            Ok(g) => g,
            Err(_) => return Ok(()), // poisoned: skip tick and continue
        };

        let msg = core.next_message();
        self.publisher.publish(&msg)?;
        Ok(())
    }
}

/// Small context object captured by the polling timer closure.
struct ParamPollCtx {
    timer_store: Arc<Mutex<Option<rclrs::Timer>>>,
    comp: Arc<PublisherComponent>,
    period_param: rclrs::MandatoryParameter<f64>,
    last_period_ms: AtomicU64,
}

/// Resource Container: owns lifecycle, configuration, and the mutable timer resource.
///
/// This node demonstrates "rebuild-on-update" behaviour: we replace the publish timer
/// when the period parameter changes (publisher + core remain stable).
struct Lesson05PublisherNode {
    pub node: Node,
    _publisher_component: Arc<PublisherComponent>,

    // Publish timer is hot-swapped when period changes.
    _timer: Arc<Mutex<Option<rclrs::Timer>>>,

    // Hotfix polling timer keep-alive.
    _param_poll_timer: rclrs::Timer,
}

impl Lesson05PublisherNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let logger = node.logger().clone();

        // Declare once and keep a handle for polling.
        // This avoids node.get_parameter() (not in rclrs v0.6.x) and avoids double-declare.
        let period_param = utils::declare_parameter(&node, PERIOD_PARAM, 1.0_f64)?;
        let initial_period_s = period_param.get();
        
        // Be boring: if invalid, log and fall back to 1.0s.
        let initial_period_ms = match period_s_to_ms_strict(initial_period_s) {
            Ok(ms) => ms,
            Err(e) => {
                log_error!( &logger, "Invalid initial {}='{}': {}. Defaulting to 1.0s.", PERIOD_PARAM, initial_period_s, e );
                1000_u64
            }
        };

        let comp = Arc::new(PublisherComponent::new(&node)?);

        let timer_store: Arc<Mutex<Option<rclrs::Timer>>> = Arc::new(Mutex::new(None));

        // Initial publish timer.
        Self::create_or_update_timer_ms(&node, &timer_store, Arc::clone(&comp), initial_period_ms)?;

        // Hotfix adapter: poll parameter and apply updates when it changes.
        let param_poll_timer = Self::_create_param_timer( &node, Arc::clone(&timer_store), Arc::clone(&comp), period_param, initial_period_ms )?;

        log_info!(&logger, "Lesson 05 Publisher started.");
        Ok(Self { node, _publisher_component: comp, _timer: timer_store, _param_poll_timer: param_poll_timer })
    }

    fn create_or_update_timer_ms( node: &Node, timer_store: &Arc<Mutex<Option<rclrs::Timer>>>, comp: Arc<PublisherComponent>, period_ms: u64 ) -> Result<(), RclrsError> {
        let logger = node.logger().clone();

        // Create timer before swapping it in.
        let tick_logger = logger.clone();
        let new_timer = node.create_timer_repeating(Duration::from_millis(period_ms), move || {
            match comp.on_tick() {
                Ok(_) => {
                    comp.tick_failed.store(false, Ordering::Relaxed);
                }
                Err(e) => {
                    if !comp.tick_failed.swap(true, Ordering::Relaxed) {
                        log_error!(&tick_logger, "Tick failed: {}", e);
                    }
                }
            }
        })?;

        // Swap it in; drop old timer when replaced.
        let mut guard = match timer_store.lock() {
            Ok(g) => g,
            Err(_) => {
                log_error!(node.logger(), "timer_store mutex poisoned; keeping existing timer");
                return Ok(());
            }
        };
        guard.replace(new_timer);

        log_info!( &logger, "Timer updated to {:.3}s", (period_ms as f64) / 1000.0 );
        Ok(())
    }

    /// Hotfix adapter: poll the period parameter and apply updates when it changes.
    fn _create_param_timer( node: &Node, timer_store: Arc<Mutex<Option<rclrs::Timer>>>, comp: Arc<PublisherComponent>,
        period_param: rclrs::MandatoryParameter<f64>, initial_period_ms: u64 ) -> Result<rclrs::Timer, RclrsError> {
        // Same borrow/move fix as subscriber: one clone for the call, one for the closure.
        let node_for_timer = node.clone();
        let node_for_cb = node.clone();

        let ctx = Arc::new(ParamPollCtx { timer_store, comp, period_param, last_period_ms: AtomicU64::new(initial_period_ms) });

        node_for_timer.create_timer_repeating(PARAM_POLL_PERIOD, move || {
            Self::_check_param(&node_for_cb, &ctx);
        })
    }

    fn _check_param(node: &Node, ctx: &ParamPollCtx) {
        let current_s = ctx.period_param.get();

        let current_ms = match period_s_to_ms_strict(current_s) {
            Ok(ms) => ms,
            Err(e) => {
                log_error!( node.logger(), "Ignoring update {}='{}': {}", PERIOD_PARAM, current_s, e );
                return;
            }
        };

        let last_ms = ctx.last_period_ms.load(Ordering::Relaxed);
        if current_ms == last_ms {
            return;
        }

        match Self::on_param_change(node, &ctx.timer_store, &ctx.comp, current_ms) {
            Ok(applied_ms) => {
                ctx.last_period_ms.store(applied_ms, Ordering::Relaxed);
            }
            Err(e) => {
                log_error!( node.logger(), "Ignoring update {}='{}': {}", PERIOD_PARAM, current_s, e );
            }
        }
    }

    /// Callback-shaped handler: apply a new publish period (already validated to ms).
    fn on_param_change( node: &Node, timer_store: &Arc<Mutex<Option<rclrs::Timer>>>, comp: &Arc<PublisherComponent>, new_period_ms: u64, ) -> Result<u64, RclrsError> {
        Self::create_or_update_timer_ms(node, timer_store, Arc::clone(comp), new_period_ms)?;
        Ok(new_period_ms)
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
            log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}
