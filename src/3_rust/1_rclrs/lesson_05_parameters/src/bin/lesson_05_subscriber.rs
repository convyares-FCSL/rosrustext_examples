use std::sync::{atomic::{AtomicI64, Ordering},Arc, Mutex};
use std::time::Duration;

use rclrs::{
    log_error, log_info, log_warn, Context, CreateBasicExecutor, Executor, Logger, Node, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics, utils};

// Pure logic library (shared with publisher)
use lesson_05_logic::{StreamEvent, TelemetryStreamValidator};

const NODE_NAME: &str = "lesson_05_subscriber";
const RESET_MAX_PARAM: &str = "reset_max_value";

/// Hotfix:
/// rclrs v0.6.x does not expose on-set parameter callbacks.
/// Until that lands (or rosrustext bridges it), we poll parameters at a fixed rate.
///
/// NOTE: polling is control-plane only; data-plane remains event-driven.
const PARAM_POLL_PERIOD: Duration = Duration::from_secs(1);

/// Encapsulates the subscription and its stream-validation logic.
///
/// Thread-safety: shared between the subscription callback (data plane) and the
/// parameter polling loop (control plane), so the validator is protected by a Mutex.
struct SubscriberComponent {
    _sub: rclrs::Subscription<MsgCount>,
    validator: Arc<Mutex<TelemetryStreamValidator>>,
    _logger: Logger, // RAII keep-alive
}

impl SubscriberComponent {
    fn new(node: &Node, reset_max_value: i64) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        // Topic/QoS are centralized (same pattern as publisher).
        let topic_name = topics::telemetry(node);
        let qos_profile = qos::telemetry(node);

        // Shared validator state (updated in-place via polling loop).
        let validator = Arc::new(Mutex::new(TelemetryStreamValidator::new(reset_max_value)));

        // Use explicit options to keep the “production pattern” consistent with other lessons.
        let mut options = rclrs::SubscriptionOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        let sub = Self::create_subscription(node, options, logger.clone(), Arc::clone(&validator))?;

        Ok(Self { _sub: sub, validator, _logger: logger })
    }

    fn create_subscription( node: &Node, options: rclrs::SubscriptionOptions, logger: Logger, 
        validator: Arc<Mutex<TelemetryStreamValidator>>, ) -> Result<rclrs::Subscription<MsgCount>, RclrsError> {
        node.create_subscription::<MsgCount, _>(options, move |msg: MsgCount| {
            // Lock validator to ensure correctness during concurrent control-plane updates.
            let mut v = match validator.lock() {
                Ok(g) => g,
                Err(_) => return, // poisoned: drop sample and continue
            };

            Self::on_msg(&logger, &mut v, &msg);
        })
    }

    fn on_msg(logger: &Logger, v: &mut TelemetryStreamValidator, msg: &MsgCount) {
        let decision = v.on_count(msg.count);

        match decision.event {
            StreamEvent::Reset | StreamEvent::OutOfOrder => log_warn!(logger, "{}", decision.message),
            StreamEvent::Valid => log_info!(logger, "{}", decision.message),
        }
    }
}

/// Resource Container: owns lifecycle, configuration, and the control-plane polling resource.
///
/// This node demonstrates "in-place update" behaviour: we mutate the validator behind a
/// Mutex when the parameter changes (subscription remains stable).
struct Lesson05SubscriberNode {
    pub node: Node,
    _subscriber_component: Arc<SubscriberComponent>,

    // Hotfix polling timer keep-alive.
    _param_poll_timer: rclrs::Timer,
}

/// Small context object captured by the polling timer closure.
struct ParamPollCtx {
    validator: Arc<Mutex<TelemetryStreamValidator>>,
    reset_param: rclrs::MandatoryParameter<i64>,
    last_reset_max: AtomicI64,
}

impl Lesson05SubscriberNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;
        let logger = node.logger().clone();

        // 1) Declare parameter once and keep a handle for polling.
        //
        // This is the rclrs-friendly equivalent of "declare if missing, otherwise get".
        let reset_param = utils::declare_parameter(&node, RESET_MAX_PARAM, 1_i64)?;

        // 2) Read + sanitise initial value (robust and boring).
        let mut initial_reset_max: i64 = reset_param.get();
        if initial_reset_max < 0 {
            log_error!( &logger, "Invalid initial {}='{}' (must be >= 0). Defaulting to 0.", RESET_MAX_PARAM, initial_reset_max );
            initial_reset_max = 0;
        }

        // 3) Build subscriber component (owns subscription + validator state).
        let sub = Arc::new(SubscriberComponent::new(&node, initial_reset_max)?);

        // 4) Hotfix adapter: poll the reset parameter and apply updates when it changes.
        let param_poll_timer = Self::_create_param_timer( &node, Arc::clone(&sub.validator), reset_param, initial_reset_max )?;

        log_info!(&logger, "Lesson 05 Subscriber started.");
        Ok(Self { node, _subscriber_component: sub, _param_poll_timer: param_poll_timer })
    }

    fn _create_param_timer( node: &Node, validator: Arc<Mutex<TelemetryStreamValidator>>, 
        reset_param: rclrs::MandatoryParameter<i64>, initial_reset_max: i64 ) -> Result<rclrs::Timer, RclrsError> {
        // Avoid "move out of node because it is borrowed" by using one clone for the call
        // and another clone for the closure.
        let node_for_timer = node.clone();
        let node_for_cb = node.clone();

        let ctx = Arc::new(ParamPollCtx { validator, reset_param, last_reset_max: AtomicI64::new(initial_reset_max) });

        node_for_timer.create_timer_repeating(PARAM_POLL_PERIOD, move || {
            Self::_check_param(&node_for_cb, &ctx);
        })
    }

    fn _check_param(node: &Node, ctx: &ParamPollCtx) {
        let current = ctx.reset_param.get();

        // Fast-path: no change.
        let last = ctx.last_reset_max.load(Ordering::Relaxed);
        if current == last {
            return;
        }

        // Validate before applying. If invalid, keep the last good configuration.
        if let Err(reason) = Self::on_param_change(node, &ctx.validator, current) {
            log_error!( node.logger(), "Ignoring update {}='{}': {}", RESET_MAX_PARAM, current, reason );
            return;
        }

        ctx.last_reset_max.store(current, Ordering::Relaxed);
    }

    /// Callback-shaped handler: validate + apply a new reset tolerance.
    ///
    /// Written as if it were called by a real on-set-parameters callback.
    /// The hotfix polling loop calls this when it detects a change.
    fn on_param_change( node: &Node, validator: &Arc<Mutex<TelemetryStreamValidator>>, new_reset_max: i64, ) -> Result<(), &'static str> {
        if new_reset_max < 0 {
            return Err("must be >= 0");
        }

        let mut v = match validator.lock() {
            Ok(g) => g,
            Err(_) => return Err("validator state unavailable"),
        };

        v.set_reset_max_value(new_reset_max);
        log_info!(node.logger(), "Updated {} -> {}", RESET_MAX_PARAM, new_reset_max);
        Ok(())
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
