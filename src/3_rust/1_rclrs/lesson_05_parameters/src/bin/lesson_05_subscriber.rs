use std::sync::{Arc, Mutex};

use rclrs::{
    log_error, log_info, log_warn, parameter::Parameter, Context, CreateBasicExecutor, Executor,
    Logger, Node, RclrsError, SpinOptions, RclrsErrorFilter,
};
use rcl_interfaces::msg::SetParametersResult;

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics, utils};

// Pure logic library (shared with publisher)
use lesson_05_parameters_rclrs::{StreamEvent, TelemetryStreamValidator};

const NODE_NAME: &str = "lesson_05_subscriber";
const RESET_MAX_PARAM: &str = "reset_max_value";

/// Encapsulates the subscription and its stream-validation logic.
///
/// Thread-safety: shared between the subscription callback (data plane) and the
/// parameter callback (control plane), so the validator is protected by a Mutex.
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

        // Shared validator state (updated in-place via parameter callback).
        let validator = Arc::new(Mutex::new(TelemetryStreamValidator::new(reset_max_value)));

        // Use explicit options to keep the “production pattern” consistent with other lessons.
        let mut options = rclrs::SubscriptionOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        let sub = Self::create_subscription(node, options, logger.clone(), Arc::clone(&validator))?;

        Ok(Self { _sub: sub, validator, _logger: logger })
    }

    fn create_subscription(node: &Node, options: rclrs::SubscriptionOptions,
        logger: Logger, validator: Arc<Mutex<TelemetryStreamValidator>>, ) -> Result<rclrs::Subscription<MsgCount>, RclrsError> {
        node.create_subscription::<MsgCount, _>(options, move |msg: MsgCount| {
            // Lock validator to ensure correctness during concurrent parameter updates.
            let mut v = match validator.lock() {
                Ok(g) => g,
                Err(_) => return, // poisoned: drop sample and continue
            };

            Self::on_msg(&logger, &mut v, &msg);
        })
    }

    fn on_msg(logger: &Logger, v: &mut TelemetryStreamValidator, msg: &MsgCount) {
        // MsgCount.count is i64 (per interface), preserve sign and semantics.
        let decision = v.on_count(msg.count);

        match decision.event {
            StreamEvent::Reset | StreamEvent::OutOfOrder => log_warn!(logger, "{}", decision.message),
            StreamEvent::Valid => log_info!(logger, "{}", decision.message),
        }
    }
}

/// Resource Container: manages lifecycle, configuration, and the control-plane callback.
///
/// This node demonstrates "in-place" parameter updates: we mutate the validator behind a
/// Mutex without rebuilding the ROS subscription resource.
struct Lesson05SubscriberNode {
    pub node: Node,
    _subscriber_component: Arc<SubscriberComponent>,
}

impl Lesson05SubscriberNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        // 1) Declare/get parameter (via utils) so configuration remains centralized.
        let reset_max = utils::get_or_declare_parameter(&node, RESET_MAX_PARAM, 1_i64, "parameter");

        // 2) Build subscriber component (owns subscription + validator state).
        let sub = Arc::new(SubscriberComponent::new(&node, reset_max)?);

        // 3) Register parameter callback for in-place validator updates.
        // Capture only what is required and keep ownership explicit.
        let validator_clone = Arc::clone(&sub.validator);
        let node_clone = node.clone();

        node.add_on_set_parameters_callback(move |params| {Self::on_parameters(&node_clone, &validator_clone, params)});

        log_info!(node.logger(), "Lesson 05 Subscriber started.");
        Ok(Self { node, _subscriber_component: sub })
    }

    fn on_parameters(node: &Node, validator: &Arc<Mutex<TelemetryStreamValidator>>, params: &[Parameter] ) -> SetParametersResult {
        // Fast-path: ignore unrelated parameters.
        let Some(param) = params.iter().find(|p| p.name == RESET_MAX_PARAM) else {
            return SetParametersResult { successful: true, reason: String::new() };
        };

        Self::handle_limit_update(node, validator, param)
    }

    /// Validates and applies the new reset tolerance limit.
    ///
    /// Update is applied in-place (no subscription rebuild) to keep the data plane stable.
    fn handle_limit_update(node: &Node, validator: &Arc<Mutex<TelemetryStreamValidator>>, param: &Parameter) -> SetParametersResult {
        let val = match param.get_as::<i64>() {
            Ok(v) => v,
            Err(_) => {
                return SetParametersResult {
                    successful: false,
                    reason: "reset_max_value must be an integer".to_string(),
                }
            }
        };

        if val < 0 {
            return SetParametersResult {
                successful: false,
                reason: "reset_max_value must be >= 0".to_string(),
            };
        }

        let mut v = match validator.lock() {
            Ok(g) => g,
            Err(_) => {
                return SetParametersResult {
                    successful: false,
                    reason: "validator state unavailable".to_string(),
                }
            }
        };

        v.set_reset_max_value(val);
        log_info!(node.logger(), "Updated reset_max_value -> {}", val);

        SetParametersResult { successful: true, reason: String::new() }
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
