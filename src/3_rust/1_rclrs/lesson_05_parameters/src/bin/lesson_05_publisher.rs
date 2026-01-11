use std::sync::{Arc, Mutex};
use std::time::Duration;

use rcl_interfaces::msg::SetParametersResult;
use rclrs::{
    log_error, log_info, parameter::Parameter, Context, CreateBasicExecutor, Executor, Logger,
    Node, Publisher, RclrsError, SpinOptions,RclrsErrorFilter,
};

// Workspace deps
use lesson_interfaces::msg::MsgCount;
use utils_rclrs::{qos, topics, utils};

// Pure logic library (shared with subscriber)
use lesson_05_parameters_rclrs::TelemetryPublisherCore;

const NODE_NAME: &str = "lesson_05_publisher";
const PERIOD_PARAM: &str = "timer_period_s";

/// Encapsulates the publisher and its business logic.
///
/// Thread-safety: shared between the timer callback (data plane) and the
/// parameter callback (control plane), so the core is protected by a Mutex.
struct PublisherComponent {
    publisher: Publisher<MsgCount>,
    core: Mutex<TelemetryPublisherCore>,
    _logger: Logger, // RAII keep-alive
}

impl PublisherComponent {
    fn new(node: &Node) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        // Topic/QoS are centralized (same pattern as subscriber).
        let topic_name = topics::telemetry(node);
        let qos_profile = qos::telemetry(node);

        // Use explicit options to keep the “production pattern” consistent with other lessons.
        let mut options = rclrs::PublisherOptions::new(topic_name.as_str());
        options.qos = qos_profile;

        let publisher = Self::create_publisher(node, options)?;
        let core = Mutex::new(TelemetryPublisherCore::new());

        Ok(Self { publisher, core, _logger: logger })
    }

    fn create_publisher(node: &Node, options: rclrs::PublisherOptions ) -> Result<Publisher<MsgCount>, RclrsError> {
        node.create_publisher::<MsgCount>(options)
    }

    fn on_tick(&self) -> Result<(), RclrsError> {
        // Lock core to ensure correctness during concurrent parameter updates.
        let mut core = match self.core.lock() {
            Ok(g) => g,
            Err(_) => return Ok(()), // poisoned: skip tick and continue
        };

        let msg = core.next_message();
        self.publisher.publish(&msg)?;
        Ok(())
    }
}

/// Resource Container: owns lifecycle, configuration, and the mutable timer resource.
///
/// This node demonstrates "rebuild-on-update" behaviour: we replace the timer resource
/// when the period parameter changes (publisher + core remain stable).
struct Lesson05PublisherNode {
    pub node: Node,
    _publisher_component: Arc<PublisherComponent>,
    _timer: Arc<Mutex<Option<rclrs::Timer>>>,
}

impl Lesson05PublisherNode {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        // 1) Declare/get parameter (via utils) so configuration remains centralized.
        let initial_period_s = utils::get_or_declare_parameter(&node, PERIOD_PARAM, 1.0, "parameter");

        // 2) Build publisher component (owns publisher + core).
        let comp = Arc::new(PublisherComponent::new(&node)?);

        // 3) Create mutable timer store for atomic replacement.
        let timer_store: Arc<Mutex<Option<rclrs::Timer>>> = Arc::new(Mutex::new(None));

        // 4) Initial timer creation (validated/sanitised).
        Self::rebuild_timer(&node, &timer_store, Arc::clone(&comp), initial_period_s)?;

        // 5) Register parameter callback for timer period updates.
        // Capture only what is required and keep ownership explicit.
        let node_clone = node.clone();
        let timer_clone = Arc::clone(&timer_store);
        let comp_clone = Arc::clone(&comp);

        node.add_on_set_parameters_callback(move |params| {
            Self::on_parameters(&node_clone, &timer_clone, &comp_clone, params)
        });

        log_info!(node.logger(), "Lesson 05 Publisher started.");
        Ok(Self { node, _publisher_component: comp, _timer: timer_store })
    }

    fn rebuild_timer(node: &Node, timer_store: &Arc<Mutex<Option<rclrs::Timer>>>, comp: Arc<PublisherComponent>, period_s: f64 ) -> Result<(), RclrsError> {
        let period_s = Self::validate_period_or_default(node.logger(), period_s);

        // Create the timer *before* swapping it in, to avoid holding the mutex across creation.
        let logger = node.logger().clone();
        let new_timer = node.create_timer_repeating(
            Duration::from_secs_f64(period_s),
            move || {
                if let Err(e) = comp.on_tick() {
                    log_error!(logger, "Tick failed: {}", e);
                }
            },
        )?;

        {
            let mut guard = timer_store.lock().unwrap();
            *guard = Some(new_timer);
        }

        log_info!(node.logger(), "Timer updated to {:.2}s", period_s);
        Ok(())
    }

    fn validate_period_or_default(logger: &Logger, val: f64) -> f64 {
        if val.is_finite() && val > 0.0 {
            return val;
        }
        let fallback = 1.0;
        log_info!(logger, "Invalid timer period. Defaulting to {}s.", fallback);
        fallback
    }

    fn on_parameters(node: &Node, timer_store: &Arc<Mutex<Option<rclrs::Timer>>>, comp: &Arc<PublisherComponent>, params: &[Parameter]) -> SetParametersResult {
        // Fast-path: ignore unrelated parameters.
        let Some(param) = params.iter().find(|p| p.name == PERIOD_PARAM) else {
            return SetParametersResult { successful: true, reason: String::new() };
        };

        Self::handle_period_update(node, timer_store, comp, param)
    }

    /// Validates and applies the new timer period.
    ///
    /// Update is applied by rebuilding the timer resource to keep the data plane stable.
    fn handle_period_update(node: &Node, timer_store: &Arc<Mutex<Option<rclrs::Timer>>>, comp: &Arc<PublisherComponent>, param: &Parameter ) -> SetParametersResult {
        let val = match param.get_as::<f64>() {
            Ok(v) => v,
            Err(_) => {
                return SetParametersResult {
                    successful: false,
                    reason: "timer_period_s must be a double".to_string(),
                }
            }
        };

        if !val.is_finite() || val <= 0.0 {
            return SetParametersResult {
                successful: false,
                reason: "timer_period_s must be finite and > 0.0".to_string(),
            };
        }

        match Self::rebuild_timer(node, timer_store, Arc::clone(comp), val) {
            Ok(_) => SetParametersResult { successful: true, reason: String::new() },
            Err(e) => SetParametersResult {
                successful: false,
                reason: format!("Failed to update timer: {}", e),
            },
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
            log_error!(node.node.logger(), "Executor stopped with error: {err}");
            err
        })?;

    Ok(())
}
