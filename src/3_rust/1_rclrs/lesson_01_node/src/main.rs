use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use rclrs::{
    log_info, Context, CreateBasicExecutor, Executor, MandatoryParameter, Node, RclrsError,
    RclrsErrorFilter, SpinOptions,
};

const NODE_NAME: &str = "lesson_01_node";

/// Struct representing the lesson 01 node.
/// We store handles so they stay alive for the lifetime of the process.
struct Lesson01Node {
    /// Node handle (kept alive; also useful for logging in main on errors).
    _node_handle: Node,

    /// Timer handle (must be kept alive; dropping it stops the timer).
    _timer_handle: rclrs::Timer,
}

/// Implementation of the lesson 01 node.
impl Lesson01Node {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node(NODE_NAME)?;

        // Declare parameter with a default; "mandatory" gives a handle with .get().
        let period_handle = node.declare_parameter("timer_period_s").default(1.0).mandatory()?;

        // Tick counter: shared, mutable state for callbacks.
        let tick = Arc::new(AtomicU64::new(0));

        // Create timer from the parameter value (read once for this lesson).
        let timer = create_timer_from_param(&node, period_handle, tick)?;

        log_info!(node.logger(), "Lesson 01 node started...");

        Ok(Self {
            _node_handle: node,
            _timer_handle: timer,
        })
    }
}

/// Create a repeating timer using the provided period parameter (read once here).
fn create_timer_from_param(node: &Node,timer_period_s: MandatoryParameter<f64>, tick: Arc<AtomicU64>) -> Result<rclrs::Timer, RclrsError> {
    let mut period_s = timer_period_s.get();

    if period_s <= 0.0 {
        period_s = 1.0;
        log_info!(node.logger(), "Invalid timer period specified. Defaulting to {} seconds.", period_s );
    }

    // Own clones in the callback so it is `'static`.
    let node_cb = node.clone();
    let tick_cb = tick.clone();

    let timer = node.create_timer_repeating(
        Duration::from_secs_f64(period_s), 
        move || { on_tick(&node_cb, &tick_cb);}
    )?;

    Ok(timer)
}

/// Timer callback helper.
fn on_tick(node: &Node, tick: &Arc<AtomicU64>) {
    let n = tick.fetch_add(1, Ordering::Relaxed) + 1;
    log_info!(node.logger(), "tick {}", n);
}

fn main() -> Result<(), RcslrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = Lesson01Node::new(&executor)?;

    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()
        .map_err(|err| {
            rclrs::log_error!( node._node_handle.logger(), "Executor stopped with error: {err}" );
            err
        })?;

    Ok(())
}
