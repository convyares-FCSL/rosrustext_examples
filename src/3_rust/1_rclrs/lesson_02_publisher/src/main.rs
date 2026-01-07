use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use rclrs::{log_error, Context, CreateBasicExecutor, RclrsError, RclrsErrorFilter, SpinOptions};
use std_msgs::msg::String as StringMsg;
use utils_rclrs::{qos, topics};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_02_publisher")?;

    let publisher = node.create_publisher::<StringMsg>(
        &topics::from_params(&node, "chatter", "/tutorial/chatter"),
        qos::from_parameters(&node),
    )?;
    let count = Arc::new(AtomicU64::new(0));
    let count_for_timer = Arc::clone(&count);

    let _timer = node.create_timer_repeating(Duration::from_millis(500), move || {
        let next = count_for_timer.fetch_add(1, Ordering::Relaxed);
        let msg = StringMsg {
            data: format!("Hello {}", next),
        };
        if let Err(err) = publisher.publish(msg) {
            log_error!("lesson_02_publisher", "Publish error: {err}");
        }
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
