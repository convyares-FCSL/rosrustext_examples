use std::time::Duration;

use rclrs::{log_info, Context, CreateBasicExecutor, RclrsError, RclrsErrorFilter, SpinOptions};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_01_node")?;

    let mut count: u64 = 0;
    let _timer = node.create_timer_repeating(Duration::from_millis(500), move || {
        log_info!("lesson_01_node", "Tick {}", count);
        count += 1;
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
