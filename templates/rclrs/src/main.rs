use std::time::Duration;

mod qos;
mod services;
mod topics;

use rclrs::{Context, RclrsError, SpinOptions};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_node")?;

    let mut count: u64 = 0;
    let _timer = node.create_timer_repeating(Duration::from_millis(500), move || {
        println!("Tick {}", count);
        count += 1;
    })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
