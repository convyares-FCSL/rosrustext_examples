use rclrs::{log_info, Context, CreateBasicExecutor, RclrsError, RclrsErrorFilter, SpinOptions};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_00_bootstrap")?;
    
    // rclrs provides logging macros similar to rclcpp/rclpy
    log_info!(node.logger(), "Lesson 00 bootstrap node started.");
    
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
