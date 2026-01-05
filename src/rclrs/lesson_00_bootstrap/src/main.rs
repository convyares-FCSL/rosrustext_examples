use rclrs::{Context, RclrsError, SpinOptions};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let _node = executor.create_node("lesson_00_bootstrap")?;

    println!("Lesson 00 bootstrap node started.");

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
