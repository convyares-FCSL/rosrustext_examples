use rclrs::{Context, RclrsError, SpinOptions};
use std_msgs::msg::String as StringMsg;
use utils_rclrs::topics;

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_03_subscriber")?;

    let _subscription = node.create_subscription(
        topics::CHATTER,
        |msg: StringMsg| {
            println!("Heard: {}", msg.data);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
