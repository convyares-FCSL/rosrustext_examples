use rclrs::{log_info, Context, CreateBasicExecutor, RclrsError, RclrsErrorFilter, SpinOptions};
use std_msgs::msg::String as StringMsg;
use utils_rclrs::{qos, topics};

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lesson_03_subscriber")?;

    let _subscription = node.create_subscription::<StringMsg, _>(
        &topics::from_params(&node, "chatter", "/tutorial/chatter"),
        qos::from_parameters(&node),
        move |msg: StringMsg| {
            log_info!("lesson_03_subscriber", "Heard: {}", msg.data);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
