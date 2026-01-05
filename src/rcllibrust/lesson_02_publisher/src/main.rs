include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use roslibrust::traits::{Publish, Ros};
use std::time::Duration;
use utils_rcllibrust::topics;

async fn publish_loop(ros: impl Ros) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = ros.advertise::<std_msgs::String>(topics::CHATTER).await?;

    let mut interval = tokio::time::interval(Duration::from_millis(500));
    let mut count: u64 = 0;

    loop {
        interval.tick().await;
        let msg = std_msgs::String {
            data: format!("Hello {}", count),
        };
        publisher.publish(&msg).await?;
        count += 1;
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    tokio::spawn(async move {
        if let Err(err) = publish_loop(ros).await {
            eprintln!("Publisher task error: {err}");
        }
    });

    tokio::signal::ctrl_c().await?;
    Ok(())
}
