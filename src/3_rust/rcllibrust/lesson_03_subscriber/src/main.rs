include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use roslibrust::traits::{Ros, Subscribe};
use utils_rcllibrust::topics;

async fn subscribe_loop(ros: impl Ros) -> Result<(), Box<dyn std::error::Error>> {
    let mut subscriber = ros.subscribe::<std_msgs::String>(topics::CHATTER).await?;

    loop {
        let msg = subscriber.next().await;
        println!("Heard: {}", msg.data);
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    tokio::spawn(async move {
        if let Err(err) = subscribe_loop(ros).await {
            eprintln!("Subscriber task error: {err}");
        }
    });

    tokio::signal::ctrl_c().await?;
    Ok(())
}
