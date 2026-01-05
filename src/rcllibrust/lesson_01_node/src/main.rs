use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let _ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    let mut interval = tokio::time::interval(Duration::from_millis(500));
    let mut count: u64 = 0;

    loop {
        tokio::select! {
            _ = interval.tick() => {
                println!("Tick {}", count);
                count += 1;
            }
            _ = tokio::signal::ctrl_c() => break,
        }
    }

    Ok(())
}
