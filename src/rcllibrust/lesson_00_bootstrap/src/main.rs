#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let _ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    println!("Lesson 00 bootstrap client connected.");

    tokio::signal::ctrl_c().await?;
    Ok(())
}
