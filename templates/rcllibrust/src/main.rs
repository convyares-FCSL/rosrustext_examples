mod qos;
mod services;
mod topics;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let _ros = roslibrust::rosbridge::ClientHandle::new("ws://localhost:9090").await?;
    tokio::signal::ctrl_c().await?;
    Ok(())
}
