use roslibrust::rosbridge::ClientHandle;

const ROSBRIDGE_URL: &str = "ws://localhost:9090";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging from RUST_LOG (e.g. RUST_LOG=info).
    env_logger::init();

    // Run the program and report any failure once at the boundary.
    run_once().await.map_err(|err| {
        eprintln!("lesson_00_bootstrap_rcllibrust: {err}");
        err
    })
}

async fn run_once() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to rosbridge (WebSocket JSON API gateway to ROS).
    let ros: ClientHandle = ClientHandle::new(ROSBRIDGE_URL).await?;

    // Log an informational message indicating the client has connected.
    log::info!("Lesson 00 bootstrap client connected to {}.", ROSBRIDGE_URL);

    // Explicitly drop the handle to close the connection before exiting.
    drop(ros);

    // Return success.
    Ok(())
}
