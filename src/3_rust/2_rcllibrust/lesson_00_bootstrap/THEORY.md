# Lesson 00 Breakdown: The Async Client

## Architecture: The Async Client Pattern

Unlike `rclrs` (which uses native DDS and a spin-loop), `rcllibrust` is an **Async Client**. It does not participate in the ROS discovery graph directly; instead, it connects to a gateway (`rosbridge_server`) via WebSockets.



This shifts the architecture from a "Polled Executor" model to an **"Event-Driven Async"** model powered by the **Tokio** runtime.

1.  **The Runtime**: We use `tokio` to manage asynchronous tasks (sending/receiving JSON packets).
2.  **The Handle**: We use a `ClientHandle` to represent the active connection.
3.  **RAII**: When the `ClientHandle` is dropped, the WebSocket connection is automatically closed.

---

## Code Walkthrough

### 1. The Async Entry Point (`main`)

In Rust, `main` functions cannot be `async` by default. We use the `#[tokio::main]` macro to wrap our code. This macro initializes the Tokio runtime and blocks until our async code finishes.

```rust
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Initialize Logging
    // Unlike rclrs, we don't use a ROS logger. We use standard Rust logging
    // which prints to stdout/stderr based on the RUST_LOG env var.
    env_logger::init();

    // 2. The Logic Boundary
    // We delegate the actual work to 'run_once'.
    // This allows us to catch and format errors cleanly at the top level
    // before the program exits.
    run_once().await.map_err(|err| {
        eprintln!("lesson_00_bootstrap_rcllibrust: {err}");
        err
    })
}

```

### 2. The Connection Logic (`run_once`)

This function demonstrates the "Connect -> Do Work -> Disconnect" lifecycle.

```rust
async fn run_once() -> Result<(), Box<dyn std::error::Error>> {
    // Step A: The Handshake
    // ClientHandle::new() initiates the WebSocket connection.
    // The 'await' keyword suspends this function until the connection
    // is fully established or fails.
    let ros: ClientHandle = ClientHandle::new(ROSBRIDGE_URL).await?;

    // Step B: The Work
    // At this point, we are connected. We can log success.
    log::info!("Lesson 00 bootstrap client connected to {}.", ROSBRIDGE_URL);

    // Step C: Cleanup (RAII)
    // The 'ros' variable owns the connection.
    // By calling drop(ros), we explicitly destroy the handle.
    // This sends a close signal to the WebSocket and frees resources.
    // (Note: Simply letting 'ros' go out of scope at the end of the function
    // would achieve the exact same thing, but we make it explicit here).
    drop(ros);

    Ok(())
}

```

### Key Concepts

* **`Box<dyn std::error::Error>`**: This is a "Catch-All" error type. Since network operations can fail in many ways (DNS error, Connection Refused, Timeout), we use a dynamic error type to bubble up *any* failure to the main function.
* **`await`**: This keyword is the heart of async Rust. It tells the program: "Pause this function here, go do other work (like handling heartbeats), and come back when this specific task (connecting) is finished."
* **No Spin Loop**: Notice there is no `spin()`. The `roslibrust` client handles the background listening loop for you automatically in a separate Tokio task spawned during `ClientHandle::new()`.