# Lesson 02 Breakdown: Manual Message Mapping

## The Problem: No Colcon, No Generated Code
In the `rclrs` track, `colcon` builds the `lesson_interfaces` package, generates C++ code, and then `rosidl_generator_rs` generates Rust bindings.
In this **Cargo-only** track, we don't have access to that build pipeline. We are just a standard Rust binary talking JSON over a WebSocket.

## The Solution: Manual Definition (`serde`)
We must define a Rust struct that *looks exactly like* the JSON message `rosbridge` expects.

```rust
/// 1. Serde: Deriving Serialize lets us turn this struct into JSON.
///    Deriving Deserialize lets us turn JSON back into this struct.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MsgCount {
    pub count: i64,
}

/// 2. Trait Implementation: This tells roslibrust exactly what
///    ROS message type this struct represents.
impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
}

```

When you call `publisher.publish(&msg)`, `roslibrust`:

1. Checks `ROS_TYPE_NAME` to tell rosbridge what we are sending.
2. Uses `serde` to serialize the message into `{"count": 123}`.
3. Sends the JSON string over the WebSocket.

## Architecture: Async Publishing Component

We stick to the **Logic/Resource Separation** pattern, but with async twists and references.

```rust
struct PublisherChatter {
    publisher: Publisher<MsgCount>, // Holds the advertisement handle
    count: AtomicU64,
}

impl PublisherChatter {
    async fn on_tick(&self) {
        // ... logic ...
        let msg = MsgCount { count: n as i64 };
        
        // PUBLISHING IS ASYNC & BORROWED
        // 1. We pass &msg (borrowed) because the publisher doesn't need to own it.
        // 2. We 'await' the result because sending a network packet is async.
        if let Err(e) = self.publisher.publish(&msg).await {
            log::error!("Failed to publish: {}", e);
        }
    }
}

```

## Workspace Dependencies (`utils_roslibrust`)

Instead of defining constants locally, we import them from our shared workspace library to ensure consistency across lessons.

```rust
// Cargo.toml
// utils_roslibrust = { path = "../utils_roslibrust" }

use utils_roslibrust::{qos, topics};

```

* **`topics::CHATTER`**: Returns the topic string (`"/chatter"`).
* **`qos::get_options("telemetry")`**: Returns a `roslibrust::rosbridge::AdvertiseOptions` struct.
* This struct is specific to the Rosbridge Protocol.
* It handles fields like `queue_size` and `latching`, mapping our "telemetry" profile to the correct underlying settings.
