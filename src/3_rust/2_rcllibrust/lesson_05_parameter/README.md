# Lesson 05 (Rust / roslibrust) – Parameters & Central Configuration

## Goal

Revisit the publisher–subscriber pair using **fully parameterised configuration** in a client-library context.

By the end of this lesson, the system:

* Loads **topic names and behaviour settings** from central YAML configuration files.
* Applies configuration at startup by parsing these files locally.
* Demonstrates how client nodes adapt to system configuration rather than hardcoding values.
* Uses the `utils_roslibrust` crate to standardize configuration loading.

This lesson establishes configuration as a **system-level concern**, decoupling the Rust client logic from the definitions on the ROS network.

> Assumes completion of Lessons 00–04.

---

## What’s New in Lesson 05

Compared to earlier lessons:

* **No hardcoded strings** for topics or timeout periods.
* **Client-Side Configuration**: The node reads the same YAML files as the ROS 2 nodes (Python/C++) to ensure consistency.
* **Centralized Logic**: Configuration is loaded from `lesson_interfaces/config`, maintaining a Single Source of Truth.

This is the point where the client becomes "smart"—it arrives at the network already knowing how to behave.

---

## Architecture Overview (roslibrust)

Lesson 05 uses a **Client-Side Loading** strategy.

* **Configuration (Layer 1)**:
  * `topics_config.yaml`: Defines topic names.
  * `qos_config.yaml`: Defines reliability and durability.

* **Adapter (Layer 2)**:
  * `utils_roslibrust`: A library that parses these YAML files and provides typed accessors (e.g., `topics::telemetry(cfg)`).

* **Application (Layer 3)**:
  * **Startup Phase**:
    1. Parse CLI arguments (`--params-file`).
    2. Load and merge YAML files into a `Config` object.
    3. Resolve parameters (Topic Name, Period).
    4. Connect to `rosbridge_websocket`.

This differs from `rclrs` (which uses ROS 2 parameters) because `roslibrust` is an external client. It reads the config *files* directly rather than querying the *parameter server*.

---

## Configuration Model

Lesson 05 relies on **YAML files** located in `src/4_interfaces/lesson_interfaces/config/`.

**Key Parameters Used:**

| Parameter Key | Purpose | Default (if missing) |
| :--- | :--- | :--- |
| `topics.telemetry` | The topic to publish/subscribe to | `/telemetry` |
| `publisher.period_ms` | Loop duration in milliseconds | `1000` |
| `subscriber.reset_max_value` | Threshold for resetting stream stats | `10` |

---

## Build

Since this package is ignored by `colcon`, we build it as a standard Rust project from its source directory.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_05_parameter
cargo build

```

---

## Prerequisite: The Bridge

We need a running ROS bridge to communicate, even though configuration is loaded locally.

**1. Start the Bridge**
In Terminal 1:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

```

---

## Run – Publisher

Run the publisher, passing the path to the shared configuration file using `cargo run`.

In Terminal 2:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_05_parameter

# Run via cargo, passing args after '--'
# Correct path: ../../../ (up 3 levels to 'src')
RUST_LOG=info cargo run --bin publisher -- --params-file ../../../4_interfaces/lesson_interfaces/config/topics_config.yaml

```

**Expected output:**

```text
[INFO] Loaded configuration from 1 files.
[INFO] Connected to ws://localhost:9090
[INFO] Configuration: topic='/tutorial/telemetry', period=1000ms
[INFO] Lesson 05 node started (publisher)...

```

*Note: If you run without arguments, the code includes a dev-convenience check that attempts to find the file automatically.*

---

## Run – Subscriber

In Terminal 3:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_05_parameter

# Run via cargo, passing args after '--'
RUST_LOG=info cargo run --bin subscriber -- --params-file ../../../4_interfaces/lesson_interfaces/config/topics_config.yaml

```

**Expected behaviour:**

* It loads the *same* file, ensuring it connects to the *same* topic (`/tutorial/telemetry`).
* It logs received messages.

---

## Runtime Updates (Restart Pattern)

Since `roslibrust` reads files at startup, "runtime updates" are achieved by **editing the file and restarting the client**.

1. **Edit the Config:**
Open `src/4_interfaces/lesson_interfaces/config/topics_config.yaml` and change `telemetry` to `/tutorial/custom_topic`.
2. **Restart Clients:**
Stop and restart both Publisher and Subscriber.
3. **Observe:**
Both nodes immediately sync to the new topic `/tutorial/custom_topic` without recompiling code.

---

## Architecture Notes

* **Client vs. Node**: `roslibrust` acts as a client. It configures itself *before* joining the network.
* **`utils_roslibrust` Dependency**: This lesson relies on the shared utils crate to parse YAML, ensuring the parsing logic is identical across all Rust-based lessons.
* **Sync Configuration**: Unlike the previous async service-call approach, this configuration is synchronous and guaranteed to exist before the main loop starts.

---

## What This Lesson Proves

When Lesson 05 works correctly, you have demonstrated:

1. **Single Source of Truth**: Rust clients use the exact same configuration files as the rest of the ROS system.
2. **Code Reusability**: The binary is generic; its behavior is defined entirely by the injected YAML.
3. **Robustness**: The client handles missing files by falling back to sensible defaults (or internal dev paths).

---

## What Comes Next

Lesson 06 introduces **Lifecycle Management**, exploring how to control the state of the node (Active/Inactive) remotely, mirroring the Managed Node concept in ROS 2.

```

```