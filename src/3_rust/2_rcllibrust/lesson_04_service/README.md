# Lesson 04 (roslibrust) – Services & Logic Separation

## Goal

Create a service server and client that:

* connect to ROS 2 via `rosbridge` (WebSocket)
* implement the `ComputeStats` service:
  * input: `float64[] data`
  * output: `float64 sum`, `float64 average`, `string status`
* separate **Business Logic** (pure Rust library) from **Transport / Middleware**
* verify logic correctness using `cargo test` (no ROS required)
* support cross-language service calls (ROS CLI, Rust, C++, Python)
* shut down cleanly on Ctrl+C

This lesson focuses on **professional service design over a bridge**:
testability, adapter patterns, and asynchronous request handling.

> Prerequisite: `lesson_interfaces/srv/ComputeStats.srv` exists and `rosbridge_server` is running.

---

## Prerequisites

### Terminal 1 – rosbridge

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
````

---

## Build

This is a **Cargo-only Rust crate** (no colcon, no generated Rust interfaces).

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_04_service
cargo build
```

---

## Verify: Unit Testing (Logic Only)

All computation logic lives in `src/lib.rs` and has **no ROS dependencies**.

```bash
cargo test
```

Expected output (example):

```text
running 4 tests
test tests::test_compute_empty_list ... ok
test tests::test_compute_floats ... ok
test tests::test_compute_normal_values ... ok
test tests::test_compute_single_element ... ok
```

This validates algorithm correctness before any ROS nodes are started.

---

## Run: Service Server (roslibrust)

### Terminal 2

```bash
RUST_LOG=info cargo run --bin service_server
```

Expected output (example):

```text
[lesson_04_service_server] Connected to ws://localhost:9090
[lesson_04_service_server] Started (rosbridge). Ctrl+C to exit.
[lesson_04_service_server] Service name: /compute_stats
[lesson_04_service_server] Service type: lesson_interfaces/srv/ComputeStats
```

The server advertises `/compute_stats` and remains active until Ctrl+C.

---

## Verify: Service Logic

### Method 1 – ROS 2 CLI

### Terminal 3

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /compute_stats lesson_interfaces/srv/ComputeStats "{data: [10.0, 20.0, 30.0]}"
```

Expected response:

```text
response:
lesson_interfaces.srv.ComputeStats_Response(sum=60.0, average=20.0, status='Success')
```

Server logs (example):

```text
Incoming request on /compute_stats with 3 samples.
Computation complete: Sum=60.00, Avg=20.00
```

---

### Method 2 – Rust Client (roslibrust)

Keep the server running.

### Terminal 4

```bash
RUST_LOG=info cargo run --bin service_client
```

Expected output (example):

```text
[lesson_04_service_client] Connected to ws://localhost:9090
[lesson_04_service_client] Started (rosbridge).
[lesson_04_service_client] Calling /compute_stats with 3 samples...
[lesson_04_service_client] Response received -> Sum: 61.40, Avg: 20.47, Status: 'Success'
```

The client performs a non-blocking service call over rosbridge and exits after receiving the response.

---

## Runtime Inspection

Verify the service is visible on the ROS graph:

```bash
ros2 service list -t
```

Expected entry:

```text
/compute_stats [lesson_interfaces/srv/ComputeStats]
```

---

## Notes

* **Logic Separation**
  All business logic is implemented in `lib.rs` and reused by both server and client.

* **Manual Message / Service Definitions**
  Because this is a Cargo-only crate, service request/response types are defined manually using `serde` and `RosServiceType`.

* **Async Execution Model**
  Roslibrust uses Tokio; service callbacks and service calls are asynchronous and do not block the runtime.

* **Cross-Language Compatibility**
  The service can be called from:

  * ROS 2 CLI
  * Rust (roslibrust)
  * C++ and Python nodes using the same `.srv`

For architectural rationale and patterns, see `lesson_breakdown.md`.

```

---

### One next step  
When you’re ready, the natural follow-on is to **centralise `/compute_stats` in `utils_roslibrust::services`** so Lesson 04 (rclrs) and Lesson 04 (roslibrust) share the same naming contract.