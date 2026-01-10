# Lesson 04 (Rust) – Services & Logic Separation

## Goal

Create a service server and client that:

* Implements the `ComputeStats` service (sum, average, status).
* Separates **Business Logic** (pure Rust library) from **Middleware** (ROS node).
* Uses shared configuration for service names (`utils_rclrs`).
* Uses the `rclrs` **Executor** pattern for async request handling.

This lesson focuses on **Rust idiomatic architecture** and the **Client–Server** pattern.

---

## Build (workspace overlay)

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_rclrs lesson_interfaces
colcon build --packages-select lesson_04_service_rclrs
source ~/ros2_ws_tutorial/install/setup.bash
````

Note: If colcon warns about overriding packages already in an underlay, you may need:

```bash
colcon build --packages-select utils_rclrs lesson_interfaces --allow-overriding utils_rclrs lesson_interfaces
```

---

## Verify: Unit Testing (Logic Only)

Because the math logic is in `src/lib.rs`, we can test it using `cargo test` without ROS 2.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/1_rclrs/lesson_04_service
cargo test
```

Expected: 4 tests pass.

---

## Run: The Server

Terminal 1:

```bash
source ~/ros2_ws_tutorial/install/setup.bash
ros2 run lesson_04_service_rclrs service_server
```

Expected output:

```text
[INFO] [lesson_04_service_server]: Lesson 04 service server started. Ctrl+C to exit.
```

---

## Verify: Service Logic

### Method 1 – Manual CLI Test

Terminal 2:

```bash
source ~/ros2_ws_tutorial/install/setup.bash
ros2 service call /compute_stats lesson_interfaces/srv/ComputeStats "{data: [10.0, 20.0, 30.0]}"
```

Server logs:

```text
[INFO] [lesson_04_service_server]: Incoming request with 3 samples.
[INFO] [lesson_04_service_server]: Computation complete: Sum=60.00, Avg=20.00
```

CLI response:

```text
response:
lesson_interfaces.srv.ComputeStats_Response(sum=60.0, average=20.0, status='Success')
```

> If your configured service name differs, check it with `ros2 service list -t`
> and use that name in the `ros2 service call`.

### Method 2 – Rust Client Node

Terminal 2:

```bash
source ~/ros2_ws_tutorial/install/setup.bash
ros2 run lesson_04_service_rclrs service_client
```

Expected output:

```text
[INFO] [lesson_04_service_client]: Lesson 04 service client started.
[INFO] [lesson_04_service_client]: Waiting for service...
[INFO] [lesson_04_service_client]: Sending request with 3 samples...
[INFO] [lesson_04_service_client]: Result -> Sum: 61.40, Avg: 20.47, Status: 'Success'
```

---

## Runtime Inspection

```bash
ros2 service list -t
```

Verify `/compute_stats` is listed with type `lesson_interfaces/srv/ComputeStats`.

---

## Notes

* **Executor pattern (rclrs 0.6.x):** the client uses `call_then(...)` to register a callback, and the callback is executed while `executor.spin(...)` is running.
* **Separation of concerns:** `compute(&[f64]) -> StatsResult` stays pure and testable; ROS nodes only translate requests/responses and log.