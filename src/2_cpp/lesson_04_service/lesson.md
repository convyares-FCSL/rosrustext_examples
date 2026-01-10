# Lesson 04 (C++) – Services & Logic Separation

## Goal

Create a service server and client that:

* implements the `ComputeStats` service (sum, average, status)
* separates **Business Logic** (Header-Only C++) from **Middleware** (ROS Node)
* uses shared configuration for service names (`utils_cpp`)
* validates request/response flow asynchronously
* shuts down cleanly on Ctrl+C

This lesson focuses on **testable architecture** and the **Client-Server pattern**.

---

## Build

Since we updated `utils_cpp` with new service definitions, we must rebuild it first.

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_cpp
colcon build --packages-select lesson_04_service_cpp
source install/setup.bash
```

---

## Verify: Unit Testing (New)

Because we separated the math logic (`stats_logic.hpp`) from the ROS node, we can test it using Google Test (GTest) without launching any nodes.

Run the tests via the build system:

```bash
colcon test --packages-select lesson_04_service_cpp --ctest-args -V
colcon test-result --all --verbose
```

**Expected output:**

```text
build/lesson_04_service_cpp/test_results/lesson_04_service_cpp/test_stats_logic.gtest.xml: 4 tests, 0 errors, 0 failures, 0 skipped
```

This confirms that the business logic handles normal values, empty lists, and single elements correctly.

---

## Run: The Server

Terminal 1:

```bash
ros2 run lesson_04_service_cpp service_server
```

**Expected output (idle):**

```text
[INFO] [lesson_04_service_server]: Lesson 04 service node started (server). Ctrl+C to exit.
[INFO] [lesson_04_service_server]: Service "compute_stats" is ready.
```

The server waits silently for incoming requests.

---

## Verify: Service Logic

### Method 1 – Manual CLI Test

Send a request using the ROS 2 command-line tool to verify the server logic.

Terminal 2:

```bash
source install/setup.bash
ros2 service call /compute_stats lesson_interfaces/srv/ComputeStats "{data: [10.0, 20.0, 30.0]}"
```

Terminal 1 output (Server):

```text
[INFO] [lesson_04_service_server]: Incoming request with 3 samples.
[INFO] [lesson_04_service_server]: Computation Complete: Sum=60.00, Avg=20.00
```

Terminal 2 output (Client):

```text
response:
lesson_interfaces.srv.ComputeStats_Response(sum=60.0, average=20.0, status='Success')
```

### Method 2 – C++ Client Node

Keep the server running. In a second terminal, run the programmatic client.

Terminal 2:

```bash
source install/setup.bash
ros2 run lesson_04_service_cpp service_client
```

Terminal 2 output:

```text
[INFO] [lesson_04_service_client]: Service "compute_stats" available.
[INFO] [lesson_04_service_client]: Sending request with 3 samples...
[INFO] [lesson_04_service_client]: Result -> Sum: 61.40, Avg: 20.47, Status: 'Success'
```

The client will exit automatically after receiving the response.

---

## Runtime Inspection

Check that the service is active and correctly typed:

```bash
ros2 service list -t
```

Verify that `/compute_stats` is listed with type `lesson_interfaces/srv/ComputeStats`.

---

## Notes

* **Separation of Concerns**: The math logic lives in `stats_logic.hpp` (pure C++), not inside the Node class. This allows us to unit test the math without ROS.
* **Service Name**: Loaded from `utils_cpp` (not hardcoded).
* **Async Clients**: The client uses `async_send_request` to avoid blocking the thread while waiting for a response.

For architectural rationale and design patterns, see `lesson_breakdown.md`.

---

## Summary

Lesson 04 (C++) validates that your system can:

* Handle asynchronous request/response interactions.
* Decouple **Business Logic** (Computation) from **Middleware** (Transport).
* Verify logic correctness via automated Unit Tests (`colcon test`).

This pattern is critical for building reliable distributed applications where core algorithms (Signal Processing, Data Aggregation, Financial Logic) must be verifiable independent of the ROS transport layer.