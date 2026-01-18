# Lesson 04 (Python) – Services & Logic Separation

## Goal

Create a service server and client that:

* implements the `ComputeStats` service (sum, average, status)
* separates **Business Logic** (pure Python) from **Middleware** (ROS Node)
* uses shared configuration for service names (`utils_py`)
* validates request/response flow asynchronously
* shuts down cleanly on Ctrl+C

This lesson focuses on **testable architecture** and the **Client-Server pattern**.

---

## Build (server + client)

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_04_service_py --symlink-install
source install/setup.bash
```

---

## Verify: Unit Testing (New)

Because we separated the math logic from the ROS node, we can test it directly using standard Python tools.

**Method 1: The ROS Way (Standard)**
This runs the tests defined in `setup.py` via the build tool.

```bash
colcon test --packages-select lesson_04_service_py --pytest-args -v
colcon test-result --all --verbose
```

**Method 2: The Developer Way (Fast)**
You can run `pytest` directly on the logic file without building.

```bash
cd src/1_python/lesson_04_service
pytest test/test_stats_logic.py -v
```

**Expected output:**

```text
test/test_stats_logic.py::test_compute_normal_values PASSED
test/test_stats_logic.py::test_compute_empty_list PASSED
...
4 passed
```

---

## Run: The Server

Terminal 1:

```bash
ros2 run lesson_04_service_py service_server
```

**Expected output (idle):**

```text
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
ros2 service call /tutorial/compute_stats lesson_interfaces/srv/ComputeStats "{data: [10.0, 20.0, 30.0]}"
```

Terminal 1 output (Server):

```text
[INFO] [lesson_04_service_server]: Incoming request with 3 samples.
```

Terminal 2 output (Client):

```text
response:
lesson_interfaces.srv.ComputeStats_Response(sum=60.0, average=20.0, status='Success')
```

### Method 2 – Python Client Node

Keep the server running. In a second terminal, run the programmatic client.

Terminal 2:

```bash
source install/setup.bash
ros2 run lesson_04_service_py service_client
```

Terminal 2 output:

```text
[INFO] [lesson_04_service_client]: Service "compute_stats" available.
[INFO] [lesson_04_service_client]: Sending request: [10.5, 20.2, 30.7]
[INFO] [lesson_04_service_client]: Result -> Sum: 61.40, Avg: 20.47, Status: 'Success'
```

The client will exit automatically after receiving the response.

---

## Runtime Inspection

Check that the service is active and correctly typed:

```bash
ros2 service list -t
```

Verify that `/tutorial/compute_stats` is listed with type `lesson_interfaces/srv/ComputeStats`.

---

## Notes

* **Separation of Concerns**: The math logic lives in `StatsLogic` (pure Python), not inside the Node class. This allows us to unit test the math without ROS.
* **Service Name**: Loaded from `utils_py` (not hardcoded).
* **Async Clients**: The client uses `call_async` + `spin` to avoid blocking the thread while waiting for a response.

For architectural rationale and design patterns, see `lesson_breakdown.md`.

---

## Summary

Lesson 04 validates that your system can:

* Handle request/response interactions.
* Decouple Business Logic (Computation) from Middleware (Transport).
* Use custom service types defined in the workspace.

This pattern is critical for building reliable distributed applications where core algorithms (Signal Processing, Data Aggregation, Financial Logic) must be verifiable independent of the ROS transport layer.