
# Lesson 06 – Orchestration (Lifecycle Manager)

## Purpose

This package provides a **Lifecycle Manager** node that orchestrates the startup and shutdown of existing Lesson 06 lifecycle-managed nodes.

It drives other nodes through their lifecycle state transitions using standard ROS 2 lifecycle services.

This package contains **only the manager**.
The managed Publisher / Subscriber nodes must be started separately.

---

## Build

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_06_orchestration --symlink-install
source install/setup.bash
```

---

## 1. Basic Usage (Same-Language Nodes)

For same-language pairs (Python↔Python, C++↔C++, rclrs↔rclrs), lifecycle orchestration **and data flow both work by default**.

The manager expects the following lifecycle nodes:

* `/lesson_06_lifecycle_publisher`
* `/lesson_06_lifecycle_subscriber`

---

### Python Nodes

**Terminal 1 – Publisher**

```bash
ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_publisher
```

**Terminal 2 – Subscriber**

```bash
ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber
```

**Terminal 3 – Lifecycle Manager**

```bash
ros2 run lesson_06_orchestration lifecycle_manager
```

**Expected behavior**

* Manager configures both nodes
* Subscriber activates
* Publisher activates
* Subscriber receives messages

---

### C++ Nodes

**Terminal 1 – Publisher**

```bash
ros2 run lesson_06_lifecycle_cpp lesson_06_lifecycle_publisher
```

**Terminal 2 – Subscriber**

```bash
ros2 run lesson_06_lifecycle_cpp lesson_06_lifecycle_subscriber
```

**Terminal 3 – Lifecycle Manager**

```bash
ros2 run lesson_06_orchestration lifecycle_manager
```
---

### Rust (rclrs) Nodes


**Terminal 1 – Publisher**

```bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_publisher
```

**Terminal 2 – Subscriber**

```bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_subscriber
```

**Terminal 3 – Lifecycle Manager**

```bash
ros2 run lesson_06_orchestration lifecycle_manager
```
---

## 2. Mixed-Language Orchestration

Mixed-language orchestration works at the **lifecycle level** without any changes.

However, **data flow may not work on the first run** unless a shared configuration is applied.

---

### Example: C++ Publisher + Rust Subscriber (Initial Run)

**Terminal 1 – C++ Publisher**

```bash
ros2 run lesson_06_lifecycle_cpp lesson_06_lifecycle_publisher
```

**Terminal 2 – Rust Subscriber**

```bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_subscriber
```

**Terminal 3 – Lifecycle Manager**

```bash
ros2 run lesson_06_orchestration lifecycle_manager
```

**Expected behavior**

* Both nodes reach `Active`
* Manager reports successful orchestration
* Subscriber may receive **no data**

This is expected for mixed-language runs without shared configuration.

---

<details>
<summary><strong>Enable Data Flow for Mixed-Language Systems</strong></summary>

To align topic names and QoS across languages, restart the **publisher and subscriber** with the shared parameter file:

```
src/4_interfaces/lesson_interfaces/config/topics_config.yaml
```

**Restart C++ Publisher**

```bash
ros2 run lesson_06_lifecycle_cpp lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml
```

**Restart Rust Subscriber**

```bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml
```

The lifecycle manager does **not** need to be restarted.

**Result**

* Lifecycle state remains `Active`
* Subscriber begins receiving data

</details>

---

## 3. Manager Configuration

You can override the managed node list and behavior using ROS parameters.

```bash
ros2 run lesson_06_orchestration lifecycle_manager --ros-args \
  -p node_names:="['/my_pub', '/my_sub']" \
  -p autostart:=True \
  -p timeout_s:=5.0
```

### Parameters

* `node_names` – fully qualified node names to manage
* `autostart` – configure and activate on startup (default: true)
* `timeout_s` – lifecycle service timeout (default: 10.0)
* `continue_on_error` – continue on failure (default: false)
* `shutdown_on_exit` – best-effort shutdown on exit (default: true)

---

## 4. Stopping the Manager

Press `Ctrl+C` in the manager terminal.

* Managed nodes are requested to deactivate and shutdown
* The manager exits once the sequence completes or times out

Managed nodes are separate processes and may need to be stopped manually if unresponsive.

---

## 5. Nav2 Lifecycle Manager Compatibility

The lifecycle nodes used in this lesson are compatible with the standard Nav2 lifecycle manager.

<details>
<summary><strong>Install Nav2 (Jazzy)</strong></summary>

```bash
sudo apt update
sudo apt install ros-jazzy-nav2-bringup
```

Verify:

```bash
ros2 pkg list | grep nav2_lifecycle_manager
```

</details>

### Example

```bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml
```

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['lesson_06_lifecycle_publisher']" \
  -p autostart:=True
```

Verify:

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
```

Expected:

```
Active [3]
```
