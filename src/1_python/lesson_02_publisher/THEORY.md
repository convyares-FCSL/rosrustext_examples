# Lesson 02 Breakdown: Publishers & Shared Interfaces

## Architecture: The Helper Pattern

In Lesson 01, we put timer logic inside a helper method. In Lesson 02, we extend this pattern to **Publishers**.

Instead of a giant `__init__` method, we break initialization into distinct steps:
1.  `declare_parameter`
2.  `_create_publisher`
3.  `_create_timer_from_param`

This makes the code readable and easier to maintain.

## New Concept: External Messages

We are no longer using built-in strings. We are using a **Custom Message** defined in another package.

```python
from lesson_interfaces.msg import MsgCount

```

* **The Source:** This class is generated from `MsgCount.msg` in the `lesson_interfaces` package.
* **The Usage:** Unlike C++ or Rust where you pass a struct, in Python, you instantiate the class (`msg = MsgCount()`) and populate its fields (`msg.count = ...`).

## New Concept: Shared Library (`utils_py`)

Hardcoding topic names and QoS settings is brittle. If you change the topic name in the publisher, you break the subscriber.

We use a local module `utils_py` to centralize this configuration.

```python
from utils_py import topics, qos

```

1. **Topic Consistency**: `topics.chatter(self)` returns the standardized topic name string.
2. **QoS Configuration**: `qos.telemetry(self)` returns a specific Quality of Service profile (e.g., Reliable vs Best Effort). This ensures both publisher and subscriber "speak the same language" regarding data delivery guarantees.

## Code Walkthrough

### 1. The Publisher Setup

```python
def _create_publisher(self) -> None:
    # 1. Get Topic Name (Centralized)
    self.topic_name = topics.chatter(self)

    # 2. Get QoS Profile (Centralized)
    self.qos_profile = qos.telemetry(self)

    # 3. Create Publisher
    # create_publisher(msg_type, topic, qos_profile)
    self._publisher = self.create_publisher(MsgCount, self.topic_name, self.qos_profile)

```

* **Type Safety**: We pass the class `MsgCount` (not an instance) as the first argument. ROS 2 checks that any data sent matches this structure.

### 2. The Timer Setup (Parameter Driven)

We reuse the robust pattern from Lesson 01, allowing the publish rate to be changed via the command line.

```python
# ... inside _create_timer_from_param ...
self.period = float(self.get_parameter("timer_period_s").value)
# ... validation ...
self._timer = self.create_timer(self.period, self._tick)

```

### 3. The Callback (Publishing Logic)

This acts as the "heartbeat" of the node.

```python
def _tick(self) -> None:
    # 1. Instantiate the Message Object
    msg = MsgCount()
    
    # 2. Populate Fields
    # (Matches int64 in the .msg file)
    msg.count = self._count

    # 3. Publish
    # This serializes the object and sends it over the DDS middleware.
    self._publisher.publish(msg)

    # 4. Update State
    self._count += 1

```

### 4. Type Hinting

Notice the use of Python type hints:

```python
def main(args: Optional[List[str]] = None) -> None:

```

While ROS 2 Python doesn't *enforce* types at runtime, adding hints (like `-> None`) is a best practice. It allows static analysis tools (mypy, vscode) to catch errors before you run the code.

### 5. Main Execution

The `main` function follows the standard robust pattern:

1. **Init** (`rclpy.init`)
2. **Try/Spin** (`rclpy.spin(node)`) to run the event loop.
3. **Catch** `KeyboardInterrupt` for clean exit.
4. **Finally** block for resource cleanup (`destroy_node`, `shutdown`).