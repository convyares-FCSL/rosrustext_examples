# Lesson 01 Breakdown: The Event Loop & Parameters

## Architecture: The Event-Driven Node

In Lesson 00, we ran the node once and exited. In Lesson 01, the node enters an **Event Loop**.



1.  **Configuration**: The node reads a **Parameter** (`timer_period_s`) at startup to determine how fast it should run.
2.  **Scheduling**: It creates a **Timer**, asking the ROS 2 Executor to "wake it up" every $X$ seconds.
3.  **Spinning**: The main thread blocks on `rclpy.spin()`. It sits idle until the timer fires, executes the callback, and then goes back to sleep.

## Code Walkthrough

### 1. Declaring Parameters
In ROS 2, parameters are not just global variables; they are strictly typed and managed by the node.

```python
# Inside __init__
self.declare_parameter("timer_period_s", 1.0)

```

* **Declaration**: You *must* declare parameters before reading them. This registers the parameter name with the ROS 2 system, allowing tools like `ros2 param list` to see it.
* **Default Value**: We provide `1.0` as a default. If the user doesn't specify a value at runtime, this is what we get.

### 2. The Setup Helper Pattern

Instead of putting complex logic inside `__init__`, we extract the timer setup into a helper method.

```python
def _start_timer_from_param(self) -> None:
    # 1. Read the parameter
    period = float(self.get_parameter("timer_period_s").value)

    # 2. Validation (Sanity Check)
    if period <= 0.0:
        self.get_logger().warn(f"timer_period_s={period} is invalid; using 1.0s")
        period = 1.0

    # 3. Create the Timer
    # self.create_timer(interval, callback)
    self._timer = self.create_timer(period, self._on_timer)

```

* **Validation**: Validating inputs is a hallmark of production-grade code. A negative timer period would cause a crash or undefined behavior.
* **Cleanliness**: This keeps `__init__` readable. If we later want to support *dynamic* parameter updates (changing the speed while the node is running), we can reuse this function!

### 3. The Callback (The Logic)

This is the "Business Logic". It happens asynchronously whenever the timer fires.

```python
def _on_timer(self) -> None:
    self._tick += 1
    self.get_logger().info(f"tick {self._tick}")

```

### 4. The Main Loop & Clean Shutdown

The `main` function has evolved to handle a long-running process.

```python
def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = Lesson01Node()
        
        # BLOCKING CALL: This line will run forever until Ctrl+C is pressed.
        rclpy.spin(node)

    # Catch Ctrl+C (KeyboardInterrupt) specifically
    except KeyboardInterrupt:
        pass # Do nothing, just exit the try block to reach 'finally'

    except Exception as exc:
        # ... standard error logging ...

    finally:
        # ... standard cleanup ...

```

**Why catch `KeyboardInterrupt`?**

* In Python, pressing `Ctrl+C` raises a `KeyboardInterrupt` exception.
* If you don't catch it, Python prints a messy "Traceback" to the console, which looks like a crash.
* By catching it with `pass`, we allow the code to flow naturally into the `finally` block, ensuring `node.destroy_node()` and `rclpy.shutdown()` are called cleanly. This is the "polite" way to stop a Python node.