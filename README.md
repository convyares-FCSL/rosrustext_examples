# ROS 2 lessons workspace

This workspace is being reset into a structured, multi-language lesson series.


## Goals
- Provide a consistent lesson sequence across Python, C++, rclrs, and rcllibrust.
- Keep code simple, readable, and close in style across languages.
- Share the same topic/service/action names to make cross-language comparisons easy.

## Planned layout
- `src/`
   - `python/`
     - `lesson_XX_topic/`
     - `utils_py/` (shared utilities, QOS profiles, python specific)
  - `cpp/`
    - `lesson_XX_topic/`
    - `utils_cpp/` (shared utilities, QOS profiles, cpp specific)
  - `rust/`
    - `rclrs/`
      - `lesson_XX_topic/`
    - `rcllibrust/`
      - `lesson_XX_topic/`
    - `utils_rust/` (shared utilities, QOS profiles, rust specific)
  - `4_interfaces/` (package `lesson_interfaces` for shared msg/srv/action definitions and shared configuration)
  - `src/4_interfaces/config/*.yaml` (central configuration for all lessons)
- `templates/` (starting templates for each language)

## Lesson roadmap
Lesson 00 - workspace bootstrap
- One minimal package per language.
- Build and run instructions.

Lesson 01 - simple node
- Timer + logging only.
- Clean shutdown and basic options.

Lesson 02 - publisher
- Publish `std_msgs/String` on `chatter`.
- Parameter for publish rate.
- **New Pattern**: Uses parameters from `src/4_interfaces/config/topics_config.yaml` and `src/4_interfaces/config/qos_config.yaml`.

Lesson 03 - subscriber
- Subscribe to `chatter`.
- QoS selection and logging.
- **New Pattern**: Uses parameters from `src/4_interfaces/config/topics_config.yaml` and `src/4_interfaces/config/qos_config.yaml`.

Lesson 04 - services
- `example_interfaces/srv/AddTwoInts` server and client.
- Input validation and error handling.

Lesson 05 - parameters
- Declare + validate parameters.
- Runtime updates with callback.

Lesson 06 - lifecycle publisher (gated)
- Lifecycle node with publisher created on configure.
- Publish only when active; disable on deactivate.

Lesson 07 - actions
- `CountUntil` action server and client.
- Feedback, result, and cancel handling.

Lesson 08 - executors and callback groups (optional)
- Multi-threaded executor and callback group isolation.
- Demonstrate non-blocking patterns.

Each lesson package includes a `lesson.md` with the goal, build, and run steps.

## Templates
Templates are in `templates/` and are intended to be copied into `src/`.
- Copy into `src/<language>/lesson_XX_topic/`.
- Rename the package name in the manifest/build file and code.
- Keep naming consistent with the lesson layout shown above.

Template locations:
- `templates/python`
- `templates/cpp`
- `templates/rclrs`
- `templates/rcllibrust`

## Rust notes
- Rust edition: 2021.
- rclrs source: `https://github.com/ros2-rust/ros2_rust` (tag `v0.6.0`).
- rcllibrust source: `https://github.com/RosLibRust/roslibrust`.

## Build notes
- `src/rcllibrust` is built with `cargo` and is excluded from `colcon` via `COLCON_IGNORE`.


### Using Parameters
This example shows a "professional" pattern where configuration is split by type in `src/4_interfaces/config/`:
- `topics_config.yaml` for topic names.
- `qos_config.yaml` for QoS defaults and profile selection (`telemetry`, `commands`, `state_latched`, `events`, `reliable_data`, `static_data_latched`).
- `services_config.yaml` for service names (used in Lesson 04+).
Shared utility modules (`utils_py` and `utils_rclrs`) handle the boilerplate of declaring and reading these parameters.

To run with the configuration:
```bash
# Example for Python publisher
ros2 run lesson_02_publisher_py publisher --ros-args \
  --params-file src/4_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/config/qos_config.yaml
```

Launch File Power: This sets you up perfectly for later lessons on Launch Files, where you can show one launch file starting a C++ talker and a Rust listener, both sharing the same configuration files.

Good tutorials : 
https://www.youtube.com/@RoboticsBackEnd/posts {has udemy course :https://www.udemy.com/user/edouard-renard/ }
