# ROS 2 lessons workspace

This workspace is being reset into a structured, multi-language lesson series.
The previous learning project is archived in `src_archive/`.

## Goals
- Provide a consistent lesson sequence across Python, C++, rclrs, and rcllibrust.
- Keep code simple, readable, and close in style across languages.
- Share the same topic/service/action names to make cross-language comparisons easy.

## Planned layout
- `src/`
  - `interfaces/` (package `lesson_interfaces` for shared msg/srv/action definitions)
  - `python/lesson_XX_topic/`
  - `python/utils_py/` (shared utilities)
  - `cpp/lesson_XX_topic/`
  - `cpp/utils_cpp/` (shared utilities)
  - `rclrs/lesson_XX_topic/`
  - `rclrs/utils_rclrs/` (shared utilities)
  - `rcllibrust/lesson_XX_topic/`
  - `rcllibrust/utils_rcllibrust/` (shared utilities)
- `templates/` (starting templates for each language)
- `src_archive/` (original learning workspace snapshot)

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

Lesson 03 - subscriber
- Subscribe to `chatter`.
- QoS selection and logging.

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


Good tutorials : 
https://www.youtube.com/@RoboticsBackEnd/posts {has udemy course :https://www.udemy.com/user/edouard-renard/ }
