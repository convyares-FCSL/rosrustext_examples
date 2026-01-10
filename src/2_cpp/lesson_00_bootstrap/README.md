# Lesson 00 (C++) - Bootstrap

## Goal
Create the smallest possible `rclcpp` package and verify it runs.

## Where To Run Commands
- Workspace commands (`colcon build ...`, `source install/setup.bash`) assume you start from the workspace root: `~/ros2_ws_tutorial`.
- ROS setup (`source /opt/ros/jazzy/setup.bash`) and `ros2 run ...` can be run from any directory.

## Official Resources
- [ROS 2 Documentation: Developing a ROS 2 package](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html) :contentReference[oaicite:2]{index=2}
- [ROS 2 Documentation: Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) :contentReference[oaicite:3]{index=3}
- [rclcpp API: rclcpp::spin_some(node)](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/function_namespacerclcpp_1a5e16488d62cc48e5520101f9f4f4102a.html) :contentReference[oaicite:4]{index=4}

---

## Building the Workspace

### 1. Source the ROS 2 Environment
This must be done in **every new terminal** before building or running.
```bash
source /opt/ros/jazzy/setup.bash
````

### 2. Build the Package

Navigate to your workspace root and build the lesson.

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_00_bootstrap_cpp
```

* Unlike the Python lesson, C++ changes require a rebuild (and re-sourcing) to take effect.

---

## Running the Node

### 1. Source the Workspace

After a successful build, source your local installation:

```bash
source install/setup.bash
```

Optional: auto-source ROS + workspace in every new terminal by adding this to `~/.bashrc` (or `~/.zshrc`):

```bash
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi
```

### 2. Execute

```bash
ros2 run lesson_00_bootstrap_cpp lesson_00_bootstrap
```

**Expected Output:**
`[INFO] [lesson_00_bootstrap]: Lesson 00 bootstrap node started...`

This lesson node prints once and exits.

---

## Notes

* This lesson intentionally performs a single non-blocking tick (`rclcpp::spin_some`) and then exits. ([docs.ros.org][1])
* Later lessons replace `spin_some` with `rclcpp::spin(node)` for continuous execution, as shown in the official C++ tutorial pattern. ([docs.ros.org][2])