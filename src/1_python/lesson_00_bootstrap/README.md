# Lesson 00 (Python) - Bootstrap

## Goal
Create the smallest possible `rclpy` package and verify it runs.

## Where To Run Commands
- Workspace commands (`colcon build ...`, `source install/setup.bash`) assume you start from the workspace root: `~/ros2_ws_tutorial`.
- ROS setup (`source /opt/ros/jazzy/setup.bash`) and `ros2 run ...` can be run from any directory.

## Official Resources
- [ROS 2 Documentation: Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Documentation: Writing a simple node (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

## Step-by-Step Setup (Fresh System)

Follow these steps in order to prepare a fresh system for ROS 2 Python development.

### 1. Install ROS 2
Installation commands below use **Jazzy** (for Ubuntu 24.04). 
*Note: ROS distributions are tied to OS versions. If you are on Ubuntu 22.04, you must use **Humble**. Replace `jazzy` with `humble` in all commands below.*
``` bash
sudo apt update && sudo apt install ros-jazzy-desktop
```

### 2. Install Build Tools
``` bash
sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep git
```

### 3. Initialize `rosdep` (Recommended)
`rosdep` helps install ROS package dependencies automatically.
``` bash
sudo rosdep init
rosdep update
```

### 4. Python Packaging Note (Ubuntu 24.04+)
Modern Ubuntu protects the system Python environment (PEP 668). If you ever need to install Python tooling via `pip`, prefer a virtual environment, or use `--user --break-system-packages` for user-local installs.
``` bash
# Preferred: venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip

# Alternative: user-local tools (no sudo)
python3 -m pip install --user <package> --break-system-packages
```

---

## Building the Workspace

### 1. Source the ROS 2 Environment
This must be done in **every new terminal** before building or running.
``` bash
source /opt/ros/jazzy/setup.bash
```

### 2. Build the Package
Navigate to your workspace root and build the lesson.
``` bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_00_bootstrap_py --symlink-install
```
- Note for python only, build with `--symlink-install` so you can edit `.py` files without rebuilding (and without re-sourcing in the same terminal).
---

## Running the Node

### 1. Source the Workspace
After a successful build, source your local installation:
``` bash
source install/setup.bash
```

Optional: auto-source the workspace in every new terminal by adding this to `~/.bashrc` (or `~/.zshrc`):
``` bash
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi
```

### 2. Execute
``` bash
ros2 run lesson_00_bootstrap_py lesson_00_bootstrap
```

**Expected Output:**
`[INFO] [lesson_00_bootstrap]: Lesson 00 bootstrap node started...`

This lesson node prints once and exits.

---

## Troubleshooting Knowledge
- **Seeing a stack trace on Ctrl+C**: This should exit cleanly; rebuild and re-source after updating the node: `colcon build --packages-select lesson_00_bootstrap_py` then `source install/setup.bash`.
- **`ros2: command not found` / `ImportError: No module named rclpy`**: The ROS environment is not sourced; run `source /opt/ros/jazzy/setup.bash`.
- **`Package 'lesson_00_bootstrap_py' not found`**: Rebuild and re-source `install/setup.bash` in the same terminal.
- **`No executable found`**: The executable name is `lesson_00_bootstrap` (from `setup.py` `console_scripts`), not the package name.
- **`externally-managed-environment`**: On Ubuntu 24.04+, install Python tools using a venv, or `--user --break-system-packages` (no sudo) for user-local tooling.
