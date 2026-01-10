# Lesson 00 (Rust / rclrs) - Bootstrap

## Goal
Create the smallest possible `rclrs` package and verify it runs.

## Where To Run Commands
- Workspace commands (`colcon build ...`, `source install/setup.bash`) assume you start from the workspace root: `~/ros2_ws_tutorial`.
- ROS setup (`source /opt/ros/jazzy/setup.bash`) and `ros2 run ...` can be run from any directory.

## Official Resources
- [Official ros2_rust Building Guide](https://github.com/ros2-rust/ros2_rust/blob/main/docs/building.md)

---

## Step-by-Step Setup (Fresh System)

Follow these steps in order to prepare a fresh system for ROS 2 Rust development.

### 1. Install ROS 2
Installation commands below use **Jazzy** (for Ubuntu 24.04). 
*Note: ROS distributions are tied to OS versions. If you are on Ubuntu 22.04, you must use **Humble**. Replace `jazzy` with `humble` in all commands below.*
``` bash
sudo apt update && sudo apt install ros-jazzy-desktop
```
### 2. Install Rust
Install the Rust toolchain via `rustup`:
``` bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
# Follow prompts (usually option 1), then restart your terminal or run:
source $HOME/.cargo/env
```
### 3. Install Build Tools & Extensions
Modern Ubuntu (24.04+) protects the system Python environment. To install `colcon` extensions, we use the `--break-system-packages` flag.
``` bash
# Install base build tools
sudo apt install python3-pip python3-colcon-common-extensions git build-essential

# Install Rust-specific colcon extensions
python3 -m pip install --user colcon-cargo colcon-ros-cargo --break-system-packages
```
**Why is `--break-system-packages` okay?** 
On Ubuntu 24.04, this flag is required to install Python tools into your *user* directory (`~/.local`). Because we are not using `sudo` here, it will not modify or "break" the critical system files required by the OS to run. It only affects your local user environment. See the Troubleshooting section for more details.

### 4. Install Missing Linker Dependencies
`rclrs` requires specific headers and libraries to link correctly. These are often missing in "base" installations.
``` bash
sudo apt install ros-jazzy-test-msgs ros-jazzy-rosidl-runtime-rs
```
---

## Building the Workspace

### 1. Source the ROS 2 Environment
This must be done in **every new terminal** before building or running.
``` bash
source /opt/ros/jazzy/setup.bash
```
### 2. Build the Package
Navigate to your workspace root and build the lesson. We use `--packages-up-to` to ensure the internal utility library (`utils_rclrs`) is built first.
``` bash
cd ~/ros2_ws_tutorial
colcon build --packages-up-to lesson_00_bootstrap_rclrs
```
- Unlike the Python lesson, Rust changes require a rebuild (and re-sourcing) to take effect.

---

## Running the Node

### 1. Source the Workspace
After a successful build, source your local installation:
``` bash
source install/setup.bash
```

Optional: auto-source ROS + workspace in every new terminal by adding this to `~/.bashrc` (or `~/.zshrc`):
``` bash
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi
```

Optional: ensure Rust is on your PATH in new terminals (if `cargo` isn't found):
``` bash
source $HOME/.cargo/env
```

### 2. Execute
``` bash
ros2 run lesson_00_bootstrap_rclrs lesson_00_bootstrap_rclrs
```
**Expected Output:**
`[INFO] [lesson_00_bootstrap]: Lesson 00 bootstrap node started...`

This lesson node prints once and exits.

---

## Troubleshooting Knowledge
- **`externally-managed-environment`**: This is a security feature (PEP 668) in Ubuntu 24.04+. By using `--user --break-system-packages`, we tell the system to allow our local tool installation without touching the root OS Python.
- **`cargo: command not found`**: Rust isn't on your PATH; run `source $HOME/.cargo/env` (or add it to your shell rc file).
- **`linking with cc failed` / `unable to find library -ltest_msgs`**: This means Step 4 was skipped. The Rust linker requires the C-libraries for ROS 2 test messages to compile successfully.
- **`Package 'lesson_00_bootstrap_rclrs' not found` / `No executable found`**: Rebuild and re-source `install/setup.bash` in the same terminal.
- **`Failed to find package.sh`**: This happens if a dependency (like `utils_rclrs`) hasn't been built yet. Always ensure you build with `--packages-up-to` if you have made changes to the utility library.
