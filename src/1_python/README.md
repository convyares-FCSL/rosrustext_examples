## Python lessons notes

### Build with symlinks (recommended)
When developing Python packages, prefer `--symlink-install` so edits to `.py` files take effect immediately (no rebuild needed).
```bash
cd ~/ros2_ws_tutorial
colcon build --symlink-install --packages-select lesson_00_bootstrap_py
```

### Avoid re-sourcing in every new terminal
You still need to source ROS 2 and your workspace once per shell session. To make that automatic, add this to `~/.bashrc` (or `~/.zshrc`):
```bash
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi
```
