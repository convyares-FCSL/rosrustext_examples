# Templates

This directory contains starter boilerplate for creating new packages in this workspace.
Use these templates to ensure every new package starts with the correct build configuration and directory structure.

---

## Directory Structure

```text
templates/
├── cpp/                  # C++ Template (rclcpp)
│   ├── include/          # Headers (.hpp)
│   ├── src/              # Source code (.cpp)
│   ├── CMakeLists.txt    # Configured for ament_cmake
│   └── package.xml
│
├── python/               # Python Template (rclpy)
│   ├── lesson_py_template/  # Python module (Rename this!)
│   ├── resource/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── rcllibrust/           # Rust Client Template
│   ├── src/
│   └── Cargo.toml        # Configured for cargo-only build
│
└── rclrs/                # Rust Node Template
    ├── src/
    └── Cargo.toml        # Configured for colcon + rosidl artifacts

```

---

## Usage Guide

To start a new lesson, copy the appropriate folder into your `src/` tree and rename it.

### 1. Creating a new Python Package

1. **Copy the template**:
```bash
cp -r templates/python src/1_python/lesson_03_new_lesson

```


2. **Rename the module**:
You must rename the inner folder (`lesson_py_template`) to match your package name.
```bash
mv src/1_python/lesson_03_new_lesson/lesson_py_template \
   src/1_python/lesson_03_new_lesson/lesson_03_new_lesson

```


3. **Update Configuration**:
* Edit `package.xml`: Update `<name>` and `<description>`.
* Edit `setup.py`: Update `package_name` variable.



### 2. Creating a new C++ Package

1. **Copy the template**:
```bash
cp -r templates/cpp src/2_cpp/lesson_03_new_lesson

```


2. **Update Configuration**:
* Edit `CMakeLists.txt`: Change `project(lesson_cpp_template)` to your new name.
* Edit `package.xml`: Update `<name>`.



### 3. Creating a new Rust Package (`rclrs`)

1. **Copy the template**:
```bash
cp -r templates/rclrs src/3_rust/1_rclrs/lesson_03_new_lesson

```


2. **Update Configuration**:
* Edit `Cargo.toml`: Update `name` and `version`.
* **Note**: The `Cargo.toml` is pre-configured to look for interface bindings in `../../../../install`. Do not remove that dependency line if you plan to use custom messages.