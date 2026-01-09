# Python Template (`rclpy`)

This is a minimal `ament_python` package template that enforces **type hinting** and the **composition pattern**.

## Directory Structure

```text
.
├── lesson_py_template/       # Python Package (Rename this!)
│   ├── __init__.py
│   ├── node.py               # Node Logic + Main Entry Point
│   ├── qos.py                # QoS Configuration Constants
│   ├── services.py           # Service Name Constants
│   └── topics.py             # Topic Name Constants
├── resource/
│   └── lesson_py_template    # Ament Index Marker (Rename this!)
├── package.xml
├── setup.cfg
└── setup.py

```

## How to Use

1. **Copy the Template**:
Copy this folder into your `src/` directory (e.g., `src/1_python/lesson_03_my_lesson`).
2. **Rename the Package Configuration**:
* **`package.xml`**: Update `<name>` to your new package name.
* **`setup.py`**: Update the `package_name` variable at the top of the file.
* **`setup.cfg`**: Update the `[develop]` and `[install]` script paths if necessary (usually auto-handled by `package_name` in setup.py, but good to check).


3. **Rename Directories**:
You **MUST** rename the source directories to match the `package_name` you defined in step 2.
* Rename `lesson_py_template/` → `your_package_name/`
* Rename `resource/lesson_py_template` → `resource/your_package_name`


4. **Edit the Node**:
* Define your node logic in `your_package_name/node.py`.
* Update the `NODE_NAME` constant.
* Add shared constants to `topics.py`, `qos.py`, and `services.py` instead of hardcoding strings.