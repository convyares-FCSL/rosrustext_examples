# Interfaces and Tools

This directory acts as the **Central Source of Truth** for the entire workspace.
It contains the shared data structures (Messages) and shared configuration (Topics/QoS) used by Python, C++, and Rust nodes.

---

## Directory Structure

```text
4_interfaces/
├── lesson_interfaces/       # The ROS 2 Interface Package
│   ├── action/              # .action definitions
│   ├── config/              # Shared Configuration (YAML)
│   │   ├── qos_config.yaml
│   │   ├── services_config.yaml
│   │   └── topics_config.yaml
│   ├── msg/                 # .msg definitions (e.g., MsgCount.msg)
│   ├── srv/                 # .srv definitions
│   ├── act/                 # .act definitions
│   ├── CMakeLists.txt
│   └── package.xml
│
└── rosidl_rust/             # Rust Message Generator Tools
    └── rosidl_generator_rs  # The actual generator logic

```

---

## Packages

### 1. `lesson_interfaces`

This package defines the communication contract. It includes:

* **Messages**: Custom data types (like `MsgCount`) to demonstrate generating code from `.msg` files.
* **Services**: Custom data types (like `ComputeStats`) to demonstrate generating code from `.srv` files.
* **Actions**: Custom data types (like `Fibonacci`) to demonstrate generating code from `.act` files.
* **Config**: YAML files that define topic names and QoS profiles. This ensures that a Python Publisher and a Rust Subscriber always agree on the topic name (e.g., `/chatter`) without hardcoding strings in source code.

### 2. `rosidl_rust`

This directory contains `rosidl_generator_rs`. This is the tool that plugs into the ROS 2 build system (`ament`) to generate Rust code from `.msg` files.

* **Note**: You do not typically edit this package; it is a build dependency required to make Rust nodes work with custom messages.

---

## Usage in Rust

Rust nodes cannot "just import" messages like C++ or Python. You must configure them to look for the **generated artifacts** produced by this folder.

To use `lesson_interfaces` in a Rust node:

1. **Enable Generator**: Ensure `lesson_interfaces/package.xml` has this dependency so the build system knows to run the Rust generator:
```xml
<build_depend>rosidl_generator_rs</build_depend>

```


2. **Dependencies**: In your node's `Cargo.toml`, you must point to the **installed** location of the generated Rust code, not the source folder:
```toml
[dependencies]
# We point to the 'install' space because that is where Colcon puts the generated Rust code.
lesson_interfaces = { path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust" }

