# Rust Template (`rclrs`)

This is a minimal Rust package template that enforces the **State/Params/Node** separation pattern for scalability.

## Directory Structure

```text
.
├── src/
│   ├── main.rs           # Node Logic + Entry Point
│   ├── qos.rs            # QoS Configuration Constants
│   ├── services.rs       # Service Name Constants
│   └── topics.rs         # Topic Name Constants
├── Cargo.toml            # Rust Dependencies
├── package.xml           # Colcon Build Metadata
└── README.md

```

## How to Use

1. **Copy the Template**:
Copy this folder into your `src/3_rust/1_rclrs/` directory.
2. **Rename the Package**:
* **`Cargo.toml`**: Update `name` to your new package name.
* **`package.xml`**: Update `<name>` to match. This is critical for `colcon` to find it.


3. **Update Dependencies**:
* **`Cargo.toml`**: Uncomment `lesson_interfaces` to link against the generated Rust code.
* **`package.xml`**: Uncomment `<depend>lesson_interfaces</depend>` so `colcon` knows to build the messages *before* building this node.


4. **Edit the Node**:
* **`src/main.rs`**: Define your immutable config in `NodeParams` and mutable logic in `NodeState`.
* **Helpers**: Add shared constants to `topics.rs`, `qos.rs`, and `services.rs`.