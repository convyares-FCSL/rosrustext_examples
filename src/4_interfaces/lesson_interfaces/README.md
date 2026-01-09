# lesson_interfaces

Shared message/service/action definitions for lessons.

Add new interface files under:
- `msg/`
- `srv/`
- `action/`

### Configuration
Shared parameters live in `config/` and are split by type:
- `config/topics_config.yaml` for topic names.
- `config/services_config.yaml` for service names.
- `config/qos_config.yaml` for QoS defaults and profile selection.

Then update `CMakeLists.txt` to include interfaces in `rosidl_generate_interfaces()`.

### Rust Runtime & Generation

To use these messages in Rust (rclrs), the Rust bindings generator must be enabled.
This workspace includes `rosidl_generator_rs` in `src/4_interfaces/rosidl_rust`.

**To enable Rust generation for this package:**
1. Ensure `package.xml` includes:
   ```xml
   <build_depend>rosidl_generator_rs</build_depend>
   ```
2. Build the package. The Rust crate will be generated in `install/<pkg>/share/<pkg>/rust`.

