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
