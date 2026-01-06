# lesson_interfaces

Shared message/service/action definitions for lessons.

Add new interface files under:
- `msg/`
- `srv/`
- `action/`

### Configuration
The `config.yaml` file in this directory contains shared parameters for topics and QoS profiles used across all lessons.

Then update `CMakeLists.txt` to include interfaces in `rosidl_generate_interfaces()`.
