# Test Harness (Language-Oriented)

This directory contains validation suites for **all** lessons (1-6), organized by language to mirror the `build_all.sh` structure.

## Suites

### `run_python.sh`
Executes lessons 1-6 using Python nodes.
- **Checks**: Node startup, Topic presence, Data flow, Service logic, Param reads, Lifecycle orchestration.

### `run_cpp.sh`
Executes lessons 1-6 using C++ nodes.
- **Checks**: Equivalent functionality validation for C++ implementations.
- **Note**: Requires C++ packages to be built.

### `run_rclrs.sh`
Executes lessons 1-6 using Rust (rclrs) nodes.
- **Checks**: Equivalent functionality validation for Rust implementations.
- **Note**: Requires `rclrs` packages to be built.

## Usage

```bash
# Run Everything (Summary Table)
./scripts/04_tests/run_all.sh

# Run Specific Language Suite
./scripts/04_tests/run_python.sh
```

**Prerequisities**: Ensure the workspace is fully built (`scripts/build_all.sh`) before running tests. Test failures may occur if dependencies (like `lesson_interfaces` or `lesson_05_parameters_py`) are missing from the install tree.
