# Build Scripts

This directory contains modular scripts to build the workspace in a deterministic order.

## Structure

```text
scripts/
├── 01_setup/
│   └── build_interfaces.sh       # Step 1: Build shared messages
├── 02_bootstrap/
│   ├── build_python.sh           # Step 2: Build utils & bootstrap for Python
│   ├── build_cpp.sh              # Step 2: Build utils & bootstrap for C++
│   ├── build_rclrs.sh            # Step 2: Build utils & bootstrap for Native Rust
│   └── build_rcllibrust.sh       # Step 2: Build utils & bootstrap for Bridge Rust
├── 03_lessons/
│   ├── build_python.sh           # Step 3: Build all Python lessons
│   ├── build_cpp.sh              # Step 3: Build all C++ lessons
│   ├── build_rclrs.sh            # Step 3: Build all Native Rust lessons
│   └── build_rcllibrust.sh       # Step 3: Build all Bridge Rust lessons
├── build_all.sh                  # Master script: Runs everything in order
├── clean_all.sh                  # Helper: Cleans build/ install/ and target/
└── env_setup.sh                  # Helper: Sanitizes environment variables to suppress warnings
```

## Features

- **Warnings Suppression**: All scripts automatically source `env_setup.sh` to clean `AMENT_PREFIX_PATH`, removing annoying "path doesn't exist" warnings.
- **Success Summaries**: `build_all.sh` provides clear Green/Red status summaries for each build stage.


## Usage

### Build Everything
```bash
./scripts/build_all.sh
```

### Build by Track
If you prefer to build a specific language track, **you must always build dependencies (interfaces) first**.

#### 1. Setup (Required)
```bash
./scripts/01_setup/build_interfaces.sh
```

#### 2. Python Track
```bash
./scripts/02_bootstrap/build_python.sh
./scripts/03_lessons/build_python.sh
```

#### 3. C++ Track
```bash
./scripts/02_bootstrap/build_cpp.sh
./scripts/03_lessons/build_cpp.sh
```

#### 4. Native Rust (`rclrs`) Track
```bash
./scripts/02_bootstrap/build_rclrs.sh
./scripts/03_lessons/build_rclrs.sh
```

#### 5. Bridge Rust (`rcllibrust`) Track
```bash
# Note: These are pure Cargo builds and are ignored by Colcon
./scripts/02_bootstrap/build_rcllibrust.sh
./scripts/03_lessons/build_rcllibrust.sh
```
