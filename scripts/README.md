# Build Scripts

This directory contains modular scripts to build the workspace in a deterministic, repeatable order across all language tracks.

The scripts exist to make **cross-language parity reproducible**, not to hide build mechanics.

---

## Structure

```text
scripts/
├── 01_setup/
│   └── build_interfaces.sh       # Step 1: Build shared interfaces
│                                 # (Includes Rust interface generation & validation)
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
├── clean_all.sh                  # Helper: Cleans build/, install/, and target/
└── env_setup.sh                  # Helper: Sanitizes environment variables to suppress warnings
```

---

## Features

* **Deterministic ordering**
  Interfaces → utilities → lessons, mirroring dependency structure across languages.

* **Environment sanitisation**
  All scripts source `env_setup.sh` to clean `AMENT_PREFIX_PATH` and `CMAKE_PREFIX_PATH`, suppressing misleading “path does not exist” warnings.

* **Clear failure boundaries**
  Each stage fails fast and reports status explicitly; `build_all.sh` provides a green/red summary.

---

## Important Note: Rust Interface Generation

The `01_setup/build_interfaces.sh` script does **more than build custom messages**.

For Rust (`rclrs`) specifically, it also:

* Overlays core ROS interface packages (e.g. `rcl_interfaces`)
* Enables `rosidl_generator_rs`
* Generates **workspace-local Rust crates** under `install/**/share/**/rust`
* Validates that generated `Cargo.toml` files use **path dependencies**, not crates.io

This is **required** for Native Rust nodes to build reliably.

> **Do not skip this step for Rust.**
> The rationale and failure mode are documented in detail in:
> `src/3_rust/README.md` → *“Rust Interface Generation & Local Overlay”*

The script intentionally centralises this logic so Rust lessons behave like C++ and Python peers once built.

---

## Usage

### Build Everything

> **Important**
> Run Lesson 00 first to ensure system dependencies are installed.

```bash
./scripts/build_all.sh
```

---

### Build by Track

If you prefer to build a specific language track, you **must always build interfaces first**.

#### 1. Setup (Required for all tracks)

```bash
./scripts/01_setup/build_interfaces.sh
```

---

#### 2. Python Track

```bash
./scripts/02_bootstrap/build_python.sh
./scripts/03_lessons/build_python.sh
```

---

#### 3. C++ Track

```bash
./scripts/02_bootstrap/build_cpp.sh
./scripts/03_lessons/build_cpp.sh
```

---

#### 4. Native Rust (`rclrs`) Track

```bash
./scripts/02_bootstrap/build_rclrs.sh
./scripts/03_lessons/build_rclrs.sh
```

---

#### 5. Bridge Rust (`rcllibrust`) Track

```bash
# Pure Cargo builds, intentionally ignored by Colcon
./scripts/02_bootstrap/build_rcllibrust.sh
./scripts/03_lessons/build_rcllibrust.sh
```

---

## Design Intent (Why this exists)

These scripts are **not convenience wrappers**.

They encode:

* the correct build graph
* cross-language interface parity
* Rust-specific constraints that Cargo will not infer
* a clean separation between *infrastructure* and *lessons*

If a script feels “strict”, that is intentional — it prevents silent divergence between Python, C++, and Rust tracks.