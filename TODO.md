# TODO – ROS 2 Lessons Workspace

This file tracks the structural, pedagogical, and technical status of the ROS 2 multi-language lesson series.

---

## 1. High Priority (Structure & Consistency)

- [X] **Normalize Directory Layout**
  - Layout: `1_python / 2_cpp / 3_rust / 4_interfaces`
  - Naming: `lesson_XX_<topic>`

- [X] **Lesson 00 (Bootstrap)**
  - Ensures workspace builds/runs across all 4 tracks.

- [X] **Documentation Baseline**
  - [X] Restore full roadmap (Lessons 00-09) to root `README.md`.
  - [X] Document architectural progression (Container -> Event Loop -> Composition).

---

## 2. Shared Utilities & Configuration

**Strategy**: We use "Shared Utils" packages to prevent magic strings in lesson code.
* *Current State*: Utils return hardcoded constants (mirrors of the future config).
* *Future State (Lesson 05)*: Utils will read the YAML files from `4_interfaces/config`.

### Utility Packages (Structure Exists)
- [X] **`utils_py`** (`src/1_python/utils_py`)
  - [X] Package structure created.
  - [ ] **Action**: Confirm `setup.py` installs it so lessons can `import utils_py`.
- [X] **`utils_cpp`** (`src/2_cpp/utils_cpp`)
  - [X] Package structure created.
  - [ ] **Action**: Confirm `CMakeLists.txt` exports headers so lessons can `find_package(utils_cpp)`.
- [X] **`utils_rclrs`** (`src/3_rust/1_rclrs/utils_rclrs`)
  - [X] Fully functional (returning hardcoded profiles).
- [X] **`utils_roslibrust`** (`src/3_rust/2_rcllibrust/utils_roslibrust`)
  - [X] Fully functional (returning hardcoded options).

### Configuration Specs (`src/4_interfaces/config`)
- [X] **File Structure Created**:
  - `topics_config.yaml` (Defines topic names)
  - `qos_config.yaml` (Defines "telemetry", "state", etc. profiles)
  - `services_config.yaml` (Defines service names)
- [ ] **Validation**: Ensure content matches the hardcoded values currently in Utils.

---

## 3. Lesson Content Matrix

**Legend**:
- **[X]**: Completed & Verified.
- **[S]**: Structure/Folder exists (Work In Progress).
- **[ ]**: Not started.

| Lesson | Topic | Python | C++ | rclrs | rcllibrust | scripts |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **00** | **Bootstrap** | [X] | [X] | [X] | [X] | [X] |
| **01** | **Node / Timer** | [X] | [X] | [X] | [X] | [X] |
| **02** | **Publisher** | [X] | [X] | [X] | [X] | [X] |
| **03** | **Subscriber** | [X] | [X] | [X] | [X] | [X] |
| **04** | **Services** | [X] | [X] | [ ] | [ ] | [ ] |
| **05** | **Parameters** | [ ] | [ ] | [ ] | [ ] | [ ] |
| **06** | **Lifecycle** | [ ] | [ ] | [ ] | [ ] | [ ] |
| **07** | **Actions** | [ ] | [ ] | [ ] | [ ] | [ ] |
| **08** | **Executors** | [ ] | [ ] | [ ] | [ ] | [ ] |
| **09** | **Launch** | [ ] | [ ] | [ ] | [ ] | [ ] |


---

## 4. Rust-Specific Tasks

- [X] **Dependency Locking**:
  - `rclrs`: Pinned to git tag.
  - `roslibrust`: Pinned to `0.18`.
- [X] **Architecture Refactor**: Split utils into Native vs. Bridge.
- [X] **Build Isolation**: `rcllibrust` lessons marked `COLCON_IGNORE`.
- [ ] **Logging consistency**: Ensure `env_logger` (bridge) output format roughly matches ROS 2 logger (native).

---

## 5. Capstone Preparation

- [ ] Create separate Capstone repository
- [ ] Link Lesson 10 to Capstone repo
- [ ] Decide final Capstone scope:
  - languages involved
  - lifecycle usage
  - bringup structure

---

## 6. Documentation & Cleanup

- [ ] Verify all lesson packages share the same topic/service/action names
- [ ] Review top-level README after Lesson 09 is complete
- [ ] Add diagrams only where they clarify behaviour
- [ ] Remove `src_archive` once content is migrated or discarded
- [ ] Ensure `.gitignore` covers:
  - `build/`
  - `install/`
  - `log/`
  - `target/`
  - IDE files

---

## 7. Non-Goals (Explicitly Out of Scope)

- Teaching ROS 1
- Teaching DDS internals
- Teaching Rust language basics
- Teaching Python or C++ syntax
