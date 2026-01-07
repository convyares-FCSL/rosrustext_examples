# TODO – ROS 2 Lessons Workspace

This file tracks structural, pedagogical, and technical follow-ups for the
ROS 2 multi-language lesson series.

The goal is not feature creep, but clarity, consistency, and professional realism.

---

## High Priority (Structure & Consistency)

- [ ] Normalize directory layout across all tracks
  - Confirm `1_python / 2_cpp / 3_rust / 4_interfaces` is final
  - Ensure all lessons follow `lesson_XX_<topic>` naming

- [ ] Ensure Lesson 00 exists and builds for all languages
  - Python ✔
  - C++ ✔
  - rclrs ✔
  - rcllibrust ✔

- [ ] Add missing `Lesson.md` files where placeholders exist
  - lesson_01_node
  - lesson_02_publisher
  - lesson_03_subscriber

- [ ] Verify all lesson packages share the same topic/service/action names

---

## Configuration & Parameters

- [ ] Finalize config file schema in `src/4_interfaces/config/`
  - topics_config.yaml
  - qos_config.yaml
  - services_config.yaml

- [ ] Ensure utils libraries exist and are minimal:
  - utils_py
  - utils_cpp
  - utils_rclrs

- [ ] Document parameter-loading pattern clearly:
  - developer mode (source tree)
  - installed mode (package share path)

---

## Lesson Content Roadmap

- [ ] Lesson 04 – Services
- [ ] Lesson 05 – Parameters
- [ ] Lesson 06 – Lifecycle nodes
- [ ] Lesson 07 – Actions
- [ ] Lesson 08 – Executors and callback groups
- [ ] Lesson 09 – Launch files and configuration discovery

Each lesson should:
- introduce one new concept
- reuse previous patterns
- avoid hiding behaviour behind utilities unless explained

---

## Rust-Specific

- [ ] Lock rclrs version (tag/commit) and document it
- [ ] Decide whether utils_rclrs is versioned or path-based
- [ ] Ensure rcllibrust lessons remain Cargo-only
- [ ] Keep logging model differences explicit (ROS logging vs Rust logging)

---

## Capstone Preparation

- [ ] Create separate Capstone repository
- [ ] Link Lesson 10 to Capstone repo
- [ ] Decide final Capstone scope:
  - languages involved
  - lifecycle usage
  - bringup structure

---

## Documentation & Cleanup

- [ ] Review top-level README after Lesson 09 is complete
- [ ] Add diagrams only where they clarify behaviour
- [ ] Remove `src_archive` once content is migrated or discarded
- [ ] Ensure `.gitignore` covers:
  - build/
  - install/
  - log/
  - target/
  - IDE files

---

## Non-Goals (Explicitly Out of Scope)

- Teaching ROS 1
- Teaching DDS internals
- Teaching Rust language basics
- Teaching Python or C++ syntax
