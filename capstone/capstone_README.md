# ROS 2 Capstone – Multi-Language Bringup

This repository is a **capstone project** that demonstrates how a
professional ROS 2 system is structured and brought up in practice.

It is intended to be used **after completing Lessons 00–09** in the
ROS 2 Lessons Workspace.

---

## Purpose

The capstone shows how to combine:

- Multiple languages (Python, C++, Rust)
- Shared interfaces and configuration
- Lifecycle-managed nodes
- Launch files and bringup packages

into a single, coherent ROS 2 system.

This is not a step-by-step tutorial.
It is a **reference implementation**.

---

## What This Project Demonstrates

- Mixed-language ROS 2 nodes in one system
- A dedicated bringup package
- Launch file hierarchy
- Lifecycle nodes + lifecycle manager
- Shared topic, service, and action definitions
- Shared configuration (topics, QoS, services)
- Clean startup, shutdown, and error handling

---

## Architecture Overview

Example system composition:

- **C++**
  - High-rate publisher
  - Lifecycle-managed data producer

- **Rust (rclrs)**
  - Deterministic processing node
  - Explicit executor and callback group usage

- **Python**
  - Supervisor / orchestration node
  - Parameter updates and monitoring

- **Interfaces**
  - Shared messages, services, actions
  - Centralized configuration

All nodes are started via launch files and communicate using consistent naming
and configuration.

---

## Repository Layout (Planned)

```text
src/
├─ bringup/
│  ├─ launch/
│  └─ config/
│
├─ cpp_nodes/
├─ python_nodes/
├─ rust_nodes/
│  └─ rclrs/
│
├─ interfaces/
│  ├─ msg/
│  ├─ srv/
│  └─ action/
````

---

## Relationship to Lessons Workspace

This capstone builds directly on concepts introduced in the lessons:

| Lesson | Concept Used              |
| ------ | ------------------------- |
| 02–03  | Topics + QoS              |
| 04     | Services                  |
| 05     | Parameters                |
| 06     | Lifecycle                 |
| 07     | Actions                   |
| 08     | Executors                 |
| 09     | Launch + config discovery |

No new ROS concepts are introduced here.
Only composition and integration.

---

## Intended Audience

* Engineers who understand basic ROS 2 concepts
* Teams working in multi-language environments
* Developers transitioning from “tutorial ROS” to “production ROS”

---

## Status

This repository is a **placeholder**.

The implementation will be added once:

* Lesson 09 is complete
* Configuration patterns are stable
* Language tracks have converged behaviourally

---

## Non-Goals

* Teaching ROS basics
* Teaching language syntax
* Benchmarking performance
* Providing a reusable framework

This is an example, not a product.