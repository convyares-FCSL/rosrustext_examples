# THEORY — Lesson 09 (Composition & Containers)

## Lesson 09 — Deployment Is a Constraint

Lessons 06–08 showed that a system can be:

* logically correct
* lifecycle-correct
* responsive under load

…and still change behaviour when **deployment topology changes**.

Lesson 09 isolates that variable.

---

## What “Composition” Actually Means

In ROS terms, composition is not an optimisation.

It means:

* multiple nodes in one process
* one executor deciding what runs and when
* shared shutdown and failure fate

Nothing about node logic changes.
Everything about *consequences* does.

---

## What Changed (And What Didn’t)

What did **not** change:

* node code
* topics, actions, or lifecycle interfaces
* execution semantics inside each node

What **did** change:

* where execution lives
* who owns the executor
* how failure propagates

Lesson 09 moves responsibility upward, into deployment.

---

## Shared Fate Is a Multiplier

When nodes share a process:

* one crash kills all nodes
* one deadlock stalls all callbacks
* one Ctrl-C ends every responsibility at once

This is neither good nor bad.
It is a property.

Lesson 09 makes that property visible.

---

## Actions Under Shared Fate

Actions are long-lived, stateful interactions.

When an action server disappears mid-goal:

* no result may be returned
* no abort may be observed
* clients may block indefinitely

In Lesson 09 this happens without any code changes.

The only difference is deployment.

---

## Why Python Stops Here

Canonical ROS 2 composition is defined by:

* discoverable component types
* loadable/unloadable components
* container-managed executors

In this environment:

* `ros2 component types` exposes only C++ components
* Python (`rclpy`) nodes are not discoverable as components
* dynamic component load/unload is unavailable for Python

Lesson 09 documents this boundary.
It does not hide it.

---

## Historical Context

The desire for Python composition is not new:

* Experimental implementations existed as early as 2018
* Feature requests remain open in core ROS 2

Despite this, Python composition has not entered the canonical contract.

Lesson 09 treats this as a **systems constraint**, not a missing tutorial step.

---

## The Architectural Takeaway

Composition is not about fewer processes.

It is about:

* shared fate
* failure amplification
* truth under deployment pressure

Language tooling determines which truths you can expose.

Lesson 09 shows exactly where Python’s tooling currently stops—and why that matters.
