# Theory — Lesson 08 (Executors & Availability)

## The Problem: Scheduling Starvation

In Lesson 07, the node was logically correct but operationally unavailable during long-running actions. This occurred because a **SingleThreadedExecutor** serializes all callbacks. If one callback (the action execution) blocks, no others (timers, services) can run.

## The Solution: Multi-Threaded Scheduling

Lesson 08 introduces the **MultiThreadedExecutor**. This executor maintains a pool of worker threads. When a callback is ready to run, it is assigned to an available thread.

This implies that:
1.  **Concurrency** is possible (two callbacks running at the same time).
2.  **Blocking** one thread does not necessarily stall the entire node (other threads remain free).

## Callback Groups: Managing Safety

Concurrency introduces risk: shared state might be accessed simultaneously (Data Races). ROS 2 uses **Callback Groups** to define thread-safety rules.

### 1. Mutually Exclusive Group
*   **Behavior**: Only one callback from this group can run at a time.
*   **Use Case**: Protecting shared resources (like class member variables) without explicit mutexes.
*   **Applied Here**: The **Telemetry Timer** uses a Mutually Exclusive group to ensure sequential publishing.

### 2. Reentrant Group
*   **Behavior**: Multiple callbacks from this group can run concurrently (on different threads).
*   **Use Case**: Thread-safe workloads or those that must scale/overlap.
*   **Applied Here**: The **Action Server** uses a Reentrant group. This allows the "Cancel Goal" request (a service call essentially) to be processed *while* the "Execute Goal" callback is still running on another thread.

## Observational Outcome

By moving the blocking action to a threaded pool (via Reentrant group) and isolating the telemetry timer (via Mutually Exclusive group), the node enables independent progress.

*   **Action** uses Thread A (Blocking).
*   **Telemetry** uses Thread B (Periodic).
*   **Lifecycle/Services** use Thread C (On Demand).

The external observer sees a "responsive" node, even though the heavy computation is still blocking one of the threads.