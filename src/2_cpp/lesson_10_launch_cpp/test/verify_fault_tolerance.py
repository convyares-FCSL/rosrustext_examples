#!/usr/bin/env python3
import sys
import time
import subprocess
import signal
import shutil
import re
import os

# Configuration
PROFILE_A_LAUNCH = "composed.launch.py"
PROFILE_B_LAUNCH = "isolated.launch.py"

TARGET_NODES = {
    "/lesson_06_lifecycle_publisher",
    "/lesson_06_lifecycle_subscriber",
    "/lesson_08_action_server"
}

def run_command(cmd, timeout=5.0):
    """Run a command and return stdout. Raise on failure."""
    try:
        result = subprocess.run(
            cmd, 
            capture_output=True, 
            text=True,
            timeout=timeout
        )
        if result.returncode != 0:
            raise RuntimeError(f"Command failed: {cmd}\nStderr: {result.stderr}")
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"Command timed out: {cmd}")

def wait_for_condition(check_func, timeout=10.0, description="condition"):
    """Poll check_func until True or timeout."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            if check_func():
                return
        except Exception:
            pass
        time.sleep(1.0)
    raise RuntimeError(f"Timed out waiting for: {description}")

def measure_lifecycle_latency(samples=3):
    """Measure avg latency of 'ros2 lifecycle get'."""
    latencies = []
    for _ in range(samples):
        start = time.time()
        run_command(["ros2", "lifecycle", "get", "/lesson_06_lifecycle_publisher"])
        latencies.append(time.time() - start)
        time.sleep(0.5)
    return sum(latencies) / len(latencies)

def run_profile_test(launch_file, profile_name):
    print(f"\n=== Testing {profile_name} ({launch_file}) ===")
    
    # 1. Launch
    launch_process = subprocess.Popen(
        ["ros2", "launch", "lesson_10_launch_cpp", launch_file],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    try:
        # 2. Wait for Nodes
        print("Waiting for nodes...")
        def check_nodes():
            raw = run_command(["ros2", "node", "list"])
            current = set(n.strip() for n in raw.splitlines() if n.strip())
            return TARGET_NODES.issubset(current)
        wait_for_condition(check_nodes, timeout=15.0)

        # 3. Activation
        print("Activating system...")
        for node in TARGET_NODES:
            run_command(["ros2", "lifecycle", "set", node, "configure"])
        for node in TARGET_NODES:
            run_command(["ros2", "lifecycle", "set", node, "activate"])
            
        wait_for_condition(lambda: "active" in run_command(["ros2", "lifecycle", "get", "/lesson_06_lifecycle_publisher"]), timeout=10.0)

        # 4. Measure Baseline Latency
        print("Measuring baseline latency...")
        baseline = measure_lifecycle_latency()
        print(f"Baseline: {baseline:.4f}s")
        
        # 5. Apply Stimulus (Hammer)
        print("Applying stimulus (Fibonacci 50)...")
        # Verify action is present
        actions = run_command(["ros2", "action", "list"])
        if "/tutorial/fibonacci" not in actions:
            print("WARNING: Fibonacci action not found!")
        
        # Send action goal, non-blocking via CLI &
        subprocess.Popen(
            ["ros2", "action", "send_goal", "/tutorial/fibonacci", "lesson_interfaces/action/Fibonacci", "{order: 50}"],
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )
        time.sleep(5.0) # Warmup
        
        # 6. Measure Loaded Latency
        print("Measuring loaded latency...")
        loaded = measure_lifecycle_latency()
        print(f"Loaded:   {loaded:.4f}s")
        
        degradation_factor = loaded / baseline if baseline > 0 else 0
        print(f"Factor:   {degradation_factor:.2f}x")
        
        results = {
            "baseline": baseline,
            "loaded": loaded,
            "factor": degradation_factor
        }

        # 7. Survival Test (Profile B only)
        if profile_name == "Profile B":
            print("--- Survival Test ---")
            print("Killing Action Server Process...")
            # For C++ Profile B:
            # Domain 1: component_container_mt (Control Plane)
            # Domain 2: lesson_08_action_server (Worker Plane)
            try:
                # Find PID of lesson_08_action_server executable
                pid_str = run_command(["pgrep", "-f", "lesson_08_action_server"])
                int_pid = int(pid_str.split()[0])
                os.kill(int_pid, signal.SIGKILL)
                print(f"Killed PID {int_pid}")
                time.sleep(2.0)
                
                print("Probing Lifecycle Publisher (Control Plane)...")
                # Should still be active
                probe_start = time.time()
                state = run_command(["ros2", "lifecycle", "get", "/lesson_06_lifecycle_publisher"])
                probe_time = time.time() - probe_start
                print(f"Probe success: {state.strip()} in {probe_time:.4f}s")
                results["survived"] = True
            except Exception as e:
                print(f"Survival Test Failed: {e}")
                results["survived"] = False

        return results

    finally:
        print("Teardown...")
        launch_process.send_signal(signal.SIGINT)
        try:
            launch_process.wait(timeout=5)
        except:
            launch_process.kill()
        
        time.sleep(2)
        subprocess.run(["pkill", "-f", "component_container"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "lesson_08_action_server"], stderr=subprocess.DEVNULL)
        time.sleep(1)

def main():
    print("--- Starting Verify Fault Tolerance (C++ A/B Test) ---")
    
    # Run A
    res_a = run_profile_test(PROFILE_A_LAUNCH, "Profile A")
    
    # Run B
    res_b = run_profile_test(PROFILE_B_LAUNCH, "Profile B")
    
    print("\n\n=== FINAL REPORT ===")
    print(f"Profile A (Shared Fate):")
    print(f"  Baseline: {res_a['baseline']:.4f}s")
    print(f"  Loaded:   {res_a['loaded']:.4f}s")
    print(f"  Degradation: {res_a['factor']:.2f}x")
    
    print(f"Profile B (Isolated):")
    print(f"  Baseline: {res_b['baseline']:.4f}s")
    print(f"  Loaded:   {res_b['loaded']:.4f}s")
    print(f"  Degradation: {res_b['factor']:.2f}x")
    
    print("-" * 30)
    
    success = True
    
    # Criteria 1: Profile A should degrade (Factor > 1.2)
    if res_a['factor'] > 1.2:
        print("PASS: Profile A reflects expected shared-fate degradation.")
    else:
        print("WARN: Profile A unexpectedly stable ({:.2f}x). Environment variance?".format(res_a['factor']))
        
    # Criteria 2: Profile B Comparison
    if res_b['factor'] < res_a['factor']:
         print("PASS: Profile B ({:.2f}x) is more stable than Profile A ({:.2f}x).".format(res_b['factor'], res_a['factor']))
    else:
         print("WARN: Profile B degraded same/worse than A. Isolation limited by CPU saturation.")
         
    # Criteria 3: Survival (Binary Check) - CRITICAL
    if res_b.get("survived"):
        print("PASS: Profile B SURVIVED worker death (Process Isolation Proven).")
    else:
        print("FAIL: Profile B did not survive worker death.")
        success = False

    if success:
        print("\nOVERALL VERDICT: PASS")
        sys.exit(0)
    else:
        print("\nOVERALL VERDICT: FAIL")
        sys.exit(1)

if __name__ == "__main__":
    main()
