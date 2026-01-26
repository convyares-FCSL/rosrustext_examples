import subprocess
import time
import sys
import os
import signal
import shutil

def run_command(cmd, shell=False, timeout=5.0):
    """Run a command and return stdout. Raise on failure."""
    # print(f"DEBUG: Running: {cmd}")
    try:
        result = subprocess.run(
            cmd, 
            shell=shell, 
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

def main():
    print("--- 1. Launching System (Background) ---")
    
    # Check dependencies
    if not shutil.which("ros2"):
        print("ERROR: ros2 not found in PATH")
        sys.exit(1)

    # Launch in background, inheriting stdout/stderr to avoid pipe blocking
    launch_process = subprocess.Popen(
        ["ros2", "launch", "lesson_10_launch", "composed.launch.py"],
        stdout=None, # Inherit
        stderr=None, # Inherit
        text=True
    )
    
    try:
        # Wait for nodes to appear (Cold Start Contract)
        print("Waiting for nodes to initialize...")
        
        target_nodes = {
            "/lesson_06_lifecycle_publisher",
            "/lesson_06_lifecycle_subscriber",
            "/lesson_08_action_server"
        }
        
        def check_nodes():
            raw = run_command(["ros2", "node", "list"])
            current = set(n.strip() for n in raw.splitlines() if n.strip())
            return target_nodes.issubset(current)
            
        wait_for_condition(check_nodes, timeout=15.0, description="nodes to appear")
        
        print("--- 2. Asserts Cold-Start Contract ---")
        
        def check_cold_contract():
            # Assert Action Absent
            act_list = run_command(["ros2", "action", "list", "-t"])
            if "/tutorial/fibonacci" in act_list:
                raise RuntimeError("VIOLATION: Action /tutorial/fibonacci present before activation!")
                
            # Assert Telemetry Absent
            top_list = run_command(["ros2", "topic", "list"])
            if "/tutorial/telemetry" in top_list:
                raise RuntimeError("VIOLATION: Topic /tutorial/telemetry present before activation!")
            return True

        # Retry assertions briefly to allow discovery to settle if needed
        wait_for_condition(check_cold_contract, timeout=5.0, description="cold-start contract")
        print("✓ Cold-start contract verified (No action/telemetry visible)")

        print("--- 3. Driving Lifecycle Transitions ---")
        targets = [
            "/lesson_06_lifecycle_publisher",
            "/lesson_06_lifecycle_subscriber",
            "/lesson_08_action_server"
        ]
        
        for node in targets:
            print(f"Configuring {node}...")
            run_command(["ros2", "lifecycle", "set", node, "configure"])
            
        for node in targets:
            print(f"Activating {node}...")
            run_command(["ros2", "lifecycle", "set", node, "activate"])
            
        print("Waiting for activation state checks...")
        
        def check_lifecycle_active():
            # Verify all nodes report 'active' state
            for node in targets:
                state_raw = run_command(["ros2", "lifecycle", "get", node])
                if "State: active" not in state_raw: 
                    return False
            return True

        wait_for_condition(check_lifecycle_active, timeout=10.0, description="all nodes active")
        print("✓ Lifecycle state active verified for all nodes")
        
        print("--- 4. Asserts Active Contract ---")
        
        def check_active_contract():
            # Assert Action Present
            act_list = run_command(["ros2", "action", "list", "-t"])
            if "/tutorial/fibonacci" not in act_list:
                 return False # Retry
            # Assert Telemetry Present
            top_list = run_command(["ros2", "topic", "list"])
            if "/tutorial/telemetry" not in top_list:
                 return False # Retry
            return True

        wait_for_condition(check_active_contract, timeout=10.0, description="active contract (discovery)")
        print("✓ Active contract verified (Action/Telemetry present)")
        
        print("--- 4a. Action Exercise ---")
        # Send quick goal, ensure success.
        # Don't leave hanging client process.
        print("Sending short goal (order: 5)...")
        # Using subprocess.run will wait for completion, effectively ensuring cleanup
        run_command([
            "ros2", "action", "send_goal", 
            "/tutorial/fibonacci", 
            "lesson_interfaces/action/Fibonacci", 
            "{order: 5}"
        ], timeout=10.0)
        print("✓ Action goal executed successfully")

        print("--- 5. Clean Shutdown & Convergence ---")
        
    finally:
        # Terminate launch
        if launch_process.poll() is None:
            print("Terminating launch process (SIGINT)...")
            launch_process.send_signal(signal.SIGINT)
            try:
                launch_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("Launch process did not exit gracefully, killing...")
                launch_process.kill()
            
        time.sleep(2)
        
        print("Verifying graph convergence...")
        
        def check_shutdown():
            # Check nodes gone
            n_list = run_command(["ros2", "node", "list"])
            remaining_nodes = [n for n in n_list.splitlines() if n.strip() in target_nodes]
            if remaining_nodes:
                return False
                
            # Check action gone
            a_list = run_command(["ros2", "action", "list", "-t"])
            if "/tutorial/fibonacci" in a_list:
                return False
                
            # Check telemetry publishers count (Robust check instead of strict disappearance)
            # Topics may linger in DDS discovery, but publisher count must be 0
            info_raw = run_command(["ros2", "topic", "info", "/tutorial/telemetry"], timeout=2.0)
            if "Publisher count: 0" not in info_raw:
                # If topic disappeared completely, that's fine too (command might fail or return different output)
                # But if present, must have 0 pubs.
                # If command failed because topic absent, that's success.
                # Let's handle this carefully.
                # Actually, run_command raises on failure.
                # If info command fails, assume topic is gone (Success).
                pass
            
            return True

        
        try:
            # We wrap the topic info check separately to handle "topic not found" failure as success
            def check_shutdown_robust():
                 # Nodes gone?
                n_list = run_command(["ros2", "node", "list"])
                remaining_nodes = [n for n in n_list.splitlines() if n.strip() in target_nodes]
                if remaining_nodes:
                    return False
                
                # Action gone?
                a_list = run_command(["ros2", "action", "list", "-t"])
                if "/tutorial/fibonacci" in a_list:
                    return False

                # Telemetry publishers gone?
                try:
                    info_raw = run_command(["ros2", "topic", "info", "/tutorial/telemetry"], timeout=2.0)
                    if "Publisher count: 0" not in info_raw:
                        return False # Still has publishers
                except RuntimeError:
                    # Topic info failed, meaning topic likely gone from list -> Success
                    pass
                    
                return True

            wait_for_condition(check_shutdown_robust, timeout=10.0, description="graph convergence")
            print("✓ Graph converged to empty (Correct)")
        except RuntimeError as e:
            print(f"FAILURE: {e}")
            # Diagnostic dump
            print("Nodes: ", run_command(["ros2", "node", "list"]))
            print("Actions: ", run_command(["ros2", "action", "list", "-t"]))
            sys.exit(1)
             
    print("\nVERIFICATION SUCCESSFUL")

if __name__ == "__main__":
    main()
